#include "spider_gait.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DEG2RAD(x) ((x) * (M_PI / 180.0f))
#define RAD2DEG(x) ((x) * (180.0f / M_PI))

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline float wrap01(float t)
{
    t -= (float)(int)t;      /* trunc，非 floor：负相位也能正确归一 */
    if (t < 0.0f) t += 1.0f;
    return t;
}

static inline float smoothstep(float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/*
 * 摆动相内的时序（占摆动相自身的比例，取自 params.yaml swing_profile）。
 *
 *   lift  [0.00, 0.15]  离地
 *   reach [0.15, 0.55]  前摆到落点
 *   place [0.55, 0.70]  落下
 *   push  [0.70, 1.00]  支撑（足端已锚定，推进由机身积分产生）
 *
 * 水平方向在 reach 段完成，两端用 smoothstep 保证零速——落脚不拍地。
 * push 段足端锚定不动，推进来自机身积分，这是支撑足零滑移的来源。
 */
static const float SW_REACH_A = 0.15f;
static const float SW_REACH_B = 0.55f;
static const float SW_PLACE_B = 0.70f;

void gait_config_defaults(gait_config_t *cfg, gait_type_t type)
{
    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->type = type;
    cfg->stride_mm = (type == GAIT_TROT) ? SPIDER_TROT_STRIDE_MM
                                         : SPIDER_CRAWL_STRIDE_MM;
    cfg->period_s  = (type == GAIT_TROT) ? SPIDER_TROT_PERIOD_S
                                         : SPIDER_CRAWL_PERIOD_S;
    cfg->lift_mm   = (type == GAIT_TROT) ? SPIDER_TROT_LIFT_MM
                                         : SPIDER_CRAWL_LIFT_MM;
    cfg->speed_pct = 50.0f;
    cfg->heading_deg = 0.0f;
    cfg->height_target_mm = SPIDER_STAND_HEIGHT_MM;
    cfg->height_mode = GAIT_HEIGHT_SWING;
}

static float leg_duty(const gait_t *g)
{
    return (g->cfg.type == GAIT_TROT) ? SPIDER_TROT_DUTY_SWING
                                      : SPIDER_CRAWL_DUTY_SWING;
}

/* 腿 i 的摆动相起始相位 */
static float leg_phase_start(const gait_t *g, uint8_t leg)
{
    /* 爬行：FL FR RL RR 严格错开 1/4 周期
     * Trot：相 0 = FL+RR，相 1 = FR+RL
     */
    if (g->cfg.type == GAIT_TROT) {
        static const float trot_phase[4] = {0.00f, 0.50f, 0.50f, 0.00f};
        return trot_phase[leg];
    }
    return 0.25f * (float)leg;
}

bool gait_leg_swinging(const gait_t *g, uint8_t leg)
{
    if (leg >= GAIT_LEG_COUNT) {
        return false;
    }
    float start = leg_phase_start(g, leg);
    float local = wrap01(g->phase - start);
    return local < leg_duty(g);
}

void gait_foot_world(const gait_t *g, uint8_t leg, float *x_mm, float *y_mm)
{
    if (leg >= GAIT_LEG_COUNT) {
        return;
    }
    if (x_mm) *x_mm = g->foot_world_x[leg];
    if (y_mm) *y_mm = g->foot_world_y[leg];
}

static void hip_world(const gait_t *g, uint8_t leg, float *hx, float *hy);

/* 某足端相对机身中心的横向坐标（机身系 Y），用于精确差速步幅 */
static float foot_body_y(const gait_t *g, uint8_t leg)
{
    float hx, hy;
    hip_world(g, leg, &hx, &hy);
    float dx = g->foot_world_x[leg] - hx;
    float dy = g->foot_world_y[leg] - hy;
    return dx * sinf(DEG2RAD(-g->body.yaw_deg))
         + dy * cosf(DEG2RAD(-g->body.yaw_deg));
}

/* 机身位姿下某腿髋轴心的世界坐标 */
static void hip_world(const gait_t *g, uint8_t leg, float *hx, float *hy)
{
    const spider_leg_t *L = &SPIDER_LEGS[leg];
    float c = cosf(DEG2RAD(g->body.yaw_deg));
    float s = sinf(DEG2RAD(g->body.yaw_deg));
    *hx = g->body.x_mm + (L->hip_x_mm * c - L->hip_y_mm * s);
    *hy = g->body.y_mm + (L->hip_x_mm * s + L->hip_y_mm * c);
}

/*
 * 由足端世界坐标反解关节角。
 *
 * dx/dy 为足端相对髋轴心的世界向量；先旋回机身系，
 * 再投影到本腿平面：along = 沿腿平面前进量，lateral = 侧向残差。
 *
 * lateral 不为 0 说明机身沿了腿平面之外的方向移动，而 2DOF 腿无法跟随
 * —— 这部分必然变成足端滑移。它同时也是髋偏角的来源。
 */
static void solve_leg(const gait_t *g, uint8_t leg,
                      float foot_wx, float foot_wy,
                      float height_mm, float lift_mm,
                      gait_leg_cmd_t *out)
{
    float hx, hy;
    hip_world(g, leg, &hx, &hy);

    float dx = foot_wx - hx;
    float dy = foot_wy - hy;

    /* 世界 -> 机身（反向旋转 -yaw） */
    float c = cosf(DEG2RAD(-g->body.yaw_deg));
    float s = sinf(DEG2RAD(-g->body.yaw_deg));
    float bx = dx * c - dy * s;
    float by = dx * s + dy * c;

    /* 投影到腿平面：u = 平面方向，v = 平面法向 */
    float psi = DEG2RAD(SPIDER_LEGS[leg].psi_deg);
    float ux = cosf(psi), uy = sinf(psi);
    float along = bx * ux + by * uy;
    float lateral = -bx * uy + by * ux;

    float delta_psi = RAD2DEG(atan2f(lateral, along));

    /* 身高 -> 膝角；摆动腿额外抬起 lift_mm */
    float h = height_mm - lift_mm;

    leg_ik_t ik;
    leg_ik(delta_psi, h, &ik);

    out->hip_deg = ik.hip_deg;
    out->knee_beta_deg = ik.knee_beta_deg;
    out->height_mm = height_mm;
}

void gait_init(gait_t *g, const gait_config_t *cfg)
{
    if (g == NULL || cfg == NULL) {
        return;
    }
    memset(g, 0, sizeof(*g));
    g->cfg = *cfg;
    g->height_mm = cfg->height_target_mm;
    g->body.x_mm = 0.0f;
    g->body.y_mm = 0.0f;
    g->body.yaw_deg = cfg->heading_deg;

    /* 四足置于机身正下方的巡航半径处 */
    for (uint8_t leg = 0; leg < GAIT_LEG_COUNT; leg++) {
        const spider_leg_t *L = &SPIDER_LEGS[leg];
        float hx, hy;
        hip_world(g, leg, &hx, &hy);
        float fx = hx + L->r_mm * cosf(DEG2RAD(L->psi_deg));
        float fy = hy + L->r_mm * sinf(DEG2RAD(L->psi_deg));
        g->foot_world_x[leg] = fx;
        g->foot_world_y[leg] = fy;
        g->foot_from_x[leg] = fx;
        g->foot_from_y[leg] = fy;
        g->foot_to_x[leg] = fx;
        g->foot_to_y[leg] = fy;
        g->leg[leg].height_mm = g->height_mm;
    }
    g->initialised = true;
}

void gait_reset_phase(gait_t *g)
{
    g->phase = 0.0f;
}

bool gait_request_height(gait_t *g, float height_mm)
{
    if (g == NULL) {
        return false;
    }
    height_mm = clampf(height_mm, SPIDER_HEIGHT_MIN_MM, SPIDER_HEIGHT_MAX_MM);
    g->cfg.height_target_mm = height_mm;
    return true;
}

void gait_force_height(gait_t *g, float height_mm)
{
    if (g == NULL) {
        return;
    }
    g->height_mm = clampf(height_mm, SPIDER_HEIGHT_MIN_MM, SPIDER_HEIGHT_MAX_MM);
    g->cfg.height_target_mm = g->height_mm;
    for (uint8_t leg = 0; leg < GAIT_LEG_COUNT; leg++) {
        g->leg[leg].height_mm = g->height_mm;
    }
}

void gait_set_speed(gait_t *g, float speed_pct)
{
    speed_pct = clampf(speed_pct, 0.0f, SPIDER_SPEED_MAX_PCT);
    /* 防翻红线：高速 Trot 被压到上限（docs/gait_design.md §8） */
    if (g->cfg.type == GAIT_TROT) {
        speed_pct = clampf(speed_pct, 0.0f, SPIDER_TROT_MAX_SPEED_PCT);
    }
    g->cfg.speed_pct = speed_pct;
}

void gait_set_heading(gait_t *g, float heading_deg)
{
    if (g != NULL) {
        g->cfg.heading_deg = heading_deg;
    }
}

bool gait_update(gait_t *g, float dt_s, gait_leg_cmd_t *out_cmds)
{
    if (g == NULL || out_cmds == NULL || !g->initialised) {
        return false;
    }
    if (g->cfg.period_s <= 0.0f || dt_s <= 0.0f) {
        return false;
    }

    float duty = leg_duty(g);
    float speed_scale = g->cfg.speed_pct / SPIDER_SPEED_MAX_PCT;

    /* 巡航推进速度：stride / period * 速度百分比 */
    float base_speed = (g->cfg.stride_mm / g->cfg.period_s) * speed_scale;
    float step_mm = base_speed * g->cfg.period_s;   /* 每周期推进量 */

    /* 航向跟踪：转向速率受防翻红线约束（25°/s，不可解锁） */
    float yaw_err = wrap01((g->cfg.heading_deg - g->body.yaw_deg) / 360.0f) * 360.0f;
    if (yaw_err > 180.0f) yaw_err -= 360.0f;
    float yaw_rate = clampf(yaw_err * 4.0f,
                            -SPIDER_YAW_RATE_MAX_DEG_S,
                            SPIDER_YAW_RATE_MAX_DEG_S);
    yaw_rate *= speed_scale;

    /*
     * 机身运动用精确刚体运动学，两个分量各自独立：
     *   前进  v_Px = stride/period × speed%
     *   偏航  ω   = P 控制(航向误差)，受 25°/s 红线约束
     *
     * 机身瞬时转动中心（ICR）在机身侧向偏距 d = v_Px / ω 处。
     * 注意「边前进边自转」且机身不跟随 ICR 的话，足端会被推出腿平面，
     * 髋角逼近限位后支撑多边形迅速塌缩——这是转向支撑裕度掉到 4mm 的根因。
     */
    float yaw = DEG2RAD(g->body.yaw_deg);

    if (!g->reposture) {
        g->body.x_mm += base_speed * cosf(yaw) * dt_s;
        g->body.y_mm += base_speed * sinf(yaw) * dt_s;
    }
    g->body.yaw_deg = wrap01(g->body.yaw_deg / 360.0f) * 360.0f + yaw_rate * dt_s;
    yaw = DEG2RAD(g->body.yaw_deg);

    float prev_phase = g->phase;
    g->phase = wrap01(g->phase + dt_s / g->cfg.period_s);

    /* 摆动相内的推进量：前 70% 摆动 + 30% 后蹬 */
    for (uint8_t leg = 0; leg < GAIT_LEG_COUNT; leg++) {
        float start = leg_phase_start(g, leg);
        float local = wrap01(g->phase - start);
        bool swinging = local < duty;

        /* 摆动开始的那一 tick：记录起点并预定落点 */
        bool just_started = swinging && (wrap01(prev_phase - start) >= duty);
        if (g->reposture) {
            /* 原地变高：落点固定为「新半径、旧方位」——方位不变故不漂移 */
            if (just_started) {
                float psi = DEG2RAD(SPIDER_LEGS[leg].psi_deg);
                float bx = SPIDER_LEGS[leg].hip_x_mm + g->reposture_radius * cosf(psi);
                float by = SPIDER_LEGS[leg].hip_y_mm + g->reposture_radius * sinf(psi);
                float cy = cosf(yaw), sy = sinf(yaw);
                g->foot_from_x[leg] = g->foot_world_x[leg];
                g->foot_from_y[leg] = g->foot_world_y[leg];
                g->foot_to_x[leg] = g->body.x_mm + (bx * cy - by * sy);
                g->foot_to_y[leg] = g->body.y_mm + (bx * sy + by * cy);
            }
        } else if (just_started) {
            g->foot_from_x[leg] = g->foot_world_x[leg];
            g->foot_from_y[leg] = g->foot_world_y[leg];
            /*
             * 转向用「差速步幅」实现：内侧腿步幅缩短、外侧腿加长，
             * 机身自然绕内侧偏转。不用落点侧向偏移——2DOF 腿无法把脚
             * 放到机身前后方向之外，加大侧向偏移只会把足端推出支撑多边形。
             */
            /*
             * 精确差速步幅（刚体运动学）：
             *   足端世界速度沿机身前向 = v_Px - ω·y_i
             *   y_i 为该足端在机身系的横向坐标（ICR 偏距已在其中隐含）
             * 内侧腿（y 与转向同向）步幅自动缩短，外侧自动加长。
             * 这样「机身怎么转」与「脚怎么落」严格一致，足端不会脱离腿平面。
             */
            float y_i = foot_body_y(g, leg);
            float yaw_rate_s = yaw_rate * DEG2RAD(1.0f);
            float v_foot = base_speed - yaw_rate_s * y_i;
            float adv = clampf(v_foot * g->cfg.period_s, -step_mm * 2.0f,
                               step_mm * 2.0f);
            float cy = cosf(yaw), sy = sinf(yaw);
            g->foot_to_x[leg] = g->foot_from_x[leg] + adv * cy;
            g->foot_to_y[leg] = g->foot_from_y[leg] + adv * sy;
        }

        float lift = 0.0f;
        if (swinging) {
            float u = local / duty;      /* 摆动相内归一化时间 0~1 */

            /* 水平：reach 段内以 smoothstep 从起点移到落点 */
            float v = smoothstep((u - SW_REACH_A) / (SW_REACH_B - SW_REACH_A));
            g->foot_world_x[leg] = g->foot_from_x[leg]
                + (g->foot_to_x[leg] - g->foot_from_x[leg]) * v;
            g->foot_world_y[leg] = g->foot_from_y[leg]
                + (g->foot_to_y[leg] - g->foot_from_y[leg]) * v;

            /* 抬起包络：离地 -> 维持 -> 落下 */
            float env;
            if (u < SW_REACH_A) {
                env = smoothstep(u / SW_REACH_A);
            } else if (u < SW_PLACE_B) {
                env = 1.0f;
            } else {
                env = 1.0f - smoothstep((u - SW_PLACE_B) / (1.0f - SW_PLACE_B));
            }
            lift = g->cfg.lift_mm * env;
        }

        /* 摆动相内逐腿变高（零滑移）；支撑腿锁在自己的高度上 */
        float leg_h = g->leg[leg].height_mm;
        if (g->cfg.height_mode == GAIT_HEIGHT_STANDING) {
            g->height_mm = g->cfg.height_target_mm;
            leg_h = g->height_mm;
            for (uint8_t k = 0; k < GAIT_LEG_COUNT; k++) {
                g->leg[k].height_mm = g->height_mm;
            }
        } else if (swinging) {
            /* 单 tick 内收敛到目标高度（空中改膝不产生滑移） */
            leg_h = g->cfg.height_target_mm;
        }
        g->leg[leg].height_mm = leg_h;

        solve_leg(g, leg, g->foot_world_x[leg], g->foot_world_y[leg],
                  leg_h, lift, &out_cmds[leg]);
        out_cmds[leg].in_swing = swinging;
        g->leg[leg].hip_deg = out_cmds[leg].hip_deg;
        g->leg[leg].knee_beta_deg = out_cmds[leg].knee_beta_deg;
        g->leg[leg].height_mm = leg_h;
        g->leg[leg].in_swing = swinging;
    }

    /* 支撑腿的高度若全一致，则当前生效高度即该值 */
    float h_sum = 0.0f;
    for (uint8_t leg = 0; leg < GAIT_LEG_COUNT; leg++) {
        h_sum += g->leg[leg].height_mm;
    }
    g->height_mm = h_sum / GAIT_LEG_COUNT;

    return true;
}

/*
 * 支撑裕度：机身中心到「支撑足凸包」各边的最小距离（mm）。
 *
 * 只用支撑腿构造多边形——摆动腿的足端在空中，对支撑没有贡献。
 * Trot 只有 2 足支撑，凸包退化为线段，此时裕度即中心到该直线的距离；
 * 正值表示重心投影落在两点之间（静态裕度天然为 0，靠动态稳定）。
 */
static float cross2(const float *ax, const float *ay,
                    const float *bx, const float *by,
                    const float *cx, const float *cy)
{
    return (bx[0] - ax[0]) * (cy[1] - ay[1]) - (by[1] - ay[1]) * (cx[0] - ax[0]);
}

/*
 * Andrew monotone chain 凸包（n<=4）。
 * 两段式实现 + 去重，避免退化时返回多余顶点导致边长为 0。
 */
static int hull_of(const float *x, const float *y, int n,
                   float *hx, float *hy)
{
    if (n < 3) {
        for (int i = 0; i < n; i++) {
            hx[i] = x[i];
            hy[i] = y[i];
        }
        return n;
    }

    int idx[GAIT_LEG_COUNT];
    for (int i = 0; i < n; i++) {
        idx[i] = i;
    }
    for (int i = 1; i < n; i++) {          /* 插入排序：(x, y) 升序 */
        int key = idx[i];
        int j = i - 1;
        while (j >= 0 && (x[idx[j]] > x[key]
                          || (x[idx[j]] == x[key] && y[idx[j]] > y[key]))) {
            idx[j + 1] = idx[j];
            j--;
        }
        idx[j + 1] = key;
    }

    float lower[GAIT_LEG_COUNT], lower_y[GAIT_LEG_COUNT];
    int ln = 0;
    for (int i = 0; i < n; i++) {
        const float *px = &x[idx[i]];
        const float *py = &y[idx[i]];
        while (ln >= 2
               && cross2(&lower[ln - 2], &lower_y[ln - 2],
                         &lower[ln - 1], &lower_y[ln - 1], px, py) <= 0.0f) {
            ln--;
        }
        lower[ln] = *px;
        lower_y[ln] = *py;
        ln++;
    }

    float upper[GAIT_LEG_COUNT], upper_y[GAIT_LEG_COUNT];
    int un = 0;
    for (int i = n - 1; i >= 0; i--) {
        const float *px = &x[idx[i]];
        const float *py = &y[idx[i]];
        while (un >= 2
               && cross2(&upper[un - 2], &upper_y[un - 2],
                         &upper[un - 1], &upper_y[un - 1], px, py) <= 0.0f) {
            un--;
        }
        upper[un] = *px;
        upper_y[un] = *py;
        un++;
    }

    /* 合并两段，去掉两段共有的端点与重复顶点 */
    int hn = 0;
    for (int i = 0; i < ln - 1; i++) {
        hx[hn] = lower[i];
        hy[hn] = lower_y[i];
        hn++;
    }
    for (int i = 0; i < un - 1; i++) {
        if (i > 0 && upper[i] == hx[hn - 1] && upper_y[i] == hy[hn - 1]) {
            continue;                       /* 跳过重复顶点 */
        }
        hx[hn] = upper[i];
        hy[hn] = upper_y[i];
        hn++;
    }
    return hn;
}

float gait_support_margin(const gait_t *g)
{
    if (g == NULL) {
        return 0.0f;
    }

    int n = 0;
    float px[GAIT_LEG_COUNT], py[GAIT_LEG_COUNT];
    float inv_yaw = DEG2RAD(-g->body.yaw_deg);
    float c = cosf(inv_yaw), s = sinf(inv_yaw);

    for (uint8_t leg = 0; leg < GAIT_LEG_COUNT; leg++) {
        if (g->leg[leg].in_swing) {
            continue;                      /* 摆动腿不参与支撑 */
        }
        /*
         * 关键：多边形顶点须是足端相对**机身中心**的坐标，
         * 不是相对髋轴心。取相对髋会把原点挪到角上，裕度失去意义。
         */
        float dx = g->foot_world_x[leg] - g->body.x_mm;
        float dy = g->foot_world_y[leg] - g->body.y_mm;
        px[n] = dx * c - dy * s;           /* 世界 -> 机身 */
        py[n] = dx * s + dy * c;
        n++;
    }
    if (n < 2) {
        return 0.0f;                       /* 支撑不足，无法评估 */
    }

    float hull_x[GAIT_LEG_COUNT * 2], hull_y[GAIT_LEG_COUNT * 2];
    int hn = hull_of(px, py, n, hull_x, hull_y);

    if (hn < 2) {
        /* 支撑点几乎重合 */
        return sqrtf(px[0] * px[0] + py[0] * py[0]);
    }
    if (hn == 1) {
        /* 退化线段：中心到直线的距离 */
        float dx = hull_x[1] - hull_x[0];
        float dy = hull_y[1] - hull_y[0];
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 1e-3f) {
            return sqrtf(hull_x[0] * hull_x[0] + hull_y[0] * hull_y[0]);
        }
        return fabsf(dx * (0.0f - hull_y[0]) - dy * (0.0f - hull_x[0])) / len;
    }

    float min_margin = 1e9f;
    for (int i = 0; i < hn; i++) {
        int a = i, b = (i + 1) % hn;
        float dx = hull_x[b] - hull_x[a];
        float dy = hull_y[b] - hull_y[a];
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 1e-3f) {
            continue;
        }
        float dist = fabsf(dx * (0.0f - hull_y[a]) - dy * (0.0f - hull_x[a])) / len;
        if (dist < min_margin) {
            min_margin = dist;
        }
    }
    return (min_margin > 1e8f) ? 0.0f : min_margin;
}

int gait_reposture_settle(gait_t *g, gait_leg_cmd_t *out_cmds, float dt_s)
{
    if (g == NULL || out_cmds == NULL) {
        return 0;
    }
    if (fabsf(g->cfg.height_target_mm - g->height_mm) < 0.5f) {
        return 0;
    }

    /*
     * 原地换高度且不拖脚的做法：
     * 让四腿各自在摆动相里把脚放到「新半径、旧方位」的位置——
     * 半径变小则机身升高，变大则降低。脚离地移动，故零滑移；
     * 方位不变，故机身不会横向漂移。
     * 每轮推进一个周期，最多 8 轮收敛。
     */
    float radius_target = kin_radius_from_height(g->cfg.height_target_mm);
    if (fabsf(radius_target - SPIDER_CRUISE_RADIUS_MM) < 0.5f) {
        return 0;
    }

    g->reposture = true;
    g->reposture_radius = radius_target;

    /* 跑满一个完整周期让四腿各重新落一次，脚即到达新半径 */
    float t = 0.0f;
    float period = g->cfg.period_s;
    int steps = (int)(period / (dt_s > 0.0f ? dt_s : 0.01f)) + 1;
    float step = period / (float)steps;
    for (int i = 0; i < steps; i++) {
        gait_update(g, step, out_cmds);
        t += step;
    }

    g->reposture = false;

    if (fabsf(radius_target - kin_radius_from_height(g->height_mm)) < 1.0f) {
        return 1;
    }
    (void)t;
    return 1;
}