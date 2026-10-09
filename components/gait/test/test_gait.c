/*
 * 步态引擎主机端测试
 *
 * 核心验收（docs/gait_design.md §9）：
 *   - 支撑足世界坐标在支撑期内逐位不变（模型层滑移必须为 0）
 *   - 支撑期足端离地高度 == 锁定身高（摆动腿才抬脚）
 *   - 变高只发生在摆动相，支撑腿高度锁定
 *   - 髋/膝全周期不越软限位、抬脚不穿地
 *   - 相位表正确：爬行单腿摆动、Trot 对角同步
 *   - 实测速度 == stride / period
 *   - 原地变高收敛且机身不漂移
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "spider_gait.h"

static int g_fail, g_total;

static void chk(const char *name, int ok, const char *detail)
{
    g_total++;
    if (!ok) g_fail++;
    printf("[%s] %-52s %s\n", ok ? "PASS" : "FAIL", name, detail ? detail : "");
}

static void chkf(const char *name, int ok, float v)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%.2f", v);
    g_total++;
    if (!ok) g_fail++;
    printf("[%s] %-52s %s\n", ok ? "PASS" : "FAIL", name, buf);
}

#define LEG_FMT "FL(%.1f/%.1f) FR(%.1f/%.1f) RL(%.1f/%.1f) RR(%.1f/%.1f)"
#define LEG_ARGS(c) (c)[0].hip_deg, (c)[0].knee_beta_deg, \
                        (c)[1].hip_deg, (c)[1].knee_beta_deg, \
                        (c)[2].hip_deg, (c)[2].knee_beta_deg, \
                        (c)[3].hip_deg, (c)[3].knee_beta_deg

/* 找出支撑腿的足端离地高度（应为锁定身高，抬脚=0） */
static float stance_foot_height(const gait_t *g, uint8_t leg, float cruise_h)
{
    float hx = g->foot_world_x[leg], hy = g->foot_world_y[leg];
    (void)hx; (void)hy;
    return cruise_h;
}

static void run_test_gait(gait_type_t type, const char *label, float tol_deg)
{
    gait_config_t cfg;
    gait_t g;
    gait_leg_cmd_t cmds[4];

    gait_config_defaults(&cfg, type);
    cfg.speed_pct = 60.0f;
    gait_init(&g, &cfg);

    printf("\n--- %s ---\n", label);

    /* ---- 相位表：任一时刻的摆动腿数量 ---- */
    int crawl_max_swing = 0, crawl_ok = 1;
    int trot_ok = 1, trot_seen[2][4];
    memset(trot_seen, 0, sizeof(trot_seen));

    for (int i = 0; i < 400; i++) {
        gait_update(&g, cfg.period_s / 200.0f, cmds);
        int ns = 0;
        for (uint8_t l = 0; l < 4; l++) if (cmds[l].in_swing) ns++;
        if (type == GAIT_CRAWL) {
            if (ns > crawl_max_swing) crawl_max_swing = ns;
            if (ns != 1) crawl_ok = 0;     /* 爬行：恰好一条腿摆动 */
        } else {
            if (ns != 2) trot_ok = 0;
            for (uint8_t l = 0; l < 4; l++) {
                if (cmds[l].in_swing) trot_seen[0][l] = 1;
            }
        }
    }

    if (type == GAIT_CRAWL) {
        chk("爬行：任一时刻恰好 1 条腿摆动", crawl_ok, "");
    } else {
        chk("Trot：任一时刻恰好 2 条腿摆动", trot_ok, "");
        /* 对角同步：FL+RR 同时、FR+RL 同时 */
        gait_t g2;
        gait_init(&g2, &cfg);
        int diag_ok = 1;
        for (int i = 0; i < 400; i++) {
            gait_update(&g2, cfg.period_s / 200.0f, cmds);
            bool a = cmds[0].in_swing, b = cmds[3].in_swing;
            bool c = cmds[1].in_swing, d = cmds[2].in_swing;
            if (!(a == b) || !(c == d) || (a == c)) diag_ok = 0;
        }
        chk("Trot：对角腿严格同步（FL+RR / FR+RL）", diag_ok, "");
    }
    (void)trot_seen;
    (void)crawl_max_swing;

    /* ---- 支撑足世界坐标逐位不变（模型层零滑移） ---- */
    gait_t g3;
    gait_init(&g3, &cfg);
    float prev_fx[4], prev_fy[4];
    for (uint8_t l = 0; l < 4; l++) {
        prev_fx[l] = g3.foot_world_x[l];
        prev_fy[l] = g3.foot_world_y[l];
    }
    int stance_locked = 1, max_anchor_move = 0;
    for (int i = 0; i < 600; i++) {
        gait_update(&g3, cfg.period_s / 300.0f, cmds);
        for (uint8_t l = 0; l < 4; l++) {
            if (!cmds[l].in_swing) {
                /* 支撑腿：足端必须与上一 tick 完全一致 */
                if (g3.foot_world_x[l] != prev_fx[l] || g3.foot_world_y[l] != prev_fy[l]) {
                    stance_locked = 0;
                }
            } else {
                /* 摆动腿：足端允许移动 */
                float dx = g3.foot_world_x[l] - prev_fx[l];
                float dy = g3.foot_world_y[l] - prev_fy[l];
                float mv = (int)(sqrtf(dx * dx + dy * dy) * 1000.0f);
                if (mv > max_anchor_move) max_anchor_move = mv;
            }
            prev_fx[l] = g3.foot_world_x[l];
            prev_fy[l] = g3.foot_world_y[l];
        }
    }
    chk("支撑足世界坐标逐位不变（模型层零滑移）", stance_locked, "");
    printf("       摆动足单 tick 最大位移 %.3f mm\n", max_anchor_move / 1000.0f);

    /* ---- 关节限位 & 不穿地 ---- */
    gait_t g4;
    gait_init(&g4, &cfg);
    int hip_ok = 1, knee_ok = 1, ground_ok = 1;
    float min_foot_h = 1e9f, max_foot_h = -1e9f;
    float max_hip = 0.0f, max_knee = 0.0f, min_knee = 999.0f;

    for (int i = 0; i < 800; i++) {
        gait_update(&g4, cfg.period_s / 400.0f, cmds);
        for (uint8_t l = 0; l < 4; l++) {
            float a = fabsf(cmds[l].hip_deg);
            if (a > SPIDER_HIP_LIMIT_DEG + 0.01f) hip_ok = 0;
            if (a > max_hip) max_hip = a;

            float b = cmds[l].knee_beta_deg;
            if (b < SPIDER_BETA_MIN_DEG - 0.01f || b > SPIDER_BETA_MAX_DEG + 0.01f) knee_ok = 0;
            if (b > max_knee) max_knee = b;
            if (b < min_knee) min_knee = b;

            /* 足端离地高度 = 身高 - 抬起量 */
            float fh = cmds[l].height_mm - kin_height_from_beta(b);
            if (fh > max_foot_h) max_foot_h = fh;
            if (fh < min_foot_h) min_foot_h = fh;
            /*
             * 支撑脚必须贴地（离地 <=1mm）
             * 摆动脚不得穿地（离地 >= -0.5mm）。相边界处抬升为 0 属正常，
             * 故只查"不穿地"，抬起来够不够由 max_foot_h 单独断言。
             */
            if (cmds[l].in_swing) {
                if (fh < -0.5f) ground_ok = 0;
            } else {
                if (fh > 1.0f) ground_ok = 0;
            }
        }
    }
    chk("髋角全周期不越 ±60°", hip_ok, "");
    chk("膝角全周期在 [21.3,65.4] 内", knee_ok, "");
    chk("支撑脚不离地 / 摆动脚不穿地", ground_ok, "");
    printf("       髋最大 %.2f° (预算 ±%.0f°)  膝 [%.2f, %.2f]\n",
           max_hip, SPIDER_HIP_LIMIT_DEG, min_knee, max_knee);
    printf("       摆动相足端最大抬升 %.2f mm (设定 %.1f)\n",
           max_foot_h, cfg.lift_mm);
    chkf("抬脚高度达到设定值 90%", max_foot_h >= cfg.lift_mm * 0.9f, max_foot_h);

    /* ---- 实测速度 == stride/period ---- */
    gait_t g5;
    gait_init(&g5, &cfg);
    float x0 = g5.body.x_mm, y0 = g5.body.y_mm;
    int cycles = 10;
    float t = 0.0f;
    for (int i = 0; i < cycles * 400; i++) {
        gait_update(&g5, cfg.period_s / 400.0f, cmds);
        t += cfg.period_s / 400.0f;
    }
    float dist = sqrtf((g5.body.x_mm - x0) * (g5.body.x_mm - x0)
                     + (g5.body.y_mm - y0) * (g5.body.y_mm - y0));
    float speed = dist / t;
    float expect = cfg.stride_mm / cfg.period_s * (cfg.speed_pct / 100.0f);
    chkf("实测速度 == stride/period×speed%", fabsf(speed - expect) < 0.5f, speed);
    printf("       期望 %.2f mm/s，10 周期行进 %.1f mm\n", expect, dist);

    /* ---- 变高只发生在摆动相 ---- */
    {
        gait_t g6;
        gait_init(&g6, &cfg);
        gait_request_height(&g6, SPIDER_HEIGHT_MAX_MM);
        int stance_h_locked = 1;
        float prev_h[4];
        for (uint8_t l = 0; l < 4; l++) prev_h[l] = g6.leg[l].height_mm;

        for (int i = 0; i < 2000; i++) {
            gait_update(&g6, cfg.period_s / 400.0f, cmds);
            for (uint8_t l = 0; l < 4; l++) {
                if (!cmds[l].in_swing && g6.leg[l].height_mm != prev_h[l]) {
                    stance_h_locked = 0;
                }
                prev_h[l] = g6.leg[l].height_mm;
            }
        }
        chk("变高期间支撑腿高度锁定（零滑移变高）", stance_h_locked, "");
        chkf("变高收敛到目标", fabsf(g6.height_mm - SPIDER_HEIGHT_MAX_MM) < 1.5f,
             g6.height_mm);
    }

    /* ---- 支撑裕度：须扫全周期取最小，单点采样会漏掉最差相位 ---- */
    {
        gait_t gm;
        gait_leg_cmd_t gc[4];
        gait_config_defaults(&cfg, type);
        gait_init(&gm, &cfg);

        float m_static = gait_support_margin(&gm);
        chkf("四足静止支撑裕度 > 150mm", m_static > 150.0f, m_static);

        float m_min = 1e9f, m_max = 0.0f;
        for (int i = 0; i < (int)(cfg.period_s * 400); i++) {
            gait_update(&gm, cfg.period_s / 400.0f, gc);
            float m = gait_support_margin(&gm);
            if (m < m_min) m_min = m;
            if (m > m_max) m_max = m;
        }
        printf("       全周期支撑裕度 min=%.1f max=%.1f mm\n", m_min, m_max);

        /*
         * 爬行：三足支撑。裕度天花板约 20mm——中心对称四足的任意三足支撑
         * 必有一条对角边过重心（docs/gait_design.md §3.5）。已用前后外八
         * 不等（前36°/后48°）把它从 0 提到 ~18mm，这是常规装法的极限。
         * 故只要求"不塌缩"，不追求大裕度。
         *
         * Trot：两足支撑，且必为对角腿对。那条连线正好穿过重心，
         * 静态裕度恒为 0——Trot 只靠动态稳定，这是几何必然，不是缺陷，
         * 也是「默认步态为爬行 + Trot 限速 70% + 转向限速 25°/s」
         * 这三条防翻红线存在的理由。
         */
        if (type == GAIT_CRAWL) {
            chkf("爬行全周期支撑裕度 > 8mm", m_min > 8.0f, m_min);
        } else {
            chkf("Trot 裕度不为负(重心不越支撑线)", m_min > -0.5f, m_min);
            chkf("Trot 部分相位重心在支撑线内", m_max > 10.0f, m_max);
        }
    }

    /* ---- 转向：航向跟踪且受红线约束 ---- */
    {
        gait_t g7;
        gait_init(&g7, &cfg);
        g7.cfg.speed_pct = 100.0f;
        gait_set_heading(&g7, 90.0f);
        float max_rate = 0.0f;
        float prev_yaw = g7.body.yaw_deg;
        for (int i = 0; i < 4000; i++) {
            float dt = cfg.period_s / 400.0f;
            gait_update(&g7, dt, cmds);
            float dy = g7.body.yaw_deg - prev_yaw;
            if (dy > 180.0f) dy -= 360.0f;
            if (dy < -180.0f) dy += 360.0f;
            float rate = fabsf(dy) / dt;
            if (rate > max_rate) max_rate = rate;
            prev_yaw = g7.body.yaw_deg;
        }
        chkf("转向速率 <= 25°/s 红线", max_rate <= SPIDER_YAW_RATE_MAX_DEG_S + 0.5f, max_rate);
        chkf("航向跟踪到 90°±8°", fabsf(g7.body.yaw_deg - 90.0f) < 8.0f, g7.body.yaw_deg);
        /*
         * 转向中的裕度同样扫全周期取最小。
         * 爬行 3 足支撑：裕度天然受三足几何限制，只要求不塌缩。
         * Trot  2 足支撑：凸包退化为线段，裕度即中心到支撑线的距离，
         * 静态必然很小，要求 > 0 即可（动态稳定由巡��横滚裕度保证）。
         */
        float m_min = 1e9f;
        for (int i = 0; i < (int)(cfg.period_s * 400); i++) {
            float dt = cfg.period_s / 400.0f;
            gait_update(&g7, dt, cmds);
            float m = gait_support_margin(&g7);
            if (m < m_min) m_min = m;
        }
        if (type == GAIT_CRAWL) {
            chkf("爬行转向中裕度不塌缩", m_min > 12.0f, m_min);
        } else {
            chkf("Trot 转向中重心在支撑线内", m_min > 0.5f, m_min);
        }
    }

    /* ---- 原地变高：收敛且机身不漂移 ---- */
    {
        gait_t g8;
        gait_init(&g8, &cfg);
        float bx0 = g8.body.x_mm, by0 = g8.body.y_mm;
        gait_request_height(&g8, SPIDER_HEIGHT_MIN_MM);
        int rounds = 0;
        for (int k = 0; k < 8; k++) {
            rounds = gait_reposture_settle(&g8, cmds, 0.01f);
            if (rounds == 0) break;
        }
        float drift = sqrtf((g8.body.x_mm - bx0) * (g8.body.x_mm - bx0)
                          + (g8.body.y_mm - by0) * (g8.body.y_mm - by0));
        chkf("原地变高收敛", fabsf(g8.height_mm - SPIDER_HEIGHT_MIN_MM) < 3.0f,
             g8.height_mm);
        chkf("原地变高机身漂移 < 15mm", drift < 15.0f, drift);
    }

    (void)tol_deg;
    (void)stance_foot_height;
}

int main(void)
{
    printf("=== 步态引擎测试 ===\n");
    run_test_gait(GAIT_CRAWL, "爬行 4 拍", 0.1f);
    run_test_gait(GAIT_TROT, "Trot 2 拍", 0.1f);

    /* ---- 防翻约束：Trot 高速被压到 70% ---- */
    {
        gait_config_t cfg;
        gait_t g;
        gait_leg_cmd_t cmds[4];
        gait_config_defaults(&cfg, GAIT_TROT);
        gait_init(&g, &cfg);
        (void)cmds;
        gait_set_speed(&g, 100.0f);
        chkf("Trot 100% 被压到 70%", fabsf(g.cfg.speed_pct - 70.0f) < 0.01f,
             g.cfg.speed_pct);

        gait_config_defaults(&cfg, GAIT_CRAWL);
        gait_init(&g, &cfg);
        gait_set_speed(&g, 100.0f);
        chkf("爬行 100% 不受限", fabsf(g.cfg.speed_pct - 100.0f) < 0.01f,
             g.cfg.speed_pct);
    }

    /* ---- 高度范围钳制 ---- */
    {
        gait_config_t cfg;
        gait_t g;
        gait_config_defaults(&cfg, GAIT_CRAWL);
        gait_init(&g, &cfg);
        gait_request_height(&g, 999.0f);
        chkf("超限高度被钳到 150", fabsf(g.cfg.height_target_mm - 150.0f) < 0.01f,
             g.cfg.height_target_mm);
        gait_request_height(&g, -50.0f);
        chkf("负高度被钳到 90", fabsf(g.cfg.height_target_mm - 90.0f) < 0.01f,
             g.cfg.height_target_mm);
    }

    printf("\n%d/%d passed, %d failed\n", g_total - g_fail, g_total, g_fail);
    return g_fail ? 1 : 0;
}