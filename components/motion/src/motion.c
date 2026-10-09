#include "spider_motion.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spider_keyframe.h"
#include "spider_posture.h"
#include "spider_servo.h"

static const char *TAG = "motion";

#define TICK_MS          CONFIG_SPIDER_MOTION_TICK_MS
#define CMD_QUEUE_LEN    SPIDER_CMD_QUEUE_LEN
#define HEARTBEAT_MS     SPIDER_HEARTBEAT_TIMEOUT_MS

/* 侧移宏状态 */
typedef struct {
    bool    active;
    float   remaining_mm;
    int     turn_stage;      /* 0=左转 1=直行 2=右转回正 */
    float   stage_heading;
} strafe_ctx_t;

/* 棘轮微调状态 */
typedef struct {
    bool    active;
    float   pending_dx, pending_dy;
    float   achieved_x, achieved_y;
    float   phase_s;
    bool    in_fast_phase;
} nudge_ctx_t;

static struct {
    motion_cmd_t  queue[CMD_QUEUE_LEN];
    volatile int  head, tail;
    volatile bool estop_flag;
    volatile bool cal_mode;

    gait_t        gait;
    motion_state_t state;
    strafe_ctx_t  strafe;
    nudge_ctx_t   nudge;
    float         lean_roll, lean_pitch;
    float         last_hb_ms;
    bool          hb_seen;
    bool          task_started;
} s_m;

/* ---------------------------------------------------------------- 命令队列 */

esp_err_t motion_enqueue(const motion_cmd_t *cmd)
{
    if (cmd == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_m.cal_mode && cmd->type != CMD_ESTOP
        && cmd->type != CMD_ESTOP_CLEAR && cmd->type != CMD_CALIBRATE_MODE) {
        /* 标定模式与运动互斥：除急停与退出标定外一律拒绝 */
        return ESP_ERR_INVALID_STATE;
    }

    int next = (s_m.tail + 1) % CMD_QUEUE_LEN;
    if (next == s_m.head) {
        return ESP_ERR_NO_MEM;      /* 队列满 */
    }
    s_m.queue[s_m.tail] = *cmd;
    s_m.tail = next;
    return ESP_OK;
}

void motion_heartbeat(uint32_t now_ms)
{
    s_m.last_hb_ms = now_ms;
    s_m.hb_seen = true;
}

void motion_estop(void)
{
    s_m.estop_flag = true;
    /*
     * 立即硬件关断，不等下一次 tick。
     *
     * servo_all_off() 依次做两件事：拉高 OE（硬件级关断全部 16 路，一次 GPIO
     * 写，不经 I2C），再把 LEDn 全写 0。第二步覆盖 oe_gpio 未接的情况。
     * 任一步失败都只记录日志——此时急停语义已经由 OE 达成。
     */
    esp_err_t rc = servo_all_off();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "急停关断输出返回 %s", esp_err_to_name(rc));
    }
    ESP_LOGE(TAG, "急停");
}

esp_err_t motion_estop_clear(void)
{
    if (!s_m.estop_flag) {
        return ESP_OK;
    }
    /* 清除前必须先回到可站立的姿态，否则可能在半空解锁 */
    s_m.estop_flag = false;
    /* 重新使能输出（servo_output_enable 内部拉低 OE） */
    servo_output_enable(true);
    gait_force_height(&s_m.gait, SPIDER_STAND_HEIGHT_MM);
    s_m.state.mode = MOTION_IDLE;
    s_m.strafe.active = false;
    s_m.nudge.active = false;
    ESP_LOGW(TAG, "急停已清除");
    return ESP_OK;
}

/* ------------------------------------------------------------ 便捷入队 */

static esp_err_t push_simple(cmd_type_t t)
{
    motion_cmd_t c = { .type = t };
    return motion_enqueue(&c);
}

esp_err_t motion_set_posture(int posture_id)   { motion_cmd_t c = {.type = CMD_SET_POSTURE}; c.params.i = posture_id; return motion_enqueue(&c); }
esp_err_t motion_set_height(float mm)          { motion_cmd_t c = {.type = CMD_SET_HEIGHT};  c.params.f[0] = mm;         return motion_enqueue(&c); }
esp_err_t motion_set_gait(gait_type_t t)       { motion_cmd_t c = {.type = CMD_GAIT};        c.params.i = (int)t;      return motion_enqueue(&c); }
esp_err_t motion_set_speed(float pct)          { motion_cmd_t c = {.type = CMD_SET_SPEED};   c.params.f[0] = pct;       return motion_enqueue(&c); }
esp_err_t motion_stop(void)                    { return push_simple(CMD_STOP); }

esp_err_t motion_drive(float vx, float vy, float yaw)
{
    motion_cmd_t c = {.type = CMD_DRIVE};
    c.params.f[0] = vx; c.params.f[1] = vy; c.params.f[2] = yaw;
    return motion_enqueue(&c);
}

esp_err_t motion_nudge(float dx_mm, float dy_mm)
{
    motion_cmd_t c = {.type = CMD_NUDGE};
    c.params.f[0] = dx_mm; c.params.f[1] = dy_mm;
    return motion_enqueue(&c);
}

esp_err_t motion_lean(float roll_deg, float pitch_deg)
{
    motion_cmd_t c = {.type = CMD_LEAN};
    c.params.f[0] = roll_deg; c.params.f[1] = pitch_deg;
    return motion_enqueue(&c);
}

esp_err_t motion_strafe(float dx_mm)
{
    motion_cmd_t c = {.type = CMD_STRAFE};
    c.params.f[0] = dx_mm;
    return motion_enqueue(&c);
}

esp_err_t motion_play(const char *seq_name)
{
    if (seq_name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    motion_cmd_t c = {.type = CMD_PLAY};
    strncpy(c.params.s.name, seq_name, sizeof(c.params.s.name) - 1);
    c.params.s.name[sizeof(c.params.s.name) - 1] = '\0';
    return motion_enqueue(&c);
}

/* ------------------------------------------------------------ 状态查询 */

motion_mode_t motion_get_mode(void)          { return s_m.state.mode; }

const char *motion_mode_name(motion_mode_t m)
{
    switch (m) {
    case MOTION_IDLE:      return "IDLE";
    case MOTION_WALK:      return "WALK";
    case MOTION_POSTURE:   return "POSTURE";
    case MOTION_KEYFRAME:  return "KEYFRAME";
    case MOTION_NUDGE:     return "NUDGE";
    case MOTION_STRAFE:    return "STRAFE";
    case MOTION_ESTOP:     return "ESTOP";
    case MOTION_CALIBRATE: return "CALIBRATE";
    default:               return "?";
    }
}

void motion_get_state(motion_state_t *out)
{
    if (out != NULL) {
        *out = s_m.state;
    }
}

/* ------------------------------------------------------------ 命令处理 */

static void handle_cmd(const motion_cmd_t *c)
{
    switch (c->type) {
    case CMD_ESTOP:
        s_m.estop_flag = true;
        break;

    case CMD_ESTOP_CLEAR:
        motion_estop_clear();
        break;

    case CMD_CALIBRATE_MODE:
        s_m.cal_mode = (c->params.i != 0);
        servo_set_mode(s_m.cal_mode ? SERVO_MODE_CALIBRATE : SERVO_MODE_NORMAL);
        s_m.state.mode = s_m.cal_mode ? MOTION_CALIBRATE : MOTION_IDLE;
        if (!s_m.cal_mode) {
            /* 退出标定：回到站立姿态 */
            gait_force_height(&s_m.gait, SPIDER_STAND_HEIGHT_MM);
        }
        break;

    case CMD_STOP:
        s_m.state.mode = MOTION_IDLE;
        s_m.state.vx = s_m.state.vy = s_m.state.yaw_cmd = 0.0f;
        s_m.strafe.active = false;
        s_m.nudge.active = false;
        kf_stop();
        break;

    case CMD_SET_POSTURE: {
        posture_id_t pid = (posture_id_t)c->params.i;
        const posture_def_t *p = posture_get(pid);
        if (p == NULL) {
            ESP_LOGW(TAG, "未知姿态 id=%d", c->params.i);
            break;
        }
        /* 姿态切换 = 目标身高 + 髋偏移；身高由 gait 的摆动相执行（零滑移） */
        gait_request_height(&s_m.gait, p->height_mm);
        s_m.state.mode = MOTION_POSTURE;
        kf_stop();
        break;
    }

    case CMD_SET_HEIGHT:
        gait_request_height(&s_m.gait, c->params.f[0]);
        s_m.state.mode = MOTION_POSTURE;
        break;

    case CMD_GAIT:
        s_m.state.gait = (gait_type_t)c->params.i;
        s_m.gait.cfg.type = s_m.state.gait;
        gait_set_speed(&s_m.gait, s_m.state.speed_pct);
        s_m.state.mode = MOTION_WALK;
        break;

    case CMD_SET_SPEED:
        gait_set_speed(&s_m.gait, c->params.f[0]);
        s_m.state.speed_pct = s_m.gait.cfg.speed_pct;
        break;

    case CMD_DRIVE:
        s_m.state.vx = c->params.f[0];
        s_m.state.vy = c->params.f[1];
        s_m.state.yaw_cmd = c->params.f[2];
        if (fabsf(s_m.state.vx) > 1e-3f || fabsf(s_m.state.yaw_cmd) > 1e-3f
            || fabsf(s_m.state.vy) > 1e-3f) {
            if (s_m.state.mode == MOTION_IDLE || s_m.state.mode == MOTION_POSTURE) {
                s_m.state.mode = MOTION_WALK;
            }
            /* vy 是侧移宏意图，不是真横移 */
            if (fabsf(s_m.state.vy) > 0.3f && !s_m.strafe.active) {
                float dir = (s_m.state.vy > 0.0f) ? 1.0f : -1.0f;
                motion_cmd_t sc = {.type = CMD_STRAFE};
                sc.params.f[0] = dir * 50.0f;
                motion_enqueue(&sc);
            }
        }
        break;

    case CMD_NUDGE:
        s_m.nudge.active = true;
        s_m.nudge.pending_dx = c->params.f[0];
        s_m.nudge.pending_dy = c->params.f[1];
        s_m.nudge.achieved_x = s_m.nudge.achieved_y = 0.0f;
        s_m.nudge.phase_s = 0.0f;
        s_m.nudge.in_fast_phase = false;
        s_m.state.mode = MOTION_NUDGE;
        break;

    case CMD_LEAN: {
        /* 非物理红线：默认锁死，advanced 才放开（docs/gait_design.md §8） */
        float lim = s_m.state.advanced ? SPIDER_LEAN_MAX_DEG
                                        : SPIDER_LEAN_MAX_DEG * 0.5f;
        s_m.lean_roll  = fmaxf(-lim, fminf(lim, c->params.f[0]));
        s_m.lean_pitch = fmaxf(-lim, fminf(lim, c->params.f[1]));
        break;
    }

    case CMD_STRAFE:
        s_m.strafe.active = true;
        s_m.strafe.remaining_mm = c->params.f[0];
        s_m.strafe.turn_stage = 0;
        s_m.strafe.stage_heading = 0.0f;
        s_m.state.mode = MOTION_STRAFE;
        break;

    case CMD_PLAY:
        if (kf_play(c->params.s.name) == ESP_OK) {
            s_m.state.mode = MOTION_KEYFRAME;
        } else {
            ESP_LOGW(TAG, "播放失败: %s", c->params.s.name);
        }
        break;

    default:
        break;
    }
}

static float wrap180(float deg);

/* ------------------------------------------------------------ 侧移宏 */

/*
 * 侧移 = 原地转 90° → 直行 → 转回（docs/gait_design.md §5）。
 * 不用支撑足拧转做 90° 大角度（髋行程不够，会把足端推出支撑多边形）。
 */
static void update_strafe(float dt_s)
{
    if (!s_m.strafe.active) {
        return;
    }
    const float TURN = 90.0f;

    switch (s_m.strafe.turn_stage) {
    case 0: /* 左转（dx>0）或右转 */
        gait_set_heading(&s_m.gait, s_m.strafe.stage_heading
                        + ((s_m.strafe.remaining_mm > 0.0f) ? TURN : -TURN));
        if (fabsf(s_m.gait.body.yaw_deg - s_m.strafe.stage_heading
                  - ((s_m.strafe.remaining_mm > 0.0f) ? TURN : -TURN)) < 6.0f) {
            s_m.strafe.stage_heading = s_m.gait.body.yaw_deg;
            s_m.strafe.turn_stage = 1;
        }
        break;

    case 1: { /* 直行 */
        float speed = s_m.gait.cfg.speed_pct;
        s_m.strafe.remaining_mm -= (s_m.gait.cfg.stride_mm / s_m.gait.cfg.period_s)
                                   * speed * dt_s;
        if (s_m.strafe.remaining_mm <= 0.0f) {
            s_m.strafe.turn_stage = 2;
            s_m.strafe.stage_heading = s_m.gait.body.yaw_deg;
        }
        break;
    }

    case 2: /* 转回原航向 */
        gait_set_heading(&s_m.gait, s_m.strafe.stage_heading);
        {
            float origin = wrap180(s_m.strafe.stage_heading);
            if (fabsf(origin) < 6.0f) {
                gait_set_heading(&s_m.gait, 0.0f);
                s_m.strafe.active = false;
                s_m.state.mode = MOTION_IDLE;
            }
        }
        break;

    default:
        s_m.strafe.active = false;
        break;
    }
}

static float wrap180(float deg)
{
    deg = fmodf(deg, 360.0f);
    if (deg > 180.0f)  deg -= 360.0f;
    if (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* ------------------------------------------------------------ 主循环 */

static void apply_lean(const gait_leg_cmd_t *cmds, gait_leg_cmd_t *out)
{
    /* 差动膝：左右差 -> 横滚，前后差 -> 俯仰 */
    float roll_h  = s_m.lean_roll * SPIDER_CRUISE_RADIUS_MM / 57.29578f;
    float pitch_h = s_m.lean_pitch * SPIDER_CRUISE_RADIUS_MM / 57.29578f;

    for (int i = 0; i < 4; i++) {
        float dy = ((i == 0 || i == 2) ? -roll_h : roll_h)      /* 左低右高 */
                 + ((i < 2) ? -pitch_h : pitch_h);             /* 前低后高 */
        float h = cmds[i].height_mm + dy;
        if (h < SPIDER_HEIGHT_MIN_MM) h = SPIDER_HEIGHT_MIN_MM;
        if (h > SPIDER_HEIGHT_MAX_MM) h = SPIDER_HEIGHT_MAX_MM;

        leg_ik_t ik;
        leg_ik(cmds[i].hip_deg, h, &ik);
        out[i].hip_deg = ik.hip_deg;
        out[i].knee_beta_deg = ik.knee_beta_deg;
        out[i].height_mm = h;
        out[i].in_swing = cmds[i].in_swing;
    }
}

static void motion_tick(uint32_t dt_ms)
{
    float dt_s = (float)dt_ms / 1000.0f;

    /* 1) 急停优先级最高：先看标志位，再排空队列 */
    if (s_m.estop_flag) {
        s_m.state.mode = MOTION_ESTOP;
        s_m.state.estop = true;
        s_m.state.vx = s_m.state.vy = s_m.state.yaw_cmd = 0.0f;
        s_m.strafe.active = false;
        s_m.nudge.active = false;
        kf_stop();
        /* 冻结姿态：所有关节保持当前角 */
        for (int leg = 0; leg < 4; leg++) {
            for (int j = 0; j < 2; j++) {
                servo_set_target(leg, (servo_joint_t)j,
                                 servo_get_current(leg, (servo_joint_t)j));
            }
        }
        servo_tick(dt_ms);
        return;
    }
    s_m.state.estop = false;

    /* 2) 消费命令队列 */
    while (s_m.head != s_m.tail) {
        motion_cmd_t c = s_m.queue[s_m.head];
        s_m.head = (s_m.head + 1) % CMD_QUEUE_LEN;
        handle_cmd(&c);
        if (s_m.estop_flag) {
            break;
        }
    }

    /* 3) 标定模式：舵机由 servo 直控，motion 不参与 */
    if (s_m.cal_mode) {
        servo_tick(dt_ms);
        return;
    }

    /*
     * 4) 心跳超时 -> 自动停止行走（docs/protocol.md §9 不变量 3）
     *
     * 只停行走、保持姿态，不锁存急停标志。理由：Web 断连是常态
     * （手机息屏、切后台、走出范围），若锁存急停则机器人进入
     * ESTOP 态需手动 estop_clear 才能恢复，等于远程失控。
     * 真正的急停只由显式 estop 命令或物理急停键产生。
     */
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (s_m.hb_seen
        && (s_m.state.mode == MOTION_WALK
            || s_m.state.mode == MOTION_STRAFE
            || s_m.state.mode == MOTION_NUDGE
            || s_m.state.mode == MOTION_KEYFRAME)) {
        if ((uint32_t)(now - s_m.last_hb_ms) > HEARTBEAT_MS) {
            s_m.state.vx = s_m.state.vy = s_m.state.yaw_cmd = 0.0f;
            s_m.strafe.active = false;
            s_m.nudge.active = false;
            kf_stop();
            s_m.state.mode = MOTION_IDLE;
            ESP_LOGW(TAG, "心跳超时 %ums，已停止行走（保持姿态，急停未锁存）",
                     (unsigned)(now - s_m.last_hb_ms));
        }
    }

    /* 6) 关键帧优先 */
    float joints[POSTURE_JOINTS];
    if (s_m.state.mode == MOTION_KEYFRAME) {
        if (kf_update(dt_ms, joints)) {
            for (int i = 0; i < 4; i++) {
                servo_set_target(i, SERVO_JOINT_HIP, joints[i * 2]);
                servo_set_target(i, SERVO_JOINT_KNEE, joints[i * 2 + 1]);
            }
            servo_tick(dt_ms);
            return;
        }
        s_m.state.mode = MOTION_IDLE;
    }

    /* 7) 姿态/变高：静止时走 reposture（原地换高度、零滑移） */
    if (s_m.state.mode == MOTION_POSTURE) {
        gait_leg_cmd_t cmds[4];
        int rounds = gait_reposture_settle(&s_m.gait, cmds, dt_s);
        if (rounds == 0) {
            /* 收敛：交回 gait 的常驻输出 */
            s_m.state.mode = MOTION_IDLE;
            gait_update(&s_m.gait, dt_s, cmds);
        }
        gait_leg_cmd_t out[4];
        apply_lean(cmds, out);
        for (int i = 0; i < 4; i++) {
            servo_set_target(i, SERVO_JOINT_HIP, out[i].hip_deg);
            servo_set_target(i, SERVO_JOINT_KNEE, out[i].knee_beta_deg);
        }
        servo_tick(dt_ms);
        s_m.state.height_mm = s_m.gait.height_mm;
        s_m.state.support_margin_mm = gait_support_margin(&s_m.gait);
        return;
    }

    /* 8) 行走（含侧移宏 / 微调期间的步态输出） */
    gait_leg_cmd_t cmds[4];
    if (s_m.state.mode == MOTION_WALK || s_m.state.mode == MOTION_NUDGE
        || s_m.state.mode == MOTION_STRAFE) {
        /* 摇杆意图 -> 速度与航向 */
        if (s_m.state.mode == MOTION_WALK && !s_m.strafe.active) {
            float spd = fabsf(s_m.state.vx) * SPIDER_SPEED_MAX_PCT;
            gait_set_speed(&s_m.gait, spd);
            if (fabsf(s_m.state.yaw_cmd) > 1e-3f) {
                /* 摇杆转向是角速度需求，累加到航向目标上 */
                s_m.state.heading_deg = wrap180(
                    s_m.state.heading_deg
                    + s_m.state.yaw_cmd * SPIDER_YAW_RATE_MAX_DEG_S * 0.15f);
            }
            gait_set_heading(&s_m.gait, s_m.state.heading_deg);
        }
        update_strafe(dt_s);
        gait_update(&s_m.gait, dt_s, cmds);
    } else {
        /* IDLE：仍需输出，但步幅为 0，保持站姿 */
        s_m.gait.cfg.speed_pct = 0.0f;
        gait_update(&s_m.gait, dt_s, cmds);
    }

    gait_leg_cmd_t out[4];
    apply_lean(cmds, out);
    for (int i = 0; i < 4; i++) {
        servo_set_target(i, SERVO_JOINT_HIP, out[i].hip_deg);
        servo_set_target(i, SERVO_JOINT_KNEE, out[i].knee_beta_deg);
    }
    servo_tick(dt_ms);

    s_m.state.height_mm = s_m.gait.height_mm;
    s_m.state.support_margin_mm = gait_support_margin(&s_m.gait);
    s_m.state.gait = s_m.gait.cfg.type;
    s_m.state.speed_pct = s_m.gait.cfg.speed_pct;
    s_m.state.heading_deg = s_m.gait.body.yaw_deg;
}

static void motion_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(TICK_MS);

    /* 让 servo 的 NVS / PCA9685 初始化先完成 */
    vTaskDelay(pdMS_TO_TICKS(50));

    for (;;) {
        motion_tick(TICK_MS);
        vTaskDelay(period);
    }
}

esp_err_t motion_init(void)
{
    memset(&s_m, 0, sizeof(s_m));
    s_m.head = s_m.tail = 0;
    s_m.last_hb_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_m.state.mode = MOTION_IDLE;
    s_m.state.gait = GAIT_CRAWL;
    s_m.state.speed_pct = 0.0f;

    gait_config_t cfg;
    gait_config_defaults(&cfg, GAIT_CRAWL);
    cfg.speed_pct = 0.0f;
    gait_init(&s_m.gait, &cfg);

    if (!s_m.task_started) {
        if (xTaskCreate(motion_task, "motion", 4096, NULL,
                        SPIDER_MOTION_TASK_PRIO, NULL) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        s_m.task_started = true;
    }

    ESP_LOGI(TAG, "就绪 tick=%ums 队列=%d 心跳超时=%ums 姿态=%.0fmm",
             TICK_MS, CMD_QUEUE_LEN, HEARTBEAT_MS, cfg.height_target_mm);
    return ESP_OK;
}

void motion_set_manual_tick(bool enabled)
{
    (void)enabled;   /* 任务自驱，保留接口供后续调试 */
}