/* 自动生成，请勿手改 */
/* 源: tools/gait_sim/params.yaml  生成器: tools/gen_params.py */
/* 改动参数请编辑 params.yaml 后重新运行生成器 */
#pragma once

/* 该头文件亦被主机端单元测试直接包含，故自带基础类型依赖 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPIDER_PARAMS_VERSION         1
#define SPIDER_PARAMS_DESC            "机器蜘蛛 2DOF 四足参数"

/* ---- 机身 ---- */
#define SPIDER_BODY_LENGTH_MM         200.0000f
#define SPIDER_BODY_WIDTH_MM          120.0000f
#define SPIDER_LAYER_GAP_MM           30.0000f
#define SPIDER_TOTAL_MASS_G           1100.0000f

/* ---- 腿几何 ---- */
#define SPIDER_L1_MM                  100.0000f
#define SPIDER_L2_MM                  110.0000f
#define SPIDER_ALPHA_DEG              30.0000f
#define SPIDER_BETA_CRUISE_DEG        39.5000f
#define SPIDER_SPLAY_DEG              40.0000f
#define SPIDER_CRUISE_HEIGHT_MM       120.0000f
#define SPIDER_CRUISE_RADIUS_MM       171.4812f

/* ---- 软限位 ---- */
#define SPIDER_HIP_LIMIT_DEG          60.0000f
#define SPIDER_BETA_MIN_DEG           21.3000f
#define SPIDER_BETA_MAX_DEG           65.4000f
#define SPIDER_MAX_RATE_DEG_S         180.0000f
#define SPIDER_SERVO_RATED_DEG_S      428.0000f

/* ---- 姿态高度 ---- */
#define SPIDER_STAND_HEIGHT_MM        120.0000f
#define SPIDER_HEIGHT_MIN_MM          90.0000f
#define SPIDER_HEIGHT_MAX_MM          150.0000f
#define SPIDER_SIT_HEIGHT_MM          95.0000f
#define SPIDER_REST_HEIGHT_MM         92.0000f
#define SPIDER_SPAWN_HEIGHT_MM        70.0000f

/* ---- 步态 ---- */
#define SPIDER_CRAWL_STRIDE_MM        60.0000f
#define SPIDER_CRAWL_PERIOD_S         1.7000f
#define SPIDER_CRAWL_LIFT_MM          28.0000f
#define SPIDER_CRAWL_DUTY_SWING       0.2500f
#define SPIDER_TROT_STRIDE_MM         68.0000f
#define SPIDER_TROT_PERIOD_S          1.8000f
#define SPIDER_TROT_LIFT_MM           29.0000f
#define SPIDER_TROT_DUTY_SWING        0.5000f
#define SPIDER_SPEED_MAX_PCT          100.0000f
#define SPIDER_TROT_MAX_SPEED_PCT     70.0000f

/* ---- 摆动相四段（周期占比） ---- */
#define SPIDER_SWING_LIFT             0.0000, 0.1500
#define SPIDER_SWING_REACH            0.1500, 0.5500
#define SPIDER_SWING_PLACE            0.5500, 0.7000
#define SPIDER_SWING_PUSH             0.7000, 1.0000

/* ---- 转向与侧移 ---- */
#define SPIDER_TURN_INPLACE_MAX_DEG   25.0000f
#define SPIDER_YAW_RATE_MAX_DEG_S     25.0000f
#define SPIDER_FOOT_PLACE_BUDGET_DEG  45.0000f
#define SPIDER_STRAFE_MAX_MM          150.0000f

/* ---- 滑移原语 ---- */
#define SPIDER_NUDGE_MAX_MM           50.0000f
#define SPIDER_LEAN_MAX_DEG           10.0000f
#define SPIDER_SKID_TURN_MAX_DEG      10.0000f
#define SPIDER_RATCHET_SLOW_S         0.6000f
#define SPIDER_RATCHET_FAST_S         0.1500f
#define SPIDER_RATCHET_HIP_DELTA_DEG  18.0000f

/* ---- 安全 ---- */
#define SPIDER_HEARTBEAT_TIMEOUT_MS   500
#define SPIDER_ESTOP_RESPONSE_MS      50
#define SPIDER_MOTION_TICK_MS         2
#define SPIDER_MOTION_TASK_PRIO       5
#define SPIDER_WEB_TASK_PRIO          4
#define SPIDER_CMD_QUEUE_LEN          16

/* ---- 电气 ---- */
#define SPIDER_PWM_FREQ_HZ            50
#define SPIDER_PWM_RES_BITS           12
#define SPIDER_PCA9685_ADDR           0x60
#define SPIDER_SERVO_MIN_US           1000
#define SPIDER_SERVO_MAX_US           2000
#define SPIDER_SERVO_NEUTRAL_US       1500
#define SPIDER_SERVO_EDGE_MARGIN_US   8
#define SPIDER_SERVO_COUNT            8

/* ---- 腿位表（索引 = leg_index，通道 = index*2 为髋）---- */
typedef struct {
    const char *id;    /* 腿标识：FL / FR / RL / RR */
    float    psi_deg;  /* 腿平面方位角，X 轴逆时针 */
    float    r_mm;     /* 巡航足端水平半径 */
    uint16_t ch_hip;   /* PCA9685 通道 */
    uint16_t ch_knee;  /* PCA9685 通道 */
    float    hip_x_mm; /* 髋安装点机身坐标 X */
    float    hip_y_mm; /* 髋安装点机身坐标 Y */
} spider_leg_t;

#define SPIDER_LEG_COUNT 4
static const spider_leg_t SPIDER_LEGS[SPIDER_LEG_COUNT] = {
    { "FL", 36.0000f, 171.5000f, 0, 1, 100.0000f, 60.0000f },
    { "FR", 324.0000f, 171.5000f, 2, 3, 100.0000f, -60.0000f },
    { "RL", 132.0000f, 171.5000f, 4, 5, -100.0000f, 60.0000f },
    { "RR", 228.0000f, 171.5000f, 6, 7, -100.0000f, -60.0000f },
};

#ifdef __cplusplus
}
#endif
