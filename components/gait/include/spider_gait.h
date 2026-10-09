/*
 * 步态引擎：爬行 4 拍 + Trot 2 拍
 *
 * 核心设计（docs/gait_design.md §7）：
 *   支撑腿的关节角由「足端世界坐标固定」反解——足端是被钉在地面上的，
 *   机身从脚上方移过去。模型层滑移因此为 0，不是"接近 0"。
 *
 * 坐标约定见 docs/architecture.md §5：机身 X 前 / Y 左 / Z 上。
 * 本组件是纯计算，不依赖 I2C / FreeRTOS，可与 tools/gait_sim 交叉校验。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "spider_kinematics.h"
#include "spider_params.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GAIT_LEG_COUNT  4

typedef enum {
    GAIT_CRAWL = 0,
    GAIT_TROT  = 1,
} gait_type_t;

/* 变高方式（docs/gait_design.md §4） */
typedef enum {
    GAIT_HEIGHT_SWING   = 0,  /* 摆动相内逐腿变高：零滑移（默认） */
    GAIT_HEIGHT_STANDING,     /* 支撑相变高：会拖脚，仅标定模式允许 */
} gait_height_mode_t;

/* 单腿指令：关节角（度）+ 该腿当前锁定的高度 */
typedef struct {
    float hip_deg;
    float knee_beta_deg;
    float height_mm;      /* 支撑腿锁定的身高；摆动腿跟随目标 */
    bool  in_swing;
} gait_leg_cmd_t;

typedef struct {
    gait_type_t type;
    float stride_mm;
    float period_s;
    float lift_mm;
    float speed_pct;      /* 0~100 */
    float heading_deg;    /* 目标航向（度，绕机身 Z） */
    float height_target_mm;
    gait_height_mode_t height_mode;
} gait_config_t;

/* 世界坐标（mm / deg） */
typedef struct {
    float x_mm;
    float y_mm;
    float yaw_deg;
} gait_pose_t;

/* 引擎内部状态 */
typedef struct {
    gait_config_t  cfg;
    float          phase;         /* 0~1 周期相位 */
    gait_pose_t    body;          /* 机身位姿（航位推算） */
    float          height_mm;     /* 当前生效身高 */
    gait_leg_cmd_t leg[GAIT_LEG_COUNT];

    /* 每腿足端的世界坐标锚点：支撑期不变，摆动期插值 */
    float foot_world_x[GAIT_LEG_COUNT];
    float foot_world_y[GAIT_LEG_COUNT];
    float foot_from_x[GAIT_LEG_COUNT];   /* 本次摆动起点 */
    float foot_from_y[GAIT_LEG_COUNT];
    float foot_to_x[GAIT_LEG_COUNT];     /* 本次摆动落点 */
    float foot_to_y[GAIT_LEG_COUNT];

    /*
     * 原地变高模式：机身禁止平移，落点改为「新半径、旧方位」。
     * 半径变小则机身升高，变大则降低；脚在摆动相里移动，故零滑移；
     * 方位不变，故机身不横向漂移（docs/gait_design.md §4）。
     */
    bool  reposture;
    float reposture_radius;

    bool  initialised;
} gait_t;

/* 填入 params.yaml 的出厂步态参数 */
void gait_config_defaults(gait_config_t *cfg, gait_type_t type);

/* 初始化：把四足放到当前姿态正下方的地面上 */
void gait_init(gait_t *g, const gait_config_t *cfg);

/* 重置相位（重新落步），保持当前足端位置 */
void gait_reset_phase(gait_t *g);

/*
 * 推进 dt 秒，产出 8 个关节角。
 * dt 建议 <= 10ms；返回 false 表示参数非法未推进。
 */
bool gait_update(gait_t *g, float dt_s, gait_leg_cmd_t *out_cmds);

/* 请求变高（摆动相内逐腿执行，零滑移）。返回 false 表示超出高度范围 */
bool gait_request_height(gait_t *g, float height_mm);

/* 立即设定高度（支撑相直接改，会拖脚）。仅标定模式使用 */
void gait_force_height(gait_t *g, float height_mm);

/* 设置速度百分比 0~100（会被 Trot 防翻上限约束） */
void gait_set_speed(gait_t *g, float speed_pct);

/* 设置目标航向（度）。转向由落点方位角实现，不做支撑足拧转 */
void gait_set_heading(gait_t *g, float heading_deg);

/* 当前姿态下的支撑多边形最小边裕度（mm）；转向时下降，跌破阈值即风险 */
float gait_support_margin(const gait_t *g);

/* 该腿是否处于摆动相 */
bool gait_leg_swinging(const gait_t *g, uint8_t leg);

/* 足端世界坐标（调试与滑移分析用） */
void gait_foot_world(const gait_t *g, uint8_t leg, float *x_mm, float *y_mm);

/*
 * 重复执行"零位移变高"的落脚循环，把身高推向目标。
 * 每轮四腿依次重新落地（脚在摆动相里移动，故无滑移），机身基本不动。
 * 返回已收敛的轮数，0 表示无法继续（已到位）。
 * 由 motion 层在非行走状态下调用，实现"原地换高度但不拖脚"。
 */
int gait_reposture_settle(gait_t *g, gait_leg_cmd_t *out_cmds, float dt_s);

#ifdef __cplusplus
}
#endif