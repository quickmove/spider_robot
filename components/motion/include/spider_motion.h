/*
 * 运动任务：命令队列、状态机、急停、侧移宏、滑移原语、防翻约束
 *
 * 本层是舵机写入的唯一所有者（docs/architecture.md §3）。
 * 其余一切（Web / CLI / 关键帧 / 二期传感器）都只能 motion_enqueue()。
 *
 * 硬不变量（docs/protocol.md §9）：
 *   1. 急停走独立标志位，队列满也不失效
 *   2. 软限位由 servo 层强制，本层无法绕过
 *   3. 心跳超时必停
 *   4. 标定模式与行走互斥
 *   5. 防翻约束默认锁死
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "spider_gait.h"
#include "spider_params.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTION_IDLE = 0,     /* 静止 */
    MOTION_WALK,         /* 行走中 */
    MOTION_POSTURE,      /* 姿态切换（含原地变高） */
    MOTION_KEYFRAME,     /* 关键帧播放中 */
    MOTION_NUDGE,        /* 滑移微调中 */
    MOTION_STRAFE,       /* 侧移宏执行中 */
    MOTION_ESTOP,        /* 急停 */
    MOTION_CALIBRATE,    /* 标定（舵机由 servo 直控） */
} motion_mode_t;

typedef enum {
    CMD_SET_POSTURE = 0, /* params.id = posture_id_t */
    CMD_SET_HEIGHT,      /* params.f = mm */
    CMD_GAIT,            /* params.i = gait_type_t */
    CMD_SET_SPEED,       /* params.f = 0~100 */
    CMD_DRIVE,           /* vx,vy,yaw（连续意图） */
    CMD_STOP,
    CMD_NUDGE,           /* f[0]=dx mm, f[1]=dy mm */
    CMD_LEAN,            /* f[0]=roll, f[1]=pitch (deg) */
    CMD_STRAFE,          /* f[0]=dx mm */
    CMD_PLAY,            /* params.s = 序列名 */
    CMD_ESTOP,
    CMD_ESTOP_CLEAR,
    CMD_CALIBRATE_MODE,  /* params.i = servo_mode_t */
} cmd_type_t;

/* 命令体：固定大小，便于环形队列整体拷贝 */
typedef struct {
    cmd_type_t type;
    union {
        float f[4];      /* CMD_DRIVE 用 f[0..2]: vx, vy, yaw */
        int   i;
        struct {
            char name[24];
        } s;
    } params;
    uint32_t stamp_ms;
} motion_cmd_t;

typedef struct {
    motion_mode_t mode;
    gait_type_t   gait;
    float         speed_pct;
    float         heading_deg;
    float         height_mm;
    float         support_margin_mm;
    bool          estop;
    bool          advanced;        /* 解锁非物理红线约束 */
    /* 连续驱动意图（摇杆） */
    float         vx, vy, yaw_cmd;
} motion_state_t;

/* 初始化并创建 motion 任务 */
esp_err_t motion_init(void);

/* 入队（非阻塞）。队列满返回 ESP_ERR_NO_MEM */
esp_err_t motion_enqueue(const motion_cmd_t *cmd);

/* 便捷入队 */
esp_err_t motion_set_posture(int posture_id);
esp_err_t motion_set_height(float mm);
esp_err_t motion_set_gait(gait_type_t type);
esp_err_t motion_set_speed(float pct);
esp_err_t motion_drive(float vx, float vy, float yaw);
esp_err_t motion_stop(void);
esp_err_t motion_nudge(float dx_mm, float dy_mm);
esp_err_t motion_lean(float roll_deg, float pitch_deg);
esp_err_t motion_strafe(float dx_mm);
esp_err_t motion_play(const char *seq_name);

/* 急停：置独立标志位，立即生效 */
void motion_estop(void);
esp_err_t motion_estop_clear(void);

/* 心跳：收到任何有效指令时调用（M3 由 web 层调用） */
void motion_heartbeat(uint32_t now_ms);

/* 状态查询（供遥测回传） */
void motion_get_state(motion_state_t *out);
motion_mode_t motion_get_mode(void);
const char *motion_mode_name(motion_mode_t m);

/* 供 CLI / Web：手动推进若干 tick（M1 临时，M3 后由任务自身驱动） */
void motion_set_manual_tick(bool enabled);

#ifdef __cplusplus
}
#endif