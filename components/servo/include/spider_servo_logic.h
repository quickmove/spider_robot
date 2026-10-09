/*
 * 舵机纯逻辑：角度↔脉宽换算、标定校验、软限位、角速度斜坡
 *
 * 本文件不含任何 I2C / NVS / FreeRTOS 依赖，可在主机端直接编译测试，
 * 与 servo.c 的硬件 I/O 部分严格分离（docs/architecture.md §2）。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 单路标定参数 */
typedef struct {
    uint16_t min_us;      /* 端点（已含硬件余量） */
    uint16_t max_us;      /* 端点（已含硬件余量） */
    uint16_t neutral_us;  /* 机械中位 */
    int16_t  trim_us;     /* 中位微调 */
    bool     reverse;     /* 方向反转 */
} servo_cal_t;

/* 标定合法性判据 */
#define SERVO_CAL_MIN_RANGE_US   400   /* 脉宽范围下限（≈±100° 可动） */
#define SERVO_CAL_NEUTRAL_MARGIN_US 100 /* 中位距端点的最小余量 */

#define SERVO_MAX_COUNT 8

typedef enum {
    SERVO_JOINT_HIP  = 0,
    SERVO_JOINT_KNEE = 1,
} servo_joint_t;

/* 关节逻辑角区间（软限位的唯一来源） */
void servo_logic_joint_limits(servo_joint_t joint, float *lo, float *hi);

/* 出厂标定 */
void servo_logic_factory_cal(servo_cal_t *cal);

/* 标定合法性校验：范围、顺序、中位安全区 */
bool servo_logic_cal_valid(const servo_cal_t *cal);

/*
 * 角度 -> 脉宽（不含硬件 clamp 的裁剪，返回 false 表示 deg 越界被裁剪）。
 *
 * 以标定端点为锚：逻辑区间 [lo,hi] 线性映射到 [min_us, max_us]，
 * 保证「软限住」与「物理端点」一致，不会出现限住了但舵机仍顶死。
 */
bool servo_logic_deg_to_pulse(const servo_cal_t *cal, servo_joint_t joint,
                              float deg, uint16_t *pulse_us);

/* 脉宽 -> 角度（标定预览与几何校核用） */
bool servo_logic_pulse_to_deg(const servo_cal_t *cal, servo_joint_t joint,
                              uint16_t pulse_us, float *deg);

/* 软限位钳制；越界返回 true */
bool servo_logic_clamp_deg(servo_joint_t joint, float deg, float *out);

/*
 * 角速度斜坡步进：把 current 朝 target 推进不超过 max_step 度。
 * 返回 true 表示已到达 target。
 */
bool servo_logic_slew(float current_deg, float target_deg,
                      float max_step_deg, float *out_deg);

/* 标定模式下的硬限幅：脉宽不得越过端点 */
uint16_t servo_logic_clamp_pulse(const servo_cal_t *cal, uint16_t pulse_us);

#ifdef __cplusplus
}
#endif