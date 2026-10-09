/*
 * 舵机层：角度 ↔ 脉宽映射、逐机标定、软限位、角速度上限、缓启动、NVS 持久化
 *
 * 设计依据：
 *   - docs/hardware.md §7（MG996R 批次脉宽差异、端点严禁贴死）
 *   - docs/architecture.md §5.2/§5.3（关节角约定与软限位）
 *   - docs/gait_design.md §7.2（机械摩擦来源与对策）
 *
 * 关键约束：本层是软限位的唯一执行点。任何路径（Web / CLI / 关键帧）
 * 都必须经过 servo_set_target()，无法绕过限位。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "spider_params.h"
#include "spider_pca9685.h"
#include "spider_servo_logic.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_COUNT   SPIDER_SERVO_COUNT   /* 8 */
#define SERVO_LEGS    SPIDER_LEG_COUNT     /* 4 */

/* 关节索引与 servo_cal_t 定义见 spider_servo_logic.h（与纯逻辑同源） */

typedef struct {
    float     target_deg;  /* 目标角（髋：相对外八中立位；膝：beta） */
    float     current_deg; /* 当前角（受角速度限制追赶） */
    servo_cal_t cal;
    bool     valid;        /* 标定是否已加载 */
} servo_ch_t;

/* 全局模式 */
typedef enum {
    SERVO_MODE_NORMAL = 0,  /* 正常：受软限位与角速度限制 */
    SERVO_MODE_CALIBRATE,   /* 标定：单通道可直写脉宽，绕过角速度限制 */
} servo_mode_t;

/*
 * 初始化。必须在 pca9685_init() 之后调用，并传入该 PCA9685 实例。
 * 读取 NVS 标定；无标定时使用出厂默认（1000~2000µs），并置 valid=false
 * 提示尚未逐机标定。
 */
esp_err_t servo_init(const pca9685_t *dev);

/* 全部通道输出关闭（先拉高 OE 硬件关断，再写 LEDn full-off） */
esp_err_t servo_all_off(void);

/*
 * 硬件级输出使能/关断（PCA9685 的 OE 引脚，低有效）。
 *
 * 关断时不改写 LEDn 寄存器，PCA9685 保留最后一次目标，因此急停清除后
 * 恢复姿态不会先跳到全关位置。急停路径应优先走这里：一次 GPIO 写即关断
 * 全部 16 路，不经 I2C，I2C 挂死时依然有效。
 *
 * 未接 OE（Kconfig 配 -1）时本函数为空操作。
 */
void servo_output_enable(bool enable);

/* 进入/退出标定模式 */
esp_err_t servo_set_mode(servo_mode_t mode);
servo_mode_t servo_get_mode(void);

/*
 * 设置目标角（度）。内部做软限位 clamp——本层的核心安全不变量。
 * hip_deg  : [-SPIDER_HIP_LIMIT_DEG, +SPIDER_HIP_LIMIT_DEG]
 * knee_deg : [SPIDER_BETA_MIN_DEG, SPIDER_BETA_MAX_DEG]
 */
esp_err_t servo_set_target(uint8_t leg, servo_joint_t joint, float deg);

/* 批量设置：angles 按 leg0.hip, leg0.knee, leg1.hip, ... 排列 */
esp_err_t servo_set_all_targets(const float *angles_deg, size_t count);

/* 读取目标角 / 当前角 */
float servo_get_target(uint8_t leg, servo_joint_t joint);
float servo_get_current(uint8_t leg, servo_joint_t joint);

/*
 * 步进：按角速度上限把 current 推向 target，并写出 PWM。
 * 由 motion 任务以 SPIDER_MOTION_TICK_MS 周期调用。
 * 标定模式下不推进（servo_pulse_raw 自行控制输出）。
 */
esp_err_t servo_tick(uint32_t tick_ms);

/* 立即把全部通道拉到 target（忽略角速度限制，仅缓启动/初始化用） */
esp_err_t servo_snap_to_target(void);

/* 缓启动：按角速度上限从当前角缓慢走向站立姿态，避免上电突跳伤舵机 */
esp_err_t servo_soft_start(void);

/* 角度 -> 脉宽（已含标定、reverse、trim、限幅）。对外暴露供标定预览使用 */
esp_err_t servo_deg_to_pulse(uint8_t ch, float deg, uint16_t *pulse_us);

/* 标定模式下直写脉宽（绕过角度换算与角速度限制） */
esp_err_t servo_pulse_raw(uint8_t ch, uint16_t pulse_us);

/* 标定读写 */
esp_err_t servo_cal_get(uint8_t ch, servo_cal_t *out);
esp_err_t servo_cal_set(uint8_t ch, const servo_cal_t *in);
esp_err_t servo_cal_save(void);
esp_err_t servo_cal_load(void);
bool servo_cal_is_ready(void);      /* 8 路全部已标定 */

/*
 * 未落盘标记（RAM 改过但尚未 cal save）。
 *
 * servo_cal_set() 是唯一置位点——CLI 的 cal set / cal reset 与 Web 的两处
 * 都经由它，故四条路径都被覆盖。cal save / cal load 读写整组 blob，
 * 成功后一并清零。
 *
 * 用途：leg show / cal show 对未保存的通道打 '*'，避免调试期忘记
 * cal save 导致掉电丢标定（出厂默认 1000~2000µs 正是 MG996R 贴死区）。
 */
bool servo_cal_is_dirty(uint8_t ch);
uint8_t servo_cal_dirty_mask(void);
void servo_cal_reset_to_factory(void);

/* 读取某通道通道号 */
uint8_t servo_ch_to_pca_channel(uint8_t leg, servo_joint_t joint);

#ifdef __cplusplus
}
#endif