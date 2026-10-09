/*
 * PCA9685 纯算术：分频计算与脉宽→计数换算
 *
 * 与 pca9685.c 的寄存器 I/O 分离，便于主机端单元测试。
 *
 * 关键正确性：
 *   - prescale = OSC / (4096 * f) - 1，且数据手册规定不得为 0
 *     （prescale=0 时芯片会持续重置振荡器，输出停在 low）
 *   - counts = pulse_us * 4096 / (1e6 / f)，本实现用 Q16 定点避免浮点
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9685_OSC_HZ            25000000UL
#define PCA9685_PWM_RES_MAX       4095U

/*
 * prescale 是 8bit（1~255），故可达频率范围被硬件限制：
 *   f_max = OSC / (4096 * 2)              = 3052 Hz
 *   f_min = OSC / (4096 * 256)            ≈ 23.84 Hz
 * 舵机 50Hz 远在范围内；低于 24Hz 无意义（脉宽会超过周期）。
 */
#define PCA9685_MAX_FREQ_HZ       1000U   /* 上限留余量，未用到 3052 的物理极限 */
#define PCA9685_MIN_FREQ_HZ       24U

/* 由 PWM 频率求预分频值；频率越界返回 false */
bool pca9685_math_prescale(uint16_t freq_hz, uint8_t *prescale_out);

/* 由预分频值反推实际频率（四舍五入后可能有偏差） */
uint16_t pca9685_math_actual_freq(uint8_t prescale);

/* 脉宽(µs) -> 占空计数；超出 0 或一周期时收敛到合法值 */
uint16_t pca9685_math_counts_from_pulse(uint16_t pulse_us, uint16_t freq_hz);

/* 占空计数 -> 脉宽(µs)，用于反解校核 */
uint16_t pca9685_math_pulse_from_counts(uint16_t counts, uint16_t freq_hz);

/* 一周期时长(µs) = 1e6 / freq */
uint32_t pca9685_math_period_us(uint16_t freq_hz);

#ifdef __cplusplus
}
#endif