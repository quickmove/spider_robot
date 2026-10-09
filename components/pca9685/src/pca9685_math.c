#include "spider_pca9685_math.h"

#include <stddef.h>

uint32_t pca9685_math_period_us(uint16_t freq_hz)
{
    if (freq_hz == 0) {
        return 0;
    }
    return 1000000UL / (uint32_t)freq_hz;
}

bool pca9685_math_prescale(uint16_t freq_hz, uint8_t *prescale_out)
{
    if (prescale_out == NULL || freq_hz < PCA9685_MIN_FREQ_HZ
        || freq_hz > PCA9685_MAX_FREQ_HZ) {
        return false;
    }

    /* f = OSC / (4096 * (prescale + 1))  =>  prescale = OSC/(4096*f) - 1 */
    uint32_t denom = 4096UL * (uint32_t)freq_hz;
    uint32_t p = (PCA9685_OSC_HZ + denom / 2) / denom;
    if (p == 0) {
        p = 1;
    }
    if (p > 255) {
        return false;
    }
    *prescale_out = (uint8_t)p;
    return true;
}

uint16_t pca9685_math_actual_freq(uint8_t prescale)
{
    if (prescale == 0) {
        return 0;   /* 非法值：芯片会停振 */
    }
    return (uint16_t)(PCA9685_OSC_HZ / (4096UL * (uint32_t)(prescale + 1)));
}

uint16_t pca9685_math_counts_from_pulse(uint16_t pulse_us, uint16_t freq_hz)
{
    if (freq_hz == 0) {
        return 0;
    }
    /* counts = pulse_us * 4096 / period_us，用 Q16 定点避免浮点 */
    uint32_t period = pca9685_math_period_us(freq_hz);
    if (period == 0) {
        return 0;
    }
    /* 1us 对应的计数 = 4096 / period；先算 Q16 */
    uint32_t per_us_q16 = (4096UL << 16) / period;
    uint32_t counts = ((uint32_t)pulse_us * per_us_q16 + 32768UL) >> 16;

    if (counts > PCA9685_PWM_RES_MAX) {
        counts = PCA9685_PWM_RES_MAX;
    }
    return (uint16_t)counts;
}

uint16_t pca9685_math_pulse_from_counts(uint16_t counts, uint16_t freq_hz)
{
    uint32_t period = pca9685_math_period_us(freq_hz);
    if (period == 0) {
        return 0;
    }
    return (uint16_t)(((uint32_t)counts * period + 2048UL) / 4096UL);
}