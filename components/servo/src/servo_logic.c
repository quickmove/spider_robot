#include "spider_servo_logic.h"

#include <math.h>
#include <stddef.h>

#include "spider_params.h"

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void servo_logic_joint_limits(servo_joint_t joint, float *lo, float *hi)
{
    if (joint == SERVO_JOINT_HIP) {
        *lo = -SPIDER_HIP_LIMIT_DEG;
        *hi = SPIDER_HIP_LIMIT_DEG;
    } else {
        *lo = SPIDER_BETA_MIN_DEG;
        *hi = SPIDER_BETA_MAX_DEG;
    }
}

void servo_logic_factory_cal(servo_cal_t *cal)
{
    if (cal == NULL) {
        return;
    }
    cal->min_us = SPIDER_SERVO_MIN_US;
    cal->max_us = SPIDER_SERVO_MAX_US;
    cal->neutral_us = SPIDER_SERVO_NEUTRAL_US;
    cal->trim_us = 0;
    cal->reverse = false;
}

bool servo_logic_cal_valid(const servo_cal_t *cal)
{
    if (cal == NULL) {
        return false;
    }
    if (cal->min_us >= cal->max_us) {
        return false;
    }
    /* 范围过窄则等效行程不足，MG996R 至少需要 ±100° */
    if ((int32_t)(cal->max_us - cal->min_us) < SERVO_CAL_MIN_RANGE_US) {
        return false;
    }
    /* 中位必须落在端点安全区内，否则 trim 一加就顶死 */
    if (cal->neutral_us <= (uint32_t)cal->min_us + SERVO_CAL_NEUTRAL_MARGIN_US) {
        return false;
    }
    if (cal->neutral_us >= (uint32_t)cal->max_us - SERVO_CAL_NEUTRAL_MARGIN_US) {
        return false;
    }
    return true;
}

static void endpoints_for(const servo_cal_t *cal, uint16_t *p_lo, uint16_t *p_hi)
{
    if (cal->reverse) {
        *p_lo = cal->max_us;
        *p_hi = cal->min_us;
    } else {
        *p_lo = cal->min_us;
        *p_hi = cal->max_us;
    }
}

bool servo_logic_deg_to_pulse(const servo_cal_t *cal, servo_joint_t joint,
                              float deg, uint16_t *pulse_us)
{
    if (cal == NULL || pulse_us == NULL || isnan(deg)) {
        return false;
    }

    float lo, hi;
    servo_logic_joint_limits(joint, &lo, &hi);

    bool clipped = false;
    if (deg < lo) { deg = lo; clipped = true; }
    if (deg > hi) { deg = hi; clipped = true; }

    uint16_t p_lo, p_hi;
    endpoints_for(cal, &p_lo, &p_hi);

    /* 两端锚定：中点落在 min/max 均值，trim 在此叠加 */
    float span = (float)(p_hi - p_lo);
    float t = (deg - lo) / (hi - lo);
    float pulse = (float)p_lo + t * span + (float)cal->trim_us;

    /* 硬限幅：绝不让脉宽越过标定端点（端点已含硬件余量） */
    uint16_t hard_lo = (cal->min_us < cal->max_us) ? cal->min_us : cal->max_us;
    uint16_t hard_hi = (cal->min_us < cal->max_us) ? cal->max_us : cal->min_us;
    pulse = clampf(pulse, (float)hard_lo, (float)hard_hi);

    *pulse_us = (uint16_t)lrintf(pulse);
    return !clipped;
}

bool servo_logic_pulse_to_deg(const servo_cal_t *cal, servo_joint_t joint,
                              uint16_t pulse_us, float *deg)
{
    if (cal == NULL || deg == NULL) {
        return false;
    }

    float lo, hi;
    servo_logic_joint_limits(joint, &lo, &hi);

    uint16_t p_lo, p_hi;
    endpoints_for(cal, &p_lo, &p_hi);
    if (p_hi == p_lo) {
        return false;
    }

    float t = ((float)pulse_us - (float)p_lo) / (float)(p_hi - p_lo);
    *deg = lo + t * (hi - lo);
    return true;
}

bool servo_logic_clamp_deg(servo_joint_t joint, float deg, float *out)
{
    if (out == NULL || isnan(deg)) {
        return false;
    }
    float lo, hi;
    servo_logic_joint_limits(joint, &lo, &hi);
    float c = clampf(deg, lo, hi);
    *out = c;
    return c != deg;
}

bool servo_logic_slew(float current_deg, float target_deg,
                      float max_step_deg, float *out_deg)
{
    if (out_deg == NULL) {
        return false;
    }
    float diff = target_deg - current_deg;
    if (fabsf(diff) <= max_step_deg) {
        *out_deg = target_deg;
        return true;
    }
    *out_deg = current_deg + ((diff > 0.0f) ? max_step_deg : -max_step_deg);
    return false;
}

uint16_t servo_logic_clamp_pulse(const servo_cal_t *cal, uint16_t pulse_us)
{
    if (cal == NULL) {
        return pulse_us;
    }
    uint16_t lo = (cal->min_us < cal->max_us) ? cal->min_us : cal->max_us;
    uint16_t hi = (cal->min_us < cal->max_us) ? cal->max_us : cal->min_us;
    if (pulse_us < lo) return lo;
    if (pulse_us > hi) return hi;
    return pulse_us;
}