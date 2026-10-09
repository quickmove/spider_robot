/*
 * 2DOF 腿逆运动学
 *
 * 坐标系与轴向约定见 docs/architecture.md §5 与 docs/hardware.md §1。
 * 本组件为纯几何计算，不依赖 I2C / FreeRTOS，便于单元测试与仿真交叉校验。
 */

#include "spider_kinematics.h"

#include <math.h>
#include <stddef.h>

#include "spider_params.h"

#define DEG2RAD(x) ((x) * (M_PI / 180.0f))
#define RAD2DEG(x) ((x) * (180.0f / M_PI))

static inline float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/*
 * 站立高度 H(beta) = L1*sin(alpha) + L2*sin(beta)
 * 巡航高度由 beta 唯一确定，故给定目标高度可反解 beta。
 */
float kin_height_from_beta(float beta_deg)
{
    return SPIDER_L1_MM * sinf(DEG2RAD(SPIDER_ALPHA_DEG))
         + SPIDER_L2_MM * sinf(DEG2RAD(beta_deg));
}

float kin_beta_from_height(float height_mm)
{
    float s = (height_mm - SPIDER_L1_MM * sinf(DEG2RAD(SPIDER_ALPHA_DEG)))
            / SPIDER_L2_MM;
    if (s <= -1.0f) {
        return SPIDER_BETA_MIN_DEG;
    }
    if (s >= 1.0f) {
        return SPIDER_BETA_MAX_DEG;
    }
    return RAD2DEG(asinf(s));
}

/*
 * 足端水平半径 R(beta) = L1*cos(alpha) + L2*cos(beta)
 * 步幅与髋半摆角的换算依赖此值。
 */
float kin_radius_from_beta(float beta_deg)
{
    return SPIDER_L1_MM * cosf(DEG2RAD(SPIDER_ALPHA_DEG))
         + SPIDER_L2_MM * cosf(DEG2RAD(beta_deg));
}

float kin_radius_from_height(float height_mm)
{
    return kin_radius_from_beta(kin_beta_from_height(height_mm));
}

/*
 * 髋半摆角 dpsi = asin(stride / 2R)
 * 髋为竖直轴偏航，摆动方向在水平面内，故步幅与摆角通过足端半径换算。
 */
float kin_hip_sweep_from_stride(float stride_mm, float radius_mm)
{
    if (radius_mm <= 0.0f) {
        return 0.0f;
    }
    float s = stride_mm / (2.0f * radius_mm);
    if (s <= -1.0f) {
        return -SPIDER_HIP_LIMIT_DEG;
    }
    if (s >= 1.0f) {
        return SPIDER_HIP_LIMIT_DEG;
    }
    return RAD2DEG(asinf(s));
}

/*
 * 逆解：足端目标（相对巡航的髋偏角 delta_psi、高度 height_mm）-> 关节角。
 *
 * 髋角定义：0 为腿平面方位角等于 SPIDER_LEGS[i].psi_deg（外八中立位），
 * 正值表示腿向前摆。
 * 膝角返回 beta（与高度同义的直观量），由 motion 层换算为舵机脉宽。
 */
void leg_ik(float delta_psi_deg, float height_mm, leg_ik_t *out)
{
    if (out == NULL) {
        return;
    }

    float beta = kin_beta_from_height(height_mm);
    beta = clampf(beta, SPIDER_BETA_MIN_DEG, SPIDER_BETA_MAX_DEG);

    float hip = clampf(delta_psi_deg,
                       -SPIDER_HIP_LIMIT_DEG, SPIDER_HIP_LIMIT_DEG);

    out->hip_deg = hip;
    out->knee_beta_deg = beta;
    out->height_mm = kin_height_from_beta(beta);
}

/*
 * 膝角变动的运动学耦合：高度变化 : 水平位移 = 1 : cot(beta)。
 * 变高若在支撑相进行必然拖脚，故 motion 层须把变高放进摆动相。
 */
float kin_height_coupling(float beta_deg)
{
    float t = tanf(DEG2RAD(beta_deg));
    if (t <= 1e-4f) {
        return 1e4f;   /* beta 趋近 0 时耦合发散 */
    }
    return 1.0f / t;
}