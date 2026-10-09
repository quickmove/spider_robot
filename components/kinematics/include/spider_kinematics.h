/*
 * 2DOF 腿逆运动学接口
 *
 * 几何关系与推导见 docs/gait_design.md §2 与 docs/architecture.md §6。
 * 本组件为纯计算，不依赖 I2C / FreeRTOS，可与 tools/gait_sim 交叉校验。
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float hip_deg;       /* 髋角：0 为外八中立位，正值为腿向前摆 */
    float knee_beta_deg; /* 膝角 beta：小腿与水平面夹角，与高度同义 */
    float height_mm;     /* 实际达成的站立高度（限位后） */
} leg_ik_t;

/* 站立高度 <-> 膝角 */
float kin_height_from_beta(float beta_deg);
float kin_beta_from_height(float height_mm);

/* 足端水平半径 */
float kin_radius_from_beta(float beta_deg);
float kin_radius_from_height(float height_mm);

/* 步幅 -> 髋半摆角 */
float kin_hip_sweep_from_stride(float stride_mm, float radius_mm);

/* 逆解：足端目标 -> 关节角（内部含软限位） */
void leg_ik(float delta_psi_deg, float height_mm, leg_ik_t *out);

/* 膝角变动的水平/高度耦合比 1/cot(beta) */
float kin_height_coupling(float beta_deg);

#ifdef __cplusplus
}
#endif