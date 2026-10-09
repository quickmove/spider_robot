#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "spider_kinematics.h"
#include "spider_params.h"

#define DEG2RAD(x) ((x)*(M_PI/180.0f))
#define RAD2DEG(x) ((x)*(180.0f/M_PI))
static int fails = 0;
static void chk(const char*n,float ok,float v){printf("[%s] %-42s %.4f\n",ok?"PASS":"FAIL",n,v);if(!ok)fails++;}

int main(void){
  /* 几何自洽: beta -> H -> beta 往返 */
  for (float b = SPIDER_BETA_MIN_DEG; b <= SPIDER_BETA_MAX_DEG; b += 1.0f) {
    float h = kin_height_from_beta(b);
    float b2 = kin_beta_from_height(h);
    if (fabsf(b - b2) > 0.05f) { printf("roundtrip FAIL b=%.1f -> H=%.2f -> b=%.2f\n", b, h, b2); fails++; }
  }
  printf("[PASS] beta<->height roundtrip over [%0.1f,%.1f]\n", SPIDER_BETA_MIN_DEG, SPIDER_BETA_MAX_DEG);

  chk("巡航 H == 120mm", fabsf(kin_height_from_beta(SPIDER_BETA_CRUISE_DEG)-120.0f)<0.5f,
      kin_height_from_beta(SPIDER_BETA_CRUISE_DEG));
  chk("巡航 R == SPIDER_CRUISE_RADIUS_MM", fabsf(kin_radius_from_beta(SPIDER_BETA_CRUISE_DEG)-SPIDER_CRUISE_RADIUS_MM)<0.5f,
      kin_radius_from_beta(SPIDER_BETA_CRUISE_DEG));
  chk("beta_min -> H=90", fabsf(kin_height_from_beta(SPIDER_BETA_MIN_DEG)-90.0f)<1.5f,
      kin_height_from_beta(SPIDER_BETA_MIN_DEG));
  chk("beta_max -> H=150", fabsf(kin_height_from_beta(SPIDER_BETA_MAX_DEG)-150.0f)<1.5f,
      kin_height_from_beta(SPIDER_BETA_MAX_DEG));

  /* 软限位 clamp */
  leg_ik_t ik;
  leg_ik(0, 9999, &ik);
  chk("超限高被 clamp 到 beta_max", fabsf(ik.knee_beta_deg-SPIDER_BETA_MAX_DEG)<0.01f, ik.knee_beta_deg);
  leg_ik(0, -999, &ik);
  chk("负高度被 clamp 到 beta_min", fabsf(ik.knee_beta_deg-SPIDER_BETA_MIN_DEG)<0.01f, ik.knee_beta_deg);
  leg_ik(999, 120, &ik);
  chk("超限髋角被 clamp", fabsf(ik.hip_deg-SPIDER_HIP_LIMIT_DEG)<0.01f, ik.hip_deg);
  leg_ik(-999, 120, &ik);
  chk("负超限髋角被 clamp", fabsf(ik.hip_deg+SPIDER_HIP_LIMIT_DEG)<0.01f, ik.hip_deg);

  /* 步幅 -> 摆角，与 params.yaml 注释值一致 */
  float r = kin_radius_from_beta(SPIDER_BETA_CRUISE_DEG);
  float sw = kin_hip_sweep_from_stride(SPIDER_CRAWL_STRIDE_MM, r);
  chk("crawl 摆角 ~10.1deg", fabsf(sw-10.1f)<0.2f, sw);
  chk("trot 摆角 ~11.4deg", fabsf(kin_hip_sweep_from_stride(SPIDER_TROT_STRIDE_MM,r)-11.4f)<0.2f,
      kin_hip_sweep_from_stride(SPIDER_TROT_STRIDE_MM,r));

  /* 步幅还原: 2*R*sin(sweep) == stride */
  chk("步幅可逆", fabsf(2*r*sinf(DEG2RAD(sw))-SPIDER_CRAWL_STRIDE_MM)<0.5f, 2*r*sinf(DEG2RAD(sw)));

  /* 耦合 1/cot(beta): 巡航处 ~1.21 */
  chk("巡航耦合 ~1.21", fabsf(kin_height_coupling(SPIDER_BETA_CRUISE_DEG)-1.21f)<0.05f,
      kin_height_coupling(SPIDER_BETA_CRUISE_DEG));

  /* 耦合数值验证: dH/dbeta 与 dR/dbeta 之比 == 1/cot(beta) */
  {
    float b = 39.5f, e = 0.01f;
    float dh = kin_height_from_beta(b+e) - kin_height_from_beta(b-e);
    float dr = kin_radius_from_beta(b+e) - kin_radius_from_beta(b-e);
    float ratio = -dh/dr;
    chk("耦合数值验证", fabsf(ratio-kin_height_coupling(b))<0.02f, ratio);
  }

  /* IK 高度往返 */
  for (float h = 95.0f; h <= 145.0f; h += 5.0f) {
    leg_ik(0, h, &ik);
    if (fabsf(ik.height_mm - h) > 0.05f) { printf("ik height FAIL h=%.0f -> %.3f\n", h, ik.height_mm); fails++; }
  }
  printf("[PASS] leg_ik 高度往返 95~145mm\n");

  printf("\n%s (%d failures)\n", fails ? "FAILED" : "ALL PASS", fails);
  return fails ? 1 : 0;
}
