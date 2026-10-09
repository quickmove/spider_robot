/*
 * 舵机纯逻辑主机端测试
 *
 * 覆盖：标定合法性、角度↔脉宽换算、reverse/trim、软限位、角速度斜坡、脉宽硬限幅。
 * 不依赖 I2C / NVS / FreeRTOS。
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "spider_params.h"
#include "spider_servo_logic.h"

static int g_fail;
static int g_total;

static void chk(const char *name, int ok, const char *fmt, ...)
{
    (void)fmt;
    g_total++;
    if (!ok) {
        g_fail++;
        printf("[FAIL] %s\n", name);
    }
}

static void chkf(const char *name, int ok, float v)
{
    g_total++;
    if (!ok) {
        g_fail++;
    }
    printf("[%s] %-46s %9.3f\n", ok ? "PASS" : "FAIL", name, v);
}

static servo_cal_t factory_cal(void)
{
    servo_cal_t c;
    servo_logic_factory_cal(&c);
    return c;
}

int main(void)
{
    servo_cal_t cal = factory_cal();

    /* ---- 标定合法性 ---- */
    chk("出厂标定有效", servo_logic_cal_valid(&cal), "");
    {
        servo_cal_t bad = cal;
        bad.min_us = 1900; bad.max_us = 1900;
        chk("min==max 被拒", !servo_logic_cal_valid(&bad), "");
        bad = cal; bad.max_us = 1200;
        chk("范围过窄被拒 (<400us)", !servo_logic_cal_valid(&bad), "");
        bad = cal; bad.neutral_us = 1050;
        chk("中位贴近端点被拒", !servo_logic_cal_valid(&bad), "");
        bad = cal; bad.neutral_us = 1950;
        chk("中位贴近另一端被拒", !servo_logic_cal_valid(&bad), "");
        bad = cal; bad.neutral_us = 1250;
        chk("中位在安全区被接受", servo_logic_cal_valid(&bad), "");
    }

    /* ---- 关节限位 ---- */
    {
        float lo, hi;
        servo_logic_joint_limits(SERVO_JOINT_HIP, &lo, &hi);
        chkf("髋限位 == ±60", fabsf(lo + 60.0f) < 1e-3 && fabsf(hi - 60.0f) < 1e-3, hi);
        servo_logic_joint_limits(SERVO_JOINT_KNEE, &lo, &hi);
        chkf("膝限位 == [21.3,65.4]",
             fabsf(lo - SPIDER_BETA_MIN_DEG) < 1e-3
             && fabsf(hi - SPIDER_BETA_MAX_DEG) < 1e-3, hi);
    }

    /* ---- 软限位钳制 ---- */
    {
        float out;
        chk("髋 999° 钳到 +60",
            servo_logic_clamp_deg(SERVO_JOINT_HIP, 999.0f, &out) && out == 60.0f, "");
        chk("髋 -999° 钳到 -60",
            servo_logic_clamp_deg(SERVO_JOINT_HIP, -999.0f, &out) && out == -60.0f, "");
        chk("膝 0° 钳到 21.3",
            servo_logic_clamp_deg(SERVO_JOINT_KNEE, 0.0f, &out)
            && fabsf(out - SPIDER_BETA_MIN_DEG) < 1e-3, "");
        chk("膝 90° 钳到 65.4",
            servo_logic_clamp_deg(SERVO_JOINT_KNEE, 90.0f, &out)
            && fabsf(out - SPIDER_BETA_MAX_DEG) < 1e-3, "");
        chk("NaN 被拒绝", !servo_logic_clamp_deg(SERVO_JOINT_HIP, NAN, &out), "");
    }

    /* ---- 角度 -> 脉宽：端点锚定 ---- */
    {
        uint16_t p;
        servo_logic_deg_to_pulse(&cal, SERVO_JOINT_HIP, -60.0f, &p);
        chkf("髋 -60° -> min_us", p == cal.min_us, p);
        servo_logic_deg_to_pulse(&cal, SERVO_JOINT_HIP, 60.0f, &p);
        chkf("髋 +60° -> max_us", p == cal.max_us, p);
        servo_logic_deg_to_pulse(&cal, SERVO_JOINT_HIP, 0.0f, &p);
        chkf("髋 0° -> 两端均值", p == (uint16_t)((cal.min_us + cal.max_us) / 2), p);

        /* 巡航 beta 不在中位（膝角区间 21.3~65.4，中点在 43.35） */
        float beta_mid = (SPIDER_BETA_MIN_DEG + SPIDER_BETA_MAX_DEG) / 2.0f;
        servo_logic_deg_to_pulse(&cal, SERVO_JOINT_KNEE, beta_mid, &p);
        chkf("膝区间中点 -> 两端均值", p == (uint16_t)((cal.min_us + cal.max_us) / 2), p);
    }

    /* ---- 越界角度不产生越界脉宽（端死保护） ---- */
    {
        uint16_t p;
        servo_logic_deg_to_pulse(&cal, SERVO_JOINT_HIP, 1000.0f, &p);
        chk("髋 1000° 脉宽仍 <= max", p <= cal.max_us, "");
        servo_logic_deg_to_pulse(&cal, SERVO_JOINT_KNEE, -1000.0f, &p);
        chk("膝 -1000° 脉宽仍 >= min", p >= cal.min_us, "");
    }

    /* ---- reverse 对称翻转 ---- */
    {
        servo_cal_t rev = cal;
        rev.reverse = true;
        uint16_t pn, pr;
        servo_logic_deg_to_pulse(&cal, SERVO_JOINT_HIP, 30.0f, &pn);
        servo_logic_deg_to_pulse(&rev, SERVO_JOINT_HIP, 30.0f, &pr);
        chkf("reverse: +30° 镜像", (int)pn + (int)pr == (int)cal.min_us + (int)cal.max_us,
             (float)pr);
    }

    /* ---- trim 叠加 ---- */
    {
        servo_cal_t tr = cal;
        uint16_t p0, p1;
        servo_logic_deg_to_pulse(&cal, SERVO_JOINT_HIP, 0.0f, &p0);
        tr.trim_us = 20;
        servo_logic_deg_to_pulse(&tr, SERVO_JOINT_HIP, 0.0f, &p1);
        chkf("trim +20us 生效", (int)p1 == (int)p0 + 20, (float)p1);

        /* trim 过大时须被端点硬限幅截住，不能顶死 */
        tr.trim_us = 900;
        servo_logic_deg_to_pulse(&tr, SERVO_JOINT_HIP, 0.0f, &p1);
        chk("大 trim 被端点截断", p1 == cal.max_us, "");
    }

    /* ---- 单调性：脉宽随角度单调递增 ---- */
    {
        uint16_t prev = 0;
        int first = 1, mono = 1;
        for (float d = -60.0f; d <= 60.0f; d += 1.0f) {
            uint16_t p;
            servo_logic_deg_to_pulse(&cal, SERVO_JOINT_HIP, d, &p);
            if (!first && p < prev) mono = 0;
            prev = p; first = 0;
        }
        chk("髋脉宽单调递增", mono, "");
    }

    /* ---- 反解往返 ---- */
    {
        float deg;
        uint16_t p;
        int ok = 1;
        for (float d = -60.0f; d <= 60.0f; d += 5.0f) {
            servo_logic_deg_to_pulse(&cal, SERVO_JOINT_HIP, d, &p);
            servo_logic_pulse_to_deg(&cal, SERVO_JOINT_HIP, p, &deg);
            if (fabsf(deg - d) > 0.1f) ok = 0;
        }
        chk("髋 deg<->pulse 往返误差 <0.1°", ok, "");
    }

    /* ---- 角速度斜坡 ---- */
    {
        float out;
        /* 180°/s × 2ms = 0.36°/tick */
        float step = SPIDER_MAX_RATE_DEG_S * 2.0f / 1000.0f;
        servo_logic_slew(0.0f, 10.0f, step, &out);
        chkf("单 tick 位移 == 0.36°", fabsf(out - step) < 1e-4, out);
        servo_logic_slew(out, 10.0f, step, &out);
        chkf("未到目标继续推进", fabsf(out - 2 * step) < 1e-4, out);
        chk("到位返回 true", servo_logic_slew(9.99f, 10.0f, step, &out)
            && out == 10.0f, "");
        servo_logic_slew(0.0f, -10.0f, step, &out);
        chkf("负向推进", fabsf(out + step) < 1e-4, out);
    }

    /* ---- 巡航姿态的脉宽合理性（996R 工作区间 1000~2000us 内） ---- */
    {
        uint16_t p;
        servo_logic_deg_to_pulse(&cal, SERVO_JOINT_HIP, 0.0f, &p);
        chk("髋巡航 0° 落在 996R 区间", p >= 1000 && p <= 2000, "");
        servo_logic_deg_to_pulse(&cal, SERVO_JOINT_KNEE, SPIDER_BETA_CRUISE_DEG, &p);
        chk("膝巡航 beta 落在 996R 区间", p >= 1000 && p <= 2000, "");
        printf("       巡航脉宽: 髋0°=%uus 膝%.1f°=%uus\n",
               1500, SPIDER_BETA_CRUISE_DEG, p);
    }

    printf("\n%d/%d passed, %d failed\n", g_total - g_fail, g_total, g_fail);
    return g_fail ? 1 : 0;
}