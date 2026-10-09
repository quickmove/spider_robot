/*
 * PCA9685 算术主机端测试
 *
 * 重点验证：50Hz 预分频值、996R 脉宽区间的计数换算、边界与越界处理。
 */

#include <stdio.h>

#include "spider_pca9685_math.h"

static int g_fail, g_total;

static void chk(const char *name, int ok, long v)
{
    g_total++;
    if (!ok) g_fail++;
    printf("[%s] %-48s %6ld\n", ok ? "PASS" : "FAIL", name, v);
}

int main(void)
{
    uint8_t p = 0;

    /* ---- 50Hz（舵机标准）---- */
    g_total++;
    int ok = pca9685_math_prescale(50, &p);
    chk("50Hz prescale == 122 (数据手册标准值)", ok && p == 122, p);
    chk("50Hz 实际频率在 49~51", (int)pca9685_math_actual_freq(p) >= 49
        && (int)pca9685_math_actual_freq(p) <= 51,
        pca9685_math_actual_freq(p));
    chk("50Hz 周期 == 20000us", pca9685_math_period_us(50) == 20000,
        (long)pca9685_math_period_us(50));

    /* ---- 舵机脉宽 -> 计数 ---- */
    /* 50Hz: counts = pulse_us * 4096 / 20000 = pulse_us * 0.2048 */
    chk("1000us -> 205 counts", pca9685_math_counts_from_pulse(1000, 50) == 205,
        pca9685_math_counts_from_pulse(1000, 50));
    chk("1500us -> 307 counts", pca9685_math_counts_from_pulse(1500, 50) == 307,
        pca9685_math_counts_from_pulse(1500, 50));
    chk("2000us -> 410 counts", pca9685_math_counts_from_pulse(2000, 50) == 410,
        pca9685_math_counts_from_pulse(2000, 50));

    /* 反解：计数 -> 脉宽，误差应 <1 LSB */
    ok = 1;
    for (uint16_t us = 1000; us <= 2000; us += 5) {
        uint16_t c = pca9685_math_counts_from_pulse(us, 50);
        uint16_t back = pca9685_math_pulse_from_counts(c, 50);
        int err = (int)back - (int)us;
        if (err < 0) err = -err;
        if (err > 3) ok = 0;
    }
    chk("1000~2000us 往返误差 <=3us", ok, 0);

    /* 1 个计数约 4.88us，故 5us 步进内误差应为 0~3 */
    chk("12bit 分辨率 ~4.88us/LSB", pca9685_math_pulse_from_counts(1, 50) == 5,
        pca9685_math_pulse_from_counts(1, 50));

    /* ---- 边界 ---- */
    chk("pulse 0 -> 0 counts", pca9685_math_counts_from_pulse(0, 50) == 0,
        pca9685_math_counts_from_pulse(0, 50));
    chk("pulse 30000us 收敛到 4095",
        pca9685_math_counts_from_pulse(30000, 50) == 4095,
        pca9685_math_counts_from_pulse(30000, 50));
    chk("freq 0 时 counts 为 0", pca9685_math_counts_from_pulse(1500, 0) == 0,
        pca9685_math_counts_from_pulse(1500, 0));

    /* ---- 频率越界 ---- */
    chk("freq 0 被拒", !pca9685_math_prescale(0, &p), 0);
    chk("freq 2000 被拒", !pca9685_math_prescale(2000, &p), 0);
    chk("freq 1000 允许", pca9685_math_prescale(1000, &p), p);

    /* prescale 为 8bit，f_min = 25e6/(4096*256) ≈ 23.84Hz */
    chk("freq 23 被拒(低于硬件下限)", !pca9685_math_prescale(23, &p), 0);
    chk("freq 24 被接受", pca9685_math_prescale(24, &p), p);
    chk("prescale 0 视为非法(停振)", pca9685_math_actual_freq(0) == 0, 0);
    chk("50Hz 在硬件范围内", pca9685_math_prescale(50, &p), p);

    printf("\n%d/%d passed, %d failed\n", g_total - g_fail, g_total, g_fail);
    return g_fail ? 1 : 0;
}