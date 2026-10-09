/*
 * PCA9685 16 路 PWM 驱动（寄存器级，ESP-IDF 5.x 新版 I2C master API）
 *
 * 依据 docs/hardware.md §5：
 *   - I2C 400kHz，地址 0x60（可配）
 *   - PWM 50Hz（20ms 周期），12bit（4096 级）分辨率
 *   - 8 路舵机 + 8 路预留
 *
 * 舵机脉宽 1000~2000µs 在 50Hz 下对应 204.8~409.6 个计数，
 * 故 12bit 分辨率对舵机绰绰有余（1 计数 ≈ 4.88µs）。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9685_CH_COUNT     16

/* 寄存器 */
#define PCA9685_REG_MODE1        0x00
#define PCA9685_REG_MODE2        0x01
#define PCA9685_REG_SUBADR1      0x08
#define PCA9685_REG_SUBADR2      0x09
#define PCA9685_REG_SUBADR3      0x0A
#define PCA9685_REG_ALLCALLADR   0x0B
#define PCA9685_REG_LED0_LED15   0x10   /* LEDn = 0x10 + n*4 */
#define PCA9685_REG_CH0          0x40

/* MODE1 位 */
#define PCA9685_MODE1_RESTART    0x80
#define PCA9685_MODE1_EXTCLK     0x40
#define PCA9685_MODE1_AI         0x20
#define PCA9685_MODE1_SLEEP      0x10
#define PCA9685_MODE1_ALLCALL    0x01   /* All Call 地址应答控制（不是输出使能） */

/*
 * PCA9685 没有「寄存器级全局输出禁用」位。
 * 全局关断只有两条路径：物理 OE 引脚（低有效），或把 LEDn 全写 0（full off）。
 * 输出使能由 pca9685_oe_set() 驱动 OE 引脚完成，见下。
 */

/* LEDnH 位 */
#define PCA9685_LED_H_FULL_OFF   0x04

/* MODE2 位 */
#define PCA9685_MODE2_OCH        0x08
#define PCA9685_MODE2_OUTDRV     0x04   /* 1 = 推挽输出（舵机需要） */
#define PCA9685_MODE2_INV        0x01   /* 1 = 反相输出 */

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint16_t freq_hz;        /* 当前 PWM 频率（0 = 未设置） */
    uint16_t period_counts;  /* 恒为 4096 */
    int      oe_gpio;        /* OE 引脚 GPIO；< 0 表示未接 OE，输出永久使能 */
} pca9685_t;

/*
 * 初始化 I2C 总线与 PCA9685。
 * 内部完成：MODE1 复位、MODE2 推挽、输出使能、分频设置、振荡器启动。
 * freq_hz 建议 50（舵机）；范围 1~1000。
 *
 * oe_gpio 为 OE 引脚编号，建议外接 10kΩ 上拉至 VDD：
 *   - 上电/复位期间 OE 为高 → 输出硬件关断，不依赖固件是否在跑
 *   - 本函数在初始化末尾把 OE 拉低使能输出
 * 传 -1 表示不接 OE（输出永久使能，无硬件急停能力）。
 */
esp_err_t pca9685_init(pca9685_t *dev, int sda_gpio, int scl_gpio,
                       uint32_t i2c_freq_hz, uint8_t addr, uint16_t pwm_freq_hz,
                       int oe_gpio);

/*
 * 硬件级全局输出使能/关断（OE 引脚，低有效）。
 *
 * 急停路径优先走这里：一次 GPIO 写，不经 I2C，不受总线状态影响，
 * 因此即使 I2C 挂死也能可靠关断全部 16 路。
 */
void pca9685_oe_set(const pca9685_t *dev, bool enable);

/* 软复位（I2C 通用呼叫 SWRESET），需要 500µs 恢复时间 */
esp_err_t pca9685_reset(const pca9685_t *dev);

/* 探测器件：读 MODE1，失败通常意味着接线/上拉问题 */
esp_err_t pca9685_probe(const pca9685_t *dev, uint8_t *mode1_out);

/* 设置 PWM 频率（会重置 MODE1 的 SLEEP 位并重启振荡器） */
esp_err_t pca9685_set_freq(pca9685_t *dev, uint16_t freq_hz);
uint16_t pca9685_get_freq(const pca9685_t *dev);

/* 设置通道占空计数（0 = 全关，>0 为 on 段计数） */
esp_err_t pca9685_set_counts(const pca9685_t *dev, uint8_t ch, uint16_t counts);

/* 设置通道脉宽（µs）。脉宽须落在 (0, 1/freq] 内，越界自动收敛 */
esp_err_t pca9685_set_pulse_us(const pca9685_t *dev, uint8_t ch, uint16_t pulse_us);

/* 通道全关（全灭；full-off 由寄存器 bit 实现，非占空 0） */
esp_err_t pca9685_channel_off(const pca9685_t *dev, uint8_t ch);
esp_err_t pca9685_all_channels_off(const pca9685_t *dev);

/* 进入/退出睡眠（睡眠时 PWM 停止输出，舵机失去保持力） */
esp_err_t pca9685_sleep(const pca9685_t *dev, bool on);

#ifdef __cplusplus
}
#endif