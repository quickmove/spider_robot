#include "spider_pca9685.h"

#include "spider_pca9685_math.h"

#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char *TAG = "pca9685";

/* 写入：起始寄存器 + 数据 */
static esp_err_t write_reg(const pca9685_t *dev, uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[8];
    if (len + 1 > sizeof(buf)) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(dev->dev, buf, len + 1, 100);
}

static esp_err_t write_reg8(const pca9685_t *dev, uint8_t reg, uint8_t val)
{
    return write_reg(dev, reg, &val, 1);
}

static esp_err_t read_reg8(const pca9685_t *dev, uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(dev->dev, &reg, 1, val, 1, 100);
}

/* 通用呼叫软复位后需等待振荡器稳定 */
#define SWRESET_SETTLE_MS  2

esp_err_t pca9685_reset(const pca9685_t *dev)
{
    uint8_t swreset = 0x06;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev->dev, &swreset, 1, 100),
                        "spider", "SWRESET 失败");
    vTaskDelay(pdMS_TO_TICKS(SWRESET_SETTLE_MS));
    return ESP_OK;
}

esp_err_t pca9685_probe(const pca9685_t *dev, uint8_t *mode1_out)
{
    uint8_t v = 0;
    ESP_RETURN_ON_ERROR(read_reg8(dev, PCA9685_REG_MODE1, &v), "spider", "读 MODE1 失败");
    if (mode1_out) {
        *mode1_out = v;
    }
    return ESP_OK;
}

esp_err_t pca9685_set_freq(pca9685_t *dev, uint16_t freq_hz)
{
    uint8_t presc = 0;
    if (!pca9685_math_prescale(freq_hz, &presc)) {
        ESP_LOGE(TAG, "频率 %uHz 超出可设范围 [%u,%u]", freq_hz,
                 PCA9685_MIN_FREQ_HZ, PCA9685_MAX_FREQ_HZ);
        return ESP_ERR_INVALID_ARG;
    }

    /* 改分频必须先睡眠，写完再重启 */
    ESP_RETURN_ON_ERROR(write_reg8(dev, PCA9685_REG_MODE1, presc),
                        "spider", "写分频失败");

    vTaskDelay(pdMS_TO_TICKS(1));
    ESP_RETURN_ON_ERROR(write_reg8(dev, PCA9685_REG_MODE1,
                                   PCA9685_MODE1_RESTART | presc),
                        "spider", "重启振荡器失败");

    dev->freq_hz = freq_hz;
    dev->period_counts = 4096;

    ESP_LOGI(TAG, "PWM 频率 %uHz（prescale=%u，实测等效 %uHz，周期 %luµs，%u counts）",
             freq_hz, presc, pca9685_math_actual_freq(presc),
             (unsigned long)pca9685_math_period_us(freq_hz), dev->period_counts);
    return ESP_OK;
}

uint16_t pca9685_get_freq(const pca9685_t *dev)
{
    return dev->freq_hz;
}

esp_err_t pca9685_init(pca9685_t *dev, int sda_gpio, int scl_gpio,
                       uint32_t i2c_freq_hz, uint8_t addr, uint16_t pwm_freq_hz,
                       int oe_gpio)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * OE 先置高（关断输出）再做一切 I2C 操作。
     * 上电与初始化期间舵机必须是瘫软态，否则 I2C 配置过程中就会按复位默认值
     * 输出脉冲。用外部 10kΩ 上拉时这一步是冗余的（复位即高），但显式拉高
     * 可覆盖「GPIO 尚未配置、悬空」的情形。
     */
    dev->oe_gpio = oe_gpio;
    pca9685_oe_set(dev, false);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &dev->bus), "spider", "创建 I2C 总线失败");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = i2c_freq_hz,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(dev->bus, &dev_cfg, &dev->dev),
                        "spider", "注册 I2C 设备失败");

    uint8_t mode1 = 0;
    ESP_RETURN_ON_ERROR(pca9685_probe(dev, &mode1), "spider",
                        "PCA9685 无响应 (addr 0x%02x)：检查接线与上拉", addr);

    ESP_RETURN_ON_ERROR(pca9685_reset(dev), "spider", "软复位失败");

    /* MODE2：Inverted 输出 + 推挽输出（舵机需要推挽以支持反向驱动） */
    ESP_RETURN_ON_ERROR(write_reg8(dev, PCA9685_REG_MODE2,
                                   PCA9685_MODE2_OUTDRV | PCA9685_MODE2_INV),
                        "spider", "写 MODE2 失败");

    ESP_RETURN_ON_ERROR(pca9685_all_channels_off(dev), "spider", "关闭全部通道失败");

    ESP_RETURN_ON_ERROR(pca9685_set_freq(dev, pwm_freq_hz), "spider", "设置频率失败");

    /* 退出睡眠：清 SLEEP 位同时重启振荡器 */
    ESP_RETURN_ON_ERROR(write_reg8(dev, PCA9685_REG_MODE1, PCA9685_MODE1_RESTART),
                        "spider", "退出睡眠失败");

    /* 全部配置就绪后才使能输出：此前舵机一直处于硬件关断态 */
    pca9685_oe_set(dev, true);

    ESP_LOGI(TAG, "就绪 addr=0x%02x SDA=%d SCL=%d I2C=%uHz OE=%d MODE1=0x%02x",
             addr, sda_gpio, scl_gpio, (unsigned)i2c_freq_hz, oe_gpio, mode1);
    return ESP_OK;
}

void pca9685_oe_set(const pca9685_t *dev, bool enable)
{
    if (dev == NULL || dev->oe_gpio < 0) {
        return;
    }
    /* OE 低有效：高 = 关断全部 16 路输出 */
    gpio_set_level(dev->oe_gpio, enable ? 0 : 1);
}

esp_err_t pca9685_set_counts(const pca9685_t *dev, uint8_t ch, uint16_t counts)
{
    if (ch >= PCA9685_CH_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (counts > PCA9685_PWM_RES_MAX) {
        counts = PCA9685_PWM_RES_MAX;
    }

    /* LEDn 四个寄存器连续写入：LEDnL, LEDnH, LEDnL, LEDnH */
    uint8_t reg = PCA9685_REG_LED0_LED15 + (ch * 4);
    uint8_t lo = (uint8_t)(counts & 0xFF);
    uint8_t hi = (uint8_t)(counts >> 8);

    uint8_t buf[5];
    buf[0] = reg;
    buf[1] = lo;
    buf[2] = hi & 0x0F;
    buf[3] = lo;
    buf[4] = hi & 0x0F;

    return i2c_master_transmit(dev->dev, buf, sizeof(buf), 100);
}

esp_err_t pca9685_set_pulse_us(const pca9685_t *dev, uint8_t ch, uint16_t pulse_us)
{
    if (dev->freq_hz == 0) {
        return ESP_ERR_INVALID_STATE;   /* 尚未成功设置过频率 */
    }
    uint16_t counts = pca9685_math_counts_from_pulse(pulse_us, dev->freq_hz);
    return pca9685_set_counts(dev, ch, counts);
}

esp_err_t pca9685_channel_off(const pca9685_t *dev, uint8_t ch)
{
    if (ch >= PCA9685_CH_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    /* LEDnH 的 full-off 位（bit2）：与占空 0 不同，前者强制输出 low */
    uint8_t reg = PCA9685_REG_LED0_LED15 + (ch * 4);
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = 0x00;
    buf[2] = PCA9685_LED_H_FULL_OFF;
    return i2c_master_transmit(dev->dev, buf, 3, 100);
}

esp_err_t pca9685_all_channels_off(const pca9685_t *dev)
{
    for (uint8_t ch = 0; ch < PCA9685_CH_COUNT; ch++) {
        ESP_RETURN_ON_ERROR(pca9685_channel_off(dev, ch), "spider", "关闭通道 %u 失败", ch);
    }
    return ESP_OK;
}

esp_err_t pca9685_sleep(const pca9685_t *dev, bool on)
{
    if (on) {
        /* SLEEP 位会自动清除 RESTART，写完无需额外延时 */
        return write_reg8(dev, PCA9685_REG_MODE1, PCA9685_MODE1_SLEEP);
    }
    return write_reg8(dev, PCA9685_REG_MODE1, PCA9685_MODE1_RESTART);
}