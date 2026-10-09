#include "spider_servo.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "spider_servo_logic.h"

#include "esp_check.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "servo";

#define NVS_NS        "spider"
#define NVS_KEY_CAL   "cal"
#define NVS_KEY_GEOM  "geom"

static const pca9685_t *s_dev;
static servo_ch_t       s_ch[SERVO_COUNT];

/*
 * 未落盘标定的通道位掩码。
 *
 * 放本组件而非 CLI：改 RAM 标定的入口有四条（CLI 的 cal set / cal reset，
 * Web 的 REST 与 WS 两处），dirty 逻辑若散在调用侧必然漏掉某条路径。
 * 收敛到 servo_cal_set() 这一个写入点，才能保证「改了就是 dirty」。
 *
 * cal save / cal load 一次覆盖全部 8 路（blob 整体读写），故均清零。
 */
static uint8_t s_cal_dirty;
static servo_mode_t     s_mode = SERVO_MODE_NORMAL;

uint8_t servo_ch_to_pca_channel(uint8_t leg, servo_joint_t joint)
{
    uint8_t ch = (uint8_t)(leg * 2 + (uint8_t)joint);
    return (ch < SERVO_COUNT) ? ch : 0;
}

/*
 * 角度 -> 脉宽。
 * 以标定端点为锚：逻辑角区间 [lo,hi] 线性映射到 [min_us, max_us]，
 * 这样标定出的端点既是机械极限也是软限位的物理落点，
 * 不存在"限住了但舵机还在顶死"的错配。
 *
 *   pulse = min + (deg - lo)/(hi - lo) * (max - min) + trim
 *
 * reverse 时交换 min/max 再取反（相对中位对称翻转）。
 */
esp_err_t servo_deg_to_pulse(uint8_t ch, float deg, uint16_t *pulse_us)
{
    if (ch >= SERVO_COUNT || pulse_us == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    servo_logic_deg_to_pulse(&s_ch[ch].cal, (servo_joint_t)(ch % 2),
                             deg, pulse_us);
    return ESP_OK;
}

esp_err_t servo_pulse_raw(uint8_t ch, uint16_t pulse_us)
{
    if (ch >= SERVO_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    /* 标定直写同样不许越过端点，防止误操作顶死舵机 */
    pulse_us = servo_logic_clamp_pulse(&s_ch[ch].cal, pulse_us);
    return pca9685_set_pulse_us(s_dev, ch, pulse_us);
}

esp_err_t servo_set_target(uint8_t leg, servo_joint_t joint, float deg)
{
    if (leg >= SERVO_LEGS || s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ch = servo_ch_to_pca_channel(leg, joint);

    float clamped;
    bool clipped = servo_logic_clamp_deg(joint, deg, &clamped);
    if (clipped) {
        ESP_LOGW(TAG, "ch%u 目标 %.1f° 越限，钳到 %.1f°", ch, deg, clamped);
    }
    if (isnan(deg)) {
        clamped = s_ch[ch].target_deg;   /* NaN 保护：保持上一目标 */
    }

    s_ch[ch].target_deg = clamped;
    return ESP_OK;
}

esp_err_t servo_set_all_targets(const float *angles_deg, size_t count)
{
    if (angles_deg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (count > SERVO_COUNT) {
        count = SERVO_COUNT;
    }
    for (size_t i = 0; i < count; i++) {
        ESP_RETURN_ON_ERROR(servo_set_target((uint8_t)(i / 2),
                                            (servo_joint_t)(i % 2),
                                            angles_deg[i]),
                            "spider", "设置目标失败");
    }
    return ESP_OK;
}

float servo_get_target(uint8_t leg, servo_joint_t joint)
{
    if (leg >= SERVO_LEGS) {
        return 0.0f;
    }
    return s_ch[servo_ch_to_pca_channel(leg, joint)].target_deg;
}

float servo_get_current(uint8_t leg, servo_joint_t joint)
{
    if (leg >= SERVO_LEGS) {
        return 0.0f;
    }
    return s_ch[servo_ch_to_pca_channel(leg, joint)].current_deg;
}

esp_err_t servo_tick(uint32_t tick_ms)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_mode != SERVO_MODE_NORMAL) {
        return ESP_OK;   /* 标定模式由 servo_pulse_raw 直写 */
    }
#ifdef CONFIG_SPIDER_BENCH_MODE
    /*
     * 台架模式：PWM 完全交给 CLI 的 leg 命令。
     *
     * 正常模式下本函数是 PWM 的唯一写入口（main.c 从不调
     * servo_snap_to_target/servo_soft_start），所以在这里早退，
     * 就能保证「上电即静默」——否则 servo_init 设的巡航 beta 会在启动
     * 后立刻被写到 8 路，而出厂标定下那个值与实际机械零位无关。
     */
    return ESP_OK;
#endif

    /* 角速度上限：996R 带载约空载一半，SPIDER_MAX_RATE_DEG_S 为默认上限 */
    float max_step = SPIDER_MAX_RATE_DEG_S * ((float)tick_ms / 1000.0f);

    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        servo_ch_t *c = &s_ch[ch];
        servo_logic_slew(c->current_deg, c->target_deg, max_step, &c->current_deg);

        uint16_t pulse;
        if (servo_deg_to_pulse(ch, c->current_deg, &pulse) == ESP_OK) {
            ESP_RETURN_ON_ERROR(pca9685_set_pulse_us(s_dev, ch, pulse),
                                "spider", "写通道 %u 失败", ch);
        }
    }
    return ESP_OK;
}

esp_err_t servo_snap_to_target(void)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        s_ch[ch].current_deg = s_ch[ch].target_deg;
        uint16_t pulse;
        if (servo_deg_to_pulse(ch, s_ch[ch].current_deg, &pulse) == ESP_OK) {
            ESP_RETURN_ON_ERROR(pca9685_set_pulse_us(s_dev, ch, pulse),
                                "spider", "写通道 %u 失败", ch);
        }
    }
    return ESP_OK;
}

esp_err_t servo_soft_start(void)
{
    /* 目标已由调用方设为站立姿态；只把当前角对齐到目标以免突跳，
     * 但输出脉宽仍按角速度斜坡推——由 servo_tick 完成。
     * 这里把 current 设在中间值，保证是"走过去"而非"跳过去"。
     */
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        s_ch[ch].current_deg = s_ch[ch].target_deg * 0.0f;
    }
    return ESP_OK;
}

esp_err_t servo_all_off(void)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    /* 优先走 OE：一次 GPIO 写硬件关断全部 16 路，不经 I2C，I2C 挂死时仍有效 */
    servo_output_enable(false);
    /* 再把 LEDn 全写 0，覆盖未接 OE（oe_gpio < 0）的情形 */
    return pca9685_all_channels_off(s_dev);
}

void servo_output_enable(bool enable)
{
    if (s_dev == NULL) {
        return;
    }
    /*
     * 只翻 OE，不动 LEDn 寄存器：即使输出关断，PCA9685 仍保存最后一次目标，
     * 急停清除后恢复姿态时不会先跳到全关位置再起立。
     */
    pca9685_oe_set(s_dev, enable);
}

esp_err_t servo_set_mode(servo_mode_t mode)
{
    if (mode == s_mode) {
        return ESP_OK;
    }
    s_mode = mode;
    ESP_LOGW(TAG, "模式 -> %s", (mode == SERVO_MODE_CALIBRATE) ? "标定" : "正常");
    if (mode == SERVO_MODE_CALIBRATE) {
#ifdef CONFIG_SPIDER_BENCH_MODE
        /*
         * 台架模式：不做「8 路归中」。
         *
         * 归中会把全部 8 路写到 neutral_us，单腿测试时那 6 路没接舵机倒无害，
         * 但真正的问题是它与「上电全程静默」的预期矛盾——一旦敲了 cal enter
         * 就看到舵机跳一下，无法区分是机械在动还是固件在动。
         */
#else
        /* 进入标定前先全部落回中位，避免带着行走姿态进入标定模式 */
        for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
            uint16_t pulse = s_ch[ch].cal.neutral_us;
            ESP_RETURN_ON_ERROR(servo_pulse_raw(ch, pulse), "spider",
                                "标定模式归中失败 ch%u", ch);
            s_ch[ch].current_deg = s_ch[ch].target_deg = 0.0f;
        }
#endif
    }
    return ESP_OK;
}

servo_mode_t servo_get_mode(void)
{
    return s_mode;
}

esp_err_t servo_cal_get(uint8_t ch, servo_cal_t *out)
{
    if (ch >= SERVO_COUNT || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_ch[ch].cal;
    return ESP_OK;
}

esp_err_t servo_cal_set(uint8_t ch, const servo_cal_t *in)
{
    if (ch >= SERVO_COUNT || in == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!servo_logic_cal_valid(in)) {
        ESP_LOGE(TAG, "ch%u 标定非法: min=%u max=%u neu=%u（范围需 >=%uus，"
                      "中位需距两端各 >=%uus）",
                 ch, in->min_us, in->max_us, in->neutral_us,
                 SERVO_CAL_MIN_RANGE_US, SERVO_CAL_NEUTRAL_MARGIN_US);
        return ESP_ERR_INVALID_ARG;
    }
    s_ch[ch].cal = *in;
    s_ch[ch].valid = true;
    s_cal_dirty |= (uint8_t)(1u << ch);
    return ESP_OK;
}

bool servo_cal_is_dirty(uint8_t ch)
{
    if (ch >= SERVO_COUNT) {
        return false;
    }
    return (s_cal_dirty & (uint8_t)(1u << ch)) != 0;
}

uint8_t servo_cal_dirty_mask(void)
{
    return s_cal_dirty;
}

bool servo_cal_is_ready(void)
{
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        if (!s_ch[ch].valid) {
            return false;
        }
    }
    return true;
}

void servo_cal_reset_to_factory(void)
{
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        servo_logic_factory_cal(&s_ch[ch].cal);
        s_ch[ch].valid = false;
    }
    /* 全部 8 路都被改过，故整组标记未落盘 */
    s_cal_dirty = (uint8_t)((1u << SERVO_COUNT) - 1u);
}

esp_err_t servo_cal_save(void)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open(NVS_NS, NVS_READWRITE, &h), "spider", "打开 NVS 失败");
    esp_err_t err = nvs_set_blob(h, NVS_KEY_CAL, s_ch, sizeof(s_ch));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "标定已保存 (%u 路，%zu 字节)", SERVO_COUNT, sizeof(s_ch));
        /* blob 覆盖全部 8 路，故一次保存即整组干净 */
        s_cal_dirty = 0;
    }
    return err;
}

esp_err_t servo_cal_load(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "无 NVS 标定数据，使用出厂默认");
        return err;
    }

    size_t len = sizeof(s_ch);
    err = nvs_get_blob(h, NVS_KEY_CAL, s_ch, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof(s_ch)) {
        ESP_LOGW(TAG, "标定数据无效，使用出厂默认");
        servo_cal_reset_to_factory();
        return (err == ESP_OK) ? ESP_ERR_INVALID_SIZE : err;
    }

    /* 载入后做一次完整性校验：拒绝非法区间，避免坏数据导致顶死 */
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        servo_cal_t probe = s_ch[ch].cal;
        s_ch[ch].valid = false;
        if (servo_cal_set(ch, &probe) != ESP_OK) {
            ESP_LOGE(TAG, "ch%u 标定数据非法，已回退出厂", ch);
            servo_cal_reset_to_factory();
            return ESP_ERR_INVALID_ARG;
        }
        s_ch[ch].valid = true;
    }

    ESP_LOGI(TAG, "标定已载入 (%u 路)", SERVO_COUNT);
    /* 上面的校验循环经由 servo_cal_set()，会把 8 路全标为 dirty；
     * 但数据确实来自 NVS，故整组清干净 */
    s_cal_dirty = 0;
    return ESP_OK;
}

esp_err_t servo_init(const pca9685_t *dev)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_dev = dev;
    s_mode = SERVO_MODE_NORMAL;

    servo_cal_reset_to_factory();
    if (servo_cal_load() != ESP_OK) {
        ESP_LOGW(TAG, "使用出厂标定（未逐机标定，脉宽范围可能不准确）");
    }

    /*
     * 上电后的 RAM 状态即基线：从 NVS 读到就是已保存，读不到就是出厂默认。
     * 此时清掉 dirty——否则全新机器上 cal show 会把 8 路全标成「未落盘」，
     * 而用户其实什么都没改。dirty 的语义是「本次开机后被改过」。
     */
    s_cal_dirty = 0;

    /* 初始目标设为巡航姿态：髋回中立、膝为巡航 beta */
    for (uint8_t leg = 0; leg < SERVO_LEGS; leg++) {
        s_ch[leg * 2 + 0].target_deg = 0.0f;
        s_ch[leg * 2 + 0].current_deg = 0.0f;
        s_ch[leg * 2 + 1].target_deg = SPIDER_BETA_CRUISE_DEG;
        s_ch[leg * 2 + 1].current_deg = SPIDER_BETA_CRUISE_DEG;
    }

    ESP_LOGI(TAG, "就绪 模式=正常 标定=%s 心跳超时=%ums 角速度上限=%.0f°/s",
             servo_cal_is_ready() ? "完整" : "出厂默认",
             SPIDER_HEARTBEAT_TIMEOUT_MS, SPIDER_MAX_RATE_DEG_S);
    return ESP_OK;
}