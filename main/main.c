/*
 * 机器蜘蛛 M0 骨架
 *
 * M0 目标：验证 ESP-IDF 工具链可用、组件结构正确、NVS 可读写。
 * M1 起接入 pca9685 / servo / motion 等组件。
 */

#include <stdio.h>

#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "spider_cli.h"
#include "spider_gait.h"
#include "spider_keyframe.h"
#include "spider_kinematics.h"
#include "spider_motion.h"
#include "spider_params.h"
#include "spider_posture.h"
#include "spider_pca9685.h"
#include "spider_servo.h"
#include "spider_web.h"

static const char *TAG = "spider";

static const char *leg_name[SPIDER_LEG_COUNT] = {"FL", "FR", "RL", "RR"};

static pca9685_t s_pca;

/*
 * 打印由 params.yaml 派生的关键量。
 * 与主机端 tools/run_tests.sh 的断言同源，用于实机确认参数表已生效。
 */
static void spider_print_geometry(void)
{
    ESP_LOGI(TAG, "几何: L1=%.1f L2=%.1f alpha=%.1f beta_cruise=%.1f",
             SPIDER_L1_MM, SPIDER_L2_MM, SPIDER_ALPHA_DEG, SPIDER_BETA_CRUISE_DEG);
    ESP_LOGI(TAG, "巡航: H=%.1fmm R=%.1fmm 耦合=1:%.2f",
             kin_height_from_beta(SPIDER_BETA_CRUISE_DEG),
             kin_radius_from_beta(SPIDER_BETA_CRUISE_DEG),
             kin_height_coupling(SPIDER_BETA_CRUISE_DEG));
    ESP_LOGI(TAG, "高度范围: [%.1f, %.1f]mm -> beta [%.1f, %.1f]",
             kin_height_from_beta(SPIDER_BETA_MIN_DEG),
             kin_height_from_beta(SPIDER_BETA_MAX_DEG),
             SPIDER_BETA_MIN_DEG, SPIDER_BETA_MAX_DEG);
    ESP_LOGI(TAG, "步幅: crawl %.1fmm -> 髋半摆 ±%.1f° | trot %.1fmm -> ±%.1f°",
             SPIDER_CRAWL_STRIDE_MM,
             kin_hip_sweep_from_stride(SPIDER_CRAWL_STRIDE_MM,
                                      SPIDER_CRUISE_RADIUS_MM),
             SPIDER_TROT_STRIDE_MM,
             kin_hip_sweep_from_stride(SPIDER_TROT_STRIDE_MM,
                                      SPIDER_CRUISE_RADIUS_MM));
    ESP_LOGI(TAG, "巡航速度: crawl %.1fmm/s | trot %.1fmm/s",
             SPIDER_CRAWL_STRIDE_MM / SPIDER_CRAWL_PERIOD_S,
             SPIDER_TROT_STRIDE_MM / SPIDER_TROT_PERIOD_S);

    for (int i = 0; i < SPIDER_LEG_COUNT; i++) {
        const spider_leg_t *l = &SPIDER_LEGS[i];
        ESP_LOGI(TAG, "  %s: psi=%.0f° r=%.1fmm ch_hip=%u ch_knee=%u",
                 leg_name[i], l->psi_deg, l->r_mm, l->ch_hip, l->ch_knee);
    }
}

static void spider_banner(void)
{
    ESP_LOGI(TAG, "----------------------------------------");
    ESP_LOGI(TAG, " 机器蜘蛛 / Insect Spider");
    ESP_LOGI(TAG, " M0 skeleton");
    ESP_LOGI(TAG, " ESP-IDF    : %s", esp_get_idf_version());
    ESP_LOGI(TAG, " target     : %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, " CPU freq   : %d MHz", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    ESP_LOGI(TAG, " FreeRTOS   : %d Hz tick", CONFIG_FREERTOS_HZ);
    ESP_LOGI(TAG, " I2C pins   : SDA=%d SCL=%d",
             CONFIG_SPIDER_I2C_SDA_GPIO, CONFIG_SPIDER_I2C_SCL_GPIO);
    ESP_LOGI(TAG, "----------------------------------------");
}

esp_err_t spider_nvs_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS 分区需擦除，重试");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS 初始化失败: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "NVS 就绪");
    }
    return err;
}

static esp_err_t spider_hw_init(void)
{
    esp_err_t err = pca9685_init(&s_pca,
                                 CONFIG_SPIDER_I2C_SDA_GPIO,
                                 CONFIG_SPIDER_I2C_SCL_GPIO,
                                 CONFIG_SPIDER_I2C_FREQ_HZ,
                                 SPIDER_PCA9685_ADDR,
                                 CONFIG_SPIDER_PWM_FREQ_HZ,
                                 CONFIG_SPIDER_PCA9685_OE_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    err = servo_init(&s_pca);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "舵机层初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 校验脉宽换算：巡航姿态不得越过标定端点 */
    for (uint8_t ch = 0; ch < SPIDER_SERVO_COUNT; ch++) {
        float deg = servo_get_target(ch / 2, (servo_joint_t)(ch % 2));
        uint16_t pulse = 0;
        if (servo_deg_to_pulse(ch, deg, &pulse) == ESP_OK) {
            ESP_LOGI(TAG, "  ch%u %s.%s 目标 %.1f° -> %uus",
                     ch, leg_name[ch / 2],
                     (ch % 2) ? "knee" : "hip", deg, pulse);
        }
    }
    return ESP_OK;
}

void app_main(void)
{
    spider_banner();
    spider_print_geometry();

    if (spider_nvs_init() != ESP_OK) {
        ESP_LOGE(TAG, "NVS 不可用，标定与参数将无法持久化");
        return;
    }

    bool spider_hw_ok = (spider_hw_init() == ESP_OK);
    if (!spider_hw_ok) {
        ESP_LOGE(TAG, "硬件未就绪，跳过 motion 与 Web（检查 I2C 接线与上拉）");
        spider_cli_start();   /* 至少保留串口诊断手段 */
        return;
    }

    /* motion 是舵机写入的唯一所有者；servo_tick 由其任务内调用 */
    if (motion_init() != ESP_OK) {
        ESP_LOGE(TAG, "motion 初始化失败");
        return;
    }
    spider_cli_start();

    /*
     * M3：WiFi AP + httpd + WebSocket + 控制页。
     *
     * 硬件（PCA9685 / 舵机）缺失时不启动 Web——否则用户会连上热点、
     * 看到控制页，却没有任何机器人可控制，白白困惑。
     */
    if (spider_hw_ok) {
        if (web_init() != ESP_OK) {
            ESP_LOGE(TAG, "Web 启动失败，串口 CLI 仍可用");
        }
    } else {
        ESP_LOGW(TAG, "硬件未就绪，跳过 Web 启动（无机器人可控）");
    }

    ESP_LOGI(TAG, "M3 就绪：WiFi AP + WebSocket 控制页");
    if (!servo_cal_is_ready()) {
        ESP_LOGW(TAG, "尚未标定 — 架空后执行: cal enter / cal scan / cal save");
        ESP_LOGW(TAG, "标定未完成前不要通电行走，否则舵机可能顶死");
    }
}