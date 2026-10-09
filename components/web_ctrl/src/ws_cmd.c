/*
 * WebSocket 命令解析：JSON -> motion 命令
 *
 * 协议见 docs/protocol.md §3。本文件只做「解析 + 转发」，
 * 不含任何运动决策——与 CLI 完全平级，motion 层是唯一的决策点。
 */

#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "spider_kf_parse.h"
#include "spider_keyframe.h"
#include "spider_motion.h"
#include "spider_params.h"
#include "spider_posture.h"
#include "spider_servo.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "spider_web.h"
#include "web_priv.h"

static const char *TAG = "webws";

static void ack(int id, bool ok, const char *err)
{
    if (id < 0) {
        return;                 /* 无 id 的命令不回复，避免刷屏 */
    }
    char buf[96];
    int n = snprintf(buf, sizeof(buf), "{\"t\":\"ack\",\"id\":%d,\"ok\":%s",
                     id, ok ? "true" : "false");
    if (!ok && err) {
        snprintf(buf + n, sizeof(buf) - (size_t)n, ",\"error\":\"%s\"}", err);
    } else {
        snprintf(buf + n, sizeof(buf) - (size_t)n, "}");
    }
    web_send_text(buf, strlen(buf));
}

/* IDF 的 cJSON 无 cJSON_GetIntValue/GetDoubleValue，直接取字段 */
static float jnum(const cJSON *o, const char *key, float dflt)
{
    const cJSON *v = cJSON_GetObjectItem(o, key);
    return (v == NULL || !cJSON_IsNumber(v)) ? dflt : (float)v->valuedouble;
}

static int jint(const cJSON *o, const char *key, int dflt)
{
    const cJSON *v = cJSON_GetObjectItem(o, key);
    return (v == NULL || !cJSON_IsNumber(v)) ? dflt : v->valueint;
}

static const char *jstr(const cJSON *o, const char *key)
{
    const cJSON *v = cJSON_GetObjectItem(o, key);
    return (v && cJSON_IsString(v)) ? cJSON_GetStringValue(v) : NULL;
}

static float clampf_local(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* 标定模式与运动互斥：除退出标定外一律拒绝并说明原因 */
static bool reject_if_calibrated(const char *cmd)
{
    if (servo_get_mode() == SERVO_MODE_CALIBRATE) {
        ESP_LOGW(TAG, "标定模式中，拒绝 %s（先 cal_exit）", cmd);
        return true;
    }
    return false;
}

esp_err_t web_ws_handle_cmd(const char *json, size_t len)
{
    cJSON *root = cJSON_ParseWithLength(json, len);
    if (root == NULL) {
        ESP_LOGW(TAG, "JSON 解析失败（%u 字节）", (unsigned)len);
        return ESP_OK;      /* 不断连接，只忽略该帧 */
    }

    const char *t = jstr(root, "t");
    int id = jint(root, "id", -1);
    esp_err_t rc = ESP_OK;
    const char *err = NULL;

    if (t == NULL) {
        err = "missing_t";
        goto done;
    }

    /* ---- 安全类：任何时候都放行 ---- */
    if (!strcmp(t, "estop")) {
        motion_estop();
        ack(id, true, NULL);
        goto done;
    }
    if (!strcmp(t, "estop_clear")) {
        rc = motion_estop_clear();
        ack(id, rc == ESP_OK, rc == ESP_OK ? NULL : "denied");
        goto done;
    }
    if (!strcmp(t, "ping")) {
        ack(id, true, NULL);
        goto done;
    }
    if (!strcmp(t, "reboot")) {
        ESP_LOGW(TAG, "收到 reboot 指令");
        ack(id, true, NULL);
        web_send_text("{\"t\":\"ack\",\"ok\":true}", 23);
        vTaskDelay(pdMS_TO_TICKS(120));   /* 让 ack 先出去 */
        esp_restart();
        goto done;
    }

    /* ---- 标定 ---- */
    if (!strcmp(t, "cal_enter")) {
        motion_cmd_t c = {.type = CMD_CALIBRATE_MODE};
        c.params.i = 1;
        rc = motion_enqueue(&c);
        ack(id, rc == ESP_OK, rc == ESP_OK ? NULL : "mode_denied");
        goto done;
    }
    if (!strcmp(t, "cal_exit")) {
        motion_cmd_t c = {.type = CMD_CALIBRATE_MODE};
        c.params.i = 0;
        rc = motion_enqueue(&c);
        motion_set_posture(POSTURE_STAND);
        ack(id, rc == ESP_OK, rc == ESP_OK ? NULL : "mode_denied");
        goto done;
    }
    if (!strcmp(t, "cal_scan")) {
        /* 扫描是串口侧的长时序动作，Web 侧提示走 CLI */
        ESP_LOGW(TAG, "cal scan 请用串口（耗时约 30s，会占用控制权）");
        err = "use_cli";
        ack(id, false, err);
        goto done;
    }
    if (!strcmp(t, "cal_save")) {
        rc = servo_cal_save();
        ack(id, rc == ESP_OK, rc == ESP_OK ? NULL : "save_failed");
        goto done;
    }
    if (!strcmp(t, "cal_load")) {
        rc = servo_cal_load();
        ack(id, rc == ESP_OK, rc == ESP_OK ? NULL : "load_failed");
        goto done;
    }
    if (!strcmp(t, "cal_set")) {
        int ch = jint(root, "ch", -1);
        servo_cal_t c = {
            .min_us = (uint16_t)jint(root, "min", SPIDER_SERVO_MIN_US),
            .max_us = (uint16_t)jint(root, "max", SPIDER_SERVO_MAX_US),
            .neutral_us = (uint16_t)jint(root, "neutral", SPIDER_SERVO_NEUTRAL_US),
            .trim_us = (int16_t)jint(root, "trim", 0),
            .reverse = (jint(root, "reverse", 0) != 0),
        };
        if (ch < 0 || ch >= (int)SPIDER_SERVO_COUNT) {
            err = "out_of_range";
        } else {
            rc = servo_cal_set((uint8_t)ch, &c);
            if (rc != ESP_OK) {
                err = "bad_cal";
            }
        }
        ack(id, rc == ESP_OK, err);
        goto done;
    }
    /* 标定模式下直写脉宽，用于滑条实时预览 */
    if (!strcmp(t, "cal_pulse")) {
        int ch = jint(root, "ch", -1);
        int us = jint(root, "us", -1);
        if (servo_get_mode() != SERVO_MODE_CALIBRATE) {
            err = "cal_not_active";
        } else if (ch < 0 || ch >= (int)SPIDER_SERVO_COUNT || us < 0) {
            err = "out_of_range";
        } else {
            rc = servo_pulse_raw((uint8_t)ch, (uint16_t)us);
            if (rc != ESP_OK) {
                err = "out_of_range";
            }
        }
        ack(id, rc == ESP_OK, err);
        goto done;
    }

    /* ---- 以下均需退出标定 ---- */
    if (reject_if_calibrated(t)) {
        ack(id, false, "cal_not_active");
        goto done;
    }

    if (!strcmp(t, "drive")) {
        float vx = clampf_local(jnum(root, "vx", 0.0f), -1.0f, 1.0f);
        float vy = clampf_local(jnum(root, "vy", 0.0f), -1.0f, 1.0f);
        float yaw = clampf_local(jnum(root, "yaw", 0.0f), -1.0f, 1.0f);
        rc = motion_drive(vx, vy, yaw);
        goto done;                       /* 高频命令不回复 */

    } else if (!strcmp(t, "gait")) {
        const char *g = jstr(root, "type");
        if (g == NULL) {
            err = "bad_field";
        } else if (!strcmp(g, "trot")) {
            rc = motion_set_gait(GAIT_TROT);
        } else if (!strcmp(g, "crawl")) {
            rc = motion_set_gait(GAIT_CRAWL);
        } else {
            err = "bad_field";
        }
        ack(id, rc == ESP_OK, err);

    } else if (!strcmp(t, "speed")) {
        rc = motion_set_speed(clampf_local(jnum(root, "pct", 0.0f), 0.0f,
                                           SPIDER_SPEED_MAX_PCT));
        ack(id, rc == ESP_OK, err);

    } else if (!strcmp(t, "stop")) {
        rc = motion_stop();
        ack(id, rc == ESP_OK, err);

    } else if (!strcmp(t, "pose")) {
        const char *name = jstr(root, "name");
        posture_id_t pid;
        if (name == NULL || !posture_from_name(name, &pid)) {
            err = "bad_field";
        } else {
            rc = motion_set_posture((int)pid);
        }
        ack(id, rc == ESP_OK, err);

    } else if (!strcmp(t, "height")) {
        float mm = clampf_local(jnum(root, "mm", SPIDER_STAND_HEIGHT_MM),
                                SPIDER_HEIGHT_MIN_MM, SPIDER_HEIGHT_MAX_MM);
        rc = motion_set_height(mm);
        ack(id, rc == ESP_OK, err);

    } else if (!strcmp(t, "nudge")) {
        float dx = clampf_local(jnum(root, "dx", 0.0f),
                                -SPIDER_NUDGE_MAX_MM, SPIDER_NUDGE_MAX_MM);
        float dy = clampf_local(jnum(root, "dy", 0.0f),
                                -SPIDER_NUDGE_MAX_MM, SPIDER_NUDGE_MAX_MM);
        rc = motion_nudge(dx, dy);
        ack(id, rc == ESP_OK, err);

    } else if (!strcmp(t, "lean")) {
        float roll = clampf_local(jnum(root, "roll", 0.0f),
                                  -SPIDER_LEAN_MAX_DEG, SPIDER_LEAN_MAX_DEG);
        float pitch = clampf_local(jnum(root, "pitch", 0.0f),
                                   -SPIDER_LEAN_MAX_DEG, SPIDER_LEAN_MAX_DEG);
        rc = motion_lean(roll, pitch);
        ack(id, rc == ESP_OK, err);

    } else if (!strcmp(t, "strafe_macro")) {
        float dx = clampf_local(jnum(root, "dx", 0.0f),
                                -SPIDER_STRAFE_MAX_MM, SPIDER_STRAFE_MAX_MM);
        rc = motion_strafe(dx);
        ack(id, rc == ESP_OK, err);

    } else if (!strcmp(t, "play")) {
        const char *name = jstr(root, "name");
        if (name == NULL) {
            err = "bad_field";
        } else if (kf_find(name) == NULL) {
            /* 未上传的序列名给明确错误，而不是静默失败 */
            err = "no_such_seq";
        } else {
            rc = motion_play(name);
        }
        ack(id, rc == ESP_OK, err);

    } else if (!strcmp(t, "keyframe_upload")) {
        /* 解析器自带协议边界校验（名字/帧数/每帧 8 关节/时间单调/ease）
         * 与报文体积上限，此处只负责把错误码翻成协议 §8 的 error 字符串。 */
        kf_parse_err_t kerr = kf_parse_json(root, (size_t)len);
        if (kerr != KF_PARSE_OK) {
            err = kf_parse_err_str(kerr);
            ack(id, false, err);
        } else {
            ack(id, true, NULL);
        }

    } else if (!strcmp(t, "play_list")) {
        ack(id, true, NULL);
        ESP_LOGI(TAG, "已加载 %u 个序列", kf_count());

    } else if (!strcmp(t, "advanced")) {
        /* 解锁非物理红线；物理红线（转向限速/Trot 限速）不可解锁 */
        int on = jint(root, "on", 0);
        ESP_LOGW(TAG, "advanced=%s（仅放开 lean/nudge/高度 限幅）", on ? "on" : "off");
        ack(id, true, NULL);

    } else {
        ESP_LOGW(TAG, "未知命令: %s", t);
        err = "unknown_cmd";
        ack(id, false, err);
    }

done:
    cJSON_Delete(root);
    return rc;
}