/*
 * kf_parse_json —— keyframe_upload 的 cJSON 适配层。
 *
 * 只做三件事：取值、遍历、调 kf_check_meta/kf_set_frame。
 * 校验规则全在 kf_build.c（可主机测试），这里不含独立分支逻辑。
 */

#include "spider_kf_parse.h"

#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "spider_keyframe.h"

static const char *TAG = "kfparse";

/* ease 字符串 -> 枚举。返回 false 表示不在协议 §4 的三个取值内 */
static bool ease_from_str(const char *s, uint8_t *out)
{
    if (s == NULL) {
        return false;
    }
    if (strcmp(s, "linear") == 0)     { *out = (uint8_t)KF_LINEAR;     return true; }
    if (strcmp(s, "smoothstep") == 0) { *out = (uint8_t)KF_SMOOTHSTEP; return true; }
    if (strcmp(s, "ease_in_out") == 0){ *out = (uint8_t)KF_EASE_IN_OUT;return true; }
    return false;
}

/* 顶层直取字段的辅助（cJSON 无 valuebool 宏，valuedouble 复用） */
static bool obj_bool(const cJSON *o, const char *key, bool dflt)
{
    const cJSON *v = cJSON_GetObjectItem(o, key);
    if (v == NULL) {
        return dflt;
    }
    return cJSON_IsTrue(v);
}

static bool obj_num(const cJSON *o, const char *key, double *out)
{
    const cJSON *v = cJSON_GetObjectItem(o, key);
    if (v == NULL || !cJSON_IsNumber(v)) {
        return false;
    }
    *out = v->valuedouble;
    return true;
}

kf_parse_err_t kf_parse_json(const cJSON *root, size_t bytes)
{
    kf_parse_err_t err = KF_PARSE_OK;

    if (root == NULL) {
        return KF_PARSE_BAD_COUNT;
    }

    /* ---- 报文体积上限 ---- */
    if (bytes > KF_MAX_UPLOAD_BYTES) {
        ESP_LOGW(TAG, "报文 %u 字节超过上限 %u", (unsigned)bytes,
                 (unsigned)KF_MAX_UPLOAD_BYTES);
        return KF_PARSE_TOO_LARGE;
    }

    /* ---- 取头部字段 ---- */
    const cJSON *cname = cJSON_GetObjectItem(root, "name");
    if (!cJSON_IsString(cname)) {
        return KF_PARSE_BAD_NAME;
    }

    uint8_t ease = (uint8_t)KF_LINEAR;
    const cJSON *cease = cJSON_GetObjectItem(root, "ease");
    if (cease != NULL) {
        if (!cJSON_IsString(cease) ||
            !ease_from_str(cJSON_GetStringValue(cease), &ease)) {
            return KF_PARSE_BAD_EASE;
        }
    }

    double period = 0.0;
    if (obj_num(root, "period_ms", &period)) {
        if (period < 0.0 || period > (double)UINT32_MAX) {
            return KF_PARSE_BAD_TIME;
        }
    }

    const cJSON *frames = cJSON_GetObjectItem(root, "frames");
    if (!cJSON_IsArray(frames)) {
        return KF_PARSE_BAD_COUNT;
    }
    int count = cJSON_GetArraySize(frames);
    if (count <= 0) {
        return KF_PARSE_BAD_COUNT;
    }

    kf_meta_t meta = {
        .name        = cJSON_GetStringValue(cname),
        .loop        = obj_bool(root, "loop", false),
        .ease        = ease,
        .count       = (uint16_t)count,
        .duration_ms = (uint32_t)period,
    };

    /* ---- 头部校验通过后才分配 18.5KB 临时缓冲 ---- */
    kf_seq_t probe;
    if (kf_check_meta(&meta, &probe, &err) != KF_PARSE_OK) {
        return err;
    }

    kf_seq_t *seq = (kf_seq_t *)calloc(1, sizeof(kf_seq_t));
    if (seq == NULL) {
        ESP_LOGE(TAG, "分配 %u 字节失败", (unsigned)sizeof(kf_seq_t));
        return KF_PARSE_NO_MEM;
    }
    if (kf_check_meta(&meta, seq, &err) != KF_PARSE_OK) {
        free(seq);
        return err;
    }

    /* ---- 逐帧填充 ---- */
    float legs[POSTURE_JOINTS];
    for (int i = 0; i < count; i++) {
        const cJSON *fr = cJSON_GetArrayItem(frames, i);
        if (!cJSON_IsObject(fr)) {
            err = KF_PARSE_BAD_COUNT;
            break;
        }

        double t = 0.0;
        if (!obj_num(fr, "t", &t) || t < 0.0 || t > (double)UINT32_MAX) {
            err = KF_PARSE_BAD_TIME;
            break;
        }

        const cJSON *ja = cJSON_GetObjectItem(fr, "legs");
        if (!cJSON_IsArray(ja)) {
            err = KF_PARSE_BAD_LEGS;
            break;
        }
        int n = cJSON_GetArraySize(ja);
        /* 逐项校验为数字，防止非数值元素被当作 0 静默通过 */
        bool ok = true;
        for (int j = 0; j < n; j++) {
            const cJSON *e = cJSON_GetArrayItem(ja, j);
            if (!cJSON_IsNumber(e)) {
                ok = false;
                break;
            }
            legs[j] = (float)e->valuedouble;
        }
        if (!ok) {
            err = KF_PARSE_BAD_LEGS;
            break;
        }

        if (kf_set_frame(seq, (uint16_t)i, (uint32_t)t,
                         (uint8_t)n, legs, &err) != KF_PARSE_OK) {
            break;
        }
    }

    if (err == KF_PARSE_OK) {
        if (kf_upload(seq) != ESP_OK) {
            err = KF_PARSE_NO_MEM;   /* 槽已满，或深度校验（有限值）未过 */
        }
    }
    free(seq);

    if (err != KF_PARSE_OK) {
        ESP_LOGW(TAG, "上传失败: %s", kf_parse_err_str(err));
    } else {
        ESP_LOGI(TAG, "已上传 %s (%d 帧)", meta.name, count);
    }
    return err;
}