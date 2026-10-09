/*
 * 关键帧序列播放器
 *
 * 与内置姿态共用插值语义，但直接输出关节角——用于 Blender 导出的
 * 风格化动作（装死、挣扎、招手等，见 docs/insect_behavior.md）。
 *
 * 序列存于 RAM（最多 4 条），刷机后丢失、可重新上传；
 * 不落 NVS 是刻意的：动作序列属于表现层，不该占用标定存储。
 */

#include "spider_keyframe.h"

#include <math.h>
#include <string.h>

#include <stdlib.h>

#include "esp_log.h"

#include "spider_posture.h"

/*
 * 序列池用堆按需分配。
 *
 * 静态数组的代价：kf_seq_t 含 512 帧 x 36 字节 ≈ 18KB，
 * 4 条即 72KB —— 空载就吃掉四成 DRAM。
 * 改为「上传时才分配」，空载成本为 0（仅指针数组）。
 */
static kf_player_t s_player;
static kf_seq_t   *s_seqs[KF_MAX_SEQS];
static uint8_t     s_seq_count;

static inline float ease_linear(float t)  { return t; }
static inline float ease_smooth(float t) { return t * t * (3.0f - 2.0f * t); }
static inline float ease_inout(float t)
{
    return (t < 0.5f) ? (2.0f * t * t)
                       : (1.0f - 2.0f * (1.0f - t) * (1.0f - t));
}

static float apply_ease(kf_ease_t e, float t)
{
    t = (t < 0.0f) ? 0.0f : ((t > 1.0f) ? 1.0f : t);
    switch (e) {
    case KF_SMOOTHSTEP: return ease_smooth(t);
    case KF_EASE_IN_OUT: return ease_inout(t);
    case KF_LINEAR:
    default: return ease_linear(t);
    }
}

/* 序列校验：帧数、时间单调、关节值有限 */
static bool seq_valid(const kf_seq_t *s, const char **why)
{
    if (s->count == 0) {
        *why = "空序列";
        return false;
    }
    if (s->count > KF_MAX_FRAMES) {
        *why = "帧数超限";
        return false;
    }
    for (uint16_t i = 1; i < s->count; i++) {
        if (s->frames[i].t_ms <= s->frames[i - 1].t_ms) {
            *why = "时间戳非单调";
            return false;
        }
    }
    for (uint16_t i = 0; i < s->count; i++) {
        for (int j = 0; j < POSTURE_JOINTS; j++) {
            float v = s->frames[i].joints[j];
            if (isnan(v) || isinf(v)) {
                *why = "关节值非有限";
                return false;
            }
        }
    }
    return true;
}

esp_err_t kf_upload(const kf_seq_t *seq)
{
    const char *why = NULL;
    if (seq == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!seq_valid(seq, &why)) {
        ESP_LOGE("kf", "序列非法: %s", why ? why : "?");
        return ESP_ERR_INVALID_ARG;
    }

    /* 找同名序列或空槽 */
    int slot = -1;
    int empty = -1;
    for (uint8_t i = 0; i < KF_MAX_SEQS; i++) {
        if (s_seqs[i] == NULL) {
            if (empty < 0) empty = i;
            continue;
        }
        if (strcmp(s_seqs[i]->name, seq->name) == 0) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        slot = empty;
    }
    if (slot < 0) {
        ESP_LOGE("kf", "序列槽已满 (%u)，请先释放", KF_MAX_SEQS);
        return ESP_ERR_NO_MEM;
    }

    size_t bytes = sizeof(kf_seq_t);
    kf_seq_t *dst = (kf_seq_t *)malloc(bytes);
    if (dst == NULL) {
        ESP_LOGE("kf", "分配 %zu 字节失败", bytes);
        return ESP_ERR_NO_MEM;
    }
    memcpy(dst, seq, bytes);

    if (s_seqs[slot] != NULL) {
        free(s_seqs[slot]);
        ESP_LOGI("kf", "覆盖序列 %s (%u 帧)", dst->name, dst->count);
    } else {
        s_seq_count++;
        ESP_LOGI("kf", "载入序列 %s (%u 帧, loop=%d)",
                 dst->name, dst->count, dst->loop);
    }
    s_seqs[slot] = dst;
    return ESP_OK;
}

void kf_release(const char *name)
{
    if (name == NULL) {
        return;
    }
    for (uint8_t i = 0; i < KF_MAX_SEQS; i++) {
        if (s_seqs[i] != NULL && strcmp(s_seqs[i]->name, name) == 0) {
            free(s_seqs[i]);
            s_seqs[i] = NULL;
            if (s_seq_count > 0) s_seq_count--;
            ESP_LOGI("kf", "释放序列 %s", name);
            return;
        }
    }
}

void kf_release_all(void)
{
    for (uint8_t i = 0; i < KF_MAX_SEQS; i++) {
        free(s_seqs[i]);
        s_seqs[i] = NULL;
    }
    s_seq_count = 0;
}

const kf_seq_t *kf_find(const char *name)
{
    if (name == NULL) {
        return NULL;
    }
    for (uint8_t i = 0; i < KF_MAX_SEQS; i++) {
        if (s_seqs[i] != NULL && strcmp(s_seqs[i]->name, name) == 0) {
            return s_seqs[i];
        }
    }
    return NULL;
}

uint8_t kf_count(void)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < KF_MAX_SEQS; i++) {
        if (s_seqs[i] != NULL) n++;
    }
    return n;
}

const kf_seq_t *kf_at(uint8_t slot)
{
    return (slot < KF_MAX_SEQS) ? s_seqs[slot] : NULL;
}

static uint32_t seq_duration(const kf_seq_t *s)
{
    if (s->duration_ms > 0) {
        return s->duration_ms;
    }
    return (s->count > 0) ? s->frames[s->count - 1].t_ms : 0;
}

esp_err_t kf_play(const char *name)
{
    const kf_seq_t *s = kf_find(name);
    if (s == NULL) {
        ESP_LOGW("kf", "未找到序列 %s", name ? name : "(null)");
        return ESP_ERR_NOT_FOUND;
    }
    s_player.seq = s;
    s_player.t_ms = 0;
    s_player.playing = true;
    ESP_LOGI("kf", "播放 %s (%ums)", s->name, (unsigned)seq_duration(s));
    return ESP_OK;
}

void kf_stop(void)
{
    s_player.playing = false;
    s_player.seq = NULL;
}

bool kf_is_playing(void)
{
    return s_player.playing && s_player.seq != NULL;
}

const char *kf_current_name(void)
{
    return s_player.seq ? s_player.seq->name : "";
}

/*
 * 推进 dt_ms，插值出 8 个关节角。
 * 返回 false 表示已播完（一次性序列）或无播放中序列。
 */
bool kf_update(uint32_t dt_ms, float *out_joints)
{
    if (!kf_is_playing() || out_joints == NULL) {
        return false;
    }

    const kf_seq_t *s = s_player.seq;
    uint32_t dur = seq_duration(s);
    if (dur == 0) {
        kf_stop();
        return false;
    }

    s_player.t_ms += dt_ms;
    uint32_t t = s_player.t_ms;
    if (t >= dur) {
        if (s->loop) {
            t %= dur;
            s_player.t_ms = t;
        } else {
            s_player.t_ms = dur;
            /* 输出末帧后停止 */
            memcpy(out_joints, s->frames[s->count - 1].joints,
                   sizeof(float) * POSTURE_JOINTS);
            kf_stop();
            return true;
        }
    }

    /* 定位所在区间 */
    uint16_t i = 0;
    while (i + 1 < s->count && s->frames[i + 1].t_ms <= t) {
        i++;
    }
    uint16_t j = (i + 1 < s->count) ? (uint16_t)(i + 1) : i;

    float span = (float)(s->frames[j].t_ms - s->frames[i].t_ms);
    float u = (span > 0.0f) ? ((float)t - (float)s->frames[i].t_ms) / span : 1.0f;
    u = apply_ease(s->ease, u);

    for (int k = 0; k < POSTURE_JOINTS; k++) {
        float a = s->frames[i].joints[k];
        float b = s->frames[j].joints[k];
        out_joints[k] = a + (b - a) * u;
    }
    return true;
}