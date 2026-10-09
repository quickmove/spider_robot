/*
 * 关键帧序列播放器接口
 *
 * 序列格式见 docs/protocol.md §4；Blender 导出流程见 tools/blender/。
 */

#pragma once

#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

#include "spider_posture.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * RAM 中最多同时驻留的序列数。
 *
 * kf_seq_t 内联 KF_MAX_FRAMES 帧，每帧 4+32=36B，即单条约 18.5KB。
 * 2 条约占 37KB 堆——ESP32 自由堆通常 150~200KB，再多会把运行时挤紧。
 * 调试阶段一次只需跑一个动作，不值得为容量牺牲余量。
 * 需要更多时改为按实际帧数动态分配，而非直接调大此值。
 */
#define KF_MAX_SEQS  2

typedef struct {
    const kf_seq_t *seq;
    uint32_t        t_ms;
    bool            playing;
} kf_player_t;

/* 上传序列（校验失败返回 ESP_ERR_INVALID_ARG）。同名则覆盖 */
esp_err_t kf_upload(const kf_seq_t *seq);

/* 释放序列（池按需分配，不释放则一直占用堆） */
void kf_release(const char *name);
void kf_release_all(void);

const kf_seq_t *kf_find(const char *name);
uint8_t kf_count(void);
const kf_seq_t *kf_at(uint8_t i);

esp_err_t kf_play(const char *name);
void kf_stop(void);
bool kf_is_playing(void);
const char *kf_current_name(void);

/* 推进 dt_ms 并插值出 8 个关节角；返回 false 表示无播放中序列或已播完 */
bool kf_update(uint32_t dt_ms, float *out_joints);

#ifdef __cplusplus
}
#endif