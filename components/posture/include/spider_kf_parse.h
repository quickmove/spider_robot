/*
 * 关键帧上传解析（协议 keyframe_upload，docs/protocol.md §4）
 *
 * 分两层，以便在主机端做单元测试：
 *
 *   kf_check_meta() / kf_set_frame()   纯逻辑，逐帧校验并填充。不碰
 *       cJSON / ESP-IDG / 大缓冲，因此能被 tools/run_tests.sh 直接编译测试。
 *   kf_parse_json()                    薄适配层：cJSON 取值 -> 调用上面两个函数，
 *       只负责「遍历数组 + 管理临时缓冲」这层胶水代码。
 *
 * 为什么是逐帧而不是「先摊平再整体校验」：kf_seq_t 约 18.5KB，而 httpd 任务栈
 * 只有 8KB，既不能放摊平数组（512×8 float = 16KB）也不能放 seq 本身。逐帧填充
 * 到一块堆上的临时 seq 可以把峰值内存控制住。
 *
 * 深度校验（关节值有限）由 kf_upload() 内的 seq_valid() 负责，此处不重复实现。
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "spider_posture.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 前置声明，避免本头文件依赖 cJSON（主机端测试无法提供） */
typedef struct cJSON cJSON;

/* 协议 §4 报文上限。上限由 WS 接收缓冲与 cJSON 内存共同决定，见 docs/protocol.md §4 */
#define KF_MAX_UPLOAD_BYTES  4096

/* name[24]，故最长 23 字符 */
#define KF_NAME_MAX  (sizeof(((kf_seq_t *)0)->name) - 1)

typedef enum {
    KF_PARSE_OK = 0,
    KF_PARSE_BAD_NAME,     /* 缺失、超长 */
    KF_PARSE_BAD_COUNT,    /* frames 缺失或为空 */
    KF_PARSE_BAD_LEGS,     /* 每帧 legs 不是 8 个数 */
    KF_PARSE_BAD_TIME,     /* t_ms 缺失、首帧非 0、或非单调 */
    KF_PARSE_BAD_EASE,     /* ease 不在枚举内 */
    KF_PARSE_TOO_LARGE,    /* 帧数 >512 或报文超长 */
    KF_PARSE_NO_MEM,       /* 堆分配失败或序列槽已满 */
} kf_parse_err_t;

/* 错误码 -> 协议 §8 的 error 字符串（bad_field / too_large / no_mem） */
const char *kf_parse_err_str(kf_parse_err_t e);

/* 序列头部（不含帧内容） */
typedef struct {
    const char *name;
    bool        loop;
    uint8_t     ease;          /* KF_LINEAR / KF_SMOOTHSTEP / KF_EASE_IN_OUT */
    uint16_t    count;         /* 帧数，1~KF_MAX_FRAMES */
    uint32_t    duration_ms;   /* 0 = 由最后一帧 t_ms 推出 */
} kf_meta_t;

/*
 * 校验并写入头部。不触碰 frames 数组。
 * 返回 KF_PARSE_OK 表示头部有效，可继续调用 kf_set_frame()。
 */
kf_parse_err_t kf_check_meta(const kf_meta_t *m, kf_seq_t *out,
                             kf_parse_err_t *err);

/*
 * 写入第 idx 帧并校验：
 *   - idx < out->count
 *   - legs_per_frame == POSTURE_JOINTS（必须先于按固定步长拷贝校验，否则越界）
 *   - t_ms 单调不减（idx==0 时要求为 0）
 *
 * 必须按 idx 递增顺序调用（前一帧的值用于单调性比较）。
 */
kf_parse_err_t kf_set_frame(kf_seq_t *out, uint16_t idx, uint32_t t_ms,
                            uint8_t legs_per_frame, const float *legs,
                            kf_parse_err_t *err);

/*
 * 解析 keyframe_upload 报文并上传到关键帧池。
 * bytes 为原始报文长度（用于上限判定）。
 *
 * 内部会分配一块 sizeof(kf_seq_t) 的临时缓冲（约 18.5KB），
 * kf_upload() 拷贝进池后释放。无论成功失败都会释放。
 */
kf_parse_err_t kf_parse_json(const cJSON *root, size_t bytes);

#ifdef __cplusplus
}
#endif