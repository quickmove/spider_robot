/*
 * 姿态与关键帧播放器
 *
 * 两类动作共用一套插值引擎（docs/architecture.md §2）：
 *   1. 内置姿态：REST / STAND / SIT / SPAWN，只给目标身高与髋偏移
 *   2. 关键帧序列：Blender 导出或 Web 上传，逐帧关节角
 *
 * 关键约束（docs/gait_design.md §4）：
 * 站立相直接改膝会让四只脚同时拖地——1:cot(β) 耦合所致。
 * 因此 posture 不自己插值关节角，而是把「目标身高」交给 motion 层，
 * 由 gait 的摆动相逐腿执行（reposture），全程零滑移。
 * 本组件只在非行走状态、且已静止落地时负责决定目标值。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "spider_params.h"

#ifdef __cplusplus
extern "C" {
#endif

#define POSTURE_LEG_COUNT  4
#define POSTURE_JOINTS    (POSTURE_LEG_COUNT * 2)

typedef enum {
    POSTURE_REST  = 0,   /* 休息：腹部近地 */
    POSTURE_STAND = 1,   /* 站立 */
    POSTURE_SIT   = 2,   /* 蹲坐 */
    POSTURE_SPAWN = 3,   /* 翻车姿态（仰卧） */
    POSTURE_CUSTOM = 4,  /* 关键帧序列播放中 */
} posture_id_t;

/* 姿态定义：目标身高 + 四腿髋偏移（膝由身高唯一决定） */
typedef struct {
    const char *name;
    float       height_mm;
    float       hip_deg[POSTURE_LEG_COUNT];
} posture_def_t;

/* 内置姿态表（只读，勿改） */
const posture_def_t *posture_get(posture_id_t id);
const char *posture_name(posture_id_t id);
bool posture_from_name(const char *name, posture_id_t *out);

/* ---- 关键帧序列 ---- */

typedef enum {
    KF_LINEAR = 0,
    KF_SMOOTHSTEP,
    KF_EASE_IN_OUT,
} kf_ease_t;

#define KF_MAX_FRAMES  512

typedef struct {
    uint32_t t_ms;                          /* 相对起点 */
    float    joints[POSTURE_JOINTS];        /* FL.h FL.k FR.h FR.k RL.h RL.k RR.h RR.k */
} kf_frame_t;

typedef struct {
    char       name[24];
    uint32_t   duration_ms;                 /* 0 = 未知，按最后一帧时间推算 */
    bool       loop;
    kf_ease_t  ease;
    uint16_t   count;      /* uint16_t：KF_MAX_FRAMES=512 超出 uint8_t 范围 */
    kf_frame_t frames[KF_MAX_FRAMES];
} kf_seq_t;

#ifdef __cplusplus
}
#endif