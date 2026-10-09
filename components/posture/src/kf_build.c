/*
 * kf_check_meta / kf_set_frame —— keyframe_upload 的纯校验核心。
 *
 * 本文件刻意不 include 任何 ESP-IDG 或 cJSON 头，以便 tools/run_tests.sh
 * 在主机端编译并单元测试（见 components/posture/test/test_kf_build.c）。
 */

#include "spider_kf_parse.h"

#include <string.h>

const char *kf_parse_err_str(kf_parse_err_t e)
{
    switch (e) {
    case KF_PARSE_OK:        return "ok";
    case KF_PARSE_BAD_NAME:  return "bad_field";
    case KF_PARSE_BAD_COUNT: return "bad_field";
    case KF_PARSE_BAD_LEGS:  return "bad_field";
    case KF_PARSE_BAD_TIME:  return "bad_field";
    case KF_PARSE_BAD_EASE:  return "bad_field";
    case KF_PARSE_TOO_LARGE: return "too_large";
    case KF_PARSE_NO_MEM:    return "no_mem";
    }
    return "bad_field";
}

kf_parse_err_t kf_check_meta(const kf_meta_t *m, kf_seq_t *out,
                             kf_parse_err_t *err)
{
    if (err == NULL) {
        static kf_parse_err_t sink;
        err = &sink;
    }
    *err = KF_PARSE_OK;

    if (m == NULL || out == NULL) {
        *err = KF_PARSE_BAD_COUNT;
        return *err;
    }

    /* ---- 名字：缺失或空或超长 ---- */
    if (m->name == NULL || m->name[0] == '\0') {
        *err = KF_PARSE_BAD_NAME;
        return *err;
    }
    size_t nlen = strlen(m->name);
    if (nlen > KF_NAME_MAX) {
        *err = KF_PARSE_BAD_NAME;
        return *err;
    }

    /* ---- 帧数：空序列无意义，超上限先于写任何帧之前拒绝 ---- */
    if (m->count == 0) {
        *err = KF_PARSE_BAD_COUNT;
        return *err;
    }
    if (m->count > KF_MAX_FRAMES) {
        *err = KF_PARSE_TOO_LARGE;
        return *err;
    }

    /* ---- ease 枚举 ---- */
    if (m->ease > (uint8_t)KF_EASE_IN_OUT) {
        *err = KF_PARSE_BAD_EASE;
        return *err;
    }

    /* 只清头部与帧时间轴区域，长度按 count 定，避免整块 18.5KB memset 的浪费 */
    memset(out, 0, sizeof(*out));
    memcpy(out->name, m->name, nlen);
    out->name[nlen] = '\0';
    out->loop      = m->loop;
    out->ease      = (kf_ease_t)m->ease;
    out->count     = m->count;
    out->duration_ms = m->duration_ms;

    return KF_PARSE_OK;
}

kf_parse_err_t kf_set_frame(kf_seq_t *out, uint16_t idx, uint32_t t_ms,
                            uint8_t legs_per_frame, const float *legs,
                            kf_parse_err_t *err)
{
    if (err == NULL) {
        static kf_parse_err_t sink;
        err = &sink;
    }
    *err = KF_PARSE_OK;

    if (out == NULL || legs == NULL) {
        *err = KF_PARSE_BAD_LEGS;
        return *err;
    }
    if (idx >= out->count) {
        *err = KF_PARSE_BAD_COUNT;
        return *err;
    }

    /*
     * 每帧关节数必须先于拷贝校验：拷贝按 sizeof(frame.joints) 固定步长进行，
     * 若调用方只给了 3 个数就会越界读。
     */
    if (legs_per_frame != POSTURE_JOINTS) {
        *err = KF_PARSE_BAD_LEGS;
        return *err;
    }

    /*
     * 时间单调：第 0 帧必须为 0（协议 §4 的 t 是「相对起点」的毫秒），
     * 其余帧要求不减。kf_upload() 的 seq_valid() 会再查一次严格递增，
     * 这里放宽到「不减」以允许同刻多关节过渡。
     */
    if (idx == 0) {
        if (t_ms != 0) {
            *err = KF_PARSE_BAD_TIME;
            return *err;
        }
    } else if (t_ms < out->frames[idx - 1].t_ms) {
        *err = KF_PARSE_BAD_TIME;
        return *err;
    }

    out->frames[idx].t_ms = t_ms;
    memcpy(out->frames[idx].joints, legs, sizeof(out->frames[idx].joints));

    /* duration_ms 未显式给出时，用已写入的最后一帧时间推算 */
    if (out->duration_ms == 0) {
        out->duration_ms = t_ms;
    }

    return KF_PARSE_OK;
}