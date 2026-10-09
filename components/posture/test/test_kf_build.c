/*
 * kf_check_meta / kf_set_frame 主机端单元测试。
 *
 * 只编译 kf_build.c（纯逻辑，无 ESP-IDG / cJSON 依赖），
 * 因此能覆盖协议 §4 的每条边界规则。
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "spider_kf_parse.h"

static int g_pass;
static int g_fail;

#define CHECK(cond, ...)                                        \
    do {                                                        \
        if (cond) {                                             \
            g_pass++;                                           \
        } else {                                                \
            g_fail++;                                           \
            printf("[FAIL] %s:%d ", __FILE__, __LINE__);        \
            printf(__VA_ARGS__);                                \
            printf("\n");                                       \
        }                                                       \
    } while (0)

/* 合法头部：名字 "wave"，2 帧，smoothstep */
static kf_meta_t good_meta(void)
{
    kf_meta_t m = {
        .name        = "wave",
        .loop        = true,
        .ease        = (uint8_t)KF_SMOOTHSTEP,
        .count       = 2,
        .duration_ms = 0,
    };
    return m;
}

static float legs_a[POSTURE_JOINTS] = { 10, -30, 10, -30, 10, -30, 10, -30 };
static float legs_b[POSTURE_JOINTS] = { 40, -20, 10, -30, 10, -30, 10, -30 };

static void test_ok_path(void)
{
    kf_meta_t m = good_meta();
    kf_seq_t seq;
    kf_parse_err_t e = KF_PARSE_OK;

    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_OK, "合法头部被拒");
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_OK, "合法头部重复调用被拒");
    CHECK(strcmp(seq.name, "wave") == 0, "名字错: %s", seq.name);
    CHECK(seq.loop == true, "loop 丢失");
    CHECK(seq.ease == KF_SMOOTHSTEP, "ease 错");
    CHECK(seq.count == 2, "帧数错: %u", seq.count);
    /* duration_ms=0 时应由最后一帧 t_ms 推出 */
    CHECK(seq.duration_ms == 0, "初始 duration 应仍为 0");

    CHECK(kf_set_frame(&seq, 0, 0, POSTURE_JOINTS, legs_a, &e) == KF_PARSE_OK,
          "第 0 帧被拒");
    CHECK(seq.frames[0].t_ms == 0, "第 0 帧时间错");
    CHECK(seq.frames[0].joints[1] == -30.0f, "第 0 帧关节错: %f",
          seq.frames[0].joints[1]);
    CHECK(seq.duration_ms == 0, "duration 应推为第 0 帧时间");

    CHECK(kf_set_frame(&seq, 1, 250, POSTURE_JOINTS, legs_b, &e) == KF_PARSE_OK,
          "第 1 帧被拒");
    CHECK(seq.frames[1].joints[0] == 40.0f, "第 1 帧关节错");
    /* duration_ms 未显式给出 -> 推算为最后一帧时间 250 */
    CHECK(seq.duration_ms == 250, "duration 应推为 250，实为 %u", seq.duration_ms);
}

static void test_bad_name(void)
{
    kf_seq_t seq;
    kf_parse_err_t e = KF_PARSE_OK;

    kf_meta_t m = good_meta();
    m.name = NULL;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_BAD_NAME, "name=NULL 未被拒");
    CHECK(e == KF_PARSE_BAD_NAME, "错误码未回填");

    m.name = "";
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_BAD_NAME, "空名未被拒");

    /* 超长：KF_NAME_MAX + 1 */
    char longname[64];
    memset(longname, 'a', sizeof(longname) - 1);
    longname[sizeof(longname) - 1] = '\0';
    m.name = longname;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_BAD_NAME, "超长名未被拒");

    /* 边界：恰好 KF_NAME_MAX 应通过 */
    char exact[64];
    memset(exact, 'b', KF_NAME_MAX);
    exact[KF_NAME_MAX] = '\0';
    m.name = exact;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_OK, "恰好 23 字符应通过");

    /* 少一位也不该溢出 name[] */
    CHECK(strlen(seq.name) == KF_NAME_MAX, "截断后长度错: %u",
          (unsigned)strlen(seq.name));
}

static void test_bad_count(void)
{
    kf_seq_t seq;
    kf_parse_err_t e = KF_PARSE_OK;
    kf_meta_t m = good_meta();

    m.count = 0;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_BAD_COUNT, "count=0 未被拒");

    m.count = KF_MAX_FRAMES + 1;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_TOO_LARGE, "帧数超限未报 too_large");

    m.count = KF_MAX_FRAMES;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_OK, "恰好 512 帧应通过");

    CHECK(kf_check_meta(NULL, &seq, &e) == KF_PARSE_BAD_COUNT, "meta=NULL 未被拒");
    CHECK(kf_check_meta(&m, NULL, &e) == KF_PARSE_BAD_COUNT, "out=NULL 未被拒");
}

static void test_bad_ease(void)
{
    kf_seq_t seq;
    kf_parse_err_t e = KF_PARSE_OK;
    kf_meta_t m = good_meta();

    m.ease = (uint8_t)KF_EASE_IN_OUT;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_OK, "ease=2 应通过");
    m.ease = 3;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_BAD_EASE, "ease=3 未被拒");
    m.ease = 255;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_BAD_EASE, "ease=255 未被拒");
}

static void test_bad_legs(void)
{
    kf_meta_t m = good_meta();
    kf_seq_t seq;
    kf_parse_err_t e = KF_PARSE_OK;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_OK, "前置头部失败");

    /* 每帧关节数必须恰为 8，否则拷贝会越界读 */
    CHECK(kf_set_frame(&seq, 0, 0, 7, legs_a, &e) == KF_PARSE_BAD_LEGS,
          "legs=7 未被拒");
    CHECK(kf_set_frame(&seq, 0, 0, 9, legs_a, &e) == KF_PARSE_BAD_LEGS,
          "legs=9 未被拒");
    CHECK(kf_set_frame(&seq, 0, 0, 0, legs_a, &e) == KF_PARSE_BAD_LEGS,
          "legs=0 未被拒");
    CHECK(kf_set_frame(&seq, 0, 0, POSTURE_JOINTS, NULL, &e) == KF_PARSE_BAD_LEGS,
          "legs=NULL 未被拒");
}

static void test_bad_time(void)
{
    kf_meta_t m = good_meta();
    kf_seq_t seq;
    kf_parse_err_t e = KF_PARSE_OK;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_OK, "前置头部失败");

    /* 首帧必须为 0（协议 §4：t 是相对起点的毫秒） */
    CHECK(kf_set_frame(&seq, 0, 100, POSTURE_JOINTS, legs_a, &e)
          == KF_PARSE_BAD_TIME, "首帧 t=100 未被拒");

    CHECK(kf_set_frame(&seq, 0, 0, POSTURE_JOINTS, legs_a, &e) == KF_PARSE_OK,
          "首帧 t=0 应通过");

    /* 允许同刻（不减），供多关节同一时刻过渡 */
    CHECK(kf_set_frame(&seq, 1, 0, POSTURE_JOINTS, legs_b, &e) == KF_PARSE_OK,
          "同刻帧应通过（不减）");

    /* 回退应被拒 */
    kf_meta_t m2 = good_meta();
    m2.count = 3;
    CHECK(kf_check_meta(&m2, &seq, &e) == KF_PARSE_OK, "前置头部失败");
    CHECK(kf_set_frame(&seq, 0, 0, POSTURE_JOINTS, legs_a, &e) == KF_PARSE_OK,
          "首帧失败");
    CHECK(kf_set_frame(&seq, 1, 200, POSTURE_JOINTS, legs_b, &e) == KF_PARSE_OK,
          "第 1 帧失败");
    CHECK(kf_set_frame(&seq, 2, 100, POSTURE_JOINTS, legs_b, &e)
          == KF_PARSE_BAD_TIME, "时间回退未被拒");
}

static void test_index_bounds(void)
{
    kf_meta_t m = good_meta();      /* count = 2 */
    kf_seq_t seq;
    kf_parse_err_t e = KF_PARSE_OK;
    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_OK, "前置头部失败");

    CHECK(kf_set_frame(&seq, 2, 0, POSTURE_JOINTS, legs_a, &e)
          == KF_PARSE_BAD_COUNT, "idx>=count 未被拒");
    CHECK(kf_set_frame(&seq, 0xFFFF, 0, POSTURE_JOINTS, legs_a, &e)
          == KF_PARSE_BAD_COUNT, "idx=0xFFFF 未被拒");
    CHECK(kf_set_frame(NULL, 0, 0, POSTURE_JOINTS, legs_a, &e)
          == KF_PARSE_BAD_LEGS, "out=NULL 未被拒");
}

static void test_explicit_duration(void)
{
    kf_meta_t m = good_meta();
    m.duration_ms = 500;            /* 显式指定，帧时间仍为 250 */
    kf_seq_t seq;
    kf_parse_err_t e = KF_PARSE_OK;

    CHECK(kf_check_meta(&m, &seq, &e) == KF_PARSE_OK, "前置头部失败");
    CHECK(seq.duration_ms == 500, "显式 duration 未保留: %u", seq.duration_ms);
    CHECK(kf_set_frame(&seq, 0, 0, POSTURE_JOINTS, legs_a, &e) == KF_PARSE_OK,
          "第 0 帧失败");
    CHECK(seq.duration_ms == 500, "显式 duration 被帧时间覆盖: %u",
          seq.duration_ms);
}

static void test_err_str(void)
{
    /* 必须映射到协议 §8 的取值 */
    CHECK(strcmp(kf_parse_err_str(KF_PARSE_OK), "ok") == 0, "OK 串错");
    CHECK(strcmp(kf_parse_err_str(KF_PARSE_BAD_NAME), "bad_field") == 0,
          "BAD_NAME 串错");
    CHECK(strcmp(kf_parse_err_str(KF_PARSE_BAD_COUNT), "bad_field") == 0,
          "BAD_COUNT 串错");
    CHECK(strcmp(kf_parse_err_str(KF_PARSE_BAD_LEGS), "bad_field") == 0,
          "BAD_LEGS 串错");
    CHECK(strcmp(kf_parse_err_str(KF_PARSE_BAD_TIME), "bad_field") == 0,
          "BAD_TIME 串错");
    CHECK(strcmp(kf_parse_err_str(KF_PARSE_BAD_EASE), "bad_field") == 0,
          "BAD_EASE 串错");
    CHECK(strcmp(kf_parse_err_str(KF_PARSE_TOO_LARGE), "too_large") == 0,
          "TOO_LARGE 串错");
    CHECK(strcmp(kf_parse_err_str(KF_PARSE_NO_MEM), "no_mem") == 0,
          "NO_MEM 串错");
    CHECK(strcmp(kf_parse_err_str((kf_parse_err_t)99), "bad_field") == 0,
          "未知错误码未回落到 bad_field");
}

static void test_upload_bytes_limit(void)
{
    /* 协议 §4 上限：报文 4KB。单帧 62B 估算，约合 60 帧手势序列 */
    CHECK(KF_MAX_UPLOAD_BYTES == 4096, "报文上限变了: %d", KF_MAX_UPLOAD_BYTES);
    /* 上限必须容得下协议 §4 示例那样的最小序列 */
    CHECK(KF_MAX_UPLOAD_BYTES > 512, "上限小于文档示例");
    /* 每帧 8 关节，与 docs/protocol.md §4 的 legs 定义一致 */
    CHECK(POSTURE_JOINTS == 8, "关节数变了: %d", POSTURE_JOINTS);
}

int main(void)
{
    test_ok_path();
    test_bad_name();
    test_bad_count();
    test_bad_ease();
    test_bad_legs();
    test_bad_time();
    test_index_bounds();
    test_explicit_duration();
    test_err_str();
    test_upload_bytes_limit();

    printf("\n关键帧解析测试：%d 通过, %d 失败\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}