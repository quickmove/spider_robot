/*
 * 串口 CLI（M1）
 *
 * 与 Web 控制协议共用同一套命令名（见 docs/protocol.md §5），
 * 只是输出为人类可读文本而非 JSON。
 *
 * 用 fgets/stdin 而非 uart_driver_install：UART0 同时是 IDF 控制台输出，
 * 装了驱动会和 VFS 冲突。
 */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "spider_params.h"
#include "spider_cli.h"
#include "spider_gait.h"
#include "spider_motion.h"
#include "spider_posture.h"
#include "spider_servo.h"

#if CONFIG_SPIDER_DEBUG_CLI

static const char *TAG = "cli";

#define servo_cli_start spider_cli_start

static void cmd_cal_show(void)
{
    servo_cal_t cal;
    printf("ch  min   max   neu   trim  rev\n");
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        servo_cal_get(ch, &cal);
        printf("%-3d %-5u %-5u %-5u %-5d %s%s\n", ch, cal.min_us, cal.max_us,
               cal.neutral_us, cal.trim_us, cal.reverse ? "Y" : "n",
               servo_cal_is_dirty(ch) ? " *" : "");
    }
    printf("标定状态: %s\n", servo_cal_is_ready() ? "8 路完整" : "未完成（出厂默认）");
    if (servo_cal_dirty_mask() != 0) {
        printf("提示: 带 * 的通道改过但未落盘，掉电会丢失——执行 cal save\n");
    }
}

/*
 * 逐路扫动：每路从中位缓慢摆到 min 再到 max 再回中位。
 * 用于确认通道-腿位对应关系与极性，见 docs/calibration.md §2。
 */
static void cmd_cal_scan(void)
{
    if (servo_get_mode() != SERVO_MODE_CALIBRATE) {
        printf("ERR 需先 cal enter\n");
        return;
    }

    const int step_us = 20;
    const int dwell_ms = 12;

    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        servo_cal_t cal;
        servo_cal_get(ch, &cal);

        printf("ch%u (%s.%s) 扫描中…\n", ch,
               (ch % 2) ? "knee" : "hip",
               SPIDER_LEGS[ch / 2].id);
        printf("  ");

        int16_t us;
        for (us = cal.neutral_us; us >= (int16_t)cal.min_us; us -= step_us) {
            servo_pulse_raw(ch, (uint16_t)us);
            vTaskDelay(pdMS_TO_TICKS(dwell_ms));
        }
        for (us = cal.neutral_us; us <= (int16_t)cal.max_us; us += step_us) {
            servo_pulse_raw(ch, (uint16_t)us);
            vTaskDelay(pdMS_TO_TICKS(dwell_ms));
        }
        for (us = cal.neutral_us; us >= (int16_t)cal.min_us; us -= step_us) {
            servo_pulse_raw(ch, (uint16_t)us);
            vTaskDelay(pdMS_TO_TICKS(dwell_ms));
        }
        servo_pulse_raw(ch, cal.neutral_us);

        printf("OK ch%u\n", ch);
        /* 每路之间给足时间，方便肉眼观察是哪条腿 */
        vTaskDelay(pdMS_TO_TICKS(900));
    }
    printf("扫描结束。确认 8 路对应 FL/FR/RL/RR 的 hip/knee 正确后 cal save\n");
}

/*
 * leg sweep 单趟来回的时长上限（毫秒）。
 *
 * 这是安全网，不是功能约束：合法区间最宽 1000µs（1000~2000µs），
 * 三档预设里最慢的 slow 也只要 16s，碰不到 60s。反推触发 60s 所需区间
 * 分别是 fast 20000µs / normal 7500µs / slow 3750µs，均远超合法输入。
 * 它只用来拦「手填超大 step/dwell」这类误操作——因为扫描期间 CLI 忙等
 * stdin，estop 也发不进去，不能让串口长时间无响应。
 */
#define LEG_SWEEP_MAX_MS  60000

/*
 * 不指定 step/dwell 时的默认值，取 normal 档参数。
 *
 * 单独定义而非就地写字面量：preset_lookup() 与 cmd_leg_sweep() 的兜底
 * 必须同源，否则三档调整后会漂移出「无参数 ≠ normal」的不一致。
 */
#define LEG_DEFAULT_STEP_US    15
#define LEG_DEFAULT_DWELL_MS   50

/* ---- 单腿台架测试（配合 CONFIG_SPIDER_BENCH_MODE）----
 *
 * 用途：确定膝关节的机械起止角，即机械上真正能转到哪、碰壁在哪。
 *
 * 三条安全约束：
 *   1. 必须已 cal enter——标定模式是 servo_pulse_raw 的唯一合法前提
 *   2. 区间由用户显式给出，且必须落在该路已标定的 [min_us, max_us] 内；
 *      出厂默认是 1000~2000µs，正是 MG996R 的贴死区，所以先 cal set 收窄
 *   3. 不做自动堵转检测——固件无电流采样（项目内无 ADC）也无位置回读，
 *      逼近极限只能靠人眼看。leg_step 因此是单步、而非连续推进。
 */

/* 每路当前脉宽，供 leg_step 做增量基准；初值取标定中位 */
static uint16_t s_leg_pulse[SERVO_COUNT];

static bool leg_require_cal(char *cmd)
{
    if (servo_get_mode() == SERVO_MODE_CALIBRATE) {
        return true;
    }
    printf("ERR %s 需先 cal enter\n", cmd);
    return false;
}

/* 区间合法性：必须落在已标定端点内，避免扫到 MG996R 贴死区 */
static bool leg_range_ok(uint8_t ch, int a, int b)
{
    if (ch >= SERVO_COUNT) {
        printf("ERR ch%d 越界（0~%d）\n", ch, SERVO_COUNT - 1);
        return false;
    }
    servo_cal_t cal;
    servo_cal_get(ch, &cal);
    int lo = (cal.min_us < cal.max_us) ? cal.min_us : cal.max_us;
    int hi = (cal.min_us < cal.max_us) ? cal.max_us : cal.min_us;

    if (a < lo || a > hi || b < lo || b > hi) {
        printf("ERR ch%d 区间 %d~%dµs 超出已标定范围 %d~%dµs\n",
               ch, a, b, lo, hi);
        if (!servo_cal_is_ready()) {
            printf("    当前是出厂默认（尚未逐机标定），先用 cal set %d "
                   "<min> <max> <neu> <trim> <rev> 收窄\n", ch);
        }
        return false;
    }
    return true;
}

static void cmd_leg_show(void)
{
    printf("ch  腿位.关节   min   max   neu   当前\n");
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        servo_cal_t cal;
        servo_cal_get(ch, &cal);
        printf("%-3d %-6s.%-5s %-5u %-5u %-5u %uus%s\n", ch,
               SPIDER_LEGS[ch / 2].id, (ch % 2) ? "knee" : "hip",
               cal.min_us, cal.max_us, cal.neutral_us, s_leg_pulse[ch],
               servo_cal_is_dirty(ch) ? " *" : "");
    }
    printf("标定状态: %s\n", servo_cal_is_ready() ? "8 路完整" : "未完成（出厂默认）");
    /* 未落盘的通道已在上表标 *；这里给出唯一补救动作 */
    if (servo_cal_dirty_mask() != 0) {
        printf("提示: 带 * 的通道改过但未落盘，掉电会丢失——执行 cal save\n");
    }
}

/* 单步逼近：每敲一次只动 step_us，观察到阻力/到头再敲或 release */
static void cmd_leg_step(int ch, int dir, int step_us)
{
    if (!leg_require_cal("leg step")) {
        return;
    }
    if (!leg_range_ok((uint8_t)ch, s_leg_pulse[ch], s_leg_pulse[ch])) {
        return;
    }
    servo_cal_t cal;
    servo_cal_get((uint8_t)ch, &cal);
    int lo = (cal.min_us < cal.max_us) ? cal.min_us : cal.max_us;
    int hi = (cal.min_us < cal.max_us) ? cal.max_us : cal.min_us;

    int next = s_leg_pulse[ch] + dir * step_us;
    if (next < lo || next > hi) {
        printf("ERR 已到已标定边界 %dµs，不再推进（要更远需先 cal set 放宽）\n",
               (dir > 0) ? hi : lo);
        return;
    }
    if (servo_pulse_raw((uint8_t)ch, (uint16_t)next) != ESP_OK) {
        printf("ERR 写 ch%d 失败\n", ch);
        return;
    }
    s_leg_pulse[ch] = (uint16_t)next;
    printf("ch%d = %dus\n", ch, next);
}

/*
 * 命名预设：把 step/dwell 两个数字绑成一个词。
 *
 * 数字参数容易被记错顺序（写成 "1800 slow" 就会把 dwell 当成步长），
 * 而记错时 sweep_at 会静默扫错区间。预设消除了这个风险面。
 * 耗时按 600µs 区间 × 2 趟估算，实际随 from/to 跨度变化。
 */
static bool preset_lookup(const char *name, int *step_us, int *dwell_ms)
{
    if (!strcmp(name, "fast")) {
        *step_us = 20; *dwell_ms = 30;    /* 600µs 约 2s，看行程全貌 */
    } else if (!strcmp(name, "normal")) {
        *step_us = 15; *dwell_ms = 50;    /* 600µs 约 4s，中间档，也是无参数默认 */
    } else if (!strcmp(name, "slow")) {
        /* 步长 10µs ≈ 膝 0.63°/步，是肉眼可辨下限。靠 dwell 长(80ms) 细看机构
         * 反应，不是用来定位极限的——定位走 leg step（可到 1µs）。 */
        *step_us = 10; *dwell_ms = 80;    /* 600µs 约 10s */
    } else {
        return false;
    }
    return true;
}

/*
 * 计算扫到第 i 跳时的脉宽：from + i*step，超过 to_us 就钳到 to_us。
 *
 * 区间长度通常不被 step 整除，纯按整步递增会永远到不了 to_us
 * （span=600、step=250 时只到 1700），而漏掉的端点恰恰可能是机械极限所在。
 * 钳位保证末跳落在 to_us，且不会越过它。
 */
static int sweep_pulse(int from_us, int i, int step_us, int dir, int to_us)
{
    int us = from_us + i * step_us * dir;
    return ((us - to_us) * dir > 0) ? to_us : us;
}

/* 在显式区间内慢速来回扫描，用于先看清行程全貌 */
static void cmd_leg_sweep(int ch, int from_us, int to_us,
                          int step_us, int dwell_ms)
{
    if (!leg_require_cal("leg sweep")) {
        return;
    }
    if (!leg_range_ok((uint8_t)ch, from_us, to_us)) {
        return;
    }
    if (step_us <= 0) {
        /* 兜底与 normal 预设一致，使「不传 step」等价于「用 normal」 */
        step_us = LEG_DEFAULT_STEP_US;
    }
    if (dwell_ms <= 0) {
        dwell_ms = LEG_DEFAULT_DWELL_MS;
    }

    int dir = (to_us >= from_us) ? 1 : -1;
    int span = (to_us - from_us) * dir;
    if (span == 0) {
        servo_pulse_raw((uint8_t)ch, (uint16_t)from_us);
        s_leg_pulse[ch] = (uint16_t)from_us;
        printf("OK ch%d 已在 %dµs（区间为零，无需扫描）\n", ch, from_us);
        return;
    }
    /*
     * 末跳可能不满一个 step。例：区间 1200→1800、step=250 时
     * steps = 600//250 = 2，去程终点是 1700 而非 1800——差的那 100µs
     * 永远不会被扫到，而漏掉的端点恰恰可能就是机械极限所在。
     * 故末步单独补足，见 sweep_at()。
     */
    /* 跳数向上取整，末跳由 sweep_pulse 钳到 to_us */
    int steps = (span + step_us - 1) / step_us;
    /*
     * 一趟来回约 2*steps*dwell，按 LEG_SWEEP_MAX_MS 封顶。
     * 该上限是安全网而非功能约束，理由见 LEG_SWEEP_MAX_MS 的定义处。
     */
    if ((int64_t)steps * dwell_ms * 2 > LEG_SWEEP_MAX_MS) {
        printf("ERR 预计耗时 >%ds，请加大 step、减小 dwell，"
               "或改用预设 fast，或用 leg step 单步逼近\n",
               LEG_SWEEP_MAX_MS / 1000);
        return;
    }

    printf("ch%d 扫描 %d→%dµs step=%d dwell=%dms 共%d跳"
           "（扫描期间串口不响应，Ctrl-C 可能无效；到点自动结束）\n",
           ch, from_us, to_us, step_us, dwell_ms, steps + 1);

    /* 去程 0..steps，回程 steps..0；每跳都调 sweep_pulse 取得脉宽 */
    for (int pass = 0; pass < 2; pass++) {
        for (int k = 0; k <= steps; k++) {
            int i = (pass == 0) ? k : steps - k;
            int us = sweep_pulse(from_us, i, step_us, dir, to_us);
            if (servo_pulse_raw((uint8_t)ch, (uint16_t)us) != ESP_OK) {
                printf("ERR 写 ch%d 失败，已中止\n", ch);
                return;
            }
            s_leg_pulse[ch] = (uint16_t)us;
            vTaskDelay(pdMS_TO_TICKS(dwell_ms));
        }
    }
    printf("OK ch%d 扫描结束，回到 %dµs\n", ch, from_us);
}

static void cmd_leg_hold(int ch, int us)
{
    if (!leg_require_cal("leg hold")) {
        return;
    }
    if (!leg_range_ok((uint8_t)ch, us, us)) {
        return;
    }
    if (servo_pulse_raw((uint8_t)ch, (uint16_t)us) != ESP_OK) {
        printf("ERR 写 ch%d 失败\n", ch);
        return;
    }
    s_leg_pulse[ch] = (uint16_t)us;
    printf("OK ch%d 保持 %dµs（试机械阻力用；观察到了头立刻 leg release）\n",
           ch, us);
}

/* 全部回中位：见到阻力/顶住时的第一反应应该是这条 */
static void cmd_leg_release(void)
{
    if (servo_get_mode() != SERVO_MODE_CALIBRATE) {
        printf("OK 无需释放（未进标定模式）\n");
        return;
    }
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
        servo_cal_t cal;
        servo_cal_get(ch, &cal);
        servo_pulse_raw(ch, cal.neutral_us);
        s_leg_pulse[ch] = cal.neutral_us;
    }
    printf("OK 8 路已回中位\n");
}

static void print_help(void)
{
    printf(
        "机器蜘蛛 CLI\n"
        "  help                本帮助\n"
        "  ping                连通性与延迟\n"
        "\n"
        "  [标定]\n"
        "  cal show            查看 8 路标定\n"
        "  cal enter|exit      进入/退出标定模式\n"
        "  cal scan            逐路扫动，检查通道对应与极性\n"
        "  cal set <ch> <min> <max> <neu> <trim> <rev>\n"
        "  cal save|load|reset 保存/载入/恢复出厂标定\n"
        "  servo <ch> <us>     标定模式下直写脉宽\n"
        "\n"
        "  [单腿台架]（仅 SPIDER_BENCH_MODE 固件；先 cal enter）\n"
        "  leg show            8 路标定与当前脉宽\n"
        "  leg sweep <ch> <from> <to> [fast|normal|slow | step_us [dwell_ms]]\n"
        "                      在 [from,to] µs 区间内来回扫一趟\n"
        "                        fast  = step 20µs, dwell  30ms  (600µs 约  2s)\n"
        "                        normal= step 15µs, dwell  50ms  (600µs 约  4s, 默认档)\n"
        "                        slow  = step 10µs, dwell  80ms  (600µs 约 10s)\n"
        "                      定位机械极限用 leg step，不用 sweep\n"
        "  leg step  <ch> <+1|-1> [step_us]  单步逼近（找极限首选）\n"
        "  leg hold  <ch> <us>  保持指定脉宽\n"
        "  leg release          8 路回中位\n"
        "\n"
        "  [运动]（首次使用前务必完成标定）\n"
        "  pose stand|sit|rest|spawn   切换姿态\n"
        "  height <mm>         原地变高 90~150mm（摆动相执行，不拖脚）\n"
        "  gait crawl|trot     选择步态并开始行走\n"
        "  speed <0-100>       速度百分比\n"
        "  heading <deg>       目标航向\n"
        "  walk <vx> <yaw>     连续行走（vx: -1~1，yaw: -1~1）\n"
        "  walk_stop           停止行走\n"
        "  strafe <mm>         侧移宏（转向-直行-转回）\n"
        "  nudge <dx> <dy>     滑移微调（限幅 50mm）\n"
        "  lean <roll> <pitch> 差动膝姿态（限幅 ±10°）\n"
        "  estop | estop_clear 急停 / 解除\n"
        "\n"
        "  status              模式、姿态、关节角、支撑裕度\n");
}

static void handle(char *line)
{
    /* 去掉行尾 CR/LF */
    size_t n = strlen(line);
    while (n > 0 && (line[n - 1] == '\r' || line[n - 1] == '\n')) {
        line[--n] = '\0';
    }
    if (n == 0) {
        return;
    }

    char *cmd = strtok(line, " ");
    if (cmd == NULL) {
        return;
    }

    if (!strcmp(cmd, "help")) {
        print_help();

    } else if (!strcmp(cmd, "ping")) {
        printf("OK pong\n");

    } else if (!strcmp(cmd, "status")) {
        motion_state_t st;
        motion_get_state(&st);
        printf("mode=%s  servo=%s  estop=%s\n",
               motion_mode_name(st.mode),
               servo_get_mode() == SERVO_MODE_CALIBRATE ? "cal" : "normal",
               st.estop ? "YES" : "no");
        printf("gait=%s speed=%.0f%% height=%.1fmm heading=%.1f° "
               "margin=%.0fmm\n",
               st.gait == GAIT_TROT ? "trot" : "crawl",
               st.speed_pct, st.height_mm, st.heading_deg,
               st.support_margin_mm);
        for (uint8_t leg = 0; leg < SERVO_LEGS; leg++) {
            printf("  %s hip %+6.1f/%+6.1f  knee %+5.1f/%+5.1f\n",
                   SPIDER_LEGS[leg].id,
                   servo_get_target(leg, SERVO_JOINT_HIP),
                   servo_get_current(leg, SERVO_JOINT_HIP),
                   servo_get_target(leg, SERVO_JOINT_KNEE),
                   servo_get_current(leg, SERVO_JOINT_KNEE));
        }

    } else if (!strcmp(cmd, "cal")) {
        char *sub = strtok(NULL, " ");
        if (sub == NULL) {
            cmd_cal_show();
        } else if (!strcmp(sub, "show")) {
            cmd_cal_show();
        } else if (!strcmp(sub, "enter")) {
            motion_cmd_t c = {.type = CMD_CALIBRATE_MODE};
            c.params.i = 1;
            motion_enqueue(&c);
            vTaskDelay(pdMS_TO_TICKS(50));
            servo_set_mode(SERVO_MODE_CALIBRATE);
            /*
             * 台架模式下 servo_set_mode() 不再归中（保持静默），
             * 所以这里必须把 leg 的增量基准初始化成中位，否则第一次
             * leg step 会从 0µs 起跳。
             */
            for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) {
                servo_cal_t cal;
                servo_cal_get(ch, &cal);
                s_leg_pulse[ch] = cal.neutral_us;
            }
#ifdef CONFIG_SPIDER_BENCH_MODE
            printf("OK 已进入标定模式（台架模式：不归中，舵机保持静默）\n");
#else
            printf("OK 已进入标定模式（8 路归中）\n");
#endif
        } else if (!strcmp(sub, "exit")) {
            motion_cmd_t c = {.type = CMD_CALIBRATE_MODE};
            c.params.i = 0;
            motion_enqueue(&c);
            motion_set_posture(POSTURE_STAND);
            printf("OK 已退出标定模式，目标设为站立姿态\n");
        } else if (!strcmp(sub, "scan")) {
            cmd_cal_scan();
        } else if (!strcmp(sub, "save")) {
            printf("%s cal_save\n", servo_cal_save() == ESP_OK ? "OK" : "ERR");
        } else if (!strcmp(sub, "load")) {
            printf("%s cal_load\n", servo_cal_load() == ESP_OK ? "OK" : "ERR");
        } else if (!strcmp(sub, "reset")) {
            servo_cal_reset_to_factory();
            printf("OK 已恢复出厂标定（未落盘，需 cal save 才生效）\n");
        } else if (!strcmp(sub, "set")) {
            int ch, mn, mx, neu, trim, rev;
            if (sscanf(line, "%*s %*s %d %d %d %d %d %d", &ch, &mn, &mx,
                       &neu, &trim, &rev) == 6) {
                servo_cal_t cal = {
                    .min_us = (uint16_t)mn, .max_us = (uint16_t)mx,
                    .neutral_us = (uint16_t)neu, .trim_us = (int16_t)trim,
                    .reverse = (rev != 0),
                };
                esp_err_t err = servo_cal_set((uint8_t)ch, &cal);
                if (err == ESP_OK) {
                    printf("OK ch%d 已设置，需 cal save 落盘\n", ch);
                } else {
                    printf("ERR ch%d 非法: %s\n", ch, esp_err_to_name(err));
                }
            } else {
                printf("ERR 用法: cal set <ch> <min> <max> <neu> <trim> <rev>\n");
            }
        } else {
            printf("ERR 未知 cal 子命令: %s\n", sub);
        }

    } else if (!strcmp(cmd, "servo")) {
        int ch, us;
        if (servo_get_mode() != SERVO_MODE_CALIBRATE) {
            printf("ERR 需先 cal enter\n");
        } else if (sscanf(line, "%*s %d %d", &ch, &us) == 2) {
            esp_err_t err = servo_pulse_raw((uint8_t)ch, (uint16_t)us);
            if (err == ESP_OK && ch >= 0 && ch < SERVO_COUNT) {
                s_leg_pulse[ch] = (uint16_t)us;
            }
            printf("%s ch%d = %dus\n", err == ESP_OK ? "OK" : "ERR", ch, us);
        } else {
            printf("ERR 用法: servo <ch> <us>\n");
        }

    } else if (!strcmp(cmd, "leg")) {
        char *sub = strtok(NULL, " ");
        if (sub == NULL || !strcmp(sub, "show")) {
            cmd_leg_show();
        } else if (!strcmp(sub, "sweep")) {
            /*
             * 参数表：<ch> <from> <to> [preset | step_us [dwell_ms]]
             *
             * 第四、五个参数按位置取 token 再判断是数字还是预设名，
             * 而不用一条 %d %d 的 sscanf：那样「slow」会解析失败，
             * 只留下已解析的前三个数，于是静默按默认 step/dwell 执行——
             * 用户以为在用慢档，实际跑的是默认档。
             */
            int ch, from, to;
            int step = LEG_DEFAULT_STEP_US, dwell = LEG_DEFAULT_DWELL_MS;
            char *a4 = strtok(NULL, " ");
            char *a5 = strtok(NULL, " ");

            if (sscanf(line, "%*s %*s %d %d %d", &ch, &from, &to) != 3) {
                printf("ERR 用法: leg sweep <ch> <from_us> <to_us> "
                       "[preset|step_us] [dwell_ms]\n"
                       "    preset: fast | normal | slow\n");
            } else if (a4 != NULL && preset_lookup(a4, &step, &dwell)) {
                /* 预设命中；a5 若同时给出则忽略并提示，避免误解 */
                if (a5 != NULL) {
                    printf("提示: 已选预设 %s，忽略额外的 dwell \"%s\"\n",
                           a4, a5);
                }
                cmd_leg_sweep(ch, from, to, step, dwell);
            } else if (a4 != NULL) {
                /*
                 * 数字参数做上界校验：steps = span / step_us 是 int 运算，
                 * step 过大时 span/step 会被截断甚至溢出成负数，从而绕过
                 * 时长上限检查。限制到 1000µs 已远超机械分辨率需要。
                 */
                long v = atol(a4);
                if (v <= 0 || v > 1000) {
                    printf("ERR step_us 需为 1~1000（收到 \"%s\"）\n", a4);
                    return;
                }
                step = (int)v;
                if (a5 != NULL) {
                    long w = atol(a5);
                    if (w <= 0 || w > LEG_SWEEP_MAX_MS) {
                        printf("ERR dwell_ms 需为 1~%d（收到 \"%s\"）\n",
                               LEG_SWEEP_MAX_MS, a5);
                        return;
                    }
                    dwell = (int)w;
                }
                cmd_leg_sweep(ch, from, to, step, dwell);
            } else {
                cmd_leg_sweep(ch, from, to, step, dwell);
            }
        } else if (!strcmp(sub, "step")) {
            int ch, dir, step_us = 5;
            int n = sscanf(line, "%*s %*s %d %d %d", &ch, &dir, &step_us);
            if (n == 2) {
                cmd_leg_step(ch, dir, step_us);
            } else {
                printf("ERR 用法: leg step <ch> <+1|-1> [step_us]\n");
            }
        } else if (!strcmp(sub, "hold")) {
            int ch, us;
            if (sscanf(line, "%*s %*s %d %d", &ch, &us) == 2) {
                cmd_leg_hold(ch, us);
            } else {
                printf("ERR 用法: leg hold <ch> <us>\n");
            }
        } else if (!strcmp(sub, "release")) {
            cmd_leg_release();
        } else {
            printf("ERR 未知 leg 子命令: %s（help 查看用法）\n", sub);
        }

    } else if (!strcmp(cmd, "pose")) {
        char name[16];
        if (sscanf(line, "%*s %15s", name) == 1) {
            posture_id_t pid;
            if (!posture_from_name(name, &pid)) {
                printf("ERR 未知姿态: %s\n", name);
            } else {
                motion_set_posture((int)pid);
                printf("OK 姿态 -> %s（%.0fmm，摆动相执行约需 1 个周期）\n",
                       name, posture_get(pid)->height_mm);
            }
        } else {
            printf("ERR 用法: pose stand|sit|rest|spawn\n");
        }

    } else if (!strcmp(cmd, "height")) {
        float mm;
        if (sscanf(line, "%*s %f", &mm) == 1) {
            motion_set_height(mm);
            printf("OK 目标高度 %.0fmm（范围 %.0f~%.0f）\n", mm,
                   SPIDER_HEIGHT_MIN_MM, SPIDER_HEIGHT_MAX_MM);
        } else {
            printf("ERR 用法: height <mm>\n");
        }

    } else if (!strcmp(cmd, "gait")) {
        char g[8];
        if (sscanf(line, "%*s %7s", g) == 1) {
            if (!strcmp(g, "crawl") || !strcmp(g, "trot")) {
                motion_set_gait(!strcmp(g, "trot") ? GAIT_TROT : GAIT_CRAWL);
                if (!strcmp(g, "trot")) {
                    printf("OK trot（高速上限 %0.f%%，防翻红线）\n",
                           SPIDER_TROT_MAX_SPEED_PCT);
                } else {
                    printf("OK crawl\n");
                }
            } else {
                printf("ERR 用法: gait crawl|trot\n");
            }
        } else {
            printf("ERR 用法: gait crawl|trot\n");
        }

    } else if (!strcmp(cmd, "speed")) {
        float pct;
        if (sscanf(line, "%*s %f", &pct) == 1) {
            motion_set_speed(pct);
            printf("OK speed %.0f%%\n", pct);
        } else {
            printf("ERR 用法: speed <0-100>\n");
        }

    } else if (!strcmp(cmd, "heading")) {
        float deg;
        if (sscanf(line, "%*s %f", &deg) == 1) {
            motion_cmd_t c = {.type = CMD_DRIVE};
            c.params.f[2] = 0.0f;
            motion_enqueue(&c);
            motion_stop();
            printf("OK heading %.0f°（用 walk 的 yaw 控制转向更直观）\n", deg);
        } else {
            printf("ERR 用法: heading <deg>\n");
        }

    } else if (!strcmp(cmd, "walk")) {
        float vx, yaw;
        if (sscanf(line, "%*s %f %f", &vx, &yaw) == 2) {
            motion_drive(vx, 0.0f, yaw);
            printf("OK walk vx=%.2f yaw=%.2f（walk_stop 停止）\n", vx, yaw);
        } else {
            printf("ERR 用法: walk <vx> <yaw>\n");
        }

    } else if (!strcmp(cmd, "walk_stop")) {
        motion_stop();
        printf("OK 已停止\n");

    } else if (!strcmp(cmd, "strafe")) {
        float mm;
        if (sscanf(line, "%*s %f", &mm) == 1) {
            if (fabsf(mm) > SPIDER_STRAFE_MAX_MM) {
                printf("ERR 超过单次上限 %.0fmm\n", SPIDER_STRAFE_MAX_MM);
            } else {
                motion_strafe(mm);
                printf("OK 侧移宏 %.0fmm（转向-直行-转回，约 %.0fs）\n",
                       mm, fabsf(mm) / 20.0f + 3.0f);
            }
        } else {
            printf("ERR 用法: strafe <mm>\n");
        }

    } else if (!strcmp(cmd, "nudge")) {
        float dx, dy;
        if (sscanf(line, "%*s %f %f", &dx, &dy) == 2) {
            if (fabsf(dx) > SPIDER_NUDGE_MAX_MM || fabsf(dy) > SPIDER_NUDGE_MAX_MM) {
                printf("ERR 超过限幅 %.0fmm\n", SPIDER_NUDGE_MAX_MM);
            } else {
                motion_nudge(dx, dy);
                printf("OK 微调 (%.0f, %.0f)mm（受控打滑，依赖地面摩擦）\n", dx, dy);
            }
        } else {
            printf("ERR 用法: nudge <dx> <dy>\n");
        }

    } else if (!strcmp(cmd, "lean")) {
        float r, p;
        if (sscanf(line, "%*s %f %f", &r, &p) == 2) {
            motion_lean(r, p);
            printf("OK lean roll=%.1f° pitch=%.1f°（限幅 ±%.0f°）\n",
                   r, p, SPIDER_LEAN_MAX_DEG);
        } else {
            printf("ERR 用法: lean <roll> <pitch>\n");
        }

    } else if (!strcmp(cmd, "estop")) {
        motion_estop();
        printf("OK 急停\n");

    } else if (!strcmp(cmd, "estop_clear")) {
        printf("%s estop_clear\n", motion_estop_clear() == ESP_OK ? "OK" : "ERR");

    } else {
        printf("ERR 未知命令: %s（help 查看用法）\n", cmd);
    }
}

static void cli_task(void *arg)
{
    (void)arg;
    char line[128];

    ESP_LOGI(TAG, "CLI 就绪（help 查看命令）。机器人请架空后执行 cal scan");

    while (1) {
        if (fgets(line, sizeof(line), stdin) != NULL) {
            handle(line);
            printf("\n");
            fflush(stdout);
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

esp_err_t servo_cli_start(void)
{
    xTaskCreate(cli_task, "cli", 4096, NULL, 3, NULL);
    return ESP_OK;
}

#else /* CONFIG_SPIDER_DEBUG_CLI */

esp_err_t servo_cli_start(void)
{
    return ESP_OK;   /* CLI 关闭（Kconfig） */
}

#endif /* CONFIG_SPIDER_DEBUG_CLI */