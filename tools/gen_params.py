#!/usr/bin/env python3
"""params.yaml -> spider_params.h

tools/gait_sim/params.yaml 是几何/步态/时序参数的唯一数据源。
本脚本把它编译成固件头文件，并在生成时校验派生量的自洽性
（见 docs/gait_design.md 的几何关系与 §9 验收判据）。

用法:
    python3 tools/gen_params.py                 # 生成到 components/kinematics/include/
    python3 tools/gen_params.py --check         # 只校验，不写文件
    python3 tools/gen_params.py --out PATH      # 指定输出路径

需要: pyyaml
"""

import argparse
import math
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("需要 pyyaml: pip3 install pyyaml")

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "tools" / "gait_sim" / "params.yaml"
DEFAULT_OUT = REPO / "components" / "kinematics" / "include" / "spider_params.h"

FLOAT_FMT = "{:.4f}"


class Checker:
    """收集校验结果；任一 FAIL 则不写文件，WARN 不阻断。"""

    def __init__(self) -> None:
        self.rows: list[tuple[str, str, str]] = []

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        self.rows.append((name, "PASS" if ok else "FAIL", detail))
        return ok

    def warn(self, name: str, detail: str = "") -> None:
        self.rows.append((name, "WARN", detail))

    def report(self) -> bool:
        width = max(len(n) for n, _, _ in self.rows)
        all_ok = True
        for name, status, detail in self.rows:
            if status == "FAIL":
                all_ok = False
            tail = f"  {detail}" if detail else ""
            print(f"[{status}] {name:<{width}}{tail}")
        return all_ok


def leg_height(l1: float, l2: float, alpha: float, beta: float) -> float:
    """站立高度 H(beta) = L1*sin(alpha) + L2*sin(beta)"""
    return l1 * math.sin(math.radians(alpha)) + l2 * math.sin(math.radians(beta))


def leg_radius(l1: float, l2: float, alpha: float, beta: float) -> float:
    """足端水平半径 R(beta) = L1*cos(alpha) + L2*cos(beta)"""
    return l1 * math.cos(math.radians(alpha)) + l2 * math.cos(math.radians(beta))


def derive_beta(l1: float, l2: float, alpha: float, height: float) -> float:
    """反解 beta = asin((H - L1*sin(alpha)) / L2)"""
    s = (height - l1 * math.sin(math.radians(alpha))) / l2
    s = max(-1.0, min(1.0, s))
    return math.degrees(math.asin(s))


def support_polygon(d: dict) -> tuple[bool, list[tuple[float, float]], str]:
    """按 FL,FR,RR,RL 顺序取四足端，检查凸性与重心内含。

    返回 (是否合法, 顶点列表, 说明)。
    配错 psi_deg 会让多边形非凸或重心落在外面——那种配置机器人静态就站不稳。
    """
    r = d["legs"][0]["r_mm"]
    pts: dict[str, tuple[float, float]] = {}
    for ld in d["legs"]:
        hx, hy = ld["hip"]
        a = math.radians(ld["psi_deg"])
        pts[ld["id"]] = (hx + r * math.cos(a), hy + r * math.sin(a))

    poly = [pts["FL"], pts["FR"], pts["RR"], pts["RL"]]

    # 叉积同号 => 凸
    cross = []
    for i in range(4):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % 4]
        x3, y3 = poly[(i + 2) % 4]
        cross.append((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2))
    convex = all(v > 0 for v in cross) or all(v < 0 for v in cross)

    # 射线法判内含（机身中心 = 原点）
    px = py = 0.0
    inside = False
    j = 3
    for i in range(4):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i

    margin = 0.0
    if convex and inside:
        # 重心到各边的最小距离，作为静态稳定裕度（mm）
        for i in range(4):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % 4]
            dx, dy = x2 - x1, y2 - y1
            L = math.hypot(dx, dy)
            if L > 0:
                margin = min(margin if margin else 1e9, abs(dx * py - dy * px + x2 * y1 - y2 * x1) / L)
    return convex and inside, poly, f"margin={margin:.1f}mm"


def three_leg_margin(d: dict) -> float:
    """最差一组三足支撑的静态裕度（mm）

    中心对称四足布局的裕度恒为 0（见 params.yaml 说明），
    故此处只报告数值，是否可接受由 docs/gait_design.md 的判据决定。
    """
    r = d["legs"][0]["r_mm"]
    pts = []
    for ld in d["legs"]:
        hx, hy = ld["hip"]
        a = math.radians(ld["psi_deg"])
        pts.append((hx + r * math.cos(a), hy + r * math.sin(a)))

    worst = float("inf")
    for i in range(4):
        tri = [pts[j] for j in range(4) if j != i]
        for k in range(3):
            x1, y1 = tri[k]
            x2, y2 = tri[(k + 1) % 3]
            dx, dy = x2 - x1, y2 - y1
            ln = math.hypot(dx, dy)
            if ln < 1e-6:
                continue
            worst = min(worst, abs(dx * (0.0 - y1) - dy * (0.0 - x1)) / ln)
    return 0.0 if worst == float("inf") else worst


def validate(d: dict) -> Checker:
    c = Checker()
    leg = d["leg"]
    l1, l2 = leg["L1_mm"], leg["L2_mm"]
    alpha, beta = leg["alpha_deg"], leg["beta_cruise_deg"]

    h_cruise = leg_height(l1, l2, alpha, beta)
    r_cruise = leg_radius(l1, l2, alpha, beta)

    c.check("巡航高度 115~125mm", 115.0 <= h_cruise <= 125.0, f"H={h_cruise:.1f}mm")

    lo = d["limits"]["beta_min_deg"]
    hi = d["limits"]["beta_max_deg"]
    c.check("巡航 beta 在软限位内", lo < beta < hi, f"[{lo}, {hi}]")
    c.check("软限位 beta_min 对应 H=90",
            abs(leg_height(l1, l2, alpha, lo) - 90.0) < 1.5,
            f"{leg_height(l1, l2, alpha, lo):.1f}mm")
    c.check("软限位 beta_max 对应 H=150",
            abs(leg_height(l1, l2, alpha, hi) - 150.0) < 1.5,
            f"{leg_height(l1, l2, alpha, hi):.1f}mm")

    hr = d["posture"]["height_range_mm"]
    c.check("height_range 与软限位一致",
            abs(leg_height(l1, l2, alpha, lo) - hr[0]) < 1.5
            and abs(leg_height(l1, l2, alpha, hi) - hr[1]) < 1.5,
            f"{hr}")

    for legdef in d["legs"]:
        c.check(f"腿 {legdef['id']} r_mm == R(beta_cruise)",
                abs(legdef["r_mm"] - r_cruise) < 0.2,
                f"{legdef['r_mm']} vs {r_cruise:.1f}")

    ok_poly, poly, note = support_polygon(d)
    c.check("四足支撑多边形凸且含重心", ok_poly,
            note if ok_poly else f"顶点 {[(round(x), round(y)) for x, y in poly]}")

    # 三足支撑裕度：中心对称时必然为 0，故仅报告不阻断
    m3 = three_leg_margin(d)
    if m3 < 15.0:
        c.warn("三足支撑裕度偏低", f"{m3:.1f}mm（<15mm，对角方向接近翻转临界）")
    else:
        c.check("三足支撑裕度 >= 15mm", True, f"{m3:.1f}mm")

    for name in ("crawl", "trot"):
        g = d["gait"][name]
        speed = g["stride_mm"] / g["period_s"]
        sweep = math.degrees(math.asin(g["stride_mm"] / (2.0 * r_cruise)))
        c.check(f"{name} 速度 35~40mm/s", 35.0 <= speed <= 40.0, f"{speed:.1f}mm/s")
        c.check(f"{name} 髋半摆不越限", sweep < d["limits"]["hip_deg"],
                f"±{sweep:.1f}deg")
        c.check(f"{name} 周期 >= 1.5s", g["period_s"] >= 1.5, f"{g['period_s']}s")
        c.check(f"{name} 抬脚 >= 20mm", g["lift_mm"] >= 20.0, f"{g['lift_mm']}mm")
        # 抬脚高度受膝软限位约束：摆到 H - lift 不得低于 height_min
        max_lift = h_cruise - d["posture"]["height_range_mm"][0]
        c.check(f"{name} 抬脚可达 (<= {max_lift:.0f}mm)",
                g["lift_mm"] <= max_lift + 0.01,
                f"{g['lift_mm']}mm vs 上限 {max_lift:.1f}mm")

    # 冻结状态是告警而非阻断：几何自洽才能编译，冻结是"能否出打印件"的放行条件
    if d["meta"]["freeze_status"] != "frozen":
        c.warn("参数未冻结", f"{d['meta']['freeze_status']}"
                            "（gait_sim 扫参通过后方可出打印件，见 docs/gait_design.md §9）")

    return c


def emit(d: dict, out: Path) -> None:
    leg = d["leg"]
    lim = d["limits"]
    pos = d["posture"]
    g = d["gait"]
    saf = d["safety"]
    slip = d["slip_primitives"]
    turn = d["turn"]
    st = d["strafe_macro"]
    el = d["electrical"]

    def f(x: float) -> str:
        return FLOAT_FMT.format(x)

    sections: list[tuple[str, list[tuple[str, str]]]] = []

    def sec(title: str, entries: list[tuple[str, str]]) -> None:
        sections.append((title, entries))

    sec("机身", [
        ("SPIDER_BODY_LENGTH_MM", f"{f(d['body']['length_mm'])}f"),
        ("SPIDER_BODY_WIDTH_MM", f"{f(d['body']['width_mm'])}f"),
        ("SPIDER_LAYER_GAP_MM", f"{f(d['body']['layer_gap_mm'])}f"),
        ("SPIDER_TOTAL_MASS_G", f"{f(d['body']['total_mass_g'])}f"),
    ])

    sec("腿几何", [
        ("SPIDER_L1_MM", f"{f(leg['L1_mm'])}f"),
        ("SPIDER_L2_MM", f"{f(leg['L2_mm'])}f"),
        ("SPIDER_ALPHA_DEG", f"{f(leg['alpha_deg'])}f"),
        ("SPIDER_BETA_CRUISE_DEG", f"{f(leg['beta_cruise_deg'])}f"),
        ("SPIDER_SPLAY_DEG", f"{f(leg['splay_deg'])}f"),
        ("SPIDER_CRUISE_HEIGHT_MM", f"{f(pos['stand_height_mm'])}f"),
        ("SPIDER_CRUISE_RADIUS_MM",
         f"{f(leg_radius(leg['L1_mm'], leg['L2_mm'], leg['alpha_deg'], leg['beta_cruise_deg']))}f"),
    ])

    sec("软限位", [
        ("SPIDER_HIP_LIMIT_DEG", f"{f(lim['hip_deg'])}f"),
        ("SPIDER_BETA_MIN_DEG", f"{f(lim['beta_min_deg'])}f"),
        ("SPIDER_BETA_MAX_DEG", f"{f(lim['beta_max_deg'])}f"),
        ("SPIDER_MAX_RATE_DEG_S", f"{f(lim['max_rate_deg_s'])}f"),
        ("SPIDER_SERVO_RATED_DEG_S", f"{f(lim['servo_rated_deg_s'])}f"),
    ])

    sec("姿态高度", [
        ("SPIDER_STAND_HEIGHT_MM", f"{f(pos['stand_height_mm'])}f"),
        ("SPIDER_HEIGHT_MIN_MM", f"{f(pos['height_range_mm'][0])}f"),
        ("SPIDER_HEIGHT_MAX_MM", f"{f(pos['height_range_mm'][1])}f"),
        ("SPIDER_SIT_HEIGHT_MM", f"{f(pos['sit_height_mm'])}f"),
        ("SPIDER_REST_HEIGHT_MM", f"{f(pos['rest_height_mm'])}f"),
        ("SPIDER_SPAWN_HEIGHT_MM", f"{f(pos['spawn_height_mm'])}f"),
    ])

    gait_macros = []
    for name in ("crawl", "trot"):
        up = name.upper()
        gg = g[name]
        gait_macros += [
            (f"SPIDER_{up}_STRIDE_MM", f"{f(gg['stride_mm'])}f"),
            (f"SPIDER_{up}_PERIOD_S", f"{f(gg['period_s'])}f"),
            (f"SPIDER_{up}_LIFT_MM", f"{f(gg['lift_mm'])}f"),
            (f"SPIDER_{up}_DUTY_SWING", f"{f(gg['duty_swing'])}f"),
        ]
    gait_macros += [
        ("SPIDER_SPEED_MAX_PCT", f"{f(g['speed']['max_pct'])}f"),
        ("SPIDER_TROT_MAX_SPEED_PCT", f"{f(g['speed']['trot_max_speed_pct'])}f"),
    ]
    swing = [(f"SPIDER_SWING_{k.upper()}",
              f"{f(g['swing_profile'][k][0])}, {f(g['swing_profile'][k][1])}")
             for k in ("lift", "reach", "place", "push")]
    sec("步态", gait_macros)
    sec("摆动相四段（周期占比）", swing)

    sec("转向与侧移", [
        ("SPIDER_TURN_INPLACE_MAX_DEG", f"{f(turn['in_place_max_deg'])}f"),
        ("SPIDER_YAW_RATE_MAX_DEG_S", f"{f(turn['max_yaw_rate_deg_s'])}f"),
        ("SPIDER_FOOT_PLACE_BUDGET_DEG", f"{f(turn['foot_placement_budget_deg'])}f"),
        ("SPIDER_STRAFE_MAX_MM", f"{f(st['max_mm'])}f"),
    ])

    sec("滑移原语", [
        ("SPIDER_NUDGE_MAX_MM", f"{f(slip['nudge_max_mm'])}f"),
        ("SPIDER_LEAN_MAX_DEG", f"{f(slip['lean_max_deg'])}f"),
        ("SPIDER_SKID_TURN_MAX_DEG", f"{f(slip['skid_turn_max_deg'])}f"),
        ("SPIDER_RATCHET_SLOW_S", f"{f(slip['ratchet']['slow_phase_s'])}f"),
        ("SPIDER_RATCHET_FAST_S", f"{f(slip['ratchet']['fast_phase_s'])}f"),
        ("SPIDER_RATCHET_HIP_DELTA_DEG", f"{f(slip['ratchet']['hip_delta_deg'])}f"),
    ])

    sec("安全", [
        ("SPIDER_HEARTBEAT_TIMEOUT_MS", str(saf["heartbeat_timeout_ms"])),
        ("SPIDER_ESTOP_RESPONSE_MS", str(saf["estop_response_ms"])),
        ("SPIDER_MOTION_TICK_MS", str(saf["move_task_tick_ms"])),
        ("SPIDER_MOTION_TASK_PRIO", str(saf["motion_task_priority"])),
        ("SPIDER_WEB_TASK_PRIO", str(saf["web_task_priority"])),
        ("SPIDER_CMD_QUEUE_LEN", str(saf["estop_command_queue_len"])),
    ])

    sec("电气", [
        ("SPIDER_PWM_FREQ_HZ", str(el["pca9685"]["pwm_freq_hz"])),
        ("SPIDER_PWM_RES_BITS", str(el["pca9685"]["pwm_res_bits"])),
        ("SPIDER_PCA9685_ADDR", f"0x{el['pca9685']['addr']:02x}"),
        ("SPIDER_SERVO_MIN_US", str(el["servo"]["min_pulse_us"])),
        ("SPIDER_SERVO_MAX_US", str(el["servo"]["max_pulse_us"])),
        ("SPIDER_SERVO_NEUTRAL_US", str(el["servo"]["neutral_us"])),
        ("SPIDER_SERVO_EDGE_MARGIN_US", str(el["servo"]["endpoint_margin_us"])),
        ("SPIDER_SERVO_COUNT", str(d["servo_map"]["channels"])),
    ])

    pad = max(len(n) for _, entries in sections for n, _ in entries) + 2

    L: list[str] = []
    a = L.append
    a("/* 自动生成，请勿手改 */")
    a("/* 源: tools/gait_sim/params.yaml  生成器: tools/gen_params.py */")
    a("/* 改动参数请编辑 params.yaml 后重新运行生成器 */")
    a("#pragma once")
    a("")
    a("/* 该头文件亦被主机端单元测试直接包含，故自带基础类型依赖 */")
    a("#include <stdint.h>")
    a("")
    a("#ifdef __cplusplus")
    a('extern "C" {')
    a("#endif")
    a("")
    a(f'#define SPIDER_PARAMS_VERSION{" " * (pad - len("SPIDER_PARAMS_VERSION"))}'
      f'{d["meta"]["version"]}')
    a(f'#define SPIDER_PARAMS_DESC{" " * (pad - len("SPIDER_PARAMS_DESC"))}'
      f'"{d["meta"]["description"]}"')
    a("")

    for title, entries in sections:
        a(f"/* ---- {title} ---- */")
        for name, value in entries:
            a(f"#define {name}{' ' * (pad - len(name))}{value}")
        a("")

    a("/* ---- 腿位表（索引 = leg_index，通道 = index*2 为髋）---- */")
    a("typedef struct {")
    a("    const char *id;    /* 腿标识：FL / FR / RL / RR */")
    a("    float    psi_deg;  /* 腿平面方位角，X 轴逆时针 */")
    a("    float    r_mm;     /* 巡航足端水平半径 */")
    a("    uint16_t ch_hip;   /* PCA9685 通道 */")
    a("    uint16_t ch_knee;  /* PCA9685 通道 */")
    a("    float    hip_x_mm; /* 髋安装点机身坐标 X */")
    a("    float    hip_y_mm; /* 髋安装点机身坐标 Y */")
    a("} spider_leg_t;")
    a("")
    a(f"#define SPIDER_LEG_COUNT {len(d['legs'])}")
    a("static const spider_leg_t SPIDER_LEGS[SPIDER_LEG_COUNT] = {")
    # mapping 条目为 [channel, leg_id, joint]，按腿聚合成 (hip, knee)
    smap: dict[str, dict[str, int]] = {}
    for ch, leg_id, joint in d["servo_map"]["mapping"]:
        smap.setdefault(leg_id, {})[joint] = ch
    for ld in d["legs"]:
        ch_h = smap[ld["id"]]["hip"]
        ch_k = smap[ld["id"]]["knee"]
        hx, hy = ld["hip"]
        a(f'    {{ "{ld["id"]}", {f(ld["psi_deg"])}f, {f(ld["r_mm"])}f, '
          f"{ch_h}, {ch_k}, {f(hx)}f, {f(hy)}f }},")
    a("};")
    a("")
    a("#ifdef __cplusplus")
    a("}")
    a("#endif")

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(L) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true", help="只校验不写文件")
    ap.add_argument("--out", type=Path, default=None, help="输出头文件路径")
    ap.add_argument("--src", type=Path, default=SRC, help="params.yaml 路径")
    args = ap.parse_args()

    d = yaml.safe_load(args.src.read_text(encoding="utf-8"))

    print(f"参数校验: {args.src}")
    ok = validate(d).report()
    print()

    if not ok:
        print("校验未通过，未写出头文件。请先修正 params.yaml。")
        return 1

    if args.check:
        print("校验通过 (--check，未写文件)")
        return 0

    out = args.out or DEFAULT_OUT
    emit(d, out)
    print(f"已生成 {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())