#!/usr/bin/env python3
"""
preview.py — 步态预览与扫参（打印前必过）

出 3D 动画预览四腿运动，并按 docs/gait_design.md §9 逐项检查验收判据。
这个工具的价值在于：**看到的就是实机将执行的**——参数与固件同源于
params.yaml，不存在换算偏差。

用法:
    python3 tools/gait_sim/preview.py                     # crawl 预览 + 检查
    python3 tools/gait_sim/preview.py --gait trot
    python3 tools/gait_sim/preview.py --no-anim           # 只跑检查
    python3 tools/gait_sim/preview.py --check-only        # 只打印验收报告
    python3 tools/gait_sim/preview.py --sweep alpha       # 扫参
    python3 tools/gait_sim/preview.py --set alpha=32 beta_cruise=41 \
                                       L1=105 L2=108 --check-only

依赖: numpy, matplotlib, pyyaml
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from gait import GaitSim, Sample           # noqa: E402
from ik import (Geometry, beta_from_height, height_from_beta,   # noqa: E402
                radius_from_beta, hip_sweep_from_stride)

DT = 0.005


# ----------------------------------------------------------------- 验收检查

class Report:
    def __init__(self) -> None:
        self.rows: list[tuple[str, bool, bool, str]] = []

    def add(self, name: str, ok: bool, detail: str, warn: bool = False) -> None:
        self.rows.append((name, ok, warn, detail))

    def show(self) -> bool:
        w = max(len(r[0]) for r in self.rows)
        all_ok = True
        for name, ok, warn, detail in self.rows:
            if not ok and not warn:
                all_ok = False
            tag = "PASS" if ok else ("WARN" if warn else "FAIL")
            print(f"[{tag}] {name:<{w}}  {detail}")
        return all_ok


def check(p: dict, gait_type: str) -> bool:
    geom = Geometry.from_params(p)
    cfg = p["gait"][gait_type]
    stride = float(cfg["stride_mm"])
    period = float(cfg["period_s"])
    lift = float(cfg["lift_mm"])
    rep = Report()

    # ---- 几何 ----
    h = height_from_beta(geom, geom.beta_cruise_deg)
    rep.add("巡航高度 115~125mm", 115.0 <= h <= 125.0, f"{h:.1f}mm")

    h_min = float(p["posture"]["height_range_mm"][0])
    h_max = float(p["posture"]["height_range_mm"][1])
    lim = p["limits"]
    rep.add("软限位 beta 下限 -> H=90",
            abs(height_from_beta(geom, float(lim["beta_min_deg"])) - 90.0) < 1.5,
            f"{height_from_beta(geom, float(lim['beta_min_deg'])):.1f}mm")
    rep.add("软限位 beta 上限 -> H=150",
            abs(height_from_beta(geom, float(lim["beta_max_deg"])) - 150.0) < 1.5,
            f"{height_from_beta(geom, float(lim['beta_max_deg'])):.1f}mm")
    rep.add("巡航 beta 在软限位内",
            float(lim["beta_min_deg"]) < geom.beta_cruise_deg < float(lim["beta_max_deg"]),
            f"[{lim['beta_min_deg']}, {lim['beta_max_deg']}]")

    # ---- 抬脚可行性 ----
    max_lift = h - h_min
    rep.add("抬脚高度可达", lift <= max_lift + 0.01,
            f"{lift:.0f}mm <= 上限 {max_lift:.0f}mm")

    # ---- 步幅 / 速度 ----
    r = radius_from_beta(geom, geom.beta_cruise_deg)
    sweep = hip_sweep_from_stride(geom, stride, r)
    rep.add("髋半摆不越 ±60°", abs(sweep) <= geom.hip_limit_deg, f"±{sweep:.1f}°")

    speed = stride / period
    rep.add("巡航速度 35~40mm/s", 35.0 <= speed <= 40.0, f"{speed:.1f}mm/s")
    rep.add("周期 >= 1.5s（996R 带载）", period >= 1.5, f"{period}s")

    # ---- 三足支撑裕度 ----
    m3 = three_leg_margin(p)
    rep.add("三足支撑裕度 >= 15mm", m3 >= 15.0, f"{m3:.1f}mm",
            warn=(m3 < 15.0))

    # ---- 仿真实测 ----
    sim = GaitSim.from_params(p, gait_type, speed_pct=60.0)
    samples = sim.run(2 * period, DT)

    all_betas = [c.knee_beta_deg for s in samples for c in s.cmds]
    min_h, max_h = min(all_betas), max(all_betas)
    rep.add("膝角全周期在软限位内",
            min_h >= float(lim["beta_min_deg"]) - 0.01
            and max_h <= float(lim["beta_max_deg"]) + 0.01,
            f"[{min_h:.1f}, {max_h:.1f}]")

    max_hip = max(s.max_hip_deg for s in samples)
    rep.add("髋角全周期不越限", max_hip <= geom.hip_limit_deg + 0.01,
            f"{max_hip:.1f}°")

    max_lift_actual = max(max(s.stance_foot_h) for s in samples)
    rep.add("抬脚高度达到设定值", max_lift_actual >= lift * 0.9,
            f"{max_lift_actual:.1f}mm / 设定 {lift:.0f}mm")

    # 穿地检查：摆动足不得低于地面
    ground_ok = True
    for s in samples:
        for i, c in enumerate(s.cmds):
            if c.in_swing and s.stance_foot_h[i] < -0.5:
                ground_ok = False
    rep.add("摆动足不穿地", ground_ok, "")

    # 支撑足零滑移（模型层）
    slip = stance_slip(sim, period)
    limit = float(p["slip_budget"]["crawl_max_mm_per_cycle"]
                  if gait_type == "crawl"
                  else p["slip_budget"]["trot_max_mm_per_cycle"])
    rep.add("支撑足模型层滑移 == 0", slip == 0.0, f"{slip:.6f}mm/周期",
            warn=False)
    rep.add(f"物理层滑移预算 <= {limit:.0f}mm", slip <= limit, f"{slip:.3f}mm")

    # 支撑裕度
    margins = [s.support_margin_mm for s in samples]
    rep.add("全周期支撑裕度 > 0", min(margins) > 0.0,
            f"min {min(margins):.1f} / max {max(margins):.1f} mm")

    # 速度实测
    dist = math.hypot(samples[-1].body_x - samples[0].body_x,
                      samples[-1].body_y - samples[0].body_y)
    dur = len(samples) * DT
    v = dist / dur
    expect = stride / period * 0.6
    rep.add("实测速度 == stride/period×speed%", abs(v - expect) < 0.5,
            f"{v:.1f} vs 期望 {expect:.1f} mm/s")

    print()
    ok = rep.show()
    print()
    print("=" * 62)
    print(f"  {gait_type} 验收: {'全部通过' if ok else '存在未通过项'}")
    print("=" * 62)
    return ok


def three_leg_margin(p: dict) -> float:
    """最差一组三足支撑的静态裕度（mm）"""
    import itertools
    r = p["legs"][0]["r_mm"]
    pts = []
    for L in p["legs"]:
        hx, hy = L["hip"]
        a = math.radians(L["psi_deg"])
        pts.append((hx + r * math.cos(a), hy + r * math.sin(a)))
    worst = float("inf")
    for skip in range(4):
        tri = [pts[j] for j in range(4) if j != skip]
        for k in range(3):
            x1, y1 = tri[k]
            x2, y2 = tri[(k + 1) % 3]
            dx, dy = x2 - x1, y2 - y1
            ln = math.hypot(dx, dy)
            if ln < 1e-6:
                continue
            worst = min(worst, abs(dx * (0.0 - y1) - dy * (0.0 - x1)) / ln)
    return 0.0 if worst == float("inf") else worst


def stance_slip(sim: GaitSim, period: float) -> float:
    """支撑足在支撑期内的最大位移（mm/周期）

    模型层应恒为 0：支撑腿足端锚定不动，机身从其上方移过。
    非 0 即代码 bug，不是「误差」。
    """
    n = int(period / DT)
    prev = list(sim.foot)
    worst = 0.0
    for _ in range(n):
        sim.step(DT)
        for leg in range(4):
            if not sim.leg_swing[leg]:
                d = math.hypot(sim.foot[leg][0] - prev[leg][0],
                               sim.foot[leg][1] - prev[leg][1])
                worst = max(worst, d)
        prev = list(sim.foot)
    return worst


# ----------------------------------------------------------------- 3D 预览

def animate(p: dict, gait_type: str, save: str | None = None) -> None:
    import numpy as np
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    geom = Geometry.from_params(p)
    sim = GaitSim.from_params(p, gait_type, speed_pct=60.0)
    n = int(2 * sim.cfg.period_s / DT)
    samples = sim.run(2 * sim.cfg.period_s, DT)

    fig = plt.figure(figsize=(11, 6))
    ax = fig.add_subplot(111, projection="3d")

    body = np.array([[-100, -60, 0], [100, -60, 0], [100, 60, 0], [-100, 60, 0],
                     [-100, -60, 0]])
    stride = max(1, n // 240)

    def draw(k: int):
        s = samples[k]
        ax.clear()
        yaw = math.radians(s.yaw_deg)
        cy, sy = math.cos(yaw), math.sin(yaw)

        def to_world(x, y):
            return s.body_x + x * cy - y * sy, s.body_y + x * sy + y * cy

        bx, by = to_world(0, 0)
        h = height_from_beta(geom, geom.beta_cruise_deg)
        wx = [to_world(px, py)[0] for px, py in body]
        wy = [to_world(px, py)[1] for px, py in body]
        ax.plot(wx, wy, [h] * len(wx), "k-", lw=3, label="body")
        ax.scatter([bx], [by], [h], c="k", marker="s")  # 机身中心（重心近似）

        colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
        for leg in range(4):
            L = sim.legs[leg]
            hx, hy = L["hip"]
            whx, why = to_world(hx, hy)
            fx, fy = s.foot_xy[leg]
            c = s.cmds[leg]
            b = beta_from_height(geom, c.height_mm)
            r_shin = geom.L2
            beta = math.radians(b)
            # 膝位置：髋下方沿腿平面方向
            kdir = math.radians(float(L["psi_deg"]))
            kx = whx + math.cos(kdir) * r_shin * math.cos(beta) * 0.5
            ky = why + math.sin(kdir) * r_shin * math.cos(beta) * 0.5
            kz = h - r_shin * math.sin(beta) * 0.5
            ax.plot([whx, kx, fx], [why, ky, fy], [h, kz, 0.0],
                    color=colors[leg], lw=2.5,
                    ls="--" if c.in_swing else "-",
                    label=f"{L['id']}{' (swing)' if c.in_swing else ''}")
            ax.scatter([fx], [fy], [0.0], color=colors[leg], marker="o")

        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.set_zlim(0, max(h * 1.4, 60))
        ax.set_title(f"{gait_type}  t={k * DT:.2f}s  "
                     f"margin={s.support_margin_mm:.0f}mm  "
                     f"lift={max(s.stance_foot_h):.0f}mm")
        ax.legend(fontsize=7, loc="upper right")
        ax.set_box_aspect((2, 2, 1))

    def loop(i):
        draw(i)
        if i + stride < n:
            timer = fig.canvas.new_timer(interval=20)
            timer.add_callback(lambda: (timer.stop(), loop(i + stride)))
            timer.start()

    if save:
        for k in range(0, n, max(1, n // 6)):
            draw(k)
            plt.savefig(f"{save}_{k // max(1, n // 6)}.png", dpi=110)
        print(f"已保存 {save}_*.png")
    else:
        loop(0)
        plt.show()


# ----------------------------------------------------------------- 扫参

def sweep(p: dict, keys: list[str], gait_type: str = "crawl") -> None:
    """在 params.yaml 声明的扫参区间内逐点评估，报告可行组合"""
    leg = p["leg"]
    ranges = leg["sweep"]
    print("扫参区间（来自 params.yaml leg.sweep）:")
    for k in keys:
        if k in ranges:
            lo, hi = ranges[k]
            print(f"  {k}: {lo} ~ {hi}")
    print()
    print(f"{'组合':<52}{'巡航H':>8}{'步幅角':>9}{'速度':>8}{'裕度':>8}  判定")
    print("-" * 92)

    def enum(k):
        if k in ranges:
            lo, hi = ranges[k]
            n = int((hi - lo) / 2) + 1
            return [lo + 2.0 * i for i in range(n)]
        return [getattr(leg, k, p["leg"][k])]

    best = []
    for a in enum(keys[0]):
        for b in enum(keys[1]) if len(keys) > 1 else [None]:
            q = {**p}
            q["leg"] = {**p["leg"]}
            q["leg"][keys[0]] = a
            if b is not None:
                q["leg"][keys[1]] = b
            g = Geometry.from_params(q)
            h = height_from_beta(g, g.beta_cruise_deg)
            r = radius_from_beta(g, g.beta_cruise_deg)
            cfg = p["gait"][gait_type]
            sw = hip_sweep_from_stride(g, float(cfg["stride_mm"]), r)
            v = float(cfg["stride_mm"]) / float(cfg["period_s"])
            m3 = three_leg_margin(q)
            ok = 115 <= h <= 125 and abs(sw) <= 60 and 35 <= v <= 40 and m3 >= 15
            label = f"{keys[0]}={a:g}" + (f" {keys[1]}={b:g}" if b is not None else "")
            best.append((ok, m3, h, label, sw, v))
            print(f"{label:<52}{h:8.1f}{sw:9.1f}{v:8.1f}{m3:8.1f}  "
                  f"{'OK' if ok else 'x'}")

    good = sorted([b for b in best if b[0]], key=lambda x: -x[1])
    print()
    if good:
        print("可行组合（按三足裕度排序，取前 5）:")
        for ok, m3, h, label, sw, v in good[:5]:
            print(f"  {label:<48} 裕度 {m3:5.1f}mm  巡航H {h:5.1f}mm")
    else:
        print("没有组合同时满足全部判据——需放宽某项或调整扫描区间。")


# ----------------------------------------------------------------- 入口

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", type=Path, default=Path(__file__).parent / "params.yaml")
    ap.add_argument("--gait", default="crawl", choices=["crawl", "trot"])
    ap.add_argument("--check-only", action="store_true", help="只跑验收检查")
    ap.add_argument("--no-anim", action="store_true", help="不显示 3D 动画")
    ap.add_argument("--save", metavar="PREFIX", help="保存若干帧 PNG 而非显示")
    ap.add_argument("--sweep", nargs="+", metavar="KEY",
                    help="扫参，KEY 取 alpha / beta_cruise / L1 / L2")
    ap.add_argument("--set", nargs="+", metavar="K=V",
                    help="临时覆盖参数，如 alpha=32 beta_cruise=41 L1=105")
    args = ap.parse_args()

    p = yaml.safe_load(args.params.read_text(encoding="utf-8"))

    # 临时覆盖（仅内存，不落盘）
    for item in args.set or []:
        if "=" not in item:
            print(f"忽略无效的 --set 项: {item}", file=sys.stderr)
            continue
        k, v = item.split("=", 1)
        if k in p.get("leg", {}):
            p["leg"][k] = float(v)
        else:
            print(f"暂不支持覆盖 leg.{k}（仅 alpha / beta_cruise / L1 / L2）",
                  file=sys.stderr)

    if args.sweep:
        sweep(p, args.sweep, args.gait)
        return 0

    ok = check(p, args.gait)

    if not args.check_only:
        print()
        try:
            animate(p, args.gait, args.save)
        except ImportError:
            print("未安装 matplotlib，跳过 3D 预览（pip3 install matplotlib）")
        except Exception as exc:                       # noqa: BLE001
            print(f"预览渲染失败: {exc}")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())