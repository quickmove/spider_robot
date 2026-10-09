"""
gait.py — 步态轨迹生成（与固件 components/gait 同算法、同参数）

设计见 docs/gait_design.md §3 与 §7：
  - 支撑腿足端在世界系固定，关节角由「足端坐标 + 机身位姿」反解 → 模型层零滑移
  - 摆动腿在摆动相内完成 lift → reach → place，落点用精确差速步幅
  - 转向 = 机身前进 + 偏航两个分量独立积分；各腿步幅 (v - ω·y_i)·T

与固件的差异仅限于浮点/整数运算，算法与相位表逐条对应。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ik import Geometry, beta_from_height, height_from_beta, radius_from_beta

SW_REACH_A = 0.15
SW_REACH_B = 0.55
SW_PLACE_B = 0.70


def _wrap01(t: float) -> float:
    t -= int(t)
    return t + 1.0 if t < 0.0 else t


def _smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


@dataclass
class GaitConfig:
    type: str = "crawl"          # crawl | trot
    stride_mm: float = 60.0
    period_s: float = 1.7
    lift_mm: float = 28.0
    speed_pct: float = 50.0
    heading_deg: float = 0.0
    height_target_mm: float = 120.0

    @staticmethod
    def from_params(p: dict, type: str) -> "GaitConfig":
        g = p["gait"][type]
        return GaitConfig(
            type=type,
            stride_mm=float(g["stride_mm"]),
            period_s=float(g["period_s"]),
            lift_mm=float(g["lift_mm"]),
            speed_pct=float(g["speed_pct"]) if "speed_pct" in g else 50.0,
            heading_deg=0.0,
            height_target_mm=float(p["posture"]["stand_height_mm"]),
        )

    @property
    def duty(self) -> float:
        return 0.5 if self.type == "trot" else 0.25

    @property
    def phase_starts(self) -> list[float]:
        """爬行：FL FR RL RR 错开 1/4；Trot：相0=FL+RR，相1=FR+RL"""
        if self.type == "trot":
            return [0.0, 0.5, 0.5, 0.0]
        return [0.0, 0.25, 0.5, 0.75]


@dataclass
class LegCmd:
    hip_deg: float = 0.0
    knee_beta_deg: float = 0.0
    height_mm: float = 120.0
    in_swing: bool = False


@dataclass
class Sample:
    t: float
    body_x: float
    body_y: float
    yaw_deg: float
    cmds: list[LegCmd]
    foot_xy: list[tuple[float, float]]
    stance_foot_h: list[float]
    support_margin_mm: float
    max_hip_deg: float


@dataclass
class GaitSim:
    """步态仿真器：与固件 gait_t 同构"""

    geom: Geometry
    legs: list[dict] = field(default_factory=list)   # psi_deg / hip / r_mm
    cfg: GaitConfig = field(default_factory=GaitConfig)

    phase: float = 0.0
    body_x: float = 0.0
    body_y: float = 0.0
    yaw_deg: float = 0.0
    height_mm: float = 120.0

    foot: list[tuple[float, float]] = field(default_factory=list)
    foot_from: list[tuple[float, float]] = field(default_factory=list)
    foot_to: list[tuple[float, float]] = field(default_factory=list)
    leg_h: list[float] = field(default_factory=list)
    leg_swing: list[bool] = field(default_factory=list)

    yaw_rate_max_deg_s: float = 25.0
    trot_speed_cap_pct: float = 70.0
    speed_max_pct: float = 100.0

    # ---------------- 初始化 ----------------

    @staticmethod
    def from_params(p: dict, type: str = "crawl", **overrides) -> "GaitSim":
        sim = GaitSim(geom=Geometry.from_params(p))
        sim.legs = [dict(l) for l in p["legs"]]
        sim.cfg = GaitConfig.from_params(p, type)
        for k, v in overrides.items():
            setattr(sim.cfg, k, v)
        sim.yaw_rate_max_deg_s = float(p["turn"]["max_yaw_rate_deg_s"])
        sim.trot_speed_cap_pct = float(p["gait"]["speed"]["trot_max_speed_pct"])
        sim.speed_max_pct = float(p["gait"]["speed"]["max_pct"])
        sim.reset()
        return sim

    def reset(self) -> None:
        """把四足放到机身正下方的巡航半径处"""
        self.phase = 0.0
        self.body_x = self.body_y = 0.0
        self.yaw_deg = 0.0
        self.height_mm = self.cfg.height_target_mm
        self.foot, self.foot_from, self.foot_to = [], [], []
        self.leg_h, self.leg_swing = [], []
        for i, L in enumerate(self.legs):
            hx, hy = self._hip_world(i)
            r = float(L["r_mm"])
            fx = hx + r * math.cos(math.radians(float(L["psi_deg"])))
            fy = hy + r * math.sin(math.radians(float(L["psi_deg"])))
            self.foot.append((fx, fy))
            self.foot_from.append((fx, fy))
            self.foot_to.append((fx, fy))
            self.leg_h.append(self.height_mm)
            self.leg_swing.append(False)

    # ---------------- 几何辅助 ----------------

    def _hip_world(self, leg: int) -> tuple[float, float]:
        L = self.legs[leg]
        c = math.cos(math.radians(self.yaw_deg))
        s = math.sin(math.radians(self.yaw_deg))
        hx, hy = float(L["hip"][0]), float(L["hip"][1])
        return (self.body_x + hx * c - hy * s,
                self.body_y + hx * s + hy * c)

    def _foot_body_y(self, leg: int) -> float:
        hx, hy = self._hip_world(leg)
        dx = self.foot[leg][0] - hx
        dy = self.foot[leg][1] - hy
        c = math.cos(math.radians(-self.yaw_deg))
        s = math.sin(math.radians(-self.yaw_deg))
        return dx * s + dy * c

    def _solve_leg(self, leg: int, foot_xy: tuple[float, float],
                   height: float, lift: float) -> LegCmd:
        hx, hy = self._hip_world(leg)
        dx = foot_xy[0] - hx
        dy = foot_xy[1] - hy
        c = math.cos(math.radians(-self.yaw_deg))
        s = math.sin(math.radians(-self.yaw_deg))
        bx = dx * c - dy * s
        by = dx * s + dy * c

        psi = math.radians(float(self.legs[leg]["psi_deg"]))
        ux, uy = math.cos(psi), math.sin(psi)
        along = bx * ux + by * uy
        lateral = -bx * uy + by * ux
        delta_psi = math.degrees(math.atan2(lateral, along))

        beta = beta_from_height(self.geom, height - lift)
        return LegCmd(
            hip_deg=max(-self.geom.hip_limit_deg,
                        min(self.geom.hip_limit_deg, delta_psi)),
            knee_beta_deg=beta,
            height_mm=height,
        )

    # ---------------- 主推进 ----------------

    def step(self, dt_s: float) -> Sample:
        cfg = self.cfg
        duty = cfg.duty
        speed_scale = cfg.speed_pct / self.speed_max_pct

        base_speed = (cfg.stride_mm / cfg.period_s) * speed_scale
        step_mm = base_speed * cfg.period_s

        # 航向跟踪（P 控制 + 红线）
        err = (cfg.heading_deg - self.yaw_deg + 180.0) % 360.0 - 180.0
        yaw_rate = max(-self.yaw_rate_max_deg_s,
                       min(self.yaw_rate_max_deg_s, err * 4.0)) * speed_scale

        # 机身：前进 + 偏航，各自独立积分
        yaw = math.radians(self.yaw_deg)
        self.body_x += base_speed * math.cos(yaw) * dt_s
        self.body_y += base_speed * math.sin(yaw) * dt_s
        self.yaw_deg = (self.yaw_deg + yaw_rate * dt_s) % 360.0
        yaw = math.radians(self.yaw_deg)

        prev_phase = self.phase
        self.phase = _wrap01(self.phase + dt_s / cfg.period_s)

        cmds: list[LegCmd] = []
        for leg in range(4):
            start = cfg.phase_starts[leg]
            local = _wrap01(self.phase - start)
            swinging = local < duty

            just_started = swinging and _wrap01(prev_phase - start) >= duty
            if just_started:
                self.foot_from[leg] = self.foot[leg]
                # 精确差速步幅：足端世界速度沿机身前向 = v - ω·y_i
                y_i = self._foot_body_y(leg)
                v_foot = base_speed - math.radians(yaw_rate) * y_i
                adv = max(-2 * step_mm, min(2 * step_mm, v_foot * cfg.period_s))
                cy, sy = math.cos(yaw), math.sin(yaw)
                self.foot_to[leg] = (self.foot_from[leg][0] + adv * cy,
                                     self.foot_from[leg][1] + adv * sy)

            lift = 0.0
            if swinging:
                u = local / duty
                v = _smoothstep((u - SW_REACH_A) / (SW_REACH_B - SW_REACH_A))
                fx, fy = self.foot_from[leg]
                tx, ty = self.foot_to[leg]
                self.foot[leg] = (fx + (tx - fx) * v, fy + (ty - fy) * v)

                if u < SW_REACH_A:
                    env = _smoothstep(u / SW_REACH_A)
                elif u < SW_PLACE_B:
                    env = 1.0
                else:
                    env = 1.0 - _smoothstep((u - SW_PLACE_B) / (1.0 - SW_PLACE_B))
                lift = cfg.lift_mm * env

            # 摆动相内逐腿变高（零滑移），支撑腿锁定各自高度
            if swinging:
                self.leg_h[leg] = cfg.height_target_mm

            cmds.append(self._solve_leg(leg, self.foot[leg], self.leg_h[leg], lift))
            cmds[leg].in_swing = swinging
            self.leg_swing[leg] = swinging

        self.height_mm = sum(self.leg_h) / 4.0

        margin = self.support_margin()
        # 足端离地高度 = 锁定身高 - 该膝角实际达成的高度（= 抬起量，与固件同号）
        foot_lift = [
            cmds[i].height_mm - height_from_beta(self.geom, cmds[i].knee_beta_deg)
            for i in range(4)
        ]
        return Sample(
            t=0.0,
            body_x=self.body_x,
            body_y=self.body_y,
            yaw_deg=self.yaw_deg,
            cmds=cmds,
            foot_xy=list(self.foot),
            stance_foot_h=foot_lift,
            support_margin_mm=margin,
            max_hip_deg=max(abs(c.hip_deg) for c in cmds),
        )

    def run(self, seconds: float, dt_s: float = 0.005) -> list[Sample]:
        n = int(seconds / dt_s)
        return [self.step(dt_s) for _ in range(n)]

    # ---------------- 指标 ----------------

    def support_margin(self) -> float:
        """机身中心到支撑足凸包各边的最小距离（mm）

        只算支撑腿——摆动腿的足端在空中。Trot 两足支撑时凸包退化为线段，
        裕度即中心到该直线的距离。
        """
        pts = []
        for leg in range(4):
            if self.leg_swing[leg]:
                continue
            # 顶点必须是足端相对机身中心的坐标，不是相对髋轴心
            dx = self.foot[leg][0] - self.body_x
            dy = self.foot[leg][1] - self.body_y
            c = math.cos(math.radians(-self.yaw_deg))
            s = math.sin(math.radians(-self.yaw_deg))
            pts.append((dx * c - dy * s, dx * s + dy * c))

        if len(pts) < 2:
            return 0.0

        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        if len(pts) == 2:
            (ax, ay), (bx, by) = pts
            ddx, ddy = bx - ax, by - ay
            length = math.hypot(ddx, ddy)
            if length < 1e-3:
                return math.hypot(ax, ay)
            return abs(ddx * (0.0 - ay) - ddy * (0.0 - ax)) / length

        hull = sorted(pts)
        lower: list[tuple[float, float]] = []
        for pt in hull:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], pt) <= 0:
                lower.pop()
            lower.append(pt)
        upper: list[tuple[float, float]] = []
        for pt in reversed(hull):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], pt) <= 0:
                upper.pop()
            upper.append(pt)
        poly = lower[:-1] + upper[:-1]

        if len(poly) == 2:
            (ax, ay), (bx, by) = poly
            ddx, ddy = bx - ax, by - ay
            length = math.hypot(ddx, ddy)
            return abs(ddx * (0.0 - ay) - ddy * (0.0 - ax)) / max(length, 1e-3)

        best = float("inf")
        for i in range(len(poly)):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % len(poly)]
            ddx, ddy = bx - ax, by - ay
            length = math.hypot(ddx, ddy)
            if length < 1e-3:
                continue
            best = min(best, abs(ddx * (0.0 - ay) - ddy * (0.0 - ax)) / length)
        return 0.0 if best == float("inf") else best

    def set_height_target(self, height_mm: float) -> None:
        self.cfg.height_target_mm = max(90.0, min(150.0, height_mm))

    def set_speed(self, pct: float) -> None:
        pct = max(0.0, min(self.speed_max_pct, pct))
        if self.cfg.type == "trot":
            pct = min(pct, self.trot_speed_cap_pct)
        self.cfg.speed_pct = pct

    def set_heading(self, deg: float) -> None:
        self.cfg.heading_deg = deg

    def set_type(self, type: str, params: dict) -> None:
        """切换步态并同步 params.yaml 中的该步态参数"""
        g = params["gait"][type]
        self.cfg.type = type
        self.cfg.stride_mm = float(g["stride_mm"])
        self.cfg.period_s = float(g["period_s"])
        self.cfg.lift_mm = float(g["lift_mm"])
        self.set_speed(self.cfg.speed_pct)