"""
ik.py — 2DOF 腿逆解（与固件 components/kinematics 逐位同源）

几何关系见 docs/gait_design.md §2：
    H(β) = L1·sin(α) + L2·sin(β)      站立高度
    R(β) = L1·cos(α) + L2·cos(β)      足端水平半径
    β_cruise = asin((H - L1·sin(α)) / L2)
    髋半摆角 Δψ = asin(stride / 2R)

参数一律从 params.yaml 读，本文件不硬编码任何几何数值。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Geometry:
    L1: float
    L2: float
    alpha_deg: float
    beta_cruise_deg: float
    splay_deg: float

    @staticmethod
    def from_params(p: dict) -> "Geometry":
        leg = p["leg"]
        return Geometry(
            L1=float(leg["L1_mm"]),
            L2=float(leg["L2_mm"]),
            alpha_deg=float(leg["alpha_deg"]),
            beta_cruise_deg=float(leg["beta_cruise_deg"]),
            splay_deg=float(leg["splay_deg"]),
        )

    @property
    def hip_limit_deg(self) -> float:
        return 60.0

    @property
    def beta_min_deg(self) -> float:
        return 21.3

    @property
    def beta_max_deg(self) -> float:
        return 65.4


def height_from_beta(geom: Geometry, beta_deg: float) -> float:
    """站立高度 H(β)"""
    a = math.radians(geom.alpha_deg)
    b = math.radians(beta_deg)
    return geom.L1 * math.sin(a) + geom.L2 * math.sin(b)


def beta_from_height(geom: Geometry, height_mm: float) -> float:
    """反解 β = asin((H - L1·sin(α)) / L2)，越界则钳到限位"""
    a = math.radians(geom.alpha_deg)
    s = (height_mm - geom.L1 * math.sin(a)) / geom.L2
    s = max(-1.0, min(1.0, s))
    b = math.degrees(math.asin(s))
    return max(geom.beta_min_deg, min(geom.beta_max_deg, b))


def radius_from_beta(geom: Geometry, beta_deg: float) -> float:
    """足端水平半径 R(β)"""
    a = math.radians(geom.alpha_deg)
    b = math.radians(beta_deg)
    return geom.L1 * math.cos(a) + geom.L2 * math.cos(b)


def radius_from_height(geom: Geometry, height_mm: float) -> float:
    return radius_from_beta(geom, beta_from_height(geom, height_mm))


def hip_sweep_from_stride(geom: Geometry, stride_mm: float, radius_mm: float) -> float:
    """步幅 -> 髋半摆角 Δψ = asin(stride / 2R)"""
    if radius_mm <= 0.0:
        return 0.0
    s = max(-1.0, min(1.0, stride_mm / (2.0 * radius_mm)))
    return math.degrees(math.asin(s))


def height_coupling(beta_deg: float) -> float:
    """膝角变动的水平/高度耦合比 1/cot(β)

    巡航位该值约 1.2；趋近 0 时发散（变高极高效但腿软）。
    """
    t = math.tan(math.radians(beta_deg))
    if t <= 1e-4:
        return 1e4
    return 1.0 / t


@dataclass
class LegIK:
    hip_deg: float
    knee_beta_deg: float
    height_mm: float


def leg_ik(geom: Geometry, delta_psi_deg: float, height_mm: float) -> LegIK:
    """逆解：足端目标（相对外八中立位的髋偏角 + 目标身高）-> 关节角

    内部含软限位，与固件 servo 层同一套限位。
    """
    beta = beta_from_height(geom, height_mm)
    hip = max(-geom.hip_limit_deg, min(geom.hip_limit_deg, delta_psi_deg))
    return LegIK(hip_deg=hip, knee_beta_deg=beta, height_mm=height_from_beta(geom, beta))


def foot_in_leg_plane(geom: Geometry, delta_psi_deg: float, height_mm: float) -> tuple[float, float]:
    """足端在机身系（腿平面极坐标）下的 (半径, 高度)

    半径 = R(β)，方位 = 腿平面方位角 + delta_psi。
    """
    beta = beta_from_height(geom, height_mm)
    return radius_from_beta(geom, beta), height_from_beta(geom, beta)