#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AEB 结构化诊断 overlay —— 对标 Autoware autonomous_emergency_braking node。

Autoware 的 AEB 是独立节点：输入障碍物/路径/车速，输出制动 + DiagnosticStatus
（OK/ERROR）+ RSS distance + metrics。本模块把本项目已算好的 AEB 状态（来自
LongitudinalContext）**投影**成同样的结构化诊断 `AebDecision`（level / ttc / drac /
closing），供遥测、`/jetson/gate_reason` 之外的安全层取证与报告使用。

**关键约束：只读投影，不重算控制、不改任何下发数值。**
`engaged` 直接取 `lon_ctx.aeb_active`（控制环真实决策），故诊断永远与控制一致、不会漂移；
`level` 按 config 的 TTC 阈值（与 ML AEB 标签一致：>15s safe / 5–15s warning / <5s emergency）分类；
`drac`（Deceleration Rate to Avoid Crash）= closing² / (2·dist)，与 lx/ml 的代理安全指标同式。

Python 3.6 兼容。中文注释，遵循 SOCCode 约定。
"""

from dataclasses import dataclass

from config import TTC_BRAKE_FULL, TTC_BRAKE_START

AEB_LEVEL_SAFE = "safe"
AEB_LEVEL_WARNING = "warning"
AEB_LEVEL_EMERGENCY = "emergency"

# DRAC 上限（与 ML 特征约定一致，避免近距离除法爆量）
_DRAC_CAP = 20.0


@dataclass(frozen=True)
class AebDecision:
    """AEB 结构化诊断（对标 Autoware AEB DiagnosticStatus + metrics）。"""
    level: str            # safe / warning / emergency
    engaged: bool         # 本帧纵向是否由 AEB 路径产生（= lon_ctx.aeb_active）
    ttc: float            # 碰撞时间 (s)
    drac: float           # 避撞所需减速度 (m/s²)
    dist: float           # 纵向间距 (m)
    closing_speed: float  # 接近速度 max(ego_v - lead_v, 0) (m/s)
    reason: str


def _classify(ttc, engaged):
    # AEB 已介入 → emergency；否则按 TTC 阈值分级（与 ML AEB 标签一致）。
    if engaged or ttc <= TTC_BRAKE_FULL:
        return AEB_LEVEL_EMERGENCY
    if ttc <= TTC_BRAKE_START:
        return AEB_LEVEL_WARNING
    return AEB_LEVEL_SAFE


def summarize_aeb(ttc, dist, ego_v, lead_v_proj, aeb_active):
    """把已算好的纵向/AEB 状态投影成结构化 AebDecision（只读，不改控制）。"""
    closing = max(float(ego_v) - max(0.0, float(lead_v_proj)), 0.0)
    d = float(dist)
    if d > 0.1 and closing > 1e-3:
        drac = min((closing * closing) / (2.0 * d), _DRAC_CAP)
    else:
        drac = 0.0
    engaged = bool(aeb_active)
    level = _classify(float(ttc), engaged)
    reason = "aeb_engaged" if engaged else ("ttc_" + level)
    return AebDecision(
        level=level, engaged=engaged, ttc=float(ttc), drac=drac,
        dist=d, closing_speed=closing, reason=reason)
