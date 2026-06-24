#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Safety supervisor for longitudinal command shaping.

This layer sits above nominal ACC/cruise/AEB.  It only raises braking demand;
it never adds drive.  The intent is to preserve existing controller behavior in
easy cases while adding a hard dynamic envelope for cut-in, tight-gap, and
curve/lateral-error cases.
"""

import math
from dataclasses import dataclass

from common import clamp, is_finite
from config import (
    ACC_NORMAL_BRAKE_MAX,
    LON_CMD_MAX_BRAKE_DECEL,
    SAFETY_CTE_HARD,
    SAFETY_CTE_WARN,
    SAFETY_CURVE_LAT_ACCEL,
    SAFETY_CURVE_SPEED_KP,
    SAFETY_CUTIN_BRAKE,
    SAFETY_CUTIN_TTC,
    SAFETY_DIST_BUFFER,
    SAFETY_FULL_BRAKE_MARGIN,
    SAFETY_PREBRAKE_MARGIN,
    SAFETY_PREBRAKE_MAX,
    SAFETY_REACTION_TIME,
    SAFETY_SUPERVISOR_ENABLED,
    SAFE_DIST_STANDSTILL,
    SAFE_EGO_MAX_DECEL,
    SAFE_LEAD_MAX_DECEL,
    LEAD_TIMEOUT_S,
)


@dataclass(frozen=True)
class SafetyResult:
    lon_cmd: float
    active: bool = False
    reason: str = ""
    required_brake: float = 0.0
    safe_dist: float = 0.0


def _dynamic_safe_distance(ego_v: float, lead_v: float) -> float:
    v_e = max(float(ego_v), 0.0)
    v_l = max(float(lead_v), 0.0)
    d_react = v_e * max(SAFETY_REACTION_TIME, 0.0)
    d_ego = (v_e * v_e) / (2.0 * max(SAFE_EGO_MAX_DECEL, 0.1))
    d_lead = (v_l * v_l) / (2.0 * max(SAFE_LEAD_MAX_DECEL, 0.1))
    return max(SAFE_DIST_STANDSTILL, SAFE_DIST_STANDSTILL + d_react + d_ego - d_lead)


def _raise_brake(current: float, brake: float):
    return max(current, clamp(brake, 0.0, LON_CMD_MAX_BRAKE_DECEL))


def apply_safety_supervisor(now, lon_cmd, signals, lead_ctx, lateral_ctx, lon_ctx):
    if not SAFETY_SUPERVISOR_ENABLED:
        return SafetyResult(lon_cmd=lon_cmd)

    out = float(lon_cmd)
    required = 0.0
    reasons = []
    safe_dist = 0.0

    signal_lead_fresh = (
        signals.lead_received
        and (now - signals.lead_last_rx_time) <= LEAD_TIMEOUT_S
        and float(signals.lead_x) > 0.0
    )
    signal_lateral_candidate = (
        signal_lead_fresh
        and abs(float(signals.lead_y)) <= 3.5
    )
    if signal_lateral_candidate:
        dist = float(signals.lead_x)
        lead_v = max(0.0, float(signals.lead_v))
        closing = max(0.0, float(signals.ego_v) - lead_v)
        safe_dist = _dynamic_safe_distance(signals.ego_v, lead_v) + SAFETY_DIST_BUFFER
        if dist <= safe_dist - SAFETY_FULL_BRAKE_MARGIN:
            required = LON_CMD_MAX_BRAKE_DECEL
            reasons.append("signal_safe_distance")
        elif dist < safe_dist + SAFETY_PREBRAKE_MARGIN and closing > 0.2:
            margin = max(SAFETY_PREBRAKE_MARGIN, 1e-6)
            ratio = (safe_dist + SAFETY_PREBRAKE_MARGIN - dist) / margin
            required = max(required, ratio * SAFETY_PREBRAKE_MAX)
            reasons.append("signal_prebrake")

    has_target = (
        lead_ctx.acc_has_lead
        or lead_ctx.has_lead
        or (lead_ctx.raw_has_lead and lead_ctx.lead_fresh)
    )
    if has_target:
        dist = float(lon_ctx.dist if is_finite(lon_ctx.dist) else lead_ctx.x_rel)
        lead_v = float(lon_ctx.lead_v_proj if is_finite(lon_ctx.lead_v_proj)
                       else lead_ctx.raw_lead_v_proj)
        if not lead_ctx.acc_has_lead:
            dist = float(lead_ctx.x_rel)
            lead_v = float(lead_ctx.raw_lead_v_proj)
        closing = max(0.0, float(signals.ego_v) - max(0.0, lead_v))
        ttc = dist / closing if closing > 1e-3 and dist > 0.0 else float("inf")
        safe_dist = _dynamic_safe_distance(signals.ego_v, lead_v) + SAFETY_DIST_BUFFER
        lateral_candidate = abs(float(lead_ctx.y_rel)) <= max(
            float(lead_ctx.lead_lat_gate) * 1.5,
            float(lead_ctx.lead_lat_straight) * 1.25,
            3.5,
        )

        if lateral_candidate and dist <= safe_dist - SAFETY_FULL_BRAKE_MARGIN:
            required = LON_CMD_MAX_BRAKE_DECEL
            reasons.append("safe_distance")
        elif lateral_candidate and dist < safe_dist + SAFETY_PREBRAKE_MARGIN and closing > 0.2:
            margin = max(SAFETY_PREBRAKE_MARGIN, 1e-6)
            ratio = (safe_dist + SAFETY_PREBRAKE_MARGIN - dist) / margin
            brake = ratio * SAFETY_PREBRAKE_MAX
            required = max(required, brake)
            reasons.append("prebrake")

        if (
            abs(float(lead_ctx.y_rel)) > max(float(lead_ctx.lead_lat_straight), 0.1)
            and lateral_candidate
            and is_finite(ttc)
            and ttc <= SAFETY_CUTIN_TTC
            and closing > 0.5
        ):
            required = max(required, SAFETY_CUTIN_BRAKE)
            reasons.append("cutin")

    curv = abs(float(lateral_ctx.curv_guard))
    if curv > 1e-6:
        v_curve = math.sqrt(max(SAFETY_CURVE_LAT_ACCEL, 0.1) / curv)
        over_v = max(0.0, float(signals.ego_v) - v_curve)
        if over_v > 0.0:
            required = max(required, min(LON_CMD_MAX_BRAKE_DECEL,
                                         over_v * SAFETY_CURVE_SPEED_KP))
            reasons.append("curve_speed")

    cte = abs(float(lateral_ctx.cur_off))
    if cte > SAFETY_CTE_WARN:
        denom = max(SAFETY_CTE_HARD - SAFETY_CTE_WARN, 1e-6)
        ratio = clamp((cte - SAFETY_CTE_WARN) / denom, 0.0, 1.0)
        required = max(required, ratio * ACC_NORMAL_BRAKE_MAX)
        reasons.append("lateral_error")

    if required > 0.0:
        out = _raise_brake(out, required)
        return SafetyResult(
            lon_cmd=out,
            active=True,
            reason="+".join(reasons),
            required_brake=required,
            safe_dist=safe_dist,
        )
    return SafetyResult(lon_cmd=lon_cmd, safe_dist=safe_dist)
