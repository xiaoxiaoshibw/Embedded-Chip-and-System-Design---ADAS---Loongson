# -*- coding: utf-8 -*-
"""内置 ACC/AEB/LKA 控制器（喂真实 CARLA 感知）。

用途：在「真实 CARLA 世界 + 平台内置控制」模式下产出转向角 delta(rad) 与
减速量 a_brake(+为减速 m/s²)，单位与 carla_bridge/pc/carla_link.apply_ego 一致。

接真实 Nano 后，这个控制器被 Nano 上的 ADAS.py 取代——RealHilBridge 改为从
Nano 网关读回控制量即可，本类只是「无 Nano 也能看 CARLA 闭环」的占位控制核。
与 MockHilBridge 的纵向/横向逻辑同源，便于答辩时口径一致。
"""

from __future__ import annotations

from typing import Any, Dict

from .hil_bridge import AEB_TTC, BRK_GAIN, MIN_GAP, TIME_GAP, _clamp

MAX_DELTA = 0.4363   # 25°，与 SOCCode config.MAX_DELTA 同量级


class Controller:
    def __init__(self, v_set_mps: float = 13.9):
        self.v_set = v_set_mps

    def set_target_speed(self, kmh: float) -> None:
        self.v_set = float(kmh) / 3.6

    def compute(self, perc: Dict[str, Any]) -> Dict[str, float]:
        """perc: ego_v(m/s), lead_present, lead_v, gap(m), lane_offset(m), heading_error(rad)。"""
        ego_v = float(perc.get("ego_v", 0.0))
        lead_present = bool(perc.get("lead_present", False))
        gap = float(perc.get("gap", float("inf")))
        lead_v = float(perc.get("lead_v", 0.0))

        # ── 纵向 ACC / AEB ──
        if not lead_present or gap == float("inf"):
            a_des = _clamp(0.8 * (self.v_set - ego_v), -3.0, 2.5)
        else:
            desired_gap = max(MIN_GAP, TIME_GAP * ego_v)
            gap_err = gap - desired_gap
            closing = ego_v - lead_v
            ttc = gap / closing if closing > 0.05 else float("inf")
            if ttc < AEB_TTC:
                a_des = -BRK_GAIN          # AEB 全力制动
            else:
                a_des = _clamp(0.45 * gap_err - 1.1 * closing, -BRK_GAIN, 2.5)

        # ── 横向 LKA（转向角，rad）──
        lane_offset = float(perc.get("lane_offset", 0.0))
        heading_error = float(perc.get("heading_error", 0.0))
        delta = _clamp(-0.15 * lane_offset - 0.5 * heading_error, -MAX_DELTA, MAX_DELTA)

        return {"delta": delta, "a_brake": -a_des}
