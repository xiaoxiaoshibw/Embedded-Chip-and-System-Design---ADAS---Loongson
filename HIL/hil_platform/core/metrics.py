# -*- coding: utf-8 -*-
"""指标计算与摘要生成。

逐帧累积，stop 时产出 summary.json 内容：
    result(PASS/FAIL), collision, min_ttc, max_lateral_error,
    takeover_happened, takeover_latency_ms, active_controller_final,
    safe_brake_triggered, conclusion

接管时延 = 故障注入时刻 → ESP32 实际切换控制器时刻 的毫秒差。
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from .types import CTRL_SAFE_BRAKE, StateFrame

# PASS/FAIL 判据阈值
MIN_TTC_FAIL = 1.0           # 最小 TTC 低于该值视为危险
MAX_LATERAL_FAIL = 0.8       # 最大横向误差超过该值视为失败（m）


class Metrics:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.min_ttc: float = float("inf")
        self.min_ttc_time: float = 0.0
        self.max_lateral_error: float = 0.0
        self.max_lateral_error_time: float = 0.0
        self.collision: bool = False
        self.collision_time: Optional[float] = None
        self.safe_brake_triggered: bool = False
        self.takeover_happened: bool = False
        self.takeover_latency_ms: Optional[float] = None
        self.active_controller_final: str = "none"
        self.last_t: float = 0.0
        # 接管时延：记录最近一次故障注入时刻，等首次接管时配对
        self._pending_fault_time: Optional[float] = None
        self._first_takeover_time: Optional[float] = None

    # ── 逐帧更新 ──
    def update(self, frame: StateFrame) -> None:
        self.last_t = frame.t
        ttc = frame.target.ttc
        if ttc is not None and ttc == ttc and ttc != float("inf") and ttc > 0:
            if ttc < self.min_ttc:
                self.min_ttc = ttc
                self.min_ttc_time = frame.t
        lat = abs(frame.ego.lateral_error or 0.0)
        if lat > self.max_lateral_error:
            self.max_lateral_error = lat
            self.max_lateral_error_time = frame.t

        fd = frame.target.front_distance
        if fd is not None and fd == fd and fd <= 0.0 and not self.collision:
            self.collision = True
            self.collision_time = frame.t

        if frame.esp32.safe_brake:
            self.safe_brake_triggered = True

        self.active_controller_final = frame.esp32.active_controller

    # ── 事件钩子（由 simulation_core 调用）──
    def on_fault_injected(self, sim_t: float) -> None:
        # 只记录第一次注入时间（用于接管时延）
        if self._pending_fault_time is None:
            self._pending_fault_time = sim_t

    def on_takeover(self, sim_t: float) -> Optional[float]:
        """返回本次接管时延（ms）。只对「故障注入之后」的首次接管算时延，
        避免启动瞬态接管（此时尚无 pending fault）把时延锁成无效值。"""
        self.takeover_happened = True
        if (self.takeover_latency_ms is None
                and self._pending_fault_time is not None
                and sim_t >= self._pending_fault_time):
            self._first_takeover_time = sim_t
            self.takeover_latency_ms = round(
                (sim_t - self._pending_fault_time) * 1000.0, 1)
        return self.takeover_latency_ms

    # ── 摘要 ──
    def _result(self) -> str:
        if self.collision:
            return "FAIL"
        if self.min_ttc != float("inf") and self.min_ttc < MIN_TTC_FAIL:
            return "FAIL"
        if self.max_lateral_error > MAX_LATERAL_FAIL:
            return "FAIL"
        return "PASS"

    def _conclusion(self, scenario: str) -> str:
        parts: List[str] = []
        if self.collision:
            parts.append("发生碰撞（%.1fs）" % (self.collision_time or 0.0))
        else:
            parts.append("全程未碰撞")
        if self.takeover_happened:
            parts.append(
                "ESP32 检测到主控异常并切换至备控（接管时延 %s ms）"
                % (("%.0f" % self.takeover_latency_ms) if self.takeover_latency_ms is not None else "—")
            )
        if self.safe_brake_triggered:
            parts.append("触发安全制动兜底")
        parts.append(
            "最小 TTC %.2fs，最大横向误差 %.2fm"
            % (
                self.min_ttc if self.min_ttc != float("inf") else 0.0,
                self.max_lateral_error,
            )
        )
        return "；".join(parts) + "。"

    def summary(self, scenario: str) -> Dict[str, Any]:
        return {
            "result": self._result(),
            "collision": bool(self.collision),
            "min_ttc": None if self.min_ttc == float("inf") else round(self.min_ttc, 2),
            "max_lateral_error": round(self.max_lateral_error, 3),
            "takeover_happened": bool(self.takeover_happened),
            "takeover_latency_ms": self.takeover_latency_ms,
            "active_controller_final": self.active_controller_final,
            "safe_brake_triggered": bool(self.safe_brake_triggered),
            "conclusion": self._conclusion(scenario),
        }

    def derived_events(self) -> List[dict]:
        """stop 时补充 MIN_TTC / MAX_LATERAL_ERROR / COLLISION 标记事件（供回放跳转）。"""
        evts: List[dict] = []
        if self.min_ttc != float("inf"):
            evts.append({
                "time": round(self.min_ttc_time, 3),
                "type": "MIN_TTC",
                "value": round(self.min_ttc, 2),
            })
        if self.max_lateral_error > 0:
            evts.append({
                "time": round(self.max_lateral_error_time, 3),
                "type": "MAX_LATERAL_ERROR",
                "value": round(self.max_lateral_error, 3),
            })
        if self.collision:
            evts.append({
                "time": round(self.collision_time or 0.0, 3),
                "type": "COLLISION",
            })
        return evts
