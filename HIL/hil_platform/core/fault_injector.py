# -*- coding: utf-8 -*-
"""故障注入器。

需求要求：故障逻辑**集中在此**，不散落到 Web / scenario_manager / bridge。
本模块只负责"持有当前激活的故障 + 把故障效果作用到一帧 StateFrame 的 Nano 状态上"。
ESP32 的仲裁/接管判定在 hil_bridge 的 Esp32Arbiter 里读取被注入后的 Nano 状态。

故障类型（与 /live 故障按钮 + CLI 一致）：
    heartbeat_loss  断心跳        -> alive=False
    seq_stuck       seq 停止递增   -> seq 冻结（假活：alive 仍 True 但 seq 不增）
    nan_output      输出 NaN       -> valid_output=False，控制量置 NaN
    control_delay   控制延迟       -> latency_ms 抬高到超阈值
    backup_fail     备控接管失败    -> 作用于 nano_b，使其不可用
    dual_fail       双路失败       -> A、B 同时失效 -> ESP32 安全制动
"""

from __future__ import annotations

import threading
from typing import Dict, List, Optional

from .types import ControllerState, StateFrame

FAULT_TYPES = (
    "heartbeat_loss",
    "seq_stuck",
    "nan_output",
    "control_delay",
    "backup_fail",
    "dual_fail",
)

# 注入控制延迟时抬高到的延迟值（ms），需大于仲裁器的健康延迟阈值
CONTROL_DELAY_MS = 600.0
NAN = float("nan")


class FaultInjector:
    """线程安全地持有激活故障集合，并把效果作用到帧上。"""

    def __init__(self):
        self._lock = threading.Lock()
        # active: {(fault_type, target): trigger_time_s}
        self._active: Dict[tuple, float] = {}
        # 注入后待广播的事件（由 simulation_core 取走写入 recorder）
        self._pending_events: List[dict] = []
        # 记录每个 fault 第一次注入的场景时间，供接管时延计算
        self._seq_freeze_value: Dict[str, Optional[int]] = {"nano_a": None, "nano_b": None}

    # ── 注入 / 清除 ──
    def inject(self, fault_type: str, target: str, sim_t: float) -> dict:
        if fault_type not in FAULT_TYPES:
            raise ValueError("未知故障类型：%s" % fault_type)
        # dual_fail 语义上作用于双路
        if fault_type == "dual_fail":
            target = "both"
        if fault_type == "backup_fail":
            target = "nano_b"
        if target not in ("nano_a", "nano_b", "both"):
            raise ValueError("非法故障目标：%s" % target)

        with self._lock:
            self._active[(fault_type, target)] = sim_t
            # seq_stuck 需要记录冻结起点，作用时第一次见到 seq 再冻结
            if fault_type == "seq_stuck":
                for tgt in self._targets(target):
                    self._seq_freeze_value[tgt] = None
            evt = {
                "time": round(sim_t, 3),
                "type": "FAULT_INJECTED",
                "target": target,
                "detail": fault_type,
            }
            self._pending_events.append(evt)
            return evt

    def clear_all(self) -> None:
        with self._lock:
            self._active.clear()
            self._pending_events.clear()
            self._seq_freeze_value = {"nano_a": None, "nano_b": None}

    def active_faults(self) -> List[dict]:
        with self._lock:
            return [
                {"type": ft, "target": tg, "trigger_time": t}
                for (ft, tg), t in self._active.items()
            ]

    def pop_pending_events(self) -> List[dict]:
        with self._lock:
            out = self._pending_events
            self._pending_events = []
            return out

    @staticmethod
    def _targets(target: str) -> List[str]:
        return ["nano_a", "nano_b"] if target == "both" else [target]

    # ── 作用到帧 ──
    def apply(self, frame: StateFrame) -> StateFrame:
        """把所有激活故障作用到 frame 的 nano_a / nano_b。就地修改并返回。"""
        with self._lock:
            active = dict(self._active)
            freeze = dict(self._seq_freeze_value)

        for (fault_type, target), _t in active.items():
            for tgt in self._targets(target):
                ctrl = frame.nano_a if tgt == "nano_a" else frame.nano_b
                self._apply_one(fault_type, tgt, ctrl, freeze)

        # 回写 seq 冻结值
        with self._lock:
            self._seq_freeze_value = freeze
        return frame

    def _apply_one(self, fault_type: str, tgt: str,
                   ctrl: ControllerState, freeze: Dict[str, Optional[int]]) -> None:
        if fault_type == "heartbeat_loss":
            ctrl.alive = False
        elif fault_type == "seq_stuck":
            # 第一次作用时锁定当前 seq，之后强制不变（制造"假活"）
            if freeze.get(tgt) is None:
                freeze[tgt] = ctrl.seq
            ctrl.seq = freeze[tgt]  # type: ignore[assignment]
        elif fault_type == "nan_output":
            ctrl.valid_output = False
            ctrl.throttle = NAN
            ctrl.brake = NAN
            ctrl.steer = NAN
        elif fault_type == "control_delay":
            ctrl.latency_ms = max(ctrl.latency_ms, CONTROL_DELAY_MS)
        elif fault_type in ("backup_fail", "dual_fail"):
            # 直接判定该控制器不可用（输出无效）
            ctrl.alive = False
            ctrl.valid_output = False
