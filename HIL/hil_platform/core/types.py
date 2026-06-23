# -*- coding: utf-8 -*-
"""核心数据类型：单帧仿真状态 StateFrame 及其子结构。

一帧 = Ego + Target(前车) + Nano A + Nano B + ESP32 仲裁结果 + 当前事件。
- HilBridge 产出 StateFrame；
- metrics 读 StateFrame 做聚合（min_ttc / max_lateral_error / 接管时延）；
- recorder 把 StateFrame 拍平成 states.csv 一行；
- server 把 StateFrame 序列化成 /ws/live 推送的 JSON。

字段命名与需求里的 WebSocket / states.csv 规范保持一致，避免再做映射。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

# active_controller 取值
CTRL_NANO_A = "nano_a"
CTRL_NANO_B = "nano_b"
CTRL_SAFE_BRAKE = "safe_brake"
CTRL_NONE = "none"


def _f(x: Optional[float]) -> Optional[float]:
    """把 NaN/Inf 归一成 None，便于 JSON 序列化与前端空值保护。"""
    if x is None:
        return None
    try:
        xf = float(x)
    except (TypeError, ValueError):
        return None
    if math.isnan(xf) or math.isinf(xf):
        return None
    return xf


@dataclass
class EgoState:
    speed_kmh: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0
    lateral_error: float = 0.0
    heading_error: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "speed_kmh": _f(self.speed_kmh),
            "throttle": _f(self.throttle),
            "brake": _f(self.brake),
            "steer": _f(self.steer),
            "lateral_error": _f(self.lateral_error),
            "heading_error": _f(self.heading_error),
        }


@dataclass
class TargetState:
    front_distance: float = float("inf")
    relative_speed: float = 0.0   # 前车速度 - 自车速度；负=接近
    ttc: float = float("inf")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "front_distance": _f(self.front_distance),
            "relative_speed": _f(self.relative_speed),
            "ttc": _f(self.ttc),
        }


@dataclass
class ControllerState:
    """单个 Nano 控制器（主控 A / 备控 B）的状态。"""
    alive: bool = True
    seq: int = 0
    latency_ms: float = 0.0
    valid_output: bool = True
    last_control_time: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "alive": bool(self.alive),
            "seq": int(self.seq),
            "latency_ms": _f(self.latency_ms),
            "valid_output": bool(self.valid_output),
            "last_control_time": _f(self.last_control_time),
            "throttle": _f(self.throttle),
            "brake": _f(self.brake),
            "steer": _f(self.steer),
        }


@dataclass
class Esp32State:
    """ESP32 仲裁器输出（最终回注 CARLA 的控制量）。"""
    active_controller: str = CTRL_NONE
    takeover_count: int = 0
    last_takeover_reason: Optional[str] = None
    safe_brake: bool = False
    throttle: float = 0.0
    brake: float = 0.0
    steer: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "active_controller": self.active_controller,
            "takeover_count": int(self.takeover_count),
            "last_takeover_reason": self.last_takeover_reason,
            "safe_brake": bool(self.safe_brake),
            "throttle": _f(self.throttle),
            "brake": _f(self.brake),
            "steer": _f(self.steer),
        }


@dataclass
class StateFrame:
    """一帧完整仿真状态。"""
    t: float = 0.0                       # 场景时间 scenario_time (s)
    ego: EgoState = field(default_factory=EgoState)
    target: TargetState = field(default_factory=TargetState)
    nano_a: ControllerState = field(default_factory=ControllerState)
    nano_b: ControllerState = field(default_factory=ControllerState)
    esp32: Esp32State = field(default_factory=Esp32State)
    event: Optional[str] = None          # 该帧触发的事件类型（如 TAKEOVER）

    # ── 序列化 ──
    def to_ws_dict(self, run_id: str, scenario: str, state: str) -> Dict[str, Any]:
        """转成 /ws/live 推送的 JSON（含 run 元信息）。"""
        return {
            "run_id": run_id,
            "timestamp": round(self.t, 3),
            "scenario": scenario,
            "state": state,
            "ego": self.ego.to_dict(),
            "target": self.target.to_dict(),
            "nano_a": self.nano_a.to_dict(),
            "nano_b": self.nano_b.to_dict(),
            "esp32": self.esp32.to_dict(),
            "event": self.event,
        }

    def to_csv_row(self) -> Dict[str, Any]:
        """拍平成 states.csv 一行（字段顺序见 recorder.STATE_FIELDS）。"""
        def num(x: Optional[float], nd: int = 4) -> str:
            v = _f(x)
            return "" if v is None else ("%.*f" % (nd, v))

        # 最终回注 CARLA 的 throttle/brake/steer = ESP32 仲裁输出
        return {
            "t": "%.3f" % self.t,
            "ego_speed": num(self.ego.speed_kmh, 3),
            "front_distance": num(self.target.front_distance, 3),
            "relative_speed": num(self.target.relative_speed, 3),
            "ttc": num(self.target.ttc, 3),
            "lateral_error": num(self.ego.lateral_error, 4),
            "heading_error": num(self.ego.heading_error, 4),
            "throttle": num(self.esp32.throttle, 4),
            "brake": num(self.esp32.brake, 4),
            "steer": num(self.esp32.steer, 4),
            "nano_a_alive": int(bool(self.nano_a.alive)),
            "nano_a_seq": int(self.nano_a.seq),
            "nano_a_latency_ms": num(self.nano_a.latency_ms, 1),
            "nano_a_valid_output": int(bool(self.nano_a.valid_output)),
            "nano_b_alive": int(bool(self.nano_b.alive)),
            "nano_b_seq": int(self.nano_b.seq),
            "nano_b_latency_ms": num(self.nano_b.latency_ms, 1),
            "nano_b_valid_output": int(bool(self.nano_b.valid_output)),
            "active_controller": self.esp32.active_controller,
            "takeover_count": int(self.esp32.takeover_count),
            "safe_brake": int(bool(self.esp32.safe_brake)),
            "event": self.event or "",
        }
