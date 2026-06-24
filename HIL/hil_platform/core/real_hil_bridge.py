# -*- coding: utf-8 -*-
"""RealHilBridge：用真实 CARLA 世界跑闭环，产出与 mock 同构的 StateFrame。

实现 HilBridge 接口，SimulationCore 上层零改动即可从 mock 切到真实 CARLA。

控制来源（control_source）可插拔：
- 'internal'：平台内置 Controller（ACC/AEB/LKA）——只要有 CARLA、无需 Nano 即可看闭环；
  复用 Esp32Arbiter + FaultInjector，软件模拟主备接管。
- 'nano'：真实双 Nano + ESP32。经 NanoLink 把 CARLA 真值感知发给主控 Nano 上的
  `carla_bridge/nano/hil_ros_gateway.py`，读回 ESP32 仲裁后的最终控制 + active_role + failover；
  CARLA ego 由**真实 ADAS.py 控制**，active_controller 反映**真实**主备角色，
  杀主控 Nano 时 ESP32 真切换、CARLA 不停。故障注入走真实硬件（见 nano_fault.py）。

两种来源产出同一套 StateFrame，前端 / 回放 / 记录完全一致。
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional, Tuple

from .carla_world import CarlaWorld
from .controller import Controller
from .fault_injector import FaultInjector
from .hil_bridge import BRK_GAIN, Esp32Arbiter, HilBridge, _accel_to_pedals
from .nano_link import NanoLink, build_sensor_payload
from .types import (
    CTRL_NANO_A, CTRL_NANO_B, CTRL_SAFE_BRAKE,
    ControllerState, EgoState, Esp32State, StateFrame, TargetState,
)

NOMINAL_LAT_MS = 20.0
ACTIVE_DEBOUNCE = 3   # active_controller 需连续 N 拍稳定才提交，滤掉启动/恢复瞬态抖动


class RealHilBridge(HilBridge):
    def __init__(self, host: str = "127.0.0.1", port: int = 2000,
                 town: str = "Town04", enable_camera: bool = True,
                 control_source: str = "internal",
                 gateway_host: str = "192.168.3.125", gateway_port: int = 42110):
        self.world = CarlaWorld(host, port, town, enable_camera=enable_camera)
        self.control_source = control_source
        self.gateway_host = gateway_host
        self.gateway_port = gateway_port
        self.controller = Controller()
        self.arbiter = Esp32Arbiter()
        self.nano_link: Optional[NanoLink] = None
        self.scenario = ""
        self.params: Dict[str, Any] = {}
        self.seq_a = 0
        self.seq_b = 0
        self._sensor_seq = 0
        self.collided = False
        self._last_active = CTRL_NANO_A
        self._takeovers = 0
        self._active_committed = CTRL_NANO_A
        self._active_candidate = CTRL_NANO_A
        self._active_stable = 0

    def load(self, scenario: str, params: Dict[str, Any]) -> None:
        self.scenario = scenario
        self.params = dict(params)
        self.world.load(scenario, params)
        self.controller.set_target_speed(float(params.get("ego_speed", 50.0)))
        self.arbiter.reset()
        self.seq_a = self.seq_b = self._sensor_seq = 0
        self.collided = False
        self._last_active = CTRL_NANO_A
        self._takeovers = 0
        self._active_committed = CTRL_NANO_A
        self._active_candidate = CTRL_NANO_A
        self._active_stable = 0
        if self.control_source == "nano":
            if self.nano_link is not None:
                self.nano_link.close()
            self.nano_link = NanoLink(self.gateway_host, self.gateway_port)
            self.set_runtime_params(self.params)

    def set_runtime_params(self, params: Dict[str, Any]) -> None:
        self.params.update(params)
        if self.control_source == "nano" and self.nano_link is not None:
            self.nano_link.update_runtime_params(self.params)

    # ──────────────────────────────────────────────────────────
    def step(self, dt: float, sim_t: float,
             fault_injector: FaultInjector) -> Tuple[StateFrame, List[dict]]:
        if self.control_source == "nano":
            return self._step_nano(dt, sim_t)
        return self._step_internal(dt, sim_t, fault_injector)

    # ── 内置控制器路径（软件模拟主备 + 故障）──
    def _step_internal(self, dt: float, sim_t: float,
                       fault_injector: FaultInjector) -> Tuple[StateFrame, List[dict]]:
        self.world.tick()
        perc = self.world.sense()
        self.controller.set_target_speed(float(self.params.get("ego_speed", 50.0)))
        cmd = self.controller.compute(perc)

        self.seq_a += 1
        self.seq_b += 1
        comm_delay = float(self.params.get("comm_delay_ms", 0.0))
        thr, brk = _accel_to_pedals(-cmd["a_brake"], perc["ego_v"])
        steer_disp = cmd["delta"]
        nano_a = ControllerState(alive=True, seq=self.seq_a, latency_ms=NOMINAL_LAT_MS + comm_delay,
                                 valid_output=True, last_control_time=round(sim_t, 3),
                                 throttle=thr, brake=brk, steer=steer_disp)
        nano_b = ControllerState(alive=True, seq=self.seq_b, latency_ms=NOMINAL_LAT_MS + comm_delay,
                                 valid_output=True, last_control_time=round(sim_t, 3),
                                 throttle=thr, brake=brk, steer=steer_disp)
        frame = StateFrame(t=round(sim_t, 3), nano_a=nano_a, nano_b=nano_b)

        fault_injector.apply(frame)
        events = self.arbiter.arbitrate(frame, sim_t)

        active = frame.esp32.active_controller
        if active == CTRL_SAFE_BRAKE:
            delta, a_brake = 0.0, BRK_GAIN
        else:
            delta, a_brake = cmd["delta"], cmd["a_brake"]
        steer, throttle, brake = self.world.apply(delta, a_brake)

        self.world.drive_lead(sim_t)
        self.world.update_spectator()

        frame.esp32.throttle, frame.esp32.brake, frame.esp32.steer = throttle, brake, steer
        if self.world.manual:
            frame.esp32.active_controller = "manual"
        self._fill_observation(frame, perc, throttle, brake, steer, sim_t, events)
        if frame.event is None and events:
            frame.event = events[-1]["type"]
        return frame, events

    # ── 真实 Nano + ESP32 路径 ──
    def _step_nano(self, dt: float, sim_t: float) -> Tuple[StateFrame, List[dict]]:
        self.world.tick()
        perc = self.world.sense()

        # 上行：CARLA 真值感知 → 网关（→ ROS2 → 双 Nano ADAS.py → ESP32）
        self._sensor_seq += 1
        if self.nano_link is not None:
            self.nano_link.send_sensor(build_sensor_payload(self._sensor_seq, perc["raw"]))
            ctl = self.nano_link.get_control()
        else:
            ctl = {"delta": 0.0, "a_brake": 6.0, "active_role": "unknown",
                   "failover_available": False, "actuation_stale_ms": 0, "stale": True}

        # 下行：ESP32 仲裁后的最终控制 → CARLA ego
        delta, a_brake = ctl["delta"], ctl["a_brake"]
        steer, throttle, brake = self.world.apply(delta, a_brake)
        self.world.drive_lead(sim_t)
        self.world.update_spectator()

        # active_controller：来自真实 active_role / 链路状态。
        # ADAS 角色字符串：primary（主控）/ secondary_active（备控接管中）/ secondary_standby（备控待机）。
        # 主备同发 /jetson/active_role，网关 last-write-wins，主控健康时会在
        # 'primary'↔'secondary_standby' 抖动 → 两者都算「主控在驾驶」(nano_a)；
        # 只有 'secondary_active'（主控已哑、备控真正接管）才算 nano_b。
        role = ctl.get("active_role", "unknown")
        stale = bool(ctl.get("stale", False))
        failover = bool(ctl.get("failover_available", False))
        if self.world.manual:
            raw_active = "manual"
        elif stale:
            raw_active = CTRL_SAFE_BRAKE      # 链路丢失 → ego 已被安全制动
        elif role == "secondary_active":
            raw_active = CTRL_NANO_B
        else:
            raw_active = CTRL_NANO_A          # 'primary' / 'secondary_standby' / 启动瞬态

        # 去抖：新状态需连续 ACTIVE_DEBOUNCE 拍稳定才提交，滤掉启动/恢复瞬态
        if raw_active == self._active_candidate:
            self._active_stable += 1
        else:
            self._active_candidate = raw_active
            self._active_stable = 1
        events: List[dict] = []
        if self._active_stable >= ACTIVE_DEBOUNCE and raw_active != self._active_committed:
            prev = self._active_committed
            self._active_committed = raw_active
            if raw_active in (CTRL_NANO_B, CTRL_SAFE_BRAKE) and prev == CTRL_NANO_A:
                self._takeovers += 1
                if raw_active == CTRL_NANO_B:
                    events.append({"time": round(sim_t, 3), "type": "TAKEOVER",
                                   "from": CTRL_NANO_A, "to": CTRL_NANO_B, "reason": "primary_down"})
                else:
                    events.append({"time": round(sim_t, 3), "type": "SAFE_BRAKE", "reason": "link_lost"})
        active = self._active_committed

        # 双 Nano 遥测（active 一侧为实测，另一侧为热备推断）。
        # seq 只在该侧存活时递增——主控被杀后其 seq 会冻结（与真实“假活/失活”一致）。
        lat = float(ctl.get("actuation_stale_ms", 0))
        primary_active = (active == CTRL_NANO_A)
        backup_active = (active == CTRL_NANO_B)
        primary_alive = primary_active          # 主控只在驾驶时算存活；备控接管即主控已哑
        backup_alive = backup_active or failover
        if primary_alive:
            self.seq_a += 1
        if backup_alive:
            self.seq_b += 1
        nano_a = ControllerState(
            alive=primary_alive, seq=self.seq_a,
            latency_ms=lat if primary_active else None, valid_output=primary_alive and not stale,
            last_control_time=round(sim_t, 3),
            throttle=throttle if primary_active else None,
            brake=brake if primary_active else None,
            steer=steer if primary_active else None)
        nano_b = ControllerState(
            alive=backup_alive, seq=self.seq_b,
            latency_ms=lat if backup_active else None, valid_output=backup_alive,
            last_control_time=round(sim_t, 3),
            throttle=throttle if backup_active else None,
            brake=brake if backup_active else None,
            steer=steer if backup_active else None)
        esp32 = Esp32State(
            active_controller=active, takeover_count=self._takeovers,
            last_takeover_reason=("primary_down" if active == CTRL_NANO_B else
                                  "link_lost" if active == CTRL_SAFE_BRAKE else None),
            safe_brake=(active == CTRL_SAFE_BRAKE),
            throttle=throttle, brake=brake, steer=steer)
        frame = StateFrame(t=round(sim_t, 3), nano_a=nano_a, nano_b=nano_b, esp32=esp32)

        self._fill_observation(frame, perc, throttle, brake, steer, sim_t, events)
        if frame.event is None and events:
            frame.event = events[-1]["type"]
        return frame, events

    # ── 公共：填充 ego / target / 碰撞 ──
    def _fill_observation(self, frame: StateFrame, perc: Dict[str, Any],
                          throttle: float, brake: float, steer: float,
                          sim_t: float, events: List[dict]) -> None:
        ego_v = perc["ego_v"]
        frame.ego = EgoState(
            speed_kmh=round(ego_v * 3.6, 3), throttle=throttle, brake=brake, steer=steer,
            lateral_error=round(perc["lane_offset"], 4),
            heading_error=round(perc["heading_error"], 4))
        gap = perc["gap"]
        if perc["lead_present"] and gap != float("inf"):
            closing = ego_v - perc["lead_v"]
            ttc = gap / closing if closing > 0.05 else float("inf")
            frame.target = TargetState(
                front_distance=round(max(0.0, gap), 3),
                relative_speed=round((perc["lead_v"] - ego_v) * 3.6, 3),
                ttc=round(ttc, 3) if ttc != float("inf") else float("inf"))
        else:
            frame.target = TargetState(front_distance=float("inf"), relative_speed=0.0, ttc=float("inf"))

        if self.world.collision_occurred() and not self.collided:
            self.collided = True
            frame.event = "COLLISION"
            events.append({"time": round(sim_t, 3), "type": "COLLISION"})

    def reset(self) -> None:
        if self.nano_link is not None:
            self.nano_link.close()
            self.nano_link = None
        self.world.reset()
        self.arbiter.reset()
        self.seq_a = self.seq_b = self._sensor_seq = 0
        self.collided = False
        self._last_active = CTRL_NANO_A
        self._takeovers = 0
        self._active_committed = CTRL_NANO_A
        self._active_candidate = CTRL_NANO_A
        self._active_stable = 0

    def close(self) -> None:
        if self.nano_link is not None:
            self.nano_link.close()
            self.nano_link = None
        self.world.close()
