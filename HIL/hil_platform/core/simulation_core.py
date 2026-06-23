# -*- coding: utf-8 -*-
"""SimulationCore：全平台唯一的仿真编排器。

需求关键约束：
- 底层 CARLA/Nano/ESP32 的控制权只由本类持有；CLI/Web/REST 都经此入口。
- 只有一处 tick、一处 actor 管理，避免状态不同步与 actor 残留。
- reset 时清理：bridge 状态、fault flag、metrics buffer、recorder buffer。
- stop 时自动保存 meta/states/events/summary/report。

线程模型：
- 一个后台循环线程，在 RUNNING 时按固定步长推进 bridge、跑指标、写 recorder、
  更新"最新帧"。PAUSED 时线程保留但不推进。
- 服务层（WebSocket）只读 get_live_payload()，不直接驱动仿真。
"""

from __future__ import annotations

import os
import threading
import time
from typing import Any, Dict, List, Optional

from .fault_injector import FaultInjector
from .hil_bridge import HilBridge, MockHilBridge
from .metrics import Metrics
from .parameter_manager import ParameterManager
from .recorder import Recorder, generate_run_id
from .scenario_manager import Scenario, ScenarioManager
from .state_machine import InvalidTransition, SimState, StateMachine
from .types import StateFrame

# 仿真步长：20Hz（与 CARLA 同步 0.05s 一致）
SIM_DT = 0.05


class SimulationCore:
    def __init__(self, mock: bool = True, bridge: Optional[HilBridge] = None):
        self.mock = mock
        self.bridge: HilBridge = bridge or self._make_default_bridge(mock)
        self.sm = StateMachine(SimState.IDLE)
        self.scenario_mgr = ScenarioManager()
        self.param_mgr = ParameterManager()
        self.fault_injector = FaultInjector()
        self.metrics = Metrics()
        self.recorder = Recorder()

        self._lock = threading.RLock()
        self._loop_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()

        self.scenario: Optional[Scenario] = None
        self.run_id: str = ""
        self.sim_t: float = 0.0
        self._latest: Optional[StateFrame] = None
        self._preset_fault_fired = False
        self._last_meta: Optional[Dict[str, Any]] = None
        self._loop_error: str = ""
        self._nano_fault = None   # 真实硬件故障注入器（仅 nano 模式惰性创建）

        # mock 模式即视为"已连接"
        if self.mock:
            self.sm.force(SimState.IDLE)

    @staticmethod
    def _make_default_bridge(mock: bool) -> HilBridge:
        if mock:
            return MockHilBridge()
        # 真实 CARLA：惰性导入，避免 mock 环境也加载 carla
        from .real_hil_bridge import RealHilBridge
        return RealHilBridge(
            host=os.environ.get("CARLA_HOST", "127.0.0.1"),
            port=int(os.environ.get("CARLA_PORT", "2000")),
            town=os.environ.get("CARLA_TOWN", "Town04"),
            enable_camera=os.environ.get("HIL_CAMERA", "1") != "0",
            # HIL_CONTROL: 'internal'(平台内置控制器) | 'nano'(真实双 Nano+ESP32)
            control_source=os.environ.get("HIL_CONTROL", "internal"),
            gateway_host=os.environ.get("GATEWAY_HOST", "192.168.3.125"),
            gateway_port=int(os.environ.get("GATEWAY_PORT", "42110")),
        )

    def _carla_world(self):
        """取真实 CARLA 世界（mock 模式返回 None）。"""
        try:
            from .real_hil_bridge import RealHilBridge
        except Exception:
            return None
        return self.bridge.world if isinstance(self.bridge, RealHilBridge) else None

    def _nano_fault_ctrl(self):
        """真实双 Nano 故障注入器（仅 control_source='nano' 时；否则 None → 用本地 FaultInjector）。"""
        try:
            from .real_hil_bridge import RealHilBridge
        except Exception:
            return None
        if not isinstance(self.bridge, RealHilBridge) or self.bridge.control_source != "nano":
            return None
        if self._nano_fault is None:
            from .nano_fault import NanoFaultController
            self._nano_fault = NanoFaultController(
                primary_host=os.environ.get("GATEWAY_HOST", "192.168.3.125"),
                backup_host=os.environ.get("BACKUP_HOST", "192.168.3.124"),
                primary_pw=os.environ.get("NANO_PW_PRIMARY", "yahboom"),
                backup_pw=os.environ.get("NANO_PW_BACKUP", "jetson"),
                user=os.environ.get("NANO_USER", "jetson"),
                auto_restore_s=float(os.environ.get("NANO_FAULT_RESTORE_S", "8")),
            )
            # 后台预连两台 Nano，使首次故障注入也无连接延迟
            threading.Thread(target=self._nano_fault.warmup, daemon=True).start()
        return self._nano_fault

    # ──────────────────────────────────────────────────────────
    # 自由操控世界（仅真实 CARLA 模式有效）
    # ──────────────────────────────────────────────────────────
    def world_command(self, action: str, **kwargs) -> Dict[str, Any]:
        world = self._carla_world()
        if world is None:
            raise RuntimeError("当前为 mock 模式，无 CARLA 世界可操控（启动后端时设 HIL_MOCK=0）")
        with self._lock:
            if action == "weather":
                world.set_weather(str(kwargs.get("weather", "clear")))
                return {"weather": kwargs.get("weather")}
            if action == "spawn_npc":
                n = world.spawn_npc(int(kwargs.get("count", 5)))
                return {"spawned": n}
            if action == "clear_npc":
                return {"cleared": world.clear_npc()}
            if action == "lead_speed":
                spd = kwargs.get("kmh")
                world.set_lead_speed(None if spd in (None, "") else float(spd))
                return {"lead_speed_kmh": spd}
            if action == "manual":
                world.set_manual(bool(kwargs.get("on", False)))
                return {"manual": world.manual}
            if action == "manual_cmd":
                world.manual_cmd(float(kwargs.get("throttle", 0.0)),
                                 float(kwargs.get("brake", 0.0)),
                                 float(kwargs.get("steer", 0.0)))
                return {"ok": True}
            raise ValueError("未知世界操作：%s" % action)

    def _restore_nanos(self) -> None:
        """安全兜底：恢复（SIGCONT）两台 Nano，确保停止/复位后不留冻结进程。"""
        if self._nano_fault is not None:
            try:
                self._nano_fault.restore_all()
            except Exception:
                pass

    def camera_jpeg(self) -> Optional[bytes]:
        world = self._carla_world()
        return world.camera_jpeg() if world is not None else None

    def camera_path(self) -> Optional[str]:
        world = self._carla_world()
        return world.camera_path() if world is not None else None

    # ──────────────────────────────────────────────────────────
    # 实时控制接口
    # ──────────────────────────────────────────────────────────
    def load_scenario(self, name: str, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        with self._lock:
            if self.sm.state == SimState.RUNNING:
                raise RuntimeError("仿真运行中，请先 stop 再加载场景")
            self.scenario = self.scenario_mgr.load(name)
            merged = self.param_mgr.load_scenario_defaults(self.scenario.default_params)
            if params:
                merged = self.param_mgr.apply(params)
            self.bridge.load(name, merged)
            self.fault_injector.clear_all()
            self.metrics.reset()
            self.sim_t = 0.0
            self._latest = None
            self._preset_fault_fired = False
            self._loop_error = ""
            # IDLE/STOPPED/READY/ERROR -> READY
            if self.sm.state in (SimState.STOPPED, SimState.ERROR):
                self.sm.force(SimState.READY)
            elif self.sm.state == SimState.IDLE:
                self.sm.transition(SimState.READY)
            else:
                self.sm.force(SimState.READY)
            # nano 模式：预创建并后台预连故障注入器，使首次注入近乎瞬时
            self._nano_fault_ctrl()
            return self.status()

    def start(self) -> Dict[str, Any]:
        with self._lock:
            if self.sm.state == SimState.PAUSED:
                self.sm.transition(SimState.RUNNING)
                return self.status()
            if self.sm.state != SimState.READY:
                raise InvalidTransition("仅 READY/PAUSED 可 start，当前 %s" % self.sm.state.value)
            # 开新一轮：生成 run_id，启动 recorder + 循环线程
            self.run_id = generate_run_id(self.scenario.name)  # type: ignore[union-attr]
            self.recorder.begin(
                self.run_id, self.scenario.name, self.scenario.map,  # type: ignore[union-attr]
                self.param_mgr.params)
            self.metrics.reset()
            self.sim_t = 0.0
            self._preset_fault_fired = False
            self.sm.transition(SimState.RUNNING)
            self._start_loop()
            return self.status()

    def pause(self) -> Dict[str, Any]:
        with self._lock:
            if self.sm.state != SimState.RUNNING:
                raise InvalidTransition("仅 RUNNING 可 pause")
            self.sm.transition(SimState.PAUSED)
            return self.status()

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            if self.sm.state not in (SimState.RUNNING, SimState.PAUSED):
                raise InvalidTransition("仅 RUNNING/PAUSED 可 stop")
            self._stop_loop()
            self._restore_nanos()
            self.sm.transition(SimState.STOPPED)
            meta = self._finalize_record()
            self._last_meta = meta
            try:
                self.bridge.reset()
            except Exception as exc:
                self._loop_error = "停止后清理底层资源失败：%r" % exc
            return self.status()

    def reset(self) -> Dict[str, Any]:
        """清理一切，回到 READY（若有已加载场景）或 IDLE。"""
        with self._lock:
            self._stop_loop()
            self._restore_nanos()
            self.fault_injector.clear_all()
            self.metrics.reset()
            self.recorder.reset()
            self.bridge.reset()
            self.sim_t = 0.0
            self._latest = None
            self._preset_fault_fired = False
            self._loop_error = ""
            self.run_id = ""
            if self.scenario is not None:
                # 重新按当前参数装载场景；CARLA 仍不可用时优雅退回 IDLE，不让 reset 抛错
                try:
                    self.bridge.load(self.scenario.name, self.param_mgr.params)
                    self.sm.force(SimState.READY)
                except Exception as exc:
                    self._loop_error = "复位重载场景失败：%r" % exc
                    self.scenario = None
                    self.sm.force(SimState.IDLE)
            else:
                self.sm.force(SimState.IDLE)
            return self.status()

    def update_parameters(self, updates: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            params = self.param_mgr.apply(updates)
            # 热更新部分参数到 bridge（不改变 actor，仅改目标量）
            if isinstance(self.bridge, MockHilBridge):
                if "ego_speed" in updates:
                    self.bridge.v_set = float(params["ego_speed"]) / 3.6
            else:
                # 真实 CARLA：更新内置控制器目标速度 + 天气
                world = self._carla_world()
                if world is not None and hasattr(self.bridge, "controller"):
                    if "ego_speed" in updates:
                        self.bridge.controller.set_target_speed(float(params["ego_speed"]))
                    if "weather" in updates:
                        try:
                            world.set_weather(str(params["weather"]))
                        except Exception:
                            pass
            return {"params": params}

    def inject_fault(self, fault_type: str, target: str = "nano_a") -> Dict[str, Any]:
        with self._lock:
            fc = self._nano_fault_ctrl()
            if fc is not None:
                # 真实硬件：SSH 冻结目标 Nano（真断心跳）→ 真实 ESP32 接管，到点自动恢复
                evt = fc.fault(fault_type, target, self.sim_t)
                if evt.get("type") == "FAULT_BUSY":
                    return {"event": evt}   # 窗口内重复注入：不记录、不计指标
                self.recorder.append_event(evt)
                self.metrics.on_fault_injected(evt["time"])
                return {"event": evt}
            evt = self.fault_injector.inject(fault_type, target, self.sim_t)
            # 立即记录 + 通知 metrics（pop 在循环里也会做，这里保证暂停时也记录）
            self.metrics.on_fault_injected(evt["time"])
            return {"event": evt}

    # ──────────────────────────────────────────────────────────
    # 状态/指标查询
    # ──────────────────────────────────────────────────────────
    def status(self) -> Dict[str, Any]:
        with self._lock:
            esp = self._latest.esp32 if self._latest else None
            return {
                "state": self.sm.state.value,
                "mock": self.mock,
                "control_source": getattr(self.bridge, "control_source", "mock" if self.mock else "unknown"),
                "run_id": self.run_id or None,
                "scenario": self.scenario.name if self.scenario else None,
                "scenario_title": self.scenario.title if self.scenario else None,
                "map": self.scenario.map if self.scenario else None,
                "scenario_time": round(self.sim_t, 3),
                "active_controller": esp.active_controller if esp else "none",
                "takeover": (esp.active_controller in ("nano_b", "safe_brake")) if esp else False,
                "safe_brake": esp.safe_brake if esp else False,
                "params": self.param_mgr.params,
                "active_faults": self.fault_injector.active_faults(),
                "frame_count": self.recorder.frame_count,
                "error": self._loop_error or None,
            }

    def metrics_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            scn = self.scenario.name if self.scenario else ""
            snap = self.metrics.summary(scn)
            snap["scenario_time"] = round(self.sim_t, 3)
            return snap

    def get_live_payload(self) -> Optional[Dict[str, Any]]:
        """供 /ws/live 推送的最新帧。无数据时返回 None（前端做空值保护）。"""
        with self._lock:
            if self._latest is None:
                return None
            return self._latest.to_ws_dict(
                self.run_id or "",
                self.scenario.name if self.scenario else "",
                self.sm.state.value,
            )

    @property
    def last_meta(self) -> Optional[Dict[str, Any]]:
        return self._last_meta

    # ──────────────────────────────────────────────────────────
    # 内部：后台循环
    # ──────────────────────────────────────────────────────────
    def _start_loop(self) -> None:
        self._stop_flag.clear()
        self._loop_thread = threading.Thread(
            target=self._loop, name="sim-core-loop", daemon=True)
        self._loop_thread.start()

    def _stop_loop(self) -> None:
        self._stop_flag.set()
        t = self._loop_thread
        if t is not None and t.is_alive() and threading.current_thread() is not t:
            t.join(timeout=2.0)
        self._loop_thread = None

    def _loop(self) -> None:
        next_wall = time.monotonic()
        while not self._stop_flag.is_set():
            state = self.sm.state
            if state == SimState.RUNNING:
                try:
                    self._tick()
                except Exception as exc:
                    # CARLA 中途断连/崩溃等：不让循环线程静默死掉、不拖垮后端进程。
                    # 进 ERROR 态，停循环；已记录的数据保留，用户可 reset 重来。
                    with self._lock:
                        self._loop_error = "仿真循环异常：%r" % exc
                        self.sm.force(SimState.ERROR)
                    break
            # 实时步进（mock 按墙钟 20Hz 推进）
            next_wall += SIM_DT
            sleep = next_wall - time.monotonic()
            if sleep > 0:
                time.sleep(min(sleep, SIM_DT))
            else:
                next_wall = time.monotonic()

    def _tick(self) -> None:
        with self._lock:
            self.sim_t += SIM_DT
            sim_t = self.sim_t
            # 预设故障：到点自动注入一次（逻辑仍在 fault_injector）
            self._maybe_fire_preset_fault(sim_t)
            frame, events = self.bridge.step(SIM_DT, sim_t, self.fault_injector)

            # 故障注入事件（手动/预设）写入记录 + 指标
            for fe in self.fault_injector.pop_pending_events():
                self.recorder.append_event(fe)
                self.metrics.on_fault_injected(fe["time"])
                if frame.event is None:
                    frame.event = "FAULT_INJECTED"

            # 仲裁/碰撞事件
            for e in events:
                self.recorder.append_event(e)
                if e["type"] == "TAKEOVER":
                    self.metrics.on_takeover(e["time"])

            self.metrics.update(frame)
            self.recorder.append_frame(frame)
            self._latest = frame

    def _maybe_fire_preset_fault(self, sim_t: float) -> None:
        if self._preset_fault_fired:
            return
        ftype = self.param_mgr.params.get("fault_type", "none")
        ftime = float(self.param_mgr.params.get("fault_trigger_time", 0.0) or 0.0)
        if ftype and ftype != "none" and ftime > 0 and sim_t >= ftime:
            try:
                fc = self._nano_fault_ctrl()
                if fc is not None:
                    evt = fc.fault(ftype, "nano_a", sim_t)
                    if evt.get("type") != "FAULT_BUSY":
                        self.recorder.append_event(evt)
                        self.metrics.on_fault_injected(evt["time"])
                else:
                    self.fault_injector.inject(ftype, "nano_a", sim_t)
            except ValueError:
                pass
            self._preset_fault_fired = True

    def _finalize_record(self) -> Dict[str, Any]:
        scn = self.scenario.name if self.scenario else ""
        summary = self.metrics.summary(scn)
        derived = self.metrics.derived_events()
        meta = self.recorder.finalize(summary, derived)
        return meta
