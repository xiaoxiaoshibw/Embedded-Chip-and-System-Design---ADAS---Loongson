# -*- coding: utf-8 -*-
"""HIL 桥接层：抽象接口 HilBridge + 默认实现 MockHilBridge + ESP32 仲裁器。

- HilBridge 抽象了"推进一帧仿真并产出 StateFrame"的能力。SimulationCore 只依赖
  这个接口，因此真实 CARLA+双 Nano+ESP32 链路（RealHilBridge）可平滑替换，无需
  改动上层（需求第 3、4 条）。
- MockHilBridge：无 CARLA/Nano/ESP32 也能跑通的纯软件仿真，模拟速度/TTC 变化、
  Nano A seq 卡死、ESP32 切换 Nano B、安全制动（需求 mock mode）。
- Esp32Arbiter：模拟 ESP32 的仲裁逻辑（心跳/seq 递增/合法性/延迟 + 接管时延），
  读取"被 fault_injector 作用后"的 Nano 状态做最终选择。
"""

from __future__ import annotations

import math
import random
from typing import Any, Dict, List, Optional, Tuple

from .fault_injector import FaultInjector
from .types import (
    CTRL_NANO_A,
    CTRL_NANO_B,
    CTRL_NONE,
    CTRL_SAFE_BRAKE,
    ControllerState,
    EgoState,
    Esp32State,
    StateFrame,
    TargetState,
)

# ── 物理/控制常量 ──
THR_GAIN = 3.0            # 油门=1.0 ≈ 3 m/s²
BRK_GAIN = 8.0           # 刹车=1.0 ≈ 8 m/s²
TIME_GAP = 1.6           # ACC 时距 (s)
MIN_GAP = 6.0            # 最小跟车距离 (m)
AEB_TTC = 2.2            # TTC 低于此值触发 AEB 全力制动 (s)
NOMINAL_LAT_MS = 20.0    # Nano 标称控制延迟
LATENCY_HEALTHY_MS = 300.0   # 仲裁器认为健康的延迟上限（comm_delay 仍健康，control_delay 故障超限）
TAKEOVER_GRACE_S = 0.10      # 仲裁判定接管的容错窗（叠加 1 个检测 tick ≈ 150ms 接管时延）


def _kmh(mps: float) -> float:
    return mps * 3.6


def _mps(kmh: float) -> float:
    return kmh / 3.6


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _accel_to_pedals(a_des: float, v: float) -> Tuple[float, float]:
    """期望加速度 → (throttle, brake)。"""
    if a_des >= 0:
        return _clamp(a_des / THR_GAIN, 0.0, 1.0), 0.0
    return 0.0, _clamp(-a_des / BRK_GAIN, 0.0, 1.0)


class Esp32Arbiter:
    """模拟 ESP32 仲裁器：在被故障作用后的 Nano 状态上选最终控制器。"""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.active = CTRL_NONE
        self.takeover_count = 0
        self.last_takeover_reason: Optional[str] = None
        self._last_seq = {CTRL_NANO_A: None, CTRL_NANO_B: None}
        self._unhealthy_since = {CTRL_NANO_A: None, CTRL_NANO_B: None}

    def _health(self, name: str, ctrl: ControllerState, sim_t: float) -> Tuple[bool, Optional[str]]:
        """返回 (是否健康, 不健康原因)。原因用于 last_takeover_reason。"""
        reason = None
        if not ctrl.alive:
            reason = "heartbeat_lost"
        elif not ctrl.valid_output:
            reason = "invalid_output"
        elif ctrl.latency_ms > LATENCY_HEALTHY_MS:
            reason = "control_timeout"
        else:
            last = self._last_seq[name]
            if last is not None and ctrl.seq <= last:
                reason = "seq_not_increasing"
        healthy = reason is None
        # 维护"不健康起始时刻"，用于容错窗判定（模拟检测+接管时延）
        if healthy:
            self._unhealthy_since[name] = None
        elif self._unhealthy_since[name] is None:
            self._unhealthy_since[name] = sim_t
        return healthy, reason

    def _confirmed_down(self, name: str, sim_t: float) -> bool:
        since = self._unhealthy_since[name]
        return since is not None and (sim_t - since) >= TAKEOVER_GRACE_S

    def arbitrate(self, frame: StateFrame, sim_t: float) -> List[dict]:
        """决定 active_controller 并写入 frame.esp32；返回本帧产生的事件。"""
        a_ok, a_reason = self._health(CTRL_NANO_A, frame.nano_a, sim_t)
        b_ok, b_reason = self._health(CTRL_NANO_B, frame.nano_b, sim_t)

        # 选择逻辑：优先主控；主控确认失活后切备控；都不可用 → 安全制动
        if a_ok:
            chosen = CTRL_NANO_A
            reason = None
        elif self._confirmed_down(CTRL_NANO_A, sim_t) and b_ok:
            chosen = CTRL_NANO_B
            reason = a_reason
        elif self._confirmed_down(CTRL_NANO_A, sim_t) and not b_ok:
            chosen = CTRL_SAFE_BRAKE
            reason = b_reason or a_reason
        else:
            # 主控刚异常但未确认（容错窗内）→ 维持当前；首帧默认主控
            chosen = self.active if self.active != CTRL_NONE else CTRL_NANO_A

        events: List[dict] = []
        prev = self.active
        if chosen != prev and prev != CTRL_NONE:
            # 记录接管/安全制动事件
            if chosen in (CTRL_NANO_B, CTRL_SAFE_BRAKE) and prev != chosen:
                self.takeover_count += 1
                self.last_takeover_reason = reason
            if chosen == CTRL_NANO_B and prev == CTRL_NANO_A:
                events.append({
                    "time": round(sim_t, 3), "type": "TAKEOVER",
                    "from": CTRL_NANO_A, "to": CTRL_NANO_B,
                    "reason": reason or "unknown",
                })
            elif chosen == CTRL_SAFE_BRAKE:
                events.append({
                    "time": round(sim_t, 3), "type": "SAFE_BRAKE",
                    "reason": reason or "dual_fail",
                })
        self.active = chosen

        # 计算最终输出
        if chosen == CTRL_NANO_A:
            src = frame.nano_a
            out = (src.throttle, src.brake, src.steer)
        elif chosen == CTRL_NANO_B:
            src = frame.nano_b
            out = (src.throttle, src.brake, src.steer)
        else:  # safe_brake
            out = (0.0, 1.0, 0.0)

        frame.esp32 = Esp32State(
            active_controller=chosen,
            takeover_count=self.takeover_count,
            last_takeover_reason=self.last_takeover_reason,
            safe_brake=(chosen == CTRL_SAFE_BRAKE),
            throttle=out[0], brake=out[1], steer=out[2],
        )

        # 更新 last_seq（放在最后，保证本帧用的是上一帧的值做比较）
        self._last_seq[CTRL_NANO_A] = frame.nano_a.seq
        self._last_seq[CTRL_NANO_B] = frame.nano_b.seq
        return events


class HilBridge:
    """桥接抽象接口。"""

    def load(self, scenario: str, params: Dict[str, Any]) -> None:
        raise NotImplementedError

    def step(self, dt: float, sim_t: float,
             fault_injector: FaultInjector) -> Tuple[StateFrame, List[dict]]:
        raise NotImplementedError

    def reset(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


class MockHilBridge(HilBridge):
    """纯软件仿真：无需 CARLA/Nano/ESP32。"""

    def __init__(self):
        self.arbiter = Esp32Arbiter()
        self._rng = random.Random(12345)
        self.reset()

    def reset(self) -> None:
        self.scenario = ""
        self.params: Dict[str, Any] = {}
        self.ego_v = 0.0          # m/s
        self.v_set = _mps(50.0)   # 目标速度
        self.front_present = False
        self.front_v = 0.0        # m/s
        self.front_distance = float("inf")
        self.lateral_error = 0.0
        self.heading_error = 0.0
        self.seq_a = 0
        self.seq_b = 0
        self.collided = False
        self._cut_in_done = False
        self.arbiter.reset()

    def load(self, scenario: str, params: Dict[str, Any]) -> None:
        self.reset()
        self.scenario = scenario
        self.params = dict(params)
        self.v_set = _mps(float(params.get("ego_speed", 50.0)))
        self.ego_v = self.v_set * 0.6   # 以 60% 目标速度起步
        self.front_v = _mps(float(params.get("front_speed", 35.0)))
        self.front_distance = float(params.get("front_distance", 40.0))

        # 场景差异化初始化
        if scenario == "lka_curve":
            self.front_present = False
            self.front_distance = float("inf")
        elif scenario == "cut_in":
            # 切入车初始在远处/不可见，触发后才出现
            self.front_present = False
            self.front_distance = float("inf")
        else:
            self.front_present = True

    # ── 感知噪声 ──
    def _noise(self, scale: float) -> float:
        sn = float(self.params.get("sensor_noise", 0.0))
        if sn <= 0:
            return 0.0
        return self._rng.gauss(0.0, scale * sn)

    # ── 场景驱动的前车行为 ──
    def _update_front(self, dt: float, sim_t: float) -> None:
        if self.scenario == "aeb_brake":
            # 到达预设急刹时刻（默认 8s）前车全力急刹
            brake_t = float(self.params.get("fault_trigger_time", 0.0)) or 8.0
            if sim_t >= brake_t:
                self.front_v = max(0.0, self.front_v - 9.0 * dt)
        elif self.scenario == "cut_in":
            trig = float(self.params.get("cut_in_trigger_distance", 25.0))
            if not self._cut_in_done and sim_t >= 3.0:
                # 一辆车以 cut_in_speed 切入到 trig 距离处
                self.front_present = True
                self.front_distance = trig
                self.front_v = _mps(float(self.params.get("cut_in_speed", 40.0)))
                self._cut_in_done = True
                self.lateral_error += 0.35   # 切入带来横向扰动
        elif self.scenario in ("acc_follow", "takeover", "failover"):
            # 前车做温和变速，便于观察跟车自适应
            base = _mps(float(self.params.get("front_speed", 35.0)))
            self.front_v = base * (1.0 + 0.18 * math.sin(0.12 * sim_t))

        if self.front_present and self.front_distance != float("inf"):
            self.front_distance += (self.front_v - self.ego_v) * dt

    # ── 纵向控制（Nano 的 ACC/AEB 计算）──
    def _longitudinal_cmd(self) -> float:
        if not self.front_present or self.front_distance == float("inf"):
            # 无前车 → 巡航到目标速度
            return _clamp(0.8 * (self.v_set - self.ego_v), -3.0, 2.5)
        desired_gap = max(MIN_GAP, TIME_GAP * self.ego_v)
        gap_err = self.front_distance - desired_gap
        closing = self.ego_v - self.front_v
        ttc = self._ttc()
        # AEB：TTC 过小 → 全力制动
        if ttc < AEB_TTC:
            return -BRK_GAIN
        a = 0.45 * gap_err - 1.1 * closing
        return _clamp(a, -BRK_GAIN, 2.5)

    # ── 横向控制（LKA）──
    def _lateral_cmd(self, dt: float, sim_t: float) -> float:
        # 道路扰动：弯道场景给出正弦曲率
        if self.scenario == "lka_curve":
            road_curv = 0.25 * math.sin(0.30 * sim_t)
        else:
            road_curv = 0.0
        # 横向误差一阶动力学：扰动推 + 控制拉回
        steer = -1.4 * self.lateral_error - 0.6 * self.heading_error
        self.heading_error += dt * (road_curv - 0.8 * self.heading_error) + self._noise(0.01)
        self.lateral_error += dt * (self.ego_v * 0.04 * self.heading_error
                                    + road_curv * 0.6 - 1.2 * self.lateral_error)
        self.lateral_error += self._noise(0.01)
        return _clamp(steer, -1.0, 1.0)

    def _ttc(self) -> float:
        if not self.front_present or self.front_distance == float("inf"):
            return float("inf")
        closing = self.ego_v - self.front_v
        if closing <= 0.05:
            return float("inf")
        return self.front_distance / closing

    def step(self, dt: float, sim_t: float,
             fault_injector: FaultInjector) -> Tuple[StateFrame, List[dict]]:
        # 1) 前车 / 道路推进
        self._update_front(dt, sim_t)

        # 2) Nano 控制计算（A、B 同核，B 为热备影子，输出近似）
        a_lon = self._longitudinal_cmd()
        steer = self._lateral_cmd(dt, sim_t)
        thr_a, brk_a = _accel_to_pedals(a_lon, self.ego_v)
        # 备控影子：极小差异，证明双冗余一致性
        thr_b = _clamp(thr_a + self._rng.uniform(-0.01, 0.01), 0.0, 1.0)
        brk_b = _clamp(brk_a + self._rng.uniform(-0.01, 0.01), 0.0, 1.0)
        steer_b = _clamp(steer + self._rng.uniform(-0.005, 0.005), -1.0, 1.0)

        self.seq_a += 1
        self.seq_b += 1
        comm_delay = float(self.params.get("comm_delay_ms", 0.0))
        lat_a = NOMINAL_LAT_MS + comm_delay + abs(self._rng.gauss(0, 3))
        lat_b = NOMINAL_LAT_MS + comm_delay + abs(self._rng.gauss(0, 3))

        nano_a = ControllerState(
            alive=True, seq=self.seq_a, latency_ms=round(lat_a, 1), valid_output=True,
            last_control_time=round(sim_t, 3), throttle=thr_a, brake=brk_a, steer=steer)
        nano_b = ControllerState(
            alive=True, seq=self.seq_b, latency_ms=round(lat_b, 1), valid_output=True,
            last_control_time=round(sim_t, 3), throttle=thr_b, brake=brk_b, steer=steer_b)

        frame = StateFrame(t=round(sim_t, 3), nano_a=nano_a, nano_b=nano_b)

        # 3) 故障注入作用到 Nano 状态（集中在 fault_injector）
        fault_injector.apply(frame)

        # 4) ESP32 仲裁 → 最终控制量
        events = self.arbiter.arbitrate(frame, sim_t)

        # 5) 用最终控制量闭环推进 Ego 纵向
        out_thr = frame.esp32.throttle
        out_brk = frame.esp32.brake
        a_applied = out_thr * THR_GAIN - out_brk * BRK_GAIN
        self.ego_v = max(0.0, self.ego_v + a_applied * dt)

        # 6) 碰撞判定
        if (self.front_present and self.front_distance != float("inf")
                and self.front_distance <= 0.0 and not self.collided):
            self.collided = True
            self.front_distance = 0.0
            frame.event = "COLLISION"
            events.append({"time": round(sim_t, 3), "type": "COLLISION"})

        # 7) 填充 Ego / Target 观测量
        frame.ego = EgoState(
            speed_kmh=round(_kmh(self.ego_v) + self._noise(0.3), 3),
            throttle=out_thr, brake=out_brk, steer=frame.esp32.steer,
            lateral_error=round(self.lateral_error, 4),
            heading_error=round(self.heading_error, 4),
        )
        if self.front_present and self.front_distance != float("inf"):
            frame.target = TargetState(
                front_distance=round(max(0.0, self.front_distance) + self._noise(0.3), 3),
                relative_speed=round(_kmh(self.front_v - self.ego_v), 3),
                ttc=round(self._ttc(), 3) if self._ttc() != float("inf") else float("inf"),
            )
        else:
            frame.target = TargetState(front_distance=float("inf"),
                                       relative_speed=0.0, ttc=float("inf"))

        # 8) 该帧事件标记（仲裁事件优先于无）
        if frame.event is None and events:
            frame.event = events[-1]["type"]
        return frame, events
