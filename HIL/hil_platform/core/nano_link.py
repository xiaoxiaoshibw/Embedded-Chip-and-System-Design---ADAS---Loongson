# -*- coding: utf-8 -*-
"""NanoLink：PC ↔ 真实双 Nano 网关的 TCP 客户端。

复用 `carla_bridge/nano/hil_ros_gateway.py`（已运行在主控 Nano 上，TCP 42110）的现有协议，
**不改 Nano 侧任何代码**：
  上行：把 CARLA 真值感知帧（JSON 行）发给网关 → 网关发布 ROS2 /car1_* 等 →
        两台 Nano 上的 ADAS.py 计算 → ESP32 仲裁；
  下行：网关每 ~50Hz 回执最终控制量（已是 ESP32 仲裁结果）+ active_role +
        failover_available + 时延，本类取最新值。

与 carla_bridge/pc/hil_carla_bridge.py 的 ActuationReceiver 字节级兼容（同一网关）。
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Any, Dict, Optional


class NanoLink:
    def __init__(self, gateway_host: str, tcp_port: int = 42110,
                 stale_timeout_s: float = 0.5, connect_timeout_s: float = 10.0):
        self.gateway_host = gateway_host
        self.tcp_port = tcp_port
        self._stale_timeout_s = float(stale_timeout_s)
        self._lock = threading.Lock()
        self._latest: Optional[Dict[str, Any]] = None
        self._latest_rx_t = 0.0
        self._running = True
        self._send_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._command_seq = 0
        self._runtime_params: Dict[str, Any] = {}
        try:
            self._sock = socket.create_connection(
                (gateway_host, tcp_port), timeout=connect_timeout_s)
        except OSError as exc:
            raise RuntimeError(
                "无法连接 Nano 网关 %s:%d（确认主控 Nano 的 hil_ros_gateway.py 在跑）：%s"
                % (gateway_host, tcp_port, exc))
        self._sock.settimeout(0.2)
        threading.Thread(target=self._recv_loop, name="nanolink-rx", daemon=True).start()

    def _recv_loop(self) -> None:
        buf = ""
        while self._running:
            try:
                data = self._sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            try:
                buf += data.decode("utf-8")
            except UnicodeDecodeError:
                continue
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line)
                except Exception:
                    continue
                with self._lock:
                    self._latest = msg
                    self._latest_rx_t = time.monotonic()

    def send_sensor(self, payload: Dict[str, Any]) -> None:
        with self._command_lock:
            if self._runtime_params:
                payload = dict(payload)
                payload["control"] = {
                    "seq": self._command_seq,
                    "params": dict(self._runtime_params),
                }
        data = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
        with self._send_lock:
            try:
                self._sock.sendall(data)
            except OSError:
                pass

    def update_runtime_params(self, params: Dict[str, Any]) -> int:
        clean: Dict[str, Any] = {}
        for key in ("ego_speed", "target_speed_kmh", "system_max_cruise", "road_limit_speed"):
            if key in params:
                clean[key] = params[key]
        if not clean:
            return self._command_seq
        with self._command_lock:
            self._runtime_params.update(clean)
            self._command_seq += 1
            return self._command_seq

    def get_control(self) -> Dict[str, Any]:
        with self._lock:
            msg = dict(self._latest) if self._latest else None
            age = time.monotonic() - self._latest_rx_t if self._latest_rx_t else 9999.0
        if msg is None or age > self._stale_timeout_s:
            # 链路无新控制 → 安全制动兜底（与网关 stale 语义一致）
            return {
                "delta": 0.0, "a_brake": 6.0, "psi": 0.0,
                "source": "stale", "active_role": "unknown",
                "failover_available": False, "actuation_stale_ms": int(age * 1000),
                "sensor_stale_ms": 0, "age_s": age, "stale": True,
            }
        return {
            "delta": float(msg.get("delta", 0.0)),
            "a_brake": float(msg.get("brake", 0.0)),     # 网关字段名为 brake，正=减速
            "psi": float(msg.get("psi", 0.0)),
            "source": msg.get("source", "unknown"),
            "active_role": msg.get("active_role", "unknown"),
            "failover_available": bool(msg.get("failover_available", False)),
            "actuation_stale_ms": int(msg.get("actuation_stale_ms", 0)),
            "sensor_stale_ms": int(msg.get("sensor_stale_ms", 0)),
            "age_s": age,
            "stale": False,
        }

    def close(self) -> None:
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass


def build_sensor_payload(seq: int, frame: Dict[str, Any]) -> Dict[str, Any]:
    """把 carla_link 原始感知帧转成网关期望的感知载荷（与 hil_carla_bridge 同构）。"""
    present = bool(frame.get("lead_present", False))
    return {
        "seq": seq,
        "t": float(frame.get("t", 0.0)),
        "ego": {
            "x": float(frame.get("ego_x", 0.0)),
            "y": float(frame.get("ego_y", 0.0)),
            "yaw": float(frame.get("ego_yaw", 0.0)),
            "v": float(frame.get("ego_v", 0.0)),
        },
        "road_psi": float(frame.get("road_psi", 0.0)),
        "lane_offset": float(frame.get("lane_offset", 0.0)),
        "lead": {
            "present": present,
            "x": float(frame.get("lead_x", 9999.0)),
            "y": float(frame.get("lead_y", 9999.0)),
            "yaw": float(frame.get("lead_yaw", 0.0)),
            "v": float(frame.get("lead_v", 0.0)),
            "cls": int(frame.get("lead_cls", 1 if present else 0)),
        },
    }
