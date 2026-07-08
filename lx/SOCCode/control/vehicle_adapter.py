#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""车辆/底盘接口抽象（VehicleAdapter）—— 对标 Apollo VehicleController / openpilot CarInterface。

把 ESP32 串口细节从控制层剥离：控制节点只产出 `Esp32ControlFrame`（协议中立的控制意图）
与安全制动量，由 VehicleAdapter 负责「组帧 / 下发 / 读底盘回传 / 链路健康」。换执行器
（ESP32 → CAN → 仿真）只需换实现，控制层零改。

- `Esp32VehicleAdapter`：真实 ESP32 串口（持有 `Esp32Serial` + `serial_protocol` 组帧），
  与原 ADAS.py 直发 `esp32.send(build_esp32_payload(...))` **字节级一致**。
- `SimVehicleAdapter`：离线 / HIL 用，不碰真串口，回显下发值作理想回传，便于无硬件回归。

对标要点：
- `send_control` ≈ Apollo `Update(ControlCommand)`（分发到 Throttle/Brake/Steer 后下发底盘）；
- `read_feedback` ≈ Apollo `chassis()`（读底盘回传）；
- `health` ≈ Apollo `CheckChassisCommunicationError()`（链路健康，用现成 readback_stale + tx_dropped）。

Python 3.6 兼容（JetPack 4.x）。中文注释，遵循 SOCCode 约定。
"""

from dataclasses import dataclass

from control.serial_protocol import Esp32ControlFrame, build_esp32_payload
from control.command_gate import CommandGate


@dataclass(frozen=True)
class VehicleFeedback:
    """底盘回传（对标 Apollo chassis()）。psi/delta/brake 为 ESP32 回读，stale=链路陈旧。"""
    psi: float = 0.0
    delta: float = 0.0
    brake: float = 0.0
    stale: bool = True


class VehicleAdapter(object):
    """车辆接口抽象。所有发往底盘的控制都经此层（对标 Apollo VehicleController）。

    子类实现：send_control / send_safe_stop / read_feedback / health / close。
    """

    def send_control(self, frame):
        """下发一帧常规控制（frame: Esp32ControlFrame）。"""
        raise NotImplementedError

    def send_safe_stop(self, brake_decel):
        """下发安全回退帧（转角 0 + 指定制动量），NaN/急停/感知超时/锁步失配共用。"""
        raise NotImplementedError

    def read_feedback(self):
        """读底盘回传，返回 VehicleFeedback。"""
        raise NotImplementedError

    def health(self):
        """链路健康字典（如 tx_dropped / readback_stale）。"""
        raise NotImplementedError

    def drain(self):
        """兼容旧 drain_rx 钩子；多数实现为空操作。"""
        pass

    def close(self):
        pass


class Esp32VehicleAdapter(VehicleAdapter):
    """真实 ESP32 串口适配。组帧/下发与原 ADAS.py 直发字节级一致。"""

    def __init__(self, serial_link=None):
        # 延迟导入 Esp32Serial，避免 Sim 路径依赖 pyserial。
        if serial_link is None:
            from serial_link import Esp32Serial
            serial_link = Esp32Serial()
        self._serial = serial_link

    def send_control(self, frame):
        self._serial.send(build_esp32_payload(frame))

    def send_safe_stop(self, brake_decel):
        self._serial.send(CommandGate.build_safe_fallback_payload(brake_decel))

    def read_feedback(self):
        s = self._serial
        return VehicleFeedback(
            psi=s.esp_psi, delta=s.esp_delta, brake=s.esp_brake,
            stale=s.readback_stale)

    def health(self):
        s = self._serial
        return {"tx_dropped": s.tx_dropped, "readback_stale": s.readback_stale}

    def drain(self):
        self._serial.drain_rx()

    def close(self):
        self._serial.close()


class SimVehicleAdapter(VehicleAdapter):
    """离线 / HIL 适配：不碰真串口，回显下发值作理想执行器回传。"""

    def __init__(self):
        self.last_frame = None
        self.last_brake = None
        self.tx_count = 0

    def send_control(self, frame):
        self.last_frame = frame
        self.last_brake = None
        self.tx_count += 1

    def send_safe_stop(self, brake_decel):
        self.last_brake = float(brake_decel)
        self.tx_count += 1

    def read_feedback(self):
        if self.last_brake is not None:
            return VehicleFeedback(psi=0.0, delta=0.0, brake=self.last_brake, stale=False)
        f = self.last_frame
        if f is None:
            return VehicleFeedback()
        return VehicleFeedback(psi=f.psi, delta=f.delta, brake=f.lon, stale=False)

    def health(self):
        idle = self.last_frame is None and self.last_brake is None
        return {"tx_dropped": 0, "readback_stale": idle}

    def close(self):
        pass


def make_vehicle_adapter(kind=None):
    """工厂：按 config.VEHICLE_ADAPTER（或显式 kind）选实现。默认 'esp32'（与现网一致）。"""
    if kind is None:
        try:
            from config import VEHICLE_ADAPTER as cfg_kind
            kind = cfg_kind
        except Exception:
            kind = "esp32"
    kind = (kind or "esp32").lower()
    if kind == "sim":
        return SimVehicleAdapter()
    return Esp32VehicleAdapter()
