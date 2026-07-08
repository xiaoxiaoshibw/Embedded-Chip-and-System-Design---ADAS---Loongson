# -*- coding: utf-8 -*-
"""VehicleAdapter 回归测试。

锁定契约：
  1. Esp32VehicleAdapter.send_control / send_safe_stop 产出的字节与原
     ADAS.py 直发（build_esp32_payload / CommandGate.build_safe_fallback_payload）
     **字节级一致**；
  2. read_feedback / health 正确映射底盘回传与链路健康；
  3. SimVehicleAdapter 回显下发值（离线/HIL 可用）。
"""

from control.serial_protocol import Esp32ControlFrame, build_esp32_payload
from control.command_gate import CommandGate
from control.vehicle_adapter import (
    Esp32VehicleAdapter,
    SimVehicleAdapter,
    make_vehicle_adapter,
)


class FakeSerial(object):
    """伪 Esp32Serial：捕获下发字节，暴露回读属性。"""

    def __init__(self):
        self.sent = []
        self.esp_psi = 0.1
        self.esp_delta = 0.02
        self.esp_brake = 1.5
        self.readback_stale = False
        self.tx_dropped = 3
        self.drained = 0
        self.closed = False

    def send(self, payload):
        self.sent.append(payload)

    def drain_rx(self):
        self.drained += 1

    def close(self):
        self.closed = True


def _frame():
    return Esp32ControlFrame(
        ttc=8.0, dist=20.0, psi=0.3, delta=0.05, speed=12.0, lon=-0.5,
        offset=0.1, lead_v_proj=10.0, min_safe_dist=9.0,
        lane_warn_margin=1.0, lane_hard_margin=0.7, filtered_curv=0.01)


def test_send_control_byte_identical():
    fk = FakeSerial()
    a = Esp32VehicleAdapter(serial_link=fk)
    f = _frame()
    a.send_control(f)
    assert fk.sent[-1] == build_esp32_payload(f)


def test_send_safe_stop_byte_identical():
    fk = FakeSerial()
    a = Esp32VehicleAdapter(serial_link=fk)
    for brake in (6.0, 0.0, 1.5, 2.5):
        a.send_safe_stop(brake)
        assert fk.sent[-1] == CommandGate.build_safe_fallback_payload(brake)


def test_read_feedback_maps_serial():
    fk = FakeSerial()
    a = Esp32VehicleAdapter(serial_link=fk)
    fb = a.read_feedback()
    assert (fb.psi, fb.delta, fb.brake, fb.stale) == (0.1, 0.02, 1.5, False)


def test_health_drain_close():
    fk = FakeSerial()
    a = Esp32VehicleAdapter(serial_link=fk)
    h = a.health()
    assert h['tx_dropped'] == 3 and h['readback_stale'] is False
    a.drain()
    assert fk.drained == 1
    a.close()
    assert fk.closed is True


def test_sim_adapter_echoes_control():
    a = SimVehicleAdapter()
    f = _frame()
    a.send_control(f)
    fb = a.read_feedback()
    assert fb.psi == f.psi and fb.delta == f.delta and fb.brake == f.lon
    assert fb.stale is False


def test_sim_adapter_safe_stop():
    a = SimVehicleAdapter()
    a.send_safe_stop(6.0)
    fb = a.read_feedback()
    assert fb.brake == 6.0 and fb.delta == 0.0 and fb.psi == 0.0
    assert a.health()['tx_dropped'] == 0


def test_factory_sim():
    assert isinstance(make_vehicle_adapter('sim'), SimVehicleAdapter)
