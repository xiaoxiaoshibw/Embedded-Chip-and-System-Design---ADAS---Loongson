# -*- coding: utf-8 -*-
"""HIL mock 结构化安全诊断回归：诊断随 mock 动力学真实联动，并进入 /ws/live 帧。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.hil_bridge import MockHilBridge


class _NoFault:
    def apply(self, frame):
        pass


def _run(scenario, params, n=320, dt=0.05):
    b = MockHilBridge()
    b.load(scenario, params)
    nf = _NoFault()
    frames = []
    t = 0.0
    for _ in range(n):
        f, _ev = b.step(dt, t, nf)
        frames.append(f)
        t += dt
    return frames


def test_ws_dict_carries_diagnostics():
    f = _run("acc_follow", {"ego_speed": 50, "front_speed": 40, "front_distance": 40}, n=20)[-1]
    ws = f.to_ws_dict("r", "acc_follow", "RUNNING")
    assert "diagnostics" in ws
    for k in ("gate_reason", "gate_severity", "aeb_level", "aeb_drac"):
        assert k in ws["diagnostics"]


def test_aeb_level_escalates_on_hard_brake():
    frames = _run("aeb_brake",
                  {"ego_speed": 60, "front_speed": 30, "front_distance": 22, "fault_trigger_time": 4.0})
    levels = {f.diagnostics.aeb_level for f in frames}
    assert "warning" in levels or "emergency" in levels


def test_drac_nonneg_and_capped():
    frames = _run("aeb_brake",
                  {"ego_speed": 70, "front_speed": 20, "front_distance": 18, "fault_trigger_time": 3.0})
    dracs = [f.diagnostics.aeb_drac for f in frames]
    assert all(0.0 <= d <= 20.0 for d in dracs)


def test_no_front_is_safe_and_normal():
    frames = _run("lka_curve", {"ego_speed": 50}, n=40)
    assert all(f.diagnostics.aeb_level == "safe" for f in frames)
    assert all(f.diagnostics.gate_reason == "normal" for f in frames)
