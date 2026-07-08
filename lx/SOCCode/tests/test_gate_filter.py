# -*- coding: utf-8 -*-
"""★2 CommandGate.filter_lateral 回归测试。

锁定契约：
  1. 默认（GATE_FILTER_EXT_ENABLED=False）下，gate.filter_lateral 与原 ADAS.py 内联
     `lat_smooth.update(delta, max_rate_override=...)` **逐序列字节级一致**（常规 + 接管窗）。
  2. 扩展限幅开启时，steer_cmd_diff_from_current 正确生效。
"""

from lateral import LateralSmoothing
from config import TAKEOVER_DELTA_RATE
import control.command_gate as cg
from control.command_gate import CommandGate

DT = 0.01


def _seq():
    # 一串含正负大阶跃的目标转角，覆盖坡度限速与低通
    return [0.0, 0.3, -0.4, 0.5, 0.5, -0.2, 0.1, 0.0, 0.45, -0.45, 0.2, 0.2]


def test_filter_lateral_byte_identical_normal():
    gate = CommandGate()
    ls_ref = LateralSmoothing(DT)
    ls_gate = LateralSmoothing(DT)
    for d in _seq():
        ref = ls_ref.update(d, max_rate_override=None)
        got = gate.filter_lateral(d, ls_gate, takeover_active=False)
        assert got == ref


def test_filter_lateral_byte_identical_takeover():
    gate = CommandGate()
    ls_ref = LateralSmoothing(DT)
    ls_gate = LateralSmoothing(DT)
    for d in _seq():
        ref = ls_ref.update(d, max_rate_override=TAKEOVER_DELTA_RATE)
        got = gate.filter_lateral(d, ls_gate, takeover_active=True)
        assert got == ref


def test_ext_off_ignores_esp_delta():
    # 默认 GATE_FILTER_EXT_ENABLED=False → esp_delta 不影响输出
    assert cg.GATE_FILTER_EXT_ENABLED is False
    gate = CommandGate()
    ls1 = LateralSmoothing(DT)
    ls2 = LateralSmoothing(DT)
    for d in _seq():
        a = gate.filter_lateral(d, ls1, False, esp_delta=-1.0)
        b = gate.filter_lateral(d, ls2, False, esp_delta=None)
        assert a == b


def test_ext_on_applies_steer_diff(monkeypatch):
    monkeypatch.setattr(cg, "GATE_FILTER_EXT_ENABLED", True)
    monkeypatch.setattr(cg, "GATE_STEER_DIFF_MAX", 0.05)
    gate = CommandGate()
    ls = LateralSmoothing(DT)
    # 把 lat_smooth 拉到正值；esp_delta 远在负侧 → 输出被钳到 esp_delta + 0.05
    for _ in range(50):
        ls.update(0.5, max_rate_override=None)
    out = gate.filter_lateral(0.5, ls, False, esp_delta=-1.0)
    assert abs(out - (-1.0 + 0.05)) < 1e-6


def test_ext_on_transition_profile_tighter(monkeypatch):
    # 接管/过渡期用更严的 transition 差限幅（对标 Autoware transition_filter）
    monkeypatch.setattr(cg, "GATE_FILTER_EXT_ENABLED", True)
    monkeypatch.setattr(cg, "GATE_STEER_DIFF_MAX", 0.05)
    monkeypatch.setattr(cg, "GATE_STEER_DIFF_MAX_TRANSITION", 0.02)
    gate = CommandGate()
    ls = LateralSmoothing(DT)
    for _ in range(50):
        ls.update(0.5, max_rate_override=None)
    out = gate.filter_lateral(0.5, ls, True, esp_delta=-1.0)
    assert abs(out - (-1.0 + 0.02)) < 1e-6  # 用 transition(0.02) 而非 nominal(0.05)


def test_ext_on_no_clamp_when_within_band(monkeypatch):
    monkeypatch.setattr(cg, "GATE_FILTER_EXT_ENABLED", True)
    monkeypatch.setattr(cg, "GATE_STEER_DIFF_MAX", 0.5)
    gate = CommandGate()
    ls_ref = LateralSmoothing(DT)
    ls_gate = LateralSmoothing(DT)
    # 命令幅度小、esp_delta≈输出 → 差在带内，扩展限幅不改变结果
    for d in [0.0, 0.02, 0.01, 0.015]:
        ref = ls_ref.update(d, max_rate_override=None)
        got = gate.filter_lateral(d, ls_gate, False, esp_delta=ref)
        assert abs(got - ref) < 1e-9
