# -*- coding: utf-8 -*-
"""AEB 结构化诊断 overlay 回归测试（只读投影，不改控制）。"""

from config import TTC_BRAKE_FULL, TTC_BRAKE_START
from control.aeb_overlay import (
    summarize_aeb,
    AEB_LEVEL_SAFE,
    AEB_LEVEL_WARNING,
    AEB_LEVEL_EMERGENCY,
)


def test_level_safe():
    d = summarize_aeb(ttc=20.0, dist=50.0, ego_v=10.0, lead_v_proj=10.0, aeb_active=False)
    assert d.level == AEB_LEVEL_SAFE and not d.engaged


def test_level_warning():
    d = summarize_aeb(ttc=10.0, dist=30.0, ego_v=15.0, lead_v_proj=5.0, aeb_active=False)
    assert d.level == AEB_LEVEL_WARNING


def test_level_emergency_by_ttc():
    d = summarize_aeb(ttc=3.0, dist=10.0, ego_v=15.0, lead_v_proj=0.0, aeb_active=False)
    assert d.level == AEB_LEVEL_EMERGENCY


def test_engaged_forces_emergency():
    d = summarize_aeb(ttc=99.0, dist=50.0, ego_v=10.0, lead_v_proj=10.0, aeb_active=True)
    assert d.engaged and d.level == AEB_LEVEL_EMERGENCY and d.reason == "aeb_engaged"


def test_drac_formula_and_cap():
    # closing = 20-0 = 20, dist=10 → 400/(2*10)=20，恰好等于上限 20
    d = summarize_aeb(ttc=1.0, dist=10.0, ego_v=20.0, lead_v_proj=0.0, aeb_active=False)
    assert abs(d.drac - 20.0) < 1e-6 and abs(d.closing_speed - 20.0) < 1e-6


def test_drac_zero_when_not_closing():
    d = summarize_aeb(ttc=99.0, dist=50.0, ego_v=10.0, lead_v_proj=15.0, aeb_active=False)
    assert d.drac == 0.0 and d.closing_speed == 0.0


def test_threshold_boundaries():
    assert TTC_BRAKE_FULL == 5.0 and TTC_BRAKE_START == 15.0
    # 恰好 = FULL → emergency（<=）
    assert summarize_aeb(5.0, 20.0, 10.0, 5.0, False).level == AEB_LEVEL_EMERGENCY
    # 恰好 = START → warning（<=）
    assert summarize_aeb(15.0, 30.0, 10.0, 5.0, False).level == AEB_LEVEL_WARNING
