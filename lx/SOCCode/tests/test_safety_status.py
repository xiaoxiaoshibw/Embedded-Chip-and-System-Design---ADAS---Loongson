# -*- coding: utf-8 -*-
"""统一安全状态聚合器回归测试。"""

from control.safety_status import (
    aggregate_safety_status,
    SAFETY_NOMINAL,
    SAFETY_DEGRADED,
    SAFETY_EMERGENCY,
)


def test_all_nominal():
    s = aggregate_safety_status()
    assert s.level == 0 and s.state == SAFETY_NOMINAL and s.reasons == ()


def test_aeb_warning_degraded():
    s = aggregate_safety_status(aeb_level="warning")
    assert s.level == 1 and s.state == SAFETY_DEGRADED and "aeb_warning" in s.reasons


def test_aeb_emergency():
    s = aggregate_safety_status(aeb_level="emergency")
    assert s.level == 2 and s.state == SAFETY_EMERGENCY


def test_gate_severity_drives_level():
    s = aggregate_safety_status(gate_reason="nan_block", gate_severity=2)
    assert s.level == 2 and s.state == SAFETY_EMERGENCY and "gate_nan_block" in s.reasons


def test_no_failover_degraded():
    s = aggregate_safety_status(failover_available=False)
    assert s.level == 1 and "no_failover" in s.reasons


def test_esp32_stale_degraded():
    s = aggregate_safety_status(esp32_stale=True)
    assert s.level == 1 and "esp32_link_stale" in s.reasons


def test_lockstep_fault_emergency():
    s = aggregate_safety_status(lockstep_fault=True)
    assert s.level == 2 and s.state == SAFETY_EMERGENCY and "lockstep_fault" in s.reasons


def test_combination_takes_max_and_all_reasons():
    s = aggregate_safety_status(
        gate_reason="backup_takeover", gate_severity=1, aeb_level="warning",
        failover_available=False, esp32_stale=True, lockstep_fault=False)
    assert s.level == 1 and s.state == SAFETY_DEGRADED
    for r in ("gate_backup_takeover", "aeb_warning", "no_failover", "esp32_link_stale"):
        assert r in s.reasons
    # 排序去重
    assert list(s.reasons) == sorted(set(s.reasons))


def test_emergency_wins_over_degraded():
    s = aggregate_safety_status(aeb_level="warning", lockstep_fault=True)
    assert s.level == 2 and s.state == SAFETY_EMERGENCY
