#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""virtual_esp32_watchdog 移植正确性回归：对照 main.c arbitrate() /
comm_watchdog_task() 的逐条判据。

正确性是后续 experiment_watchdog_limit.py 扫描结果可信的前提——这里先把
移植体在已知场景下的行为钉死。
"""

import random

from control.virtual_esp32_watchdog import VirtualEsp32Watchdog
from experiment_watchdog_limit import (
    run_single_kill_trial,
    run_worst_jitter_coincidence,
)


def _wd(jetson_timeout=58, watchdog_timeout=200, watchdog_check=50):
    return VirtualEsp32Watchdog(jetson_timeout, watchdog_timeout, watchdog_check)


def test_initial_state_uses_primary():
    """未收到任何帧时 arbitrate() 保持 use_secondary=False → 'pri'（对应
    main.c 初始 g_use_secondary=false，两路都不新鲜时保持上次来源）。"""
    wd = _wd()
    assert wd.arbitrate(0.0) == 'pri'
    assert wd.events == []


def test_primary_fresh_no_switch():
    """主控持续新鲜 → 一直用主控，不产生事件。"""
    wd = _wd()
    for t in range(0, 500, 10):
        wd.on_frame('pri', float(t))
        assert wd.arbitrate(float(t)) == 'pri'
    assert wd.events == []


def test_primary_timeout_switches_to_secondary():
    """主控停发、备控持续新鲜 → 超过 jetson_timeout_ms 后切到备控，且只记一次
    switch_timeout（对应 main.c SWITCH:pri_timeout_*，仅在 use_secondary 从
    False 变 True 的边沿打印一次）。"""
    wd = _wd(jetson_timeout=58)
    wd.on_frame('pri', 0.0)
    for t in range(0, 400, 10):
        wd.on_frame('sec', float(t))
    # 主控最后一帧在 t=0，jetson_timeout=58 → t=58 之前仍新鲜
    assert wd.arbitrate(50.0) == 'pri'
    assert wd.arbitrate(59.0) == 'sec'
    assert wd.arbitrate(70.0) == 'sec'      # 持续用备控，不重复记事件
    switch_events = [e for e in wd.events if e.kind == 'switch_timeout']
    assert len(switch_events) == 1
    assert switch_events[0].t_ms == 59.0


def test_primary_recovery_switches_back():
    """备控接管期间主控恢复新鲜 → 立即切回主控，记一次 switch_recovered。"""
    wd = _wd(jetson_timeout=58)
    wd.on_frame('pri', 0.0)
    wd.on_frame('sec', 0.0)
    wd.on_frame('sec', 100.0)
    assert wd.arbitrate(70.0) == 'sec'       # 主控已过期，切备控
    wd.on_frame('pri', 120.0)                # 主控恢复发帧
    assert wd.arbitrate(121.0) == 'pri'      # 立即切回
    recovered = [e for e in wd.events if e.kind == 'switch_recovered']
    assert len(recovered) == 1


def test_watchdog_no_trigger_when_either_alive():
    """任一路在 watchdog_timeout_ms 内新鲜 → 不进入紧急态（对应
    main.c pri_alive || sec_alive）。"""
    wd = _wd(watchdog_timeout=200)
    wd.on_frame('pri', 0.0)
    wd.on_frame('sec', 0.0)
    # 备控早早不发了，但主控仍新鲜
    assert wd.watchdog_check(150.0) is False
    assert wd.watchdog_check(190.0) is False
    assert not any(e.kind == 'wdt_timeout' for e in wd.events)


def test_watchdog_triggers_on_dual_silence():
    """两路都超过 watchdog_timeout_ms 无新帧 → 触发一次 wdt_timeout，此后
    持续保持 estop_active（对应固件持续发送 watchdog_emergency_brake()，
    这里用 estop_active=True 表示"持续处于该态"）。"""
    wd = _wd(watchdog_timeout=200)
    wd.on_frame('pri', 0.0)
    wd.on_frame('sec', 0.0)
    assert wd.watchdog_check(199.0) is False
    assert wd.watchdog_check(201.0) is True
    assert wd.watchdog_check(250.0) is True   # 持续紧急态
    timeouts = [e for e in wd.events if e.kind == 'wdt_timeout']
    assert len(timeouts) == 1
    assert timeouts[0].t_ms == 201.0


def test_watchdog_recovers_when_frame_resumes():
    """紧急态期间任一路恢复新鲜 → 退出紧急态，记一次 wdt_recovered。"""
    wd = _wd(watchdog_timeout=200)
    wd.on_frame('pri', 0.0)
    wd.on_frame('sec', 0.0)
    wd.watchdog_check(201.0)
    assert wd.estop_active is True
    wd.on_frame('pri', 210.0)
    assert wd.watchdog_check(215.0) is False
    recovered = [e for e in wd.events if e.kind == 'wdt_recovered']
    assert len(recovered) == 1


def test_worst_jitter_coincidence_boundary():
    """experiment_watchdog_limit.run_worst_jitter_coincidence 的物理边界：
    真实最坏单拍抖动固定为 49.44ms（2026-07 实机 LOOP TIMING 实测）。
    阈值大于抖动时长 → 不可能误触发；阈值小于抖动时长 → 必然存在触发相位。
    这条测试钉死"最后一帧补在 base 而非 base-period"这个修正（原实现有
    off-by-one，会把 WD=50 误判成会触发）。"""
    assert run_worst_jitter_coincidence(50, 12, 58, spike_ms=49.44) is False
    assert run_worst_jitter_coincidence(40, 10, 58, spike_ms=49.44) is True


def test_single_kill_cold_start_couples_with_watchdog_timeout():
    """experiment_watchdog_limit.run_single_kill_trial 的因果顺序修正：
    冷启动场景（备控延迟 58ms 才恢复发帧）下，若 WATCHDOG_TIMEOUT_MS(30) <
    冷启动延迟(58)，两路会在备控恢复前的窗口内同时被判"陈旧"，必须触发
    误报——这正是"WATCHDOG_TIMEOUT_MS 须 > JETSON_TIMEOUT_MS"这条耦合约束
    要防的场景。修复前（帧事件未按时间顺序应用）这里恒为 False，是假阴性。"""
    rng = random.Random(7)
    for _ in range(5):
        _, false_trig = run_single_kill_trial(
            rng, watchdog_timeout_ms=30, watchdog_check_ms=10,
            jetson_timeout_ms=58, backup_cold_start_delay_ms=58.0)
        assert false_trig is True


def test_single_kill_hot_standby_never_false_triggers():
    """热待机场景（当前实际部署，delay=0）：备控从死亡瞬间就已连续新鲜，
    watchdog 全程不应误报，与 WATCHDOG_TIMEOUT_MS 取值无关。"""
    rng = random.Random(8)
    for _ in range(5):
        _, false_trig = run_single_kill_trial(
            rng, watchdog_timeout_ms=30, watchdog_check_ms=10,
            jetson_timeout_ms=58, backup_cold_start_delay_ms=0.0)
        assert false_trig is False


def test_never_received_channel_is_never_fresh():
    """从未收到过帧的通道（valid=False）恒不新鲜，不会被 age 计算误判为新鲜
    （对应 main.c state_is_fresh 里 !st->valid 直接返回 false）。"""
    wd = _wd()
    # 只有主控发过帧，备控从未连接过
    wd.on_frame('pri', 0.0)
    assert wd.arbitrate(0.0) == 'pri'
    # 主控超时后，从未连接的备控不应被误判为可用来源
    assert wd.watchdog_check(300.0) is True
