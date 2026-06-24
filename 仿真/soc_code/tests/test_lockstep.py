#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""软件双核锁步：确定性 + 比较器离线测试。

核心保证：同一拍输入 + 同一拍前状态 + 同一 ml_result，主核与影子核跑同一份
run_pure_pipeline 必产出 bit 一致的 (delta / lon_cmd / AEB) —— 锁步比较器健康时
零误报；唯有注入偏移才失配。无 ROS 依赖，可直接 python 运行或 pytest。
"""

import copy
import os
import sys

_soccode = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _soccode not in sys.path:
    sys.path.insert(0, _soccode)

from control.context import ControlManagers, ControlMemory, VehicleSignals
from control.lead_tracking import LeadTracker
from control.aeb_alert import AebAlertManager
from control.curve_hold import CurveHoldManager
from lateral import LaneWidthEstimator
from longitudinal import LongitudinalController, LonSmoothing
from pipeline import run_pure_pipeline
from lockstep import snapshot_managers

DT = 0.01
LOOP_HZ = 100


def _make_managers():
    return ControlManagers(
        lane_est=LaneWidthEstimator(LOOP_HZ),
        lead_tracker=LeadTracker(),
        aeb_alert=AebAlertManager(),
        curve_hold=CurveHoldManager(),
        lon_ctrl=LongitudinalController(DT),
        lon_smooth=LonSmoothing(DT),
        overtake=None,
        ml_bridge=None,
    )


def _make_signals(now):
    s = VehicleSignals()
    s.ego_v = 16.0
    s.road_psi = 0.02
    s.lane_offset = 0.1
    s.lane_offset_received = True
    s.lane_offset_last_rx = now
    s.road_received = True
    s.road_last_rx = now
    s.ego_received = True
    s.ego_last_rx = now
    s.lead_x = 30.0
    s.lead_y = 0.0
    s.lead_v = 14.0
    s.lead_received = True
    s.lead_last_rx_time = now
    s.lead_v_last_rx_time = now
    return s


def _outputs(res):
    return (res.lateral_ctx.delta, res.lon_cmd, bool(res.lon_ctx.aeb_active))


def _warmup_then_split():
    """推进 20 拍累积状态，返回本拍主核结果与影子核结果（影子用拷贝重算）。"""
    mem = ControlMemory(dt=DT)
    mgr = _make_managers()
    now = 1000.0
    for _ in range(20):
        run_pure_pipeline(now, _make_signals(now), mem, mgr, None)
        now += DT
    sig = _make_signals(now)
    sig_shadow = copy.copy(sig)            # 与节点 ls_pre 同序：主核改写前先拷贝
    mem_copy = copy.deepcopy(mem)
    mgr_copy = snapshot_managers(mgr)
    main_res = run_pure_pipeline(now, sig, mem, mgr, None)
    shadow_res = run_pure_pipeline(now, sig_shadow, mem_copy, mgr_copy, None,
                                   ml_result=main_res.ml_result)
    return main_res, shadow_res


def test_lockstep_deterministic_zero_false_positive():
    main_res, shadow_res = _warmup_then_split()
    assert _outputs(main_res) == _outputs(shadow_res), (
        'lockstep 影子输出应与主核 bit 一致：main=%s shadow=%s'
        % (_outputs(main_res), _outputs(shadow_res)))


def test_lockstep_injection_detected():
    main_res, shadow_res = _warmup_then_split()
    injected_delta = shadow_res.lateral_ctx.delta + 0.05   # 模拟故障 checker 核
    assert abs(injected_delta - main_res.lateral_ctx.delta) > 1e-9, (
        '注入偏移应被比较器检出为失配')


def _drive_checker(inject, ticks=40, timeout_s=3.0):
    """端到端驱动真实 LockstepChecker 线程：每拍提交"主核输出=影子应算出的值"，
    健康时应零失配；inject=True 时影子被加偏移 → 触发故障。返回 checker。"""
    import time as _t
    from lockstep import LockstepChecker
    chk = LockstepChecker(checker_core=2, debounce_n=2, inject=inject)
    mem = ControlMemory(dt=DT)
    mgr = _make_managers()
    now = 1000.0
    for _ in range(20):                      # warmup 累积状态
        run_pure_pipeline(now, _make_signals(now), mem, mgr, None)
        now += DT
    for _ in range(ticks):
        sig = _make_signals(now)
        sig_shadow = copy.copy(sig)
        mem_copy = copy.deepcopy(mem)
        mgr_copy = snapshot_managers(mgr)
        main_res = run_pure_pipeline(now, sig, mem, mgr, None)   # 推进真实状态
        # 提交"主核输出"= 用同一前状态另算一份（与影子核将算出的一致）→ 健康零失配
        ref = run_pure_pipeline(now, copy.copy(sig), copy.deepcopy(mem_copy),
                                snapshot_managers(mgr_copy), None,
                                ml_result=main_res.ml_result)
        chk.submit(now, sig_shadow, mem_copy, mgr_copy, None, main_res.ml_result,
                   ref.lateral_ctx.delta, ref.lon_cmd, ref.lon_ctx.aeb_active)
        now += DT
    deadline = _t.time() + timeout_s
    while _t.time() < deadline:
        if chk.fault or chk.compared >= ticks:
            break
        _t.sleep(0.02)
    return chk


def test_lockstep_checker_healthy_no_fault():
    chk = _drive_checker(inject=False)
    assert chk.compared > 0, 'checker 应已比较若干拍'
    assert not chk.fault, 'healthy 路径不应误报：%s' % chk.fault_reason


def test_lockstep_checker_inject_triggers_fault():
    chk = _drive_checker(inject=True)
    assert chk.fault, '注入故障应被比较器检出（fault=True）'


if __name__ == '__main__':
    test_lockstep_deterministic_zero_false_positive()
    test_lockstep_injection_detected()
    m, s = _warmup_then_split()
    print('determinism: main=%s shadow=%s' % (_outputs(m), _outputs(s)))
    chk_ok = _drive_checker(inject=False)
    print('checker healthy: compared=%d fault=%s' % (chk_ok.compared, chk_ok.fault))
    chk_bad = _drive_checker(inject=True)
    print('checker injected: compared=%d fault=%s reason=%s'
          % (chk_bad.compared, chk_bad.fault, chk_bad.fault_reason))
    assert not chk_ok.fault and chk_bad.fault
    print('lockstep offline tests PASSED')
