#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AEB 预测路径碰撞检查（control/aeb_path_check.py）单元回归。

障碍元组约定：(tid, cls, x_rel, y_rel, v_proj, lat_rate)
  v_proj = 相对接近速度（静止目标 = ego_v；同速前车 = 0）
"""

import math

from config import (
    ACC_NORMAL_BRAKE_MAX,
    ACTOR_CLASS_PEDESTRIAN,
    ACTOR_CLASS_VEHICLE,
    AEB_PATH_CONFIRM_CYCLES,
    AEB_PATH_MIN_EGO_V,
    LON_CMD_MAX_BRAKE_DECEL,
)
from control.aeb_path_check import (
    AebPathChecker,
    AebPathDecision,
    fuse_path_decision,
    obstacles_from_lead_ctx,
)
from control.state import LeadContext, LongitudinalContext


def _run_until_confirm(checker, ego_v, delta, obstacles):
    """连续投喂同一输入直到消抖确认，返回最终决策。"""
    d = None
    for _ in range(AEB_PATH_CONFIRM_CYCLES + 1):
        d = checker.evaluate(ego_v, delta, obstacles)
    return d


def test_stationary_obstacle_in_path_engages():
    """正前方静止障碍：路径相交 → 消抖后触发，t_col ≈ 距离/车速。"""
    checker = AebPathChecker()
    obs = [(2, ACTOR_CLASS_VEHICLE, 15.0, 0.0, 10.0, 0.0)]
    d = _run_until_confirm(checker, 10.0, 0.0, obs)
    assert d.risk and d.engaged
    assert d.obstacle_tid == 2
    assert 1.0 < d.t_col < 1.8          # 15m/10mps −碰撞盒半长 ≈ 1.25~1.3s
    assert d.brake_cmd > 0.0


def test_close_stationary_obstacle_full_brake():
    """近距静止障碍（t_col ≤ FULL 阈值）→ 全力制动。"""
    checker = AebPathChecker()
    obs = [(2, ACTOR_CLASS_VEHICLE, 8.0, 0.0, 10.0, 0.0)]
    d = _run_until_confirm(checker, 10.0, 0.0, obs)
    assert d.engaged
    assert d.brake_cmd == LON_CMD_MAX_BRAKE_DECEL


def test_crossing_pedestrian_engages():
    """横穿行人：初始在走廊外（|y|=6m）但横向逼近 → 路径相交预测触发。

    这是现有 TTC AEB 的盲区（未被选举为主前车的目标），本层的核心价值。
    """
    checker = AebPathChecker()
    ped = [(5, ACTOR_CLASS_PEDESTRIAN, 20.0, 6.0, 10.0, 2.0)]
    d = _run_until_confirm(checker, 10.0, 0.0, ped)
    assert d.risk and d.engaged
    assert d.obstacle_tid == 5
    assert d.t_col < 2.6


def test_adjacent_lane_vehicle_no_false_positive():
    """相邻车道同向车（横向 3m、无逼近趋势）→ 不触发。"""
    checker = AebPathChecker()
    obs = [(3, ACTOR_CLASS_VEHICLE, 20.0, 3.0, 0.0, 0.0)]
    d = _run_until_confirm(checker, 10.0, 0.0, obs)
    assert not d.risk and not d.engaged
    assert d.brake_cmd == 0.0


def test_same_speed_lead_no_false_positive():
    """同速前车（v_proj=0，间距稳定）→ 纵向差恒定，不触发。"""
    checker = AebPathChecker()
    obs = [(2, ACTOR_CLASS_VEHICLE, 25.0, 0.0, 0.0, 0.0)]
    d = _run_until_confirm(checker, 15.0, 0.0, obs)
    assert not d.engaged
    # 25m @ v_proj=0：RSS = 15·0.5+225/12−225/16 ≈ 12.2m < 25m，不违反
    assert not d.rss_violated


def test_rss_violation_close_slow_lead():
    """高速逼近慢车、间距小于 RSS 最小安全距离 → rss_violated。"""
    checker = AebPathChecker()
    # ego 20，前车 15（v_proj=5），间距 10m：d_rss ≈ 29.3m ≫ 10m
    obs = [(2, ACTOR_CLASS_VEHICLE, 10.0, 0.0, 5.0, 0.0)]
    d = _run_until_confirm(checker, 20.0, 0.0, obs)
    assert d.rss_violated
    assert d.engaged
    assert d.brake_cmd >= ACC_NORMAL_BRAKE_MAX * 0.5


def test_debounce_single_tick_no_engage():
    """单拍命中不触发（消抖），且退出非对称衰减。"""
    checker = AebPathChecker()
    obs = [(2, ACTOR_CLASS_VEHICLE, 8.0, 0.0, 10.0, 0.0)]
    d1 = checker.evaluate(10.0, 0.0, obs)
    assert d1.risk and not d1.engaged
    # 命中消失 → 计数衰减回 0
    d2 = checker.evaluate(10.0, 0.0, [])
    assert not d2.engaged
    assert checker.confirm_count == 0


def test_below_min_speed_inactive():
    """低于最低车速门（standstill）→ 恒不激活。"""
    checker = AebPathChecker()
    obs = [(2, ACTOR_CLASS_VEHICLE, 3.0, 0.0, 0.5, 0.0)]
    for _ in range(AEB_PATH_CONFIRM_CYCLES + 2):
        d = checker.evaluate(AEB_PATH_MIN_EGO_V * 0.5, 0.0, obs)
    assert not d.risk and not d.engaged


def test_behind_obstacle_ignored():
    """后方目标（x_rel < −1m）→ 忽略。"""
    checker = AebPathChecker()
    obs = [(4, ACTOR_CLASS_VEHICLE, -5.0, 0.0, -10.0, 0.0)]
    d = _run_until_confirm(checker, 10.0, 0.0, obs)
    assert not d.risk


def test_curved_path_does_not_crash_and_stays_bounded():
    """带转角的路径积分：不异常、直行盲区目标在弯道路径下仍有限判定。"""
    checker = AebPathChecker()
    obs = [(2, ACTOR_CLASS_VEHICLE, 15.0, 0.0, 10.0, 0.0)]
    d = _run_until_confirm(checker, 10.0, 0.08, obs)
    # 不断言具体触发与否（依赖符号约定），只保证结果结构完好
    assert isinstance(d, AebPathDecision)
    assert math.isfinite(d.brake_cmd)


def test_fuse_only_raises_never_lowers():
    """融合只增不减：engaged 时 max() 抬高制动并置 aeb_active。"""
    ctx = LongitudinalContext(lon_cmd=1.0, aeb_active=False)
    dec = AebPathDecision(engaged=True, risk=True, brake_cmd=4.0,
                          t_col=1.0, obstacle_tid=2, rss_dist=12.0,
                          rss_violated=True)
    fused = fuse_path_decision(ctx, dec)
    assert fused.lon_cmd == 4.0
    assert fused.aeb_active
    assert fused.aeb_path_risk
    # 已有更强制动时不减小
    ctx2 = LongitudinalContext(lon_cmd=6.0, aeb_active=True)
    fused2 = fuse_path_decision(ctx2, dec)
    assert fused2.lon_cmd == 6.0


def test_fuse_inactive_only_fills_diagnostics():
    """未触发：控制量原样，仅回填诊断字段。"""
    ctx = LongitudinalContext(lon_cmd=1.5, aeb_active=False)
    dec = AebPathDecision(engaged=False, risk=True, brake_cmd=0.0,
                          t_col=2.4, obstacle_tid=2, rss_dist=8.0)
    fused = fuse_path_decision(ctx, dec)
    assert fused.lon_cmd == 1.5
    assert not fused.aeb_active
    assert fused.aeb_path_risk
    assert fused.aeb_path_tcol == 2.4
    # None 决策（未启用）→ 原对象直返
    assert fuse_path_decision(ctx, None) is ctx


def test_obstacles_from_lead_ctx():
    """离线合成：无前车 → 空；有前车 → 单元组。"""
    assert obstacles_from_lead_ctx(None) == []
    empty = LeadContext()
    assert obstacles_from_lead_ctx(empty) == []
    lead = LeadContext(x_rel=20.0, y_rel=0.5, lead_detected=True,
                       raw_lead_v_proj=3.0, lead_cls=1)
    obs = obstacles_from_lead_ctx(lead)
    assert len(obs) == 1
    assert obs[0][2] == 20.0 and obs[0][4] == 3.0


def test_pipeline_integration_crossing_pedestrian():
    """端到端：无主前车 + 显式行人障碍列表 → run_pure_pipeline 触发 AEB。

    这是现有 TTC AEB 完全覆盖不到的场景（行人不在 lead 槽位），验证
    pipeline 融合插桩、lon_ctx 诊断字段、aeb_active 通路全部打通。
    """
    import dataclasses as _dc

    from replay import build_stack
    from pipeline import run_pure_pipeline

    signals, memory, managers = build_stack()
    managers = _dc.replace(managers, aeb_path=AebPathChecker())
    signals.ego_v = 10.0

    ped = [(5, ACTOR_CLASS_PEDESTRIAN, 20.0, 6.0, 10.0, 2.0)]
    engaged_cycle = None
    res = None
    for k in range(AEB_PATH_CONFIRM_CYCLES + 3):
        now = 100.0 + k * memory.dt
        res = run_pure_pipeline(now, signals, memory, managers,
                                path_obstacles=ped)
        if res.lon_ctx.aeb_active and engaged_cycle is None:
            engaged_cycle = k
    assert engaged_cycle is not None            # 消抖后触发
    assert engaged_cycle >= AEB_PATH_CONFIRM_CYCLES - 1
    assert res.lon_ctx.aeb_path_risk
    assert res.lon_ctx.aeb_path_tcol < 2.6
    assert res.lon_ctx.lon_cmd > 0.0            # 正=制动

    # 同一栈、无障碍列表（默认 None → lead_ctx 合成路径，无前车 → 空）→ 不触发
    signals2, memory2, managers2 = build_stack()
    managers2 = _dc.replace(managers2, aeb_path=AebPathChecker())
    signals2.ego_v = 10.0
    res2 = run_pure_pipeline(100.0, signals2, memory2, managers2)
    assert not res2.lon_ctx.aeb_active


def test_pickle_roundtrip_for_lockstep():
    """锁步线格式：障碍列表 + checker 状态均可 pickle（protocol=2）。"""
    import pickle
    obs = [(2, 1, 15.0, 0.0, 10.0, 0.0)]
    assert pickle.loads(pickle.dumps(obs, protocol=2)) == obs
    checker = AebPathChecker()
    checker.evaluate(10.0, 0.0, obs)
    c2 = pickle.loads(pickle.dumps(checker, protocol=2))
    assert c2.confirm_count == checker.confirm_count
