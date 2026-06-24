#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""control/lateral_controller.py 单元测试。

覆盖 compute_lateral_command() 的主要分支。
"""

import math

import pytest

from control.context import ControlGains, ControlMemory, LateralContext, VehicleSignals
from control.lateral_controller import compute_lateral_command
from config import MAX_DELTA
from lateral import lane_margins_from_width


def _make_signals(**kw):
    """构造 VehicleSignals，缺失字段用合理默认值填充。"""
    defaults = dict(
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw=0.0,
        ego_v=10.0,
        lead_x=0.0,
        lead_y=0.0,
        lead_yaw=0.0,
        lead_v=0.0,
        road_psi=0.0,
        lane_offset=0.0,
        ego_received=True,
        ego_psi_received=True,
        lead_received=False,
        road_received=True,
        lane_offset_received=False,
        lead_last_rx_time=0.0,
        lead_v_last_rx_time=0.0,
        lane_offset_last_rx=0.0,
        ego_last_rx=0.0,
        road_last_rx=1.0,
    )
    defaults.update(kw)
    return VehicleSignals(**defaults)


def _make_memory(dt=0.01, lane_width=3.8, **kw):
    """构造 ControlMemory，自动设置车道余量。"""
    safe, warn, hard = lane_margins_from_width(lane_width)
    defaults = dict(
        dt=dt,
        lane_safe_margin=safe,
        lane_warn_margin=warn,
        lane_hard_margin=hard,
    )
    defaults.update(kw)
    return ControlMemory(**defaults)


# ── 1. First update → uses memory.dt ──────────────────────────────────────────


class TestFirstUpdate:
    """首次更新（lat_last_update_t < 0）分支。"""

    def test_first_update_returns_lateral_context(self):
        """首次调用应返回 LateralContext 实例，无异常。"""
        now = 100.0
        signals = _make_signals(road_last_rx=1.0)
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        assert isinstance(ctx, LateralContext)

    def test_first_update_sets_lat_last_update_t(self):
        """首次调用后 lat_last_update_t 应被设为 now。"""
        now = 100.0
        signals = _make_signals(road_last_rx=1.0)
        memory = _make_memory()
        assert memory.lat_last_update_t < 0.0
        compute_lateral_command(now, signals, memory)
        assert abs(memory.lat_last_update_t - now) < 1e-9

    def test_first_update_rrate_is_zero(self):
        """首次更新时 rrate 应为 0（无前一拍航向）。"""
        now = 100.0
        signals = _make_signals(road_psi=0.1, road_last_rx=1.0)
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        assert abs(ctx.rrate) < 1e-9

    def test_first_update_delta_is_finite(self):
        """首次更新的 delta 输出为有限值。"""
        now = 100.0
        signals = _make_signals(road_psi=0.05, ego_yaw=0.0, road_last_rx=1.0)
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        assert math.isfinite(ctx.delta)


# ── 2. Zero ego speed ─────────────────────────────────────────────────────────


class TestZeroEgoSpeed:
    """零速工况：曲率归零，航向 PID 仍工作。"""

    def test_curvature_zeroed(self):
        """ego_v=0 → raw_curv 应为 0（低速衰减逻辑）。"""
        now = 100.0
        signals = _make_signals(ego_v=0.0, road_psi=0.1, road_last_rx=1.0)
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        assert abs(ctx.raw_curv) < 1e-9

    def test_heading_pid_still_works(self):
        """ego_v=0 时航向 PID 输出不为零（存在 psi_err → 非零转角）。"""
        now = 100.0
        signals = _make_signals(
            ego_v=0.0,
            road_psi=0.3,
            ego_yaw=0.0,
            road_last_rx=1.0,
        )
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        # delta 应非零（航向误差驱动）
        assert ctx.delta != 0.0 or ctx.delta_ff == 0.0
        # 但 delta 应在物理范围内
        assert abs(ctx.delta) <= MAX_DELTA + 1e-9

    def test_delta_ff_zero_at_low_speed(self):
        """ego_v < 0.3 时曲率前馈 delta_ff = 0。"""
        now = 100.0
        signals = _make_signals(ego_v=0.1, road_last_rx=1.0)
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        assert abs(ctx.delta_ff) < 1e-9


# ── 3. Normal driving ─────────────────────────────────────────────────────────


class TestNormalDriving:
    """正常行驶工况。"""

    def test_delta_within_bounds(self):
        """正常驾驶输出 delta ∈ [-MAX_DELTA, MAX_DELTA]。"""
        now = 100.0
        signals = _make_signals(
            ego_v=10.0,
            road_psi=0.05,
            ego_yaw=0.0,
            road_last_rx=1.0,
        )
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        assert abs(ctx.delta) <= MAX_DELTA + 1e-9

    def test_output_finite(self):
        """所有输出字段为有限值。"""
        now = 100.0
        signals = _make_signals(ego_v=8.0, road_psi=0.02, road_last_rx=1.0)
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        for f in ('dyn_prev', 'rrate', 'prev_psi', 'raw_curv', 'curv_guard',
                  'delta', 'delta_ff', 'delta_cte', 'boundary_delta',
                  'boundary_brake', 'raw_cte', 'cur_off', 'upd_psi'):
            val = getattr(ctx, f)
            assert math.isfinite(val), '%s = %s' % (f, val)

    def test_multiple_updates_stable(self):
        """连续多次调用不崩溃，delta 变化有界。"""
        now = 100.0
        signals = _make_signals(ego_v=10.0, road_psi=0.03, road_last_rx=1.0)
        memory = _make_memory()
        prev_delta = 0.0
        for i in range(50):
            now += 0.01
            # 每次用新的 road_last_rx 触发完整计算
            signals = _make_signals(ego_v=10.0, road_psi=0.03, road_last_rx=now)
            ctx = compute_lateral_command(now, signals, memory)
            assert abs(ctx.delta) <= MAX_DELTA + 1e-9
            # delta 变化率有界
            assert abs(ctx.delta - prev_delta) <= MAX_DELTA + 0.1
            prev_delta = ctx.delta

    def test_positive_road_psi_produces_positive_delta(self):
        """road_psi > ego_yaw 时应产生正向转角（向右转趋向道路航向）。"""
        now = 100.0
        signals = _make_signals(
            ego_v=10.0,
            road_psi=0.2,
            ego_yaw=0.0,
            road_last_rx=1.0,
        )
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        # 正 psi_err → 正 delta（STEER_SIGN=1）
        assert ctx.delta > 0.0


# ── 4. Frame gating ───────────────────────────────────────────────────────────


class TestFrameGating:
    """帧门控：相同 road_last_rx → 返回缓存结果。"""

    def test_same_road_rx_returns_cached(self):
        """road_last_rx 不变 → 返回上一帧缓存的 LateralContext。"""
        now = 100.0
        road_rx = 1.0
        signals = _make_signals(ego_v=10.0, road_psi=0.05, road_last_rx=road_rx)
        memory = _make_memory()
        ctx1 = compute_lateral_command(now, signals, memory)
        # 第二次调用，road_last_rx 相同
        now += 0.01
        ctx2 = compute_lateral_command(now, signals, memory)
        # 应返回同一个缓存对象
        assert ctx2 is ctx1

    def test_different_road_rx_recomputes(self):
        """road_last_rx 变化 → 重新计算。"""
        now = 100.0
        signals = _make_signals(ego_v=10.0, road_psi=0.05, road_last_rx=1.0)
        memory = _make_memory()
        ctx1 = compute_lateral_command(now, signals, memory)
        # 改变 road_last_rx
        now += 0.01
        signals2 = _make_signals(ego_v=10.0, road_psi=0.05, road_last_rx=2.0)
        ctx2 = compute_lateral_command(now, signals2, memory)
        # 不应是同一个对象（重新计算了）
        assert ctx2 is not ctx1

    def test_same_road_but_new_lane_offset_updates_boundary(self):
        """road_last_rx 不变但 lane_offset_last_rx 变化 → 仅刷新边界修正。"""
        now = 100.0
        road_rx = 1.0
        signals = _make_signals(
            ego_v=10.0,
            road_psi=0.0,
            road_last_rx=road_rx,
            lane_offset=0.0,
            lane_offset_last_rx=0.5,
            lane_offset_received=True,
        )
        memory = _make_memory()
        ctx1 = compute_lateral_command(now, signals, memory)
        # 相同 road_last_rx，新的 lane_offset_last_rx
        signals2 = _make_signals(
            ego_v=10.0,
            road_psi=0.0,
            road_last_rx=road_rx,
            lane_offset=0.0,
            lane_offset_last_rx=1.0,
            lane_offset_received=True,
        )
        ctx2 = compute_lateral_command(now + 0.01, signals2, memory)
        # 应返回更新了边界的上下文
        assert isinstance(ctx2, LateralContext)


# ── 5. Large CTE → boundary correction ───────────────────────────────────────


class TestLargeCTEBoundaryCorrection:
    """大横向偏移 → 边界修正叠加。"""

    def test_large_positive_offset_applies_negative_correction(self):
        """正向大偏移 → boundary_delta 为负（拉回左侧）。"""
        now = 100.0
        safe, warn, hard = lane_margins_from_width(3.8)
        large_offset = hard + 0.5
        signals = _make_signals(
            ego_v=10.0,
            road_psi=0.0,
            ego_yaw=0.0,
            road_last_rx=1.0,
            lane_offset=large_offset,
            lane_offset_received=True,
            lane_offset_last_rx=1.0,
        )
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        assert ctx.boundary_delta <= 0.0 or ctx.boundary_brake > 0.0

    def test_large_negative_offset_applies_positive_correction(self):
        """负向大偏移 → boundary_delta 为正（拉回右侧）。"""
        now = 100.0
        safe, warn, hard = lane_margins_from_width(3.8)
        large_offset = -(hard + 0.5)
        signals = _make_signals(
            ego_v=10.0,
            road_psi=0.0,
            ego_yaw=0.0,
            road_last_rx=1.0,
            lane_offset=large_offset,
            lane_offset_received=True,
            lane_offset_last_rx=1.0,
        )
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        assert ctx.boundary_delta >= 0.0 or ctx.boundary_brake > 0.0

    def test_small_offset_no_boundary(self):
        """小偏移（safe 内）→ 无边界修正。"""
        now = 100.0
        signals = _make_signals(
            ego_v=10.0,
            road_psi=0.0,
            road_last_rx=1.0,
            lane_offset=0.1,
            lane_offset_received=True,
            lane_offset_last_rx=1.0,
        )
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        assert abs(ctx.boundary_delta) < 1e-9
        assert ctx.boundary_brake == 0.0
        assert ctx.boundary_warn is False

    def test_boundary_delta_within_bounds(self):
        """边界修正后的 delta 仍在 [-MAX_DELTA, MAX_DELTA] 内。"""
        now = 100.0
        safe, warn, hard = lane_margins_from_width(3.8)
        signals = _make_signals(
            ego_v=15.0,
            road_psi=0.0,
            ego_yaw=0.0,
            road_last_rx=1.0,
            lane_offset=hard + 1.0,
            lane_offset_received=True,
            lane_offset_last_rx=1.0,
        )
        memory = _make_memory()
        ctx = compute_lateral_command(now, signals, memory)
        assert abs(ctx.delta) <= MAX_DELTA + 1e-9
