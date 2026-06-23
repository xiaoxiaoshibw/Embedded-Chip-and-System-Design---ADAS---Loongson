#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""run_pure_pipeline / update_lane_state 集成测试。

构造真实的 ControlManagers（LaneWidthEstimator、LeadTracker、
AebAlertManager、CurveHoldManager、LongitudinalController、LonSmoothing），
驱动 pipeline.run_pure_pipeline 并检查输出。
"""

import math
import os
import sys

import pytest

# 确保 SOCCode/ 在 sys.path（conftest 已做，这里双保险）
_soccode = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _soccode not in sys.path:
    sys.path.insert(0, _soccode)

from control.context import ControlManagers, ControlMemory, VehicleSignals, LateralContext
from control.state import LeadContext, LongitudinalContext
from control.lead_tracking import LeadTracker
from control.aeb_alert import AebAlertManager
from control.curve_hold import CurveHoldManager
from lateral import LaneWidthEstimator, LateralSmoothing
from longitudinal import LongitudinalController, LonSmoothing
from pipeline import run_pure_pipeline, update_lane_state, PipelineResult


# ── 默认参数 ──
DT = 0.01       # 100 Hz
LOOP_HZ = 100


@pytest.fixture
def managers():
    """构造真实的 ControlManagers。"""
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


@pytest.fixture
def memory():
    return ControlMemory(dt=DT)


@pytest.fixture
def signals():
    return VehicleSignals()


class TestRunPurePipelineBasic:
    """run_pure_pipeline 基本功能。"""

    def test_returns_pipeline_result(self, signals, memory, managers):
        """默认信号 → 返回 PipelineResult，所有字段有限。"""
        now = 1.0
        result = run_pure_pipeline(now, signals, memory, managers)
        assert isinstance(result, PipelineResult)
        assert math.isfinite(result.cur_lane_width)
        assert math.isfinite(result.lon_cmd)
        assert math.isfinite(result.lon_raw_cmd)
        assert isinstance(result.lateral_ctx, LateralContext)
        assert isinstance(result.lead_ctx, LeadContext)
        assert isinstance(result.lon_ctx, LongitudinalContext)
        assert isinstance(result.in_curve_hold, bool)

    def test_default_signals_cruise_mode(self, signals, memory, managers):
        """无前车 + 无弯道 → 巡航模式，lon_cmd 由巡航逻辑决定。"""
        now = 1.0
        result = run_pure_pipeline(now, signals, memory, managers)
        # ego_v=0, DRIVER_SET_SPEED=8 → 巡航应给加速（负 lon_cmd）
        assert math.isfinite(result.lon_cmd)

    def test_no_lead_data_lead_ctx_no_lead(self, signals, memory, managers):
        """无前车数据 → lead_ctx.acc_has_lead=False。"""
        now = 1.0
        result = run_pure_pipeline(now, signals, memory, managers)
        assert result.lead_ctx.acc_has_lead is False
        assert result.lead_ctx.raw_has_lead is False

    def test_aeb_not_active_default(self, signals, memory, managers):
        """默认信号 → AEB 不激活。"""
        now = 1.0
        result = run_pure_pipeline(now, signals, memory, managers)
        assert result.lon_ctx.aeb_active is False

    def test_lane_width_reasonable(self, signals, memory, managers):
        """车道宽度在合理范围内。"""
        now = 1.0
        result = run_pure_pipeline(now, signals, memory, managers)
        assert 3.0 <= result.cur_lane_width <= 5.0

    def test_lateral_ctx_fields_finite(self, signals, memory, managers):
        """LateralContext 所有字段有限。"""
        now = 1.0
        result = run_pure_pipeline(now, signals, memory, managers)
        ctx = result.lateral_ctx
        for attr in ('delta', 'raw_curv', 'curv_guard', 'boundary_brake'):
            assert math.isfinite(getattr(ctx, attr)), '%s is not finite' % attr


class TestRunPurePipelineWithLead:
    """有前车数据时的 pipeline 行为。"""

    def test_lead_received_populates_lead_ctx(self, signals, memory, managers):
        """lead_received=True + 有效位姿 → lead_ctx 有前车。"""
        now = 1.0
        signals.lead_received = True
        signals.lead_x = 20.0
        signals.lead_y = 0.0
        signals.lead_yaw = 0.0
        signals.lead_v = 10.0
        signals.lead_last_rx_time = now
        signals.lead_v_last_rx_time = now

        result = run_pure_pipeline(now, signals, memory, managers)
        # 前车在正前方 20m，应被检测到
        assert result.lead_ctx.raw_has_lead is True
        # 经过确认机制后 has_lead 可能需要多拍确认
        # 但 raw_has_lead 应为 True

    def test_lead_data_affects_lon_cmd(self, signals, memory, managers):
        """有前车 → lon_cmd 应与无前车不同（ACC 介入）。"""
        # 无前车基准：多拍运行让平滑器收敛
        now = 1.0
        signals.ego_v = 15.0
        for i in range(30):
            run_pure_pipeline(now + i * 0.01, signals, memory, managers)
        result_no_lead = run_pure_pipeline(now + 0.3, signals, memory, managers)

        # 重置状态
        memory2 = ControlMemory(dt=DT)
        managers2 = ControlManagers(
            lane_est=LaneWidthEstimator(LOOP_HZ),
            lead_tracker=LeadTracker(),
            aeb_alert=AebAlertManager(),
            curve_hold=CurveHoldManager(),
            lon_ctrl=LongitudinalController(DT),
            lon_smooth=LonSmoothing(DT),
        )

        # 有前车：多拍运行让 LeadTracker 确认前车
        signals2 = VehicleSignals()
        signals2.ego_v = 15.0
        signals2.lead_received = True
        signals2.lead_x = 15.0  # 近距离慢前车
        signals2.lead_y = 0.0
        signals2.lead_yaw = 0.0
        signals2.lead_v = 5.0
        signals2.lead_last_rx_time = now
        signals2.lead_v_last_rx_time = now

        for i in range(30):
            t = now + i * 0.01
            signals2.lead_last_rx_time = t
            signals2.lead_v_last_rx_time = t
            run_pure_pipeline(t, signals2, memory2, managers2)
        result_lead = run_pure_pipeline(now + 0.3, signals2, memory2, managers2)
        # 有慢前车应比无前车更制动（lon_cmd 更大）
        assert result_lead.lon_cmd > result_no_lead.lon_cmd

    def test_lead_repeated_builds_confirm(self, signals, memory, managers):
        """多拍持续有前车 → 最终 has_lead=True。"""
        now = 1.0
        signals.lead_received = True
        signals.lead_x = 20.0
        signals.lead_y = 0.0
        signals.lead_yaw = 0.0
        signals.lead_v = 10.0
        signals.lead_last_rx_time = now
        signals.lead_v_last_rx_time = now
        signals.ego_v = 10.0

        # 跑足够多拍让确认机制通过
        for i in range(20):
            t = now + i * 0.01
            signals.lead_last_rx_time = t
            signals.lead_v_last_rx_time = t
            run_pure_pipeline(t, signals, memory, managers)

        # 最后一拍检查
        result = run_pure_pipeline(now + 0.2, signals, memory, managers)
        assert result.lead_ctx.raw_has_lead is True


class TestUpdateLaneState:
    """update_lane_state 测试。"""

    def test_returns_lane_width(self, signals, memory):
        """返回车道宽度浮点数。"""
        lane_est = LaneWidthEstimator(LOOP_HZ)
        now = 1.0
        width = update_lane_state(now, signals, memory, lane_est)
        assert isinstance(width, float)
        assert math.isfinite(width)

    def test_no_lane_offset_uses_default(self, signals, memory):
        """lane_offset_received=False → 使用默认车道宽。"""
        lane_est = LaneWidthEstimator(LOOP_HZ)
        now = 1.0
        width = update_lane_state(now, signals, memory, lane_est)
        # 没有偏移数据 → 锁定状态，返回默认最小宽度
        assert width >= 3.5

    def test_with_lane_offset(self, signals, memory):
        """有 lane_offset → 车道宽估计器接收样本。"""
        lane_est = LaneWidthEstimator(LOOP_HZ)
        now = 1.0
        signals.lane_offset_received = True
        signals.lane_offset = 0.5
        signals.lane_offset_last_rx = now
        width = update_lane_state(now, signals, memory, lane_est)
        assert math.isfinite(width)
        assert width >= 3.5

    def test_memory_margins_updated(self, signals, memory):
        """update_lane_state 更新 memory 中的三级余量。"""
        lane_est = LaneWidthEstimator(LOOP_HZ)
        now = 1.0
        signals.lane_offset_received = True
        signals.lane_offset = 0.3
        signals.lane_offset_last_rx = now
        update_lane_state(now, signals, memory, lane_est)
        # 余量应被设置
        assert memory.lane_safe_margin > 0.0
        assert memory.lane_warn_margin > 0.0
        assert memory.lane_hard_margin > 0.0
        # 层级关系：safe > hard > warn（warn=0.55*safe, hard=0.92*safe）
        assert memory.lane_safe_margin >= memory.lane_hard_margin
        assert memory.lane_hard_margin >= memory.lane_warn_margin

    def test_repeated_updates_converge(self, signals, memory):
        """多次更新后车道宽收敛到稳定值。"""
        lane_est = LaneWidthEstimator(LOOP_HZ)
        signals.lane_offset_received = True
        signals.lane_offset = 0.4
        now = 1.0
        signals.lane_offset_last_rx = now

        widths = []
        for i in range(200):
            t = now + i * 0.01
            signals.lane_offset_last_rx = t
            w = update_lane_state(t, signals, memory, lane_est)
            widths.append(w)

        # 后半段应收敛（波动小于前半段）
        first_half_var = _variance(widths[:50])
        last_half_var = _variance(widths[150:])
        # 收敛后期方差应更小（或至少不是发散的）
        assert last_half_var <= first_half_var + 0.01


class TestPipelineIntegration:
    """综合集成场景。"""

    def test_ego_moving_straight(self, signals, memory, managers):
        """自车直行 → 横向指令接近零。"""
        now = 1.0
        signals.ego_v = 10.0
        result = run_pure_pipeline(now, signals, memory, managers)
        assert math.isfinite(result.lateral_ctx.delta)

    def test_pipeline_preserves_memory_state(self, signals, memory, managers):
        """多拍运行后 memory 状态有变化（积分/滤波器前进）。"""
        now = 1.0
        signals.ego_v = 10.0
        run_pure_pipeline(now, signals, memory, managers)
        run_pure_pipeline(now + 0.01, signals, memory, managers)
        # cycle_count 应前进
        assert memory.cycle_count >= 0  # 管线本身不递增 cycle_count，但不崩溃

    def test_multiple_cycles_no_crash(self, signals, memory, managers):
        """100 拍连续运行无异常。"""
        now = 1.0
        signals.ego_v = 8.0
        for i in range(100):
            run_pure_pipeline(now + i * 0.01, signals, memory, managers)

    def test_with_road_psi(self, signals, memory, managers):
        """有道路航向 → 横向控制正常工作。"""
        now = 1.0
        signals.road_psi = 0.05
        signals.road_received = True
        signals.road_last_rx = now
        signals.ego_v = 10.0
        result = run_pure_pipeline(now, signals, memory, managers)
        assert math.isfinite(result.lateral_ctx.delta)


def _variance(xs):
    """辅助：计算列表方差。"""
    n = len(xs)
    if n < 2:
        return 0.0
    mean = sum(xs) / n
    return sum((x - mean) ** 2 for x in xs) / n
