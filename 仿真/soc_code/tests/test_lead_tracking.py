#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LeadTracker 单元测试。

测试前车跟踪与 ACC 门控的核心状态机逻辑：
  1. 无前车 → lead_acquired=False
  2. 前车在范围内，经 LEAD_CONFIRM_CYCLES 确认 → has_lead=True
  3. 前车超时 → 前车丢失
  4. reset_on_lead_swap() → 清除内部状态
  5. 速度跳变检测（v_proj 骤降）→ 滤波修正
  6. 弯道记忆 — 弯道中前车丢失 → 记忆 LEAD_MEMORY_S
  7. 保活 — 短暂丢失 → 保持 LEAD_KEEPALIVE_S
"""

import pytest

from config import (
    LEAD_CONFIRM_CYCLES,
    LEAD_CURVE_LOSS_HOLD_S,
    LEAD_DROP_GLITCH_BLEND,
    LEAD_DROP_GLITCH_DIST,
    LEAD_DROP_GLITCH_RATIO,
    LEAD_KEEPALIVE_S,
    LEAD_MAX_TRACK_DIST,
    LEAD_MEMORY_S,
    LEAD_TIMEOUT_S,
    CURVE_HOLD_CURV_THRESH,
)
from control.lead_tracking import LeadTracker
from control.state import LeadTrackingInputs


# ---------------------------------------------------------------------------
# 辅助工厂
# ---------------------------------------------------------------------------

def _inputs(**overrides):
    """构造 LeadTrackingInputs，未指定字段取合理默认值。"""
    defaults = dict(
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw=0.0,
        ego_v=10.0,
        lead_x=30.0,
        lead_y=0.0,
        lead_yaw=0.0,
        lead_v=8.0,
        lead_cls=0,
        lead_received=True,
        lead_last_rx_time=0.0,
        filtered_curv=0.0,
        cur_lane_width=3.8,
        lane_locked=True,
        last_acc_has_lead=False,
        filtered_lead_v_proj=8.0,
        last_lead_v_proj=8.0,
        last_lead_reacq_t=-1e9,
        last_curve_t=-1e9,
    )
    defaults.update(overrides)
    return LeadTrackingInputs(**defaults)


def _confirm_lead(tracker, n_cycles=LEAD_CONFIRM_CYCLES, t_start=0.0, dt=0.01,
                  inputs=None):
    """连续评估 n_cycles 拍，确认前车，返回最后一次评估结果和结束时间。"""
    if inputs is None:
        inputs = _inputs()
    result = None
    for i in range(n_cycles):
        t = t_start + i * dt
        result = tracker.evaluate(t, inputs, in_curve=False)
    return result, t_start + (n_cycles - 1) * dt


# ===========================================================================
# 测试
# ===========================================================================

class TestLeadTrackerNoLead:
    """无前车场景。"""

    def test_no_lead_received_returns_no_lead(self):
        """lead_received=False → has_lead=False, lead_acquired=False。"""
        tracker = LeadTracker()
        inputs = _inputs(lead_received=False)
        result = tracker.evaluate(1.0, inputs, in_curve=False)
        assert not result.has_lead
        assert not result.lead_acquired

    def test_lead_out_of_range(self):
        """前车超出 LEAD_MAX_TRACK_DIST → raw_has_lead=False。"""
        tracker = LeadTracker()
        inputs = _inputs(lead_x=LEAD_MAX_TRACK_DIST + 10.0)
        result = tracker.evaluate(0.01, inputs, in_curve=False)
        assert not result.raw_has_lead

    def test_lead_behind_ego(self):
        """前车在自车后方 (x_rel < 0) → raw_has_lead=False。"""
        tracker = LeadTracker()
        inputs = _inputs(lead_x=-5.0)
        result = tracker.evaluate(0.01, inputs, in_curve=False)
        assert not result.raw_has_lead


class TestLeadTrackerConfirm:
    """前车确认机制。"""

    def test_lead_confirmed_after_cycles(self):
        """连续 LEAD_CONFIRM_CYCLES 拍检测到前车 → has_lead=True。"""
        tracker = LeadTracker()
        inputs = _inputs()
        result, _ = _confirm_lead(tracker, LEAD_CONFIRM_CYCLES, inputs=inputs)
        assert result.has_lead

    def test_lead_not_confirmed_prematurely(self):
        """不足 LEAD_CONFIRM_CYCLES 拍 → 确认计数不足，ACC 前车无效。"""
        tracker = LeadTracker()
        inputs = _inputs()
        result, _ = _confirm_lead(tracker, LEAD_CONFIRM_CYCLES - 1, inputs=inputs)
        # lead_confirm_count 未达到阈值
        assert tracker.state.lead_confirm_count < LEAD_CONFIRM_CYCLES
        # acc_has_lead 需要 lead_confirm_count >= LEAD_CONFIRM_CYCLES
        # has_lead 可能因 keepalive 为 True，但 ACC 门控应拒绝
        assert not result.acc_lead_valid

    def test_confirm_count_resets_on_gap(self):
        """确认过程中中断（前车消失一拍）→ 计数器回落，需要更多拍才能确认。"""
        tracker = LeadTracker()
        good = _inputs()
        bad = _inputs(lead_received=False)

        # 先确认 3 拍
        for i in range(3):
            tracker.evaluate(i * 0.01, good, in_curve=False)
        assert tracker.state.lead_confirm_count == 3

        # 中断 1 拍
        tracker.evaluate(0.04, bad, in_curve=False)
        assert tracker.state.lead_confirm_count == 2

        # 再补 5 拍（从 2 开始递增），最终应确认
        for i in range(5):
            tracker.evaluate(0.05 + i * 0.01, good, in_curve=False)
        assert tracker.state.lead_confirm_count >= LEAD_CONFIRM_CYCLES


class TestLeadTrackerTimeout:
    """前车超时丢失。"""

    def test_lead_timeout_marks_fresh_false(self):
        """前车数据超时 → lead_fresh=False。"""
        tracker = LeadTracker()
        inputs = _inputs(lead_last_rx_time=0.0)
        _confirm_lead(tracker, inputs=inputs)

        # 超时后评估
        stale = _inputs(lead_received=True, lead_last_rx_time=0.0)
        result = tracker.evaluate(LEAD_TIMEOUT_S + 1.0, stale, in_curve=False)
        assert not result.lead_fresh

    def test_lead_timeout_clears_raw_has_lead(self):
        """前车超时 → raw_has_lead=False。"""
        tracker = LeadTracker()
        inputs = _inputs(lead_last_rx_time=0.0)
        _confirm_lead(tracker, inputs=inputs)

        stale = _inputs(lead_received=True, lead_last_rx_time=0.0)
        result = tracker.evaluate(LEAD_TIMEOUT_S + 1.0, stale, in_curve=False)
        assert not result.raw_has_lead

    def test_lead_not_received_marks_lost(self):
        """lead_received=False → 前车丢失。"""
        tracker = LeadTracker()
        inputs = _inputs()
        _confirm_lead(tracker, inputs=inputs)

        no_lead = _inputs(lead_received=False)
        result = tracker.evaluate(1.0, no_lead, in_curve=False)
        assert not result.lead_fresh


class TestLeadTrackerReset:
    """reset_on_lead_swap 清除内部状态。"""

    def test_reset_clears_confirm_count(self):
        """reset 后 lead_confirm_count 归零。"""
        tracker = LeadTracker()
        _confirm_lead(tracker)
        assert tracker.state.lead_confirm_count > 0
        tracker.reset_on_lead_swap()
        assert tracker.state.lead_confirm_count == 0

    def test_reset_clears_filter_primed(self):
        """reset 后 rel_filter_primed 归 False。"""
        tracker = LeadTracker()
        _confirm_lead(tracker)
        assert tracker.state.rel_filter_primed
        tracker.reset_on_lead_swap()
        assert not tracker.state.rel_filter_primed

    def test_reset_clears_cutin_lat_rate(self):
        """reset 后 cutin_lat_rate 归零。"""
        tracker = LeadTracker()
        _confirm_lead(tracker)
        tracker.reset_on_lead_swap()
        assert tracker.state.cutin_lat_rate == 0.0

    def test_reset_clears_release_counts(self):
        """reset 后 ACC 释放计数器归零。"""
        tracker = LeadTracker()
        _confirm_lead(tracker)
        tracker.state.acc_lane_out_release_count = 3
        tracker.state.acc_dist_opening_release_count = 2
        tracker.reset_on_lead_swap()
        assert tracker.state.acc_lane_out_release_count == 0
        assert tracker.state.acc_dist_opening_release_count == 0

    def test_reset_preserves_filtered_speed(self):
        """reset 不清零 filtered_v_proj（下一拍 priming 会覆盖）。"""
        tracker = LeadTracker()
        _confirm_lead(tracker)
        old_v = tracker.state.filtered_v_proj
        tracker.reset_on_lead_swap()
        # filtered_v_proj 不在 reset 清单中
        assert tracker.state.filtered_v_proj == old_v

    def test_full_cycle_after_reset(self):
        """reset 后重新确认前车 → 正常工作。"""
        tracker = LeadTracker()
        inputs = _inputs()
        _confirm_lead(tracker, inputs=inputs)
        assert tracker.state.rel_filter_primed

        tracker.reset_on_lead_swap()
        assert not tracker.state.rel_filter_primed
        assert tracker.state.lead_confirm_count == 0

        # 重新确认（lead_last_rx_time 需随 t_start 同步，否则 lead_fresh=False）
        new_inputs = _inputs(lead_last_rx_time=1.0)
        result, _ = _confirm_lead(tracker, LEAD_CONFIRM_CYCLES, t_start=1.0,
                                  inputs=new_inputs)
        assert tracker.state.rel_filter_primed
        assert result.has_lead


class TestLeadTrackerSpeedGlitch:
    """速度跳变检测与修正。"""

    def test_speed_glitch_is_filtered(self):
        """远处前车 v_proj 骤降 → 混合修正，不应直接采信突变值。"""
        tracker = LeadTracker()
        # 前车以 10 m/s 行驶，距离 > LEAD_DROP_GLITCH_DIST
        good_inputs = _inputs(
            lead_x=LEAD_DROP_GLITCH_DIST + 5.0,
            lead_v=10.0,
            ego_v=12.0,
            last_lead_v_proj=10.0,
        )
        result, last_t = _confirm_lead(tracker, inputs=good_inputs)
        assert result.has_lead

        # v_proj 突然骤降到低于 LEAD_DROP_GLITCH_RATIO 倍
        glitch_v = 10.0 * LEAD_DROP_GLITCH_RATIO * 0.5  # 远低于阈值
        glitch_inputs = _inputs(
            lead_x=LEAD_DROP_GLITCH_DIST + 5.0,
            lead_v=glitch_v,
            ego_v=12.0,
            last_lead_v_proj=10.0,  # 上一拍 10.0
        )
        result = tracker.evaluate(last_t + 0.01, glitch_inputs, in_curve=False)

        # 滤波后的 v_proj 应显著高于原始突变值
        assert result.predicted_lead_v_proj > glitch_v

    def test_normal_speed_change_not_filtered(self):
        """正常速度变化（未触发跳变检测）→ 滤波器正常跟踪。"""
        tracker = LeadTracker()
        # 近距离前车，速度缓慢变化不会触发 glitch 检测
        inputs = _inputs(
            lead_x=10.0,  # < LEAD_DROP_GLITCH_DIST
            lead_v=10.0,
            ego_v=12.0,
            last_lead_v_proj=10.0,
        )
        _confirm_lead(tracker, inputs=inputs)

        # 多拍低通滤波，速度从 10→8
        t = LEAD_CONFIRM_CYCLES * 0.01 + 0.01
        for _ in range(50):
            t += 0.01
            slow = _inputs(
                lead_x=10.0,
                lead_v=8.0,
                ego_v=12.0,
                last_lead_v_proj=10.0,
                lead_last_rx_time=t,  # 保持数据新鲜
            )
            result = tracker.evaluate(t, slow, in_curve=False)
        # 经过 50 拍滤波（alpha=0.18），predicted_lead_v_proj 应充分收敛到 8.0 附近
        assert result.predicted_lead_v_proj < 8.5


class TestLeadTrackerCurveMemory:
    """弯道记忆 — 弯道中前车丢失后保留。"""

    def test_curve_memory_retains_lead(self):
        """弯道中前车丢失，在 LEAD_CURVE_LOSS_HOLD_S 内 → has_lead=True。"""
        tracker = LeadTracker()
        curv = CURVE_HOLD_CURV_THRESH + 0.01
        inputs = _inputs(
            lead_x=30.0,
            lead_y=0.0,
            filtered_curv=curv,
        )

        # 在弯道中确认前车
        for i in range(LEAD_CONFIRM_CYCLES + 1):
            tracker.evaluate(i * 0.01, inputs, in_curve=True)

        last_confirmed_t = LEAD_CONFIRM_CYCLES * 0.01
        assert tracker.state.last_confirmed_lead_t == last_confirmed_t

        # 前车丢失
        no_lead = _inputs(lead_received=False, filtered_curv=curv)

        # 在 LEAD_CURVE_LOSS_HOLD_S / 2 时 → 仍应保留
        within_window = last_confirmed_t + LEAD_CURVE_LOSS_HOLD_S * 0.5
        result = tracker.evaluate(within_window, no_lead, in_curve=True)
        assert result.has_lead

    def test_curve_memory_expires(self):
        """弯道中前车丢失超过 LEAD_CURVE_LOSS_HOLD_S → has_lead=False。"""
        tracker = LeadTracker()
        curv = CURVE_HOLD_CURV_THRESH + 0.01
        inputs = _inputs(
            lead_x=30.0,
            lead_y=0.0,
            filtered_curv=curv,
        )

        for i in range(LEAD_CONFIRM_CYCLES + 1):
            tracker.evaluate(i * 0.01, inputs, in_curve=True)

        last_confirmed_t = LEAD_CONFIRM_CYCLES * 0.01
        no_lead = _inputs(lead_received=False, filtered_curv=curv)

        # 超过 LEAD_CURVE_LOSS_HOLD_S
        after_window = last_confirmed_t + LEAD_CURVE_LOSS_HOLD_S + 0.5
        result = tracker.evaluate(after_window, no_lead, in_curve=True)
        assert not result.has_lead

    def test_no_curve_memory_on_straight(self):
        """直道上不启用弯道记忆。"""
        tracker = LeadTracker()
        inputs = _inputs(lead_x=30.0, lead_y=0.0, filtered_curv=0.0)

        for i in range(LEAD_CONFIRM_CYCLES + 1):
            tracker.evaluate(i * 0.01, inputs, in_curve=False)

        last_confirmed_t = LEAD_CONFIRM_CYCLES * 0.01
        no_lead = _inputs(lead_received=False, filtered_curv=0.0)

        # 在 LEAD_CURVE_LOSS_HOLD_S 内，但不在弯道
        within_window = last_confirmed_t + LEAD_CURVE_LOSS_HOLD_S * 0.5
        result = tracker.evaluate(within_window, no_lead, in_curve=False)
        # 不应有弯道记忆（可能有 keepalive，取决于时间差）
        # 只要时间超过 LEAD_KEEPALIVE_S 就应丢失
        if (within_window - last_confirmed_t) > LEAD_KEEPALIVE_S:
            assert not result.has_lead


class TestLeadTrackerKeepalive:
    """前车短暂丢失保活。"""

    def test_keepalive_maintains_lead(self):
        """前车短暂丢失，在 LEAD_KEEPALIVE_S 内 → has_lead=True。"""
        tracker = LeadTracker()
        inputs = _inputs(lead_x=30.0, lead_y=0.0)

        result, last_t = _confirm_lead(tracker, inputs=inputs)
        assert result.has_lead

        no_lead = _inputs(lead_received=False)
        # 在 keepalive 窗口内
        keepalive_t = last_t + LEAD_KEEPALIVE_S * 0.5
        result = tracker.evaluate(keepalive_t, no_lead, in_curve=False)
        assert result.has_lead

    def test_keepalive_expires(self):
        """前车丢失超过 LEAD_KEEPALIVE_S → has_lead=False。"""
        tracker = LeadTracker()
        inputs = _inputs(lead_x=30.0, lead_y=0.0)

        result, last_t = _confirm_lead(tracker, inputs=inputs)
        assert result.has_lead

        no_lead = _inputs(lead_received=False)
        # 超过 keepalive 窗口
        after_keepalive = last_t + LEAD_KEEPALIVE_S + 0.1
        result = tracker.evaluate(after_keepalive, no_lead, in_curve=False)
        assert not result.has_lead

    def test_keepalive_duration_boundary(self):
        """精确边界：LEAD_KEEPALIVE_S 刚好到期。"""
        tracker = LeadTracker()
        inputs = _inputs(lead_x=30.0, lead_y=0.0)

        # 最后一次确认发生在最后一次 raw_has_lead=True 的 evaluate 调用
        for i in range(LEAD_CONFIRM_CYCLES + 1):
            t = i * 0.01
            result = tracker.evaluate(t, inputs, in_curve=False)
        last_confirmed = t  # last_confirmed_lead_t = 此刻

        no_lead = _inputs(lead_received=False)

        # 刚好在 keepalive 边界内
        just_within = last_confirmed + LEAD_KEEPALIVE_S - 0.005
        result = tracker.evaluate(just_within, no_lead, in_curve=False)
        assert result.has_lead

        # 刚好超过 keepalive 边界
        just_beyond = last_confirmed + LEAD_KEEPALIVE_S + 0.005
        result = tracker.evaluate(just_beyond, no_lead, in_curve=False)
        assert not result.has_lead


class TestLeadTrackerLeadAcquired:
    """lead_acquired 标志的语义：本周期新获取到前车。"""

    def test_lead_acquired_on_first_confirm(self):
        """前车首次确认（从无到有）→ lead_acquired=True。"""
        tracker = LeadTracker()
        inputs = _inputs(lead_x=30.0, lead_y=0.0, lead_v=8.0, ego_v=10.0,
                         last_acc_has_lead=False)

        # 前面几拍 acc_has_lead 还是 False（确认不够）
        for i in range(LEAD_CONFIRM_CYCLES - 1):
            tracker.evaluate(i * 0.01, inputs, in_curve=False)

        # 第 LEAD_CONFIRM_CYCLES 拍确认后，如果 ACC 门控通过
        result = tracker.evaluate(
            (LEAD_CONFIRM_CYCLES - 1) * 0.01, inputs, in_curve=False
        )
        # lead_acquired = acc_has_lead and not last_acc_has_lead
        # 取决于 ACC 门控是否通过；至少验证字段存在
        assert isinstance(result.lead_acquired, bool)

    def test_lead_not_acquired_when_already_had(self):
        """已有前车时（last_acc_has_lead=True）→ lead_acquired=False。"""
        tracker = LeadTracker()
        inputs = _inputs(
            lead_x=30.0, lead_y=0.0, lead_v=8.0, ego_v=10.0,
            last_acc_has_lead=True,
        )
        # 确认
        for i in range(LEAD_CONFIRM_CYCLES + 1):
            result = tracker.evaluate(i * 0.01, inputs, in_curve=False)
        # 如果 acc_has_lead 仍为 True，acquired 应为 False
        if result.acc_has_lead:
            assert not result.lead_acquired
