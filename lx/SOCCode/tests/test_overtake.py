#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OvertakeManager 单元测试。

测试超车状态机的核心逻辑：
  1. IDLE → WAIT：自车+前车均静止超过 OVT_LEAD_LONG_STILL_S
  2. WAIT → ACTIVE：确认超车
  3. ACTIVE 超时 25s → PASSING
  4. ACTIVE → PASSING：已切入左车道
  5. PASSING → RETURN：超过前车
"""

import pytest

from config import (
    LANE_DEFAULT_WIDTH,
    OVT_CONFIRM_TIME_S,
    OVT_EGO_STILL_V,
    OVT_LANE_OFFSET_M,
    OVT_LANE_SHIFT_RATE_M_S,
    OVT_LEAD_LONG_STILL_S,
    OVT_LEAD_PASSED_FWD_M,
    OVT_LEAD_STILL_V,
    OVT_RETURN_DONE_M,
    OVT_SHIFT_DONE_M,
    OVT_TRIGGER_DIST_M,
    OVT_TRIGGER_MIN_DIST_M,
    OVT_RESUME_LEAD_V,
)
from control.context import ControlMemory, VehicleSignals
from control.overtake import (
    OvertakeManager,
    OvertakeState,
    S_ACTIVE,
    S_IDLE,
    S_PASSING,
    S_RETURN,
    S_WAIT,
)
from control.state import LeadTrackerState


# ---------------------------------------------------------------------------
# 辅助工厂
# ---------------------------------------------------------------------------

def _make_signals(**overrides):
    """构造 VehicleSignals，未指定字段取合理默认值。"""
    defaults = dict(
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw=0.0,
        ego_v=0.0,
        lead_x=15.0,
        lead_y=0.0,
        lead_yaw=0.0,
        lead_v=0.0,
        lead_received=True,
    )
    defaults.update(overrides)
    return VehicleSignals(**defaults)


def _make_memory(dt=0.01, **overrides):
    """构造 ControlMemory，未指定字段取合理默认值。"""
    mem = ControlMemory(dt=dt)
    for k, v in overrides.items():
        setattr(mem, k, v)
    return mem


def _make_lt_state(**overrides):
    """构造 LeadTrackerState，未指定字段取合理默认值。"""
    defaults = dict(
        last_lead_x_rel=15.0,
        last_confirmed_lead_t=0.0,
    )
    defaults.update(overrides)
    return LeadTrackerState(**defaults)


def _drive_to_still_phase(manager, signals, memory, lt_state, duration,
                          t_start=0.01, dt=0.5):
    """模拟一段时间的静止行驶（多个 update 调用），返回结束时间。

    每次调用前更新 last_confirmed_lead_t 保持前车"最近确认"。
    """
    t = t_start
    while t < t_start + duration:
        lt_state.last_confirmed_lead_t = t
        manager.update(t, signals, memory, lt_state)
        t += dt
    return t


# ===========================================================================
# 测试
# ===========================================================================

class TestOvertakeIdleToWait:
    """IDLE → WAIT：自车和前车长期静止。"""

    def test_idle_to_wait_transition(self):
        """ego+lead 均静止超过 OVT_LEAD_LONG_STILL_S → 进入 WAIT。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0, lead_received=True)
        memory = _make_memory()
        lt_state = _make_lt_state(last_lead_x_rel=15.0)

        # 先跑一个周期初始化 still_since（> 0 的条件要求 t > 0）
        lt_state.last_confirmed_lead_t = 0.01
        mgr.update(0.01, signals, memory, lt_state)
        assert mgr.state.state == S_IDLE

        # 驱动到超过 OVT_LEAD_LONG_STILL_S
        for i in range(1, 20):
            t = 0.01 + i * 0.5
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)
            if mgr.state.state == S_WAIT:
                break

        assert mgr.state.state == S_WAIT

    def test_no_transition_with_moving_lead(self):
        """前车在移动 → 不进入 WAIT。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=2.0, lead_received=True)
        memory = _make_memory()
        lt_state = _make_lt_state()

        for i in range(20):
            t = 0.01 + i * 0.5
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)

        assert mgr.state.state == S_IDLE

    def test_no_transition_with_moving_ego(self):
        """自车在移动 → 不进入 WAIT。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=3.0, lead_v=0.0, lead_received=True)
        memory = _make_memory()
        lt_state = _make_lt_state()

        for i in range(20):
            t = 0.01 + i * 0.5
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)

        assert mgr.state.state == S_IDLE

    def test_no_transition_lead_too_far(self):
        """前车超出 OVT_TRIGGER_DIST_M → 不进入 WAIT。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0, lead_received=True)
        memory = _make_memory()
        lt_state = _make_lt_state(
            last_lead_x_rel=OVT_TRIGGER_DIST_M + 5.0,
        )

        for i in range(20):
            t = 0.01 + i * 0.5
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)

        assert mgr.state.state == S_IDLE

    def test_no_transition_lead_too_close(self):
        """前车太近（< OVT_TRIGGER_MIN_DIST_M）→ 不进入 WAIT。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0, lead_received=True)
        memory = _make_memory()
        lt_state = _make_lt_state(
            last_lead_x_rel=OVT_TRIGGER_MIN_DIST_M - 1.0,
        )

        for i in range(20):
            t = 0.01 + i * 0.5
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)

        assert mgr.state.state == S_IDLE


class TestOvertakeWaitToActive:
    """WAIT → ACTIVE：确认超车。"""

    def test_wait_to_active_on_confirm(self):
        """WAIT 停留 OVT_CONFIRM_TIME_S → 进入 ACTIVE。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0, lead_received=True)
        memory = _make_memory()
        lt_state = _make_lt_state()

        # 先进入 WAIT
        # t=0.01 初始化
        lt_state.last_confirmed_lead_t = 0.01
        mgr.update(0.01, signals, memory, lt_state)

        # 驱动到 WAIT
        for i in range(1, 20):
            t = 0.01 + i * 0.5
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)
            if mgr.state.state == S_WAIT:
                break
        assert mgr.state.state == S_WAIT
        wait_enter_t = mgr.state.state_enter_t

        # 在 WAIT 中等待 OVT_CONFIRM_TIME_S
        confirm_t = wait_enter_t + OVT_CONFIRM_TIME_S + 0.1
        lt_state.last_confirmed_lead_t = confirm_t
        mgr.update(confirm_t, signals, memory, lt_state)
        assert mgr.state.state == S_ACTIVE

    def test_wait_cancel_on_lead_resume(self):
        """WAIT 中前车恢复行驶 → 回到 IDLE。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0, lead_received=True)
        memory = _make_memory()
        lt_state = _make_lt_state()

        # 进入 WAIT
        lt_state.last_confirmed_lead_t = 0.01
        mgr.update(0.01, signals, memory, lt_state)
        for i in range(1, 20):
            t = 0.01 + i * 0.5
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)
            if mgr.state.state == S_WAIT:
                break
        assert mgr.state.state == S_WAIT

        # 前车恢复
        moving_signals = _make_signals(ego_v=0.0, lead_v=OVT_RESUME_LEAD_V + 0.5,
                                       lead_received=True)
        cancel_t = mgr.state.state_enter_t + 0.1
        lt_state.last_confirmed_lead_t = cancel_t
        mgr.update(cancel_t, moving_signals, memory, lt_state)
        assert mgr.state.state == S_IDLE

    def test_wait_cancel_on_ego_resume(self):
        """WAIT 中自车开始运动 → 回到 IDLE。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0, lead_received=True)
        memory = _make_memory()
        lt_state = _make_lt_state()

        # 进入 WAIT
        lt_state.last_confirmed_lead_t = 0.01
        mgr.update(0.01, signals, memory, lt_state)
        for i in range(1, 20):
            t = 0.01 + i * 0.5
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)
            if mgr.state.state == S_WAIT:
                break
        assert mgr.state.state == S_WAIT

        # 自车恢复
        moving_signals = _make_signals(ego_v=2.0, lead_v=0.0, lead_received=True)
        cancel_t = mgr.state.state_enter_t + 0.1
        lt_state.last_confirmed_lead_t = cancel_t
        mgr.update(cancel_t, moving_signals, memory, lt_state)
        assert mgr.state.state == S_IDLE

    def test_active_records_positions(self):
        """进入 ACTIVE 时记录 ego/lead 位置。"""
        mgr = OvertakeManager()
        signals = _make_signals(
            ego_x=10.0, ego_y=5.0, ego_v=0.0,
            lead_x=25.0, lead_y=5.0, lead_yaw=0.1, lead_v=0.0,
            lead_received=True,
        )
        memory = _make_memory()
        lt_state = _make_lt_state()

        # 直接设为 WAIT 状态
        mgr.state.state = S_WAIT
        mgr.state.state_enter_t = 0.0

        # 等待确认
        lt_state.last_confirmed_lead_t = OVT_CONFIRM_TIME_S + 0.1
        mgr.update(OVT_CONFIRM_TIME_S + 0.1, signals, memory, lt_state)
        assert mgr.state.state == S_ACTIVE
        assert mgr.state.ego_x_at_active == 10.0
        assert mgr.state.ego_y_at_active == 5.0
        assert mgr.state.lead_x_at_active == 25.0
        assert mgr.state.lead_y_at_active == 5.0
        assert mgr.state.lead_yaw_at_active == 0.1


class TestOvertakeActiveTimeout:
    """ACTIVE 超时 25s → PASSING。"""

    def test_active_timeout_to_passing(self):
        """ACTIVE 超过 25s → 进入 PASSING。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0)
        memory = _make_memory(filtered_cte=0.0)
        lt_state = _make_lt_state()

        # 直接设为 ACTIVE
        mgr.state.state = S_ACTIVE
        mgr.state.state_enter_t = 10.0
        mgr.state.lead_x_at_active = 15.0
        mgr.state.lead_y_at_active = 0.0
        mgr.state.lead_yaw_at_active = 0.0

        # 超时前 → 保持 ACTIVE
        lt_state.last_confirmed_lead_t = 34.0
        mgr.update(34.0, signals, memory, lt_state)
        assert mgr.state.state == S_ACTIVE

        # 超时 → PASSING
        lt_state.last_confirmed_lead_t = 35.1
        mgr.update(35.1, signals, memory, lt_state)
        assert mgr.state.state == S_PASSING

    def test_active_not_timeout_prematurely(self):
        """ACTIVE 未满 25s → 不进入 PASSING。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0)
        memory = _make_memory(filtered_cte=0.0)
        lt_state = _make_lt_state()

        mgr.state.state = S_ACTIVE
        mgr.state.state_enter_t = 10.0

        lt_state.last_confirmed_lead_t = 34.9
        mgr.update(34.9, signals, memory, lt_state)
        assert mgr.state.state == S_ACTIVE


class TestOvertakeActiveToPassingOffset:
    """ACTIVE → PASSING：横向偏移达到阈值。"""

    def test_active_to_passing_on_offset(self):
        """横向偏移 >= OVT_SHIFT_DONE_M → 进入 PASSING。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0)
        memory = _make_memory(filtered_cte=OVT_SHIFT_DONE_M + 0.01)
        lt_state = _make_lt_state()

        mgr.state.state = S_ACTIVE
        mgr.state.state_enter_t = 10.0

        lt_state.last_confirmed_lead_t = 11.0
        mgr.update(11.0, signals, memory, lt_state)
        assert mgr.state.state == S_PASSING

    def test_active_suppresses_lead(self):
        """ACTIVE 状态 → suppress_lead_for_overtake=True。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0)
        memory = _make_memory(filtered_cte=0.0)
        lt_state = _make_lt_state()

        mgr.state.state = S_ACTIVE
        mgr.state.state_enter_t = 10.0

        lt_state.last_confirmed_lead_t = 11.0
        mgr.update(11.0, signals, memory, lt_state)
        assert memory.suppress_lead_for_overtake

    def test_passing_suppresses_lead(self):
        """PASSING 状态 → suppress_lead_for_overtake=True。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0)
        memory = _make_memory(filtered_cte=OVT_SHIFT_DONE_M + 0.01)
        lt_state = _make_lt_state()

        mgr.state.state = S_PASSING
        mgr.state.state_enter_t = 20.0
        mgr.state.lead_x_at_active = 15.0
        mgr.state.lead_y_at_active = 0.0
        mgr.state.lead_yaw_at_active = 0.0

        lt_state.last_confirmed_lead_t = 21.0
        mgr.update(21.0, signals, memory, lt_state)
        assert memory.suppress_lead_for_overtake

    def test_target_lane_offset_ramps_up(self):
        """target_lane_offset 以 OVT_LANE_SHIFT_RATE_M_S 爬升。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0)
        memory = _make_memory(filtered_cte=0.0, target_lane_offset=0.0)
        lt_state = _make_lt_state()

        mgr.state.state = S_ACTIVE
        mgr.state.state_enter_t = 10.0

        dt = 0.01
        lt_state.last_confirmed_lead_t = 10.01
        mgr.update(10.01, signals, memory, lt_state)

        # 一个周期的增量
        expected_delta = OVT_LANE_SHIFT_RATE_M_S * dt
        assert memory.target_lane_offset == pytest.approx(expected_delta, abs=1e-6)

    def test_target_offset_capped_at_lane_offset(self):
        """target_lane_offset 不超过 OVT_LANE_OFFSET_M。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0)
        memory = _make_memory(
            filtered_cte=0.0,
            target_lane_offset=OVT_LANE_OFFSET_M - 0.001,
        )
        lt_state = _make_lt_state()

        mgr.state.state = S_ACTIVE
        mgr.state.state_enter_t = 10.0

        lt_state.last_confirmed_lead_t = 10.01
        mgr.update(10.01, signals, memory, lt_state)
        assert memory.target_lane_offset <= OVT_LANE_OFFSET_M


class TestOvertakePassingToReturn:
    """PASSING → RETURN：超过前车。"""

    def test_passed_lead_enters_return(self):
        """纵向超过前车 OVT_LEAD_PASSED_FWD_M → 进入 RETURN。"""
        mgr = OvertakeManager()
        # lead_yaw=0，ego 在 lead 前方 > OVT_LEAD_PASSED_FWD_M
        lead_x = 10.0
        lead_y = 0.0
        ego_x = lead_x + OVT_LEAD_PASSED_FWD_M + 1.0  # 15m 前方

        signals = _make_signals(
            ego_x=ego_x, ego_y=0.0, ego_v=5.0,
            lead_x=100.0, lead_y=100.0,  # 当前 lead 位置无关
            lead_v=0.0, lead_yaw=0.0,
        )
        memory = _make_memory(filtered_cte=OVT_SHIFT_DONE_M + 0.1)
        lt_state = _make_lt_state()

        # 直接设为 PASSING，记录 ACTIVE 时的位置
        mgr.state.state = S_PASSING
        mgr.state.state_enter_t = 20.0
        mgr.state.lead_x_at_active = lead_x
        mgr.state.lead_y_at_active = lead_y
        mgr.state.lead_yaw_at_active = 0.0

        lt_state.last_confirmed_lead_t = 21.0
        mgr.update(21.0, signals, memory, lt_state)
        assert mgr.state.state == S_RETURN

    def test_not_passed_stays_passing(self):
        """未超过前车 → 保持 PASSING。"""
        mgr = OvertakeManager()
        lead_x = 10.0
        ego_x = lead_x + OVT_LEAD_PASSED_FWD_M - 5.0  # 未超过

        signals = _make_signals(
            ego_x=ego_x, ego_y=0.0, ego_v=5.0,
            lead_x=100.0, lead_y=100.0,
            lead_v=0.0, lead_yaw=0.0,
        )
        memory = _make_memory(filtered_cte=OVT_SHIFT_DONE_M + 0.1)
        lt_state = _make_lt_state()

        mgr.state.state = S_PASSING
        mgr.state.state_enter_t = 20.0
        mgr.state.lead_x_at_active = lead_x
        mgr.state.lead_y_at_active = 0.0
        mgr.state.lead_yaw_at_active = 0.0

        lt_state.last_confirmed_lead_t = 21.0
        mgr.update(21.0, signals, memory, lt_state)
        assert mgr.state.state == S_PASSING

    def test_return_offset_ramps_down(self):
        """RETURN 状态 → target_lane_offset 向 0 爬升。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0)
        memory = _make_memory(
            filtered_cte=0.0,
            target_lane_offset=OVT_LANE_OFFSET_M,
        )
        lt_state = _make_lt_state()

        mgr.state.state = S_RETURN
        mgr.state.state_enter_t = 30.0

        lt_state.last_confirmed_lead_t = 30.01
        mgr.update(30.01, signals, memory, lt_state)

        # RETURN 的 target=0，offset 应减少
        assert memory.target_lane_offset < OVT_LANE_OFFSET_M

    def test_return_to_idle_on_offset_zero(self):
        """RETURN 且横向偏移 <= OVT_RETURN_DONE_M → IDLE。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0)
        memory = _make_memory(
            filtered_cte=OVT_RETURN_DONE_M - 0.01,
            target_lane_offset=0.0,
        )
        lt_state = _make_lt_state()

        mgr.state.state = S_RETURN
        mgr.state.state_enter_t = 30.0

        lt_state.last_confirmed_lead_t = 30.01
        mgr.update(30.01, signals, memory, lt_state)
        assert mgr.state.state == S_IDLE


class TestOvertakeIntegration:
    """端到端场景。"""

    def test_full_overtake_lifecycle(self):
        """完整超车流程：IDLE → WAIT → ACTIVE → PASSING → RETURN → IDLE。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0, lead_received=True)
        memory = _make_memory()
        lt_state = _make_lt_state(last_lead_x_rel=15.0)

        # ── IDLE → WAIT ──
        t = 0.01
        lt_state.last_confirmed_lead_t = t
        mgr.update(t, signals, memory, lt_state)

        # 驱动静止阶段
        while t < OVT_LEAD_LONG_STILL_S + 1.0:
            t += 0.5
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)
            if mgr.state.state == S_WAIT:
                break
        assert mgr.state.state == S_WAIT

        # ── WAIT → ACTIVE ──
        confirm_t = mgr.state.state_enter_t + OVT_CONFIRM_TIME_S + 0.1
        lt_state.last_confirmed_lead_t = confirm_t
        mgr.update(confirm_t, signals, memory, lt_state)
        assert mgr.state.state == S_ACTIVE

        # ── ACTIVE → PASSING (通过偏移) ──
        # 逐步增加 filtered_cte
        active_enter_t = mgr.state.state_enter_t
        t = active_enter_t + 0.1
        while t < active_enter_t + 30.0:
            # 模拟横向偏移逐渐增大
            progress = (t - active_enter_t) / 5.0  # 5s 内完成
            memory.filtered_cte = min(progress * OVT_LANE_OFFSET_M,
                                      OVT_SHIFT_DONE_M + 0.1)
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)
            if mgr.state.state == S_PASSING:
                break
            t += 0.1
        assert mgr.state.state == S_PASSING

        # ── PASSING → RETURN ──
        # 自车纵向超过前车
        passing_signals = _make_signals(
            ego_x=mgr.state.lead_x_at_active + OVT_LEAD_PASSED_FWD_M + 1.0,
            ego_y=0.0,
            ego_v=5.0,
            lead_v=0.0,
        )
        t += 0.1
        lt_state.last_confirmed_lead_t = t
        mgr.update(t, passing_signals, memory, lt_state)
        assert mgr.state.state == S_RETURN

        # ── RETURN → IDLE ──
        # 横向偏移归零
        memory.filtered_cte = 0.0
        t += 0.1
        lt_state.last_confirmed_lead_t = t
        idle_signals = _make_signals(ego_v=0.0, lead_v=0.0)
        mgr.update(t, idle_signals, memory, lt_state)
        assert mgr.state.state == S_IDLE

    def test_state_persists_in_active(self):
        """ACTIVE 持续不转移 → 保持 ACTIVE。"""
        mgr = OvertakeManager()
        signals = _make_signals(ego_v=0.0, lead_v=0.0)
        memory = _make_memory(filtered_cte=0.0)
        lt_state = _make_lt_state()

        mgr.state.state = S_ACTIVE
        mgr.state.state_enter_t = 10.0

        # 多个周期保持 ACTIVE（未超时、未达到偏移）
        for i in range(10):
            t = 10.1 + i * 0.5
            lt_state.last_confirmed_lead_t = t
            mgr.update(t, signals, memory, lt_state)
        assert mgr.state.state == S_ACTIVE
