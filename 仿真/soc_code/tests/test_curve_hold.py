#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CurveHoldManager 单元测试。

测试弯道保持状态机的核心逻辑：
  1. 弯道内 + 前车丢失超过 CURVE_HOLD_ACTIVATE_LOSS_S → 激活
  2. 曲率降低（直道）→ 退出
  3. 超时（CURVE_HOLD_TIMEOUT_S）→ 退出
  4. 前车重新获取并稳定 CURVE_HOLD_REACQ_STABLE_S → 退出
"""

import pytest

from config import (
    CURVE_HOLD_ACTIVATE_LOSS_S,
    CURVE_HOLD_CURV_THRESH,
    CURVE_HOLD_EXIT_CURV,
    CURVE_HOLD_REACQ_STABLE_S,
    CURVE_HOLD_TIMEOUT_S,
)
from control.curve_hold import CurveHoldManager


# ---------------------------------------------------------------------------
# 辅助常量
# ---------------------------------------------------------------------------
# 确保曲率处于"弯道"区间
CURVE_CURV = CURVE_HOLD_CURV_THRESH + 0.01
STRAIGHT_CURV = CURVE_HOLD_EXIT_CURV - 0.001  # 低于退出阈值


# ===========================================================================
# 测试
# ===========================================================================

class TestCurveHoldActivation:
    """弯道保持激活条件。"""

    def test_activates_after_loss_duration(self):
        """弯道内前车丢失超过 CURVE_HOLD_ACTIVATE_LOSS_S → active=True。"""
        mgr = CurveHoldManager()

        # t=0: 弯道内，有前车
        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert not mgr.state.active

        # t=0.01: 前车丢失（raw_has_lead=False），仍在弯道
        # loss_since 将被设为 0.01（prev_raw_has_lead=True → False 的边沿）
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert not mgr.state.active  # 还没超过阈值

        # t=0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + margin: 应激活
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.active

    def test_records_v_target_on_activation(self):
        """激活时记录目标速度 v_target = ego_v。"""
        mgr = CurveHoldManager()
        # 触发边沿
        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        # 激活
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=12.5)
        assert mgr.state.v_target == 12.5

    def test_resets_v_i_on_activation(self):
        """激活时清零积分项。"""
        mgr = CurveHoldManager()
        mgr.state.v_i = 5.0  # 模拟残留

        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.v_i == 0.0

    def test_no_activation_on_straight(self):
        """直道上不激活。"""
        mgr = CurveHoldManager()
        straight_curv = CURVE_HOLD_CURV_THRESH - 0.001

        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=straight_curv, ego_v=10.0)
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=straight_curv, ego_v=10.0)
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.5
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=straight_curv, ego_v=10.0)
        assert not mgr.state.active

    def test_no_activation_with_lead_present(self):
        """有前车时不激活（即使在弯道）。"""
        mgr = CurveHoldManager()
        for i in range(100):
            t = i * 0.01
            mgr.update(t, has_lead=True, raw_has_lead=True,
                       filtered_curv=CURVE_CURV, ego_v=10.0)
        assert not mgr.state.active


class TestCurveHoldExitStraight:
    """退出条件 1：曲率降低到直道水平。"""

    def test_exits_on_straight_road(self):
        """曲率降至 CURVE_HOLD_EXIT_CURV 以下 → active=False。"""
        mgr = CurveHoldManager()
        # 激活弯道保持
        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.active

        # 曲率降低
        exit_t = activate_t + 1.0
        result = mgr.update(exit_t, has_lead=False, raw_has_lead=False,
                            filtered_curv=STRAIGHT_CURV, ego_v=10.0)
        assert not mgr.state.active
        assert not result

    def test_resets_v_i_on_straight_exit(self):
        """直道退出时清零积分项。"""
        mgr = CurveHoldManager()
        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.state.v_i = 3.0

        exit_t = activate_t + 1.0
        mgr.update(exit_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=STRAIGHT_CURV, ego_v=10.0)
        assert mgr.state.v_i == 0.0


class TestCurveHoldExitTimeout:
    """退出条件 2：超时。"""

    def test_exits_on_timeout(self):
        """保持超过 CURVE_HOLD_TIMEOUT_S → active=False。"""
        mgr = CurveHoldManager()
        # 激活
        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.active

        # 超时
        timeout_t = activate_t + CURVE_HOLD_TIMEOUT_S + 0.1
        result = mgr.update(timeout_t, has_lead=False, raw_has_lead=False,
                            filtered_curv=CURVE_CURV, ego_v=10.0)
        assert not mgr.state.active
        assert not result

    def test_no_timeout_before_limit(self):
        """保持期内不因超时退出。"""
        mgr = CurveHoldManager()
        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)

        # 保持期内
        mid_t = activate_t + CURVE_HOLD_TIMEOUT_S * 0.5
        result = mgr.update(mid_t, has_lead=False, raw_has_lead=False,
                            filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.active
        assert result


class TestCurveHoldExitReacquire:
    """退出条件 3：前车重新获取并稳定。"""

    def test_exits_on_lead_reacquired_stable(self):
        """前车重新获取并稳定超过 CURVE_HOLD_REACQ_STABLE_S → 退出。"""
        mgr = CurveHoldManager()
        # 激活
        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.active

        # 前车重新获取
        reacq_t = activate_t + 1.0
        mgr.update(reacq_t, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.active  # 还没稳定够

        # 稳定超过阈值
        stable_t = reacq_t + CURVE_HOLD_REACQ_STABLE_S + 0.01
        result = mgr.update(stable_t, has_lead=True, raw_has_lead=True,
                            filtered_curv=CURVE_CURV, ego_v=10.0)
        assert not mgr.state.active
        assert not result

    def test_reacquire_resets_on_brief_loss(self):
        """重获过程中前车再次短暂丢失 → 重置 reacq 计时。"""
        mgr = CurveHoldManager()
        # 激活
        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)

        # 前车重获
        reacq_t = activate_t + 1.0
        mgr.update(reacq_t, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)

        # 重获过程中前车再次丢失 → reacq_since 重置
        mgr.update(reacq_t + 0.05, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.reacq_since == -1e9

    def test_reacquire_timeout_not_reached_stays_active(self):
        """前车重获但未稳定够 → 仍保持激活。"""
        mgr = CurveHoldManager()
        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)

        reacq_t = activate_t + 1.0
        # 刚重获，未稳定够
        just_before = reacq_t + CURVE_HOLD_REACQ_STABLE_S - 0.01
        result = mgr.update(just_before, has_lead=True, raw_has_lead=True,
                            filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.active
        assert result


class TestCurveHoldIntegration:
    """端到端场景。"""

    def test_full_activation_to_timeout(self):
        """完整生命周期：弯道丢失 → 激活 → 超时退出。"""
        mgr = CurveHoldManager()

        # 弯道内有前车
        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)

        # 前车丢失
        mgr.update(0.1, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)

        # 激活
        mgr.update(0.5, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.active

        # 超时退出
        mgr.update(0.5 + CURVE_HOLD_TIMEOUT_S + 0.1,
                   has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert not mgr.state.active

    def test_full_activation_to_reacquire(self):
        """完整生命周期：弯道丢失 → 激活 → 前车重获 → 退出。"""
        mgr = CurveHoldManager()

        mgr.update(0.0, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        mgr.update(0.1, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        activate_t = 0.1 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.active

        # 前车重获：第一次调用记录 reacq_since
        reacq_t = activate_t + 2.0
        mgr.update(reacq_t, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert mgr.state.active  # 刚重获，尚未稳定

        # 第二次调用：稳定超过阈值 → 退出
        stable_t = reacq_t + CURVE_HOLD_REACQ_STABLE_S + 0.01
        mgr.update(stable_t, has_lead=True, raw_has_lead=True,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        assert not mgr.state.active

    def test_update_returns_active_state(self):
        """update() 返回值 = state.active。"""
        mgr = CurveHoldManager()
        ret = mgr.update(0.0, has_lead=True, raw_has_lead=True,
                         filtered_curv=CURVE_CURV, ego_v=10.0)
        assert ret == mgr.state.active
        assert ret is False

        # 激活
        mgr.update(0.01, has_lead=False, raw_has_lead=False,
                   filtered_curv=CURVE_CURV, ego_v=10.0)
        activate_t = 0.01 + CURVE_HOLD_ACTIVATE_LOSS_S + 0.01
        ret = mgr.update(activate_t, has_lead=False, raw_has_lead=False,
                         filtered_curv=CURVE_CURV, ego_v=10.0)
        assert ret == mgr.state.active
        assert ret is True
