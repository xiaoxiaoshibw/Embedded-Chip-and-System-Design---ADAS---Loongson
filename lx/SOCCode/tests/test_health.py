#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""health.py 单元测试。

覆盖 evaluate_control_health 的各种分支：
所有信号正常、ego 卡帧、ego 未接收、road 卡帧、lead_cls 超时控制等。
"""

import pytest

from control.health import ControlHealth, evaluate_control_health


def _call(**kwargs):
    """便捷封装：填充默认参数，调用方可只写关心的字段。"""
    defaults = dict(
        peer_active=True,
        now=100.0,
        ego_received=True,
        ego_last_rx=99.9,
        road_received=True,
        road_last_rx=99.9,
        lead_ready=True,
        lane_offset_ready=True,
        stale_timeout_s=0.2,
        lead_cls_last_rx=-1e9,
        lead_cls_stale_timeout_s=0.0,
    )
    defaults.update(kwargs)
    return evaluate_control_health(**defaults)


# ── 基本 control_active 逻辑 ─────────────────────────────────────────────────


class TestControlActive:
    def test_all_fresh(self):
        """所有信号新鲜 → control_active=True。"""
        h = _call()
        assert h.control_active is True
        assert h.peer_active is True
        assert h.ego_ready is True
        assert h.road_ready is True
        assert h.ego_stale is False
        assert h.road_stale is False

    def test_peer_inactive(self):
        """peer_active=False → control_active=False。"""
        h = _call(peer_active=False)
        assert h.control_active is False

    def test_ego_stale_disables_control(self):
        """ego 已接收但超时 → ego_stale=True, control_active=False。"""
        h = _call(now=100.0, ego_last_rx=98.0, stale_timeout_s=0.2)
        assert h.ego_stale is True
        assert h.ego_ready is False
        assert h.control_active is False

    def test_ego_not_received_not_stale(self):
        """ego 从未接收 → ego_stale=False（只标记 ready=False）。"""
        h = _call(ego_received=False, ego_last_rx=-1e9)
        assert h.ego_stale is False
        assert h.ego_ready is False
        assert h.control_active is False

    def test_road_stale_disables_control(self):
        """road 已接收但超时 → road_stale=True, control_active=False。"""
        h = _call(now=100.0, road_last_rx=98.0, stale_timeout_s=0.2)
        assert h.road_stale is True
        assert h.road_ready is False
        assert h.control_active is False

    def test_road_not_received(self):
        """road 从未接收 → road_ready=False, road_stale=False。"""
        h = _call(road_received=False, road_last_rx=-1e9)
        assert h.road_stale is False
        assert h.road_ready is False
        assert h.control_active is False


# ── stale 边界条件 ───────────────────────────────────────────────────────────


class TestStaleBoundary:
    def test_exactly_at_timeout_not_stale(self):
        """now - last_rx == stale_timeout_s → 不算超时（严格大于才判定）。

        用整数避免浮点精度问题：2000 - 1999 == 1.0，stale_timeout_s=1.0。
        """
        h = _call(now=2000.0, ego_last_rx=1999.0, stale_timeout_s=1.0)
        assert h.ego_stale is False

    def test_just_over_timeout_is_stale(self):
        """now - last_rx > stale_timeout_s → stale。"""
        h = _call(now=100.0, ego_last_rx=99.799, stale_timeout_s=0.2)
        assert h.ego_stale is True

    def test_very_small_timeout(self):
        """stale_timeout_s 很小时几乎立刻判定 stale。"""
        h = _call(now=100.0, ego_last_rx=99.99, stale_timeout_s=0.001)
        assert h.ego_stale is True


# ── lead_cls 超时 ────────────────────────────────────────────────────────────


class TestLeadClsStale:
    def test_timeout_zero_skips_check(self):
        """lead_cls_stale_timeout_s=0 → 跳过 lead_cls 陈旧检查。"""
        h = _call(
            lead_cls_last_rx=50.0,
            lead_cls_stale_timeout_s=0.0,
            now=100.0,
        )
        assert h.lead_cls_stale is False

    def test_stale_when_timeout_positive(self):
        """lead_cls_stale_timeout_s>0 且超时 → lead_cls_stale=True。"""
        h = _call(
            lead_cls_last_rx=50.0,
            lead_cls_stale_timeout_s=1.0,
            now=100.0,
        )
        assert h.lead_cls_stale is True

    def test_fresh_when_within_timeout(self):
        """lead_cls 在超时窗口内 → lead_cls_stale=False。"""
        h = _call(
            lead_cls_last_rx=99.5,
            lead_cls_stale_timeout_s=1.0,
            now=100.0,
        )
        assert h.lead_cls_stale is False

    def test_lead_not_ready_skips_stale(self):
        """lead_ready=False 时即使超时也不标记 stale（条件短路）。"""
        h = _call(
            lead_ready=False,
            lead_cls_last_rx=50.0,
            lead_cls_stale_timeout_s=1.0,
            now=100.0,
        )
        assert h.lead_cls_stale is False

    def test_lead_cls_stale_does_not_affect_control_active(self):
        """lead_cls_stale 不影响 control_active（仅用于告警/遥测）。"""
        h = _call(
            lead_cls_last_rx=50.0,
            lead_cls_stale_timeout_s=1.0,
            now=100.0,
        )
        assert h.lead_cls_stale is True
        assert h.control_active is True


# ── ControlHealth 数据类 ─────────────────────────────────────────────────────


class TestControlHealthDataclass:
    def test_frozen(self):
        h = _call()
        with pytest.raises(AttributeError):
            h.peer_active = False

    def test_default_stale_fields(self):
        """默认构造时 stale 字段为 False。"""
        h = ControlHealth(
            peer_active=True, ego_ready=True, road_ready=True,
            lead_ready=True, lane_offset_ready=True,
        )
        assert h.ego_stale is False
        assert h.road_stale is False
        assert h.lead_cls_stale is False
