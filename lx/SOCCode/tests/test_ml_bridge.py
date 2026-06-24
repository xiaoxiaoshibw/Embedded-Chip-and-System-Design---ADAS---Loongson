#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ML 推理桥接模块测试。"""

import sys
import os
import numpy as np
import pytest

# 确保 SOCCode 在 path 中
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from control.context import VehicleSignals, ControlMemory
from control.state import LeadContext
from common import is_finite


# ── 降采样逻辑测试 ──

class TestMlBridgeDownsampling:
    """MlBridge 降采样频率测试。"""

    def test_sample_interval_calculation(self):
        """降采样间隔应为 LOOP_HZ // 10。"""
        from config import LOOP_HZ
        expected = max(1, LOOP_HZ // 10)
        assert expected == 10  # 100Hz / 10 = 10

    def test_cycle_count_increments(self):
        """cycle_count 应随 update 调用递增（仅启用时）。"""
        from control.ml_bridge import MlBridge
        bridge = MlBridge()
        bridge._enabled = True
        signals = VehicleSignals()
        signals.lead_received = False  # 无前车，不触发推理但会递增计数
        lead_ctx = LeadContext()
        bridge.update(0.0, signals, lead_ctx)
        assert bridge._cycle_count == 1
        bridge.update(0.01, signals, lead_ctx)
        assert bridge._cycle_count == 2


# ── 特征构建测试 ──

class TestMlBridgeFeatureBuilding:
    """MlBridge 特征构建测试。"""

    def _make_signals(self, ego_v=15.0, lead_received=True, ego_received=True):
        signals = VehicleSignals()
        signals.ego_v = ego_v
        signals.lead_received = lead_received
        signals.ego_received = ego_received
        return signals

    def _make_lead_ctx(self, x_rel=20.0, predicted_lead_v_proj=12.0, lead_accel=0.0):
        ctx = LeadContext(
            x_rel=x_rel,
            y_rel=0.0,
            predicted_lead_v_proj=predicted_lead_v_proj,
            raw_lead_v_proj=predicted_lead_v_proj,
            has_lead=True,
            lead_detected=True,
            raw_has_lead=True,
            acc_has_lead=True,
            acc_lead_valid=True,
            lead_fresh=True,
            lead_cls=1,
            acc_ttc=x_rel / max(15.0 - predicted_lead_v_proj, 0.1),
        )
        # ml_bridge 使用 getattr(lead_ctx, 'lead_accel', 0.0) 访问，
        # LeadContext 无此字段时自动回退到 0.0
        if lead_accel != 0.0:
            ctx.lead_accel = lead_accel
        return ctx

    def test_acc_features_no_lead(self):
        """无前车时 ACC 特征应返回 None。"""
        from control.ml_bridge import MlBridge
        signals = self._make_signals(lead_received=False)
        lead_ctx = self._make_lead_ctx()
        result = MlBridge._build_acc_features(signals, lead_ctx)
        assert result is None

    def test_acc_features_no_ego(self):
        """无自车数据时 ACC 特征应返回 None。"""
        from control.ml_bridge import MlBridge
        signals = self._make_signals(ego_received=False)
        lead_ctx = self._make_lead_ctx()
        result = MlBridge._build_acc_features(signals, lead_ctx)
        assert result is None

    def test_acc_features_negative_ego_v(self):
        """负速度时 ACC 特征应返回 None。"""
        from control.ml_bridge import MlBridge
        signals = self._make_signals(ego_v=-1.0)
        lead_ctx = self._make_lead_ctx()
        result = MlBridge._build_acc_features(signals, lead_ctx)
        assert result is None

    def test_acc_features_zero_gap(self):
        """距离 <= 0 时 ACC 特征应返回 None。"""
        from control.ml_bridge import MlBridge
        signals = self._make_signals()
        lead_ctx = self._make_lead_ctx(x_rel=0.0)
        result = MlBridge._build_acc_features(signals, lead_ctx)
        assert result is None

    def test_acc_features_valid(self):
        """正常输入时 ACC 特征应返回 7 维元组。"""
        from control.ml_bridge import MlBridge
        signals = self._make_signals()
        lead_ctx = self._make_lead_ctx()
        result = MlBridge._build_acc_features(signals, lead_ctx)
        assert result is not None
        assert len(result) == 7
        gap, v_ego, v_lead, rel_speed, acc_ego, acc_lead, thw = result
        assert gap == 20.0
        assert v_ego == 15.0
        assert v_lead == 12.0
        assert rel_speed == v_lead - v_ego
        assert acc_ego == 0.0  # SOC 无直接 ego_acc
        assert thw > 0

    def test_aeb_features_valid(self):
        """正常输入时 AEB 特征应返回 10 维元组。"""
        from control.ml_bridge import MlBridge
        signals = self._make_signals()
        lead_ctx = self._make_lead_ctx()
        result = MlBridge._build_aeb_features(signals, lead_ctx)
        assert result is not None
        assert len(result) == 10
        gap, v_ego, v_lead, rel_speed, ttc, inv_ttc, drac, thw, cs, acc_lead = result
        assert gap == 20.0
        assert ttc > 0
        assert inv_ttc >= 0
        assert drac >= 0

    def test_aeb_features_no_closing(self):
        """不接近时 TTC 应为 100，DRAC 应为 0。"""
        from control.ml_bridge import MlBridge
        signals = self._make_signals(ego_v=10.0)
        lead_ctx = self._make_lead_ctx(x_rel=20.0, predicted_lead_v_proj=15.0)
        result = MlBridge._build_aeb_features(signals, lead_ctx)
        assert result is not None
        _, _, _, _, ttc, inv_ttc, drac, _, _, _ = result
        assert ttc == 100.0
        assert inv_ttc == 0.0
        assert drac == 0.0


# ── 降级行为测试 ──

class TestMlBridgeFallback:
    """ML 不可用时的降级行为测试。"""

    def test_disabled_bridge_returns_none(self):
        """禁用时 update 应返回 None。"""
        from control.ml_bridge import MlBridge
        bridge = MlBridge()
        bridge._enabled = False
        signals = VehicleSignals()
        lead_ctx = LeadContext()
        result = bridge.update(0.0, signals, lead_ctx)
        assert result is None

    def test_reset_clears_state(self):
        """reset 应清空计数器和缓存结果。"""
        from control.ml_bridge import MlBridge
        bridge = MlBridge()
        bridge._cycle_count = 50
        bridge._last_result = type('MockResult', (), {'acc_pred': 1.0})()
        bridge.reset()
        assert bridge._cycle_count == 0

    def test_non_sample_cycle_returns_cached(self):
        """非采样周期应返回缓存的上一次结果。"""
        from control.ml_bridge import MlBridge, MlPrediction, _ML_SAMPLE_INTERVAL
        bridge = MlBridge()
        bridge._enabled = True
        cached = MlPrediction()
        cached.acc_pred = 42.0
        bridge._last_result = cached
        bridge._cycle_count = 1  # 非采样周期
        signals = VehicleSignals()
        lead_ctx = LeadContext()
        result = bridge.update(0.0, signals, lead_ctx)
        assert result is cached
        assert result.acc_pred == 42.0


# ── MlPrediction 数据类测试 ──

class TestMlPrediction:
    """MlPrediction 数据类测试。"""

    def test_default_values(self):
        """默认值应为安全状态。"""
        from control.ml_bridge import MlPrediction
        pred = MlPrediction()
        assert pred.acc_pred == 0.0
        assert pred.aeb_class == 0
        assert pred.aeb_probs is None
        assert pred.should_brake is False
        assert pred.brake_intensity == 0.0
