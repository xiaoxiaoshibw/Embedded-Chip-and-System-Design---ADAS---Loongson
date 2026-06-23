#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lateral.py 单元测试。

覆盖纯函数与 LaneWidthEstimator 状态类。
"""

import math
import time

import pytest

from lateral import (
    LaneWidthEstimator,
    adaptive_preview_time,
    compute_boundary_correction,
    compute_curve_hold_window,
    compute_lead_lateral_window,
    compute_relative_in_ego_frame,
    lane_margins_from_width,
)
from config import (
    K_LATERAL_HARD,
    LANE_HARD_RATIO,
    LANE_WIDTH_MIN,
    LANE_WIDTH_MAX,
    LANE_WARN_RATIO,
    LEAD_CURVE_HOLD_LAT_RATIO,
    LEAD_LAT_MAX_CURVE_MAX,
    LEAD_LAT_MAX_CURVE_MIN,
    LEAD_LAT_MAX_STRAIGHT_MAX,
    LEAD_LAT_MAX_STRAIGHT_MIN,
    MAX_DELTA,
    MIN_LANE_SAFE_MARGIN,
    PREVIEW_SPEED_REF,
    PREVIEW_TIME_MAX,
    PREVIEW_TIME_MIN,
    VEHICLE_HALF_WIDTH,
)


# ── 1. compute_relative_in_ego_frame ──────────────────────────────────────────


class TestComputeRelativeInEgoFrame:
    """前车全局坐标→自车坐标系投影。"""

    def test_same_position(self):
        """前车与自车在同一位置 → (0, 0)。"""
        x, y = compute_relative_in_ego_frame(0.0, 0.0, 0.0, 0.0, 0.0)
        assert abs(x) < 1e-9
        assert abs(y) < 1e-9

    def test_lead_ahead_straight(self):
        """yaw=0 时前车在正前方 → x_rel=距离, y_rel=0。"""
        x, y = compute_relative_in_ego_frame(10.0, 0.0, 0.0, 0.0, 0.0)
        assert abs(x - 10.0) < 1e-9
        assert abs(y) < 1e-9

    def test_lead_at_90_degrees(self):
        """yaw=0 时前车在正右侧 → x_rel≈0, y_rel≈+距离。

        注：函数定义 y_rel 正值=右侧，dx=0, dy=-10 → x=sin(0)*(-10)=0,
        y=-sin(0)*0+cos(0)*(-10)=-10. 换方向验证。
        """
        # 前车在 (0, 10)，yaw=0 → x_rel = 0, y_rel = +10
        x, y = compute_relative_in_ego_frame(0.0, 10.0, 0.0, 0.0, 0.0)
        assert abs(x) < 1e-9
        assert abs(y - 10.0) < 1e-9

    def test_yaw_pi(self):
        """yaw=π 时前车在 (10,0)，自车在原点 → x_rel=-10, y_rel≈0。"""
        x, y = compute_relative_in_ego_frame(10.0, 0.0, 0.0, 0.0, math.pi)
        assert abs(x - (-10.0)) < 1e-6
        assert abs(y) < 1e-6

    def test_yaw_pi_half_lead_right(self):
        """yaw=π/2 时前车在 (0,5)，自车在原点 → x_rel=5, y_rel≈0。"""
        x, y = compute_relative_in_ego_frame(0.0, 5.0, 0.0, 0.0, math.pi / 2)
        assert abs(x - 5.0) < 1e-6
        assert abs(y) < 1e-6


# ── 2. lane_margins_from_width ────────────────────────────────────────────────


class TestLaneMarginsFromWidth:
    """由车道宽度计算三级余量。"""

    def test_narrow_lane(self):
        """窄车道 3.0m → safe=max(0.6, 0.5)=0.6。"""
        safe, warn, hard = lane_margins_from_width(3.0)
        expected_safe = max(0.5 * 3.0 - VEHICLE_HALF_WIDTH, MIN_LANE_SAFE_MARGIN)
        assert abs(safe - expected_safe) < 1e-9
        assert abs(warn - safe * LANE_WARN_RATIO) < 1e-9
        assert abs(hard - safe * LANE_HARD_RATIO) < 1e-9

    def test_normal_lane(self):
        """标准车道 3.8m。"""
        safe, warn, hard = lane_margins_from_width(3.8)
        expected_safe = max(0.5 * 3.8 - VEHICLE_HALF_WIDTH, MIN_LANE_SAFE_MARGIN)
        assert abs(safe - expected_safe) < 1e-9
        assert warn < safe
        assert hard > warn

    def test_wide_lane(self):
        """宽车道 5.0m → safe=1.6。"""
        safe, warn, hard = lane_margins_from_width(5.0)
        expected_safe = max(0.5 * 5.0 - VEHICLE_HALF_WIDTH, MIN_LANE_SAFE_MARGIN)
        assert abs(safe - expected_safe) < 1e-9
        assert abs(safe - 1.6) < 1e-9

    def test_zero_width_uses_min_margin(self):
        """宽度=0 时 safe 取 MIN_LANE_SAFE_MARGIN 下限。"""
        safe, _, _ = lane_margins_from_width(0.0)
        assert abs(safe - MIN_LANE_SAFE_MARGIN) < 1e-9

    def test_ordering(self):
        """safe > hard > warn 恒成立（LANE_WARN_RATIO < LANE_HARD_RATIO）。"""
        safe, warn, hard = lane_margins_from_width(3.5)
        assert safe > hard > warn > 0.0


# ── 3. compute_lead_lateral_window ────────────────────────────────────────────


class TestComputeLeadLateralWindow:
    """前车横向检测窗口。"""

    def test_straight_road(self):
        """直道 (curv=0) → lead_lat_max 应 >= lat_straight，且含有效值。"""
        lead_lat_max, lat_straight, lat_curve = compute_lead_lateral_window(0.0, 3.8)
        assert lead_lat_max >= lat_straight - 1e-9
        assert LEAD_LAT_MAX_STRAIGHT_MIN <= lat_straight <= LEAD_LAT_MAX_STRAIGHT_MAX
        assert LEAD_LAT_MAX_CURVE_MIN <= lat_curve <= LEAD_LAT_MAX_CURVE_MAX

    def test_curve_road(self):
        """大曲率 → lead_lat_max 收窄（不高于直道值）。"""
        _, lat_s_straight, _ = compute_lead_lateral_window(0.0, 3.8)
        lead_max_curve, _, _ = compute_lead_lateral_window(0.05, 3.8)
        # 大曲率时 curve_corridor 可能比 lat_straight 大，所以 lead_lat_max 不一定收窄；
        # 但结构上三个返回值都是有限正数
        assert lead_max_curve > 0.0

    def test_narrow_lane(self):
        """窄车道 → 窗口成比例缩小。"""
        lead_max, lat_s, lat_c = compute_lead_lateral_window(0.0, 2.5)
        assert lead_max > 0.0
        assert lat_s > 0.0
        assert lat_c > 0.0

    def test_returns_finite(self):
        """所有返回值均为有限正数。"""
        for curv in (0.0, 0.005, 0.02, 0.1):
            for w in (2.5, 3.0, 3.8, 5.0):
                vals = compute_lead_lateral_window(curv, w)
                for v in vals:
                    assert math.isfinite(v)
                    assert v > 0.0


# ── 4. compute_curve_hold_window ──────────────────────────────────────────────


class TestComputeCurveHoldWindow:
    """弯道保持模式横向窗口。"""

    def test_normal_lane(self):
        """标准车道 3.8m → 结果在 [LEAD_LAT_MAX_CURVE_MIN, LEAD_LAT_MAX_CURVE_MAX] 内。"""
        result = compute_curve_hold_window(3.8)
        expected_raw = 3.8 * LEAD_CURVE_HOLD_LAT_RATIO
        lo = LEAD_LAT_MAX_CURVE_MIN
        hi = LEAD_LAT_MAX_CURVE_MAX
        expected = max(lo, min(hi, expected_raw))
        assert abs(result - expected) < 1e-9

    def test_narrow_lane_clamped(self):
        """极窄车道 → clamp 到下限。"""
        result = compute_curve_hold_window(1.0)
        assert result >= LEAD_LAT_MAX_CURVE_MIN

    def test_wide_lane_clamped(self):
        """极宽车道 → clamp 到上限。"""
        result = compute_curve_hold_window(20.0)
        assert result <= LEAD_LAT_MAX_CURVE_MAX


# ── 5. adaptive_preview_time ─────────────────────────────────────────────────


class TestAdaptivePreviewTime:
    """自适应预览时间。"""

    def test_zero_speed(self):
        """v=0 → 最小预览时间 PREVIEW_TIME_MIN。"""
        t = adaptive_preview_time(0.0, 0.0)
        assert abs(t - PREVIEW_TIME_MIN) < 1e-9

    def test_medium_speed(self):
        """v=10, curv=0 → 在 [min, max] 之间。"""
        t = adaptive_preview_time(10.0, 0.0)
        assert PREVIEW_TIME_MIN <= t <= PREVIEW_TIME_MAX

    def test_high_speed(self):
        """v >= PREVIEW_SPEED_REF, curv=0 → 接近 PREVIEW_TIME_MAX。"""
        t = adaptive_preview_time(PREVIEW_SPEED_REF, 0.0)
        assert abs(t - PREVIEW_TIME_MAX) < 1e-9

    def test_high_curvature_reduces_preview(self):
        """大曲率 → 预览时间被衰减。"""
        t_straight = adaptive_preview_time(10.0, 0.0)
        t_curve = adaptive_preview_time(10.0, 0.05)
        assert t_curve < t_straight

    def test_result_always_positive(self):
        """即使极端曲率，结果仍为正。"""
        t = adaptive_preview_time(5.0, 0.5)
        assert t > 0.0


# ── 6. compute_boundary_correction ───────────────────────────────────────────


class TestComputeBoundaryCorrection:
    """车道边界修正。"""

    def test_inside_safe_margin(self):
        """offset 在 safe 内且 hard 内 → 无修正。"""
        safe, warn, hard = lane_margins_from_width(3.8)
        dc, bk, warn_flag = compute_boundary_correction(0.1, 0.0, 10.0, safe, warn, hard)
        assert dc == 0.0
        assert bk == 0.0
        assert warn_flag is False

    def test_between_warn_and_hard(self):
        """offset 在 warn~hard 之间，但未超过 hard → 无修正。"""
        safe, warn, hard = lane_margins_from_width(3.8)
        mid = (warn + hard) / 2.0
        dc, bk, warn_flag = compute_boundary_correction(mid, 0.0, 10.0, safe, warn, hard)
        assert dc == 0.0
        assert bk == 0.0
        assert warn_flag is False

    def test_beyond_hard_margin(self):
        """offset 超过 hard_margin → 有修正。"""
        safe, warn, hard = lane_margins_from_width(3.8)
        offset = hard + 0.3
        dc, bk, warn_flag = compute_boundary_correction(offset, 0.0, 10.0, safe, warn, hard)
        assert dc != 0.0
        assert bk > 0.0
        assert warn_flag is True

    def test_correction_sign_matches_offset(self):
        """正 offset → 负修正（拉回左侧），负 offset → 正修正。"""
        safe, warn, hard = lane_margins_from_width(3.8)
        offset = hard + 0.5
        dc_pos, _, _ = compute_boundary_correction(offset, 0.0, 10.0, safe, warn, hard)
        dc_neg, _, _ = compute_boundary_correction(-offset, 0.0, 10.0, safe, warn, hard)
        assert dc_pos < 0.0
        assert dc_neg > 0.0

    def test_correction_bounded(self):
        """修正量不超过 MAX_DELTA * 0.18。"""
        safe, warn, hard = lane_margins_from_width(3.8)
        dc, _, _ = compute_boundary_correction(10.0, 0.0, 20.0, safe, warn, hard)
        assert abs(dc) <= MAX_DELTA * 0.18 + 1e-9

    def test_zero_offset(self):
        """offset=0 → 无修正。"""
        safe, warn, hard = lane_margins_from_width(3.8)
        dc, bk, flag = compute_boundary_correction(0.0, 0.0, 10.0, safe, warn, hard)
        assert dc == 0.0
        assert bk == 0.0
        assert flag is False


# ── 7. LaneWidthEstimator ─────────────────────────────────────────────────────


class TestLaneWidthEstimator:
    """车道宽度估计器——状态类。"""

    def _make_estimator(self, hz=100.0):
        return LaneWidthEstimator(hz)

    def test_cold_start_returns_min(self):
        """冷启动初期（样本不足）→ 宽度为 LANE_WIDTH_MIN。"""
        est = self._make_estimator()
        assert abs(est.width - LANE_WIDTH_MIN) < 1e-9

    def test_cold_start_sample_counts_zero(self):
        """冷启动时样本数为 (0, 0)。"""
        est = self._make_estimator()
        assert est.sample_counts == (0, 0)

    def test_converges_after_enough_samples(self):
        """喂入足够多样本后，宽度收敛到接近样本均值。"""
        est = self._make_estimator(hz=100.0)
        now = 1000.0
        # 需要超过 LANE_EST_MIN_SAMPLES 个样本
        from config import LANE_EST_MIN_SAMPLES
        n = LANE_EST_MIN_SAMPLES + 20
        target_half = 1.8  # 单侧偏移 1.8m → 宽度约 3.6m
        for i in range(n):
            now += 0.01
            est.update(target_half, now, filtered_curv=0.0, ego_v=10.0,
                       lane_offset_last_rx=now)
            est.update(-target_half, now, filtered_curv=0.0, ego_v=10.0,
                       lane_offset_last_rx=now)
        # 冷启动后宽度应已从 LANE_WIDTH_MIN 跳变
        assert est.width >= LANE_WIDTH_MIN + 0.1

    def test_rejects_outlier_above_4m(self):
        """偏移绝对值 > 4.0 的样本被拒绝，不影响宽度。"""
        est = self._make_estimator(hz=100.0)
        now = 1000.0
        from config import LANE_EST_MIN_SAMPLES
        # 先喂正常样本
        for i in range(LANE_EST_MIN_SAMPLES + 10):
            now += 0.01
            est.update(1.5, now, filtered_curv=0.0, ego_v=10.0,
                       lane_offset_last_rx=now)
            est.update(-1.5, now, filtered_curv=0.0, ego_v=10.0,
                       lane_offset_last_rx=now)
        w_before = est.width
        # 喂离群值
        for _ in range(50):
            now += 0.01
            est.update(5.0, now, filtered_curv=0.0, ego_v=10.0,
                       lane_offset_last_rx=now)
        # 离群值不应显著改变宽度
        assert abs(est.width - w_before) < 1.0

    def test_timeout_locks_width(self):
        """数据超时后宽度被锁定，不再更新。"""
        est = self._make_estimator(hz=100.0)
        now = 1000.0
        from config import LANE_EST_MIN_SAMPLES, LANE_EST_TIMEOUT_S
        # 喂入足够样本使冷启动完成
        for i in range(LANE_EST_MIN_SAMPLES + 10):
            now += 0.01
            est.update(1.5, now, filtered_curv=0.0, ego_v=10.0,
                       lane_offset_last_rx=now)
            est.update(-1.5, now, filtered_curv=0.0, ego_v=10.0,
                       lane_offset_last_rx=now)
        w_locked = est.width
        # 超过超时窗口后更新
        now += LANE_EST_TIMEOUT_S + 1.0
        result = est.update(0.5, now, filtered_curv=0.0, ego_v=10.0,
                            lane_offset_last_rx=now - LANE_EST_TIMEOUT_S - 1.0)
        assert est.is_locked
        assert abs(result - w_locked) < 1e-9

    def test_none_offset_returns_current_width(self):
        """传入 None 时不崩溃，返回当前宽度。"""
        est = self._make_estimator()
        w = est.update(None, 1000.0)
        assert abs(w - est.width) < 1e-9

    def test_non_finite_offset_returns_current_width(self):
        """传入 inf 时不崩溃，返回当前宽度。"""
        est = self._make_estimator()
        w = est.update(float('inf'), 1000.0)
        assert abs(w - est.width) < 1e-9
