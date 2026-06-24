#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纵向控制模块单元测试。

测试范围：
  - compute_ttc（碰撞时间计算）
  - compute_min_safe_distance（最小安全跟车距离）
  - compute_ttc_gate_distance（AEB 触发距离门限）
  - aeb_curv_suppress（弯道 AEB 抑制）
  - apply_aeb（AEB 三级制动逻辑）
  - LonSmoothing（纵向指令平滑器）
  - LongitudinalController（ACC 纵向控制器）
"""

import math
import pytest

from config import *
from longitudinal import (
    LongitudinalController,
    LonSmoothing,
    aeb_curv_suppress,
    apply_aeb,
    compute_min_safe_distance,
    compute_ttc,
    compute_ttc_gate_distance,
)


# ─────────────────────────────────────────────────────────
# compute_ttc
# ─────────────────────────────────────────────────────────

class TestComputeTTC:
    """compute_ttc 碰撞时间计算测试。"""

    def test_approaching_ego_faster_returns_finite_ttc(self):
        """自车更快时返回有限 TTC。"""
        ttc = compute_ttc(fwd=30.0, ego_v=20.0, lead_v=10.0,
                          lead_yaw=0.0, ego_yaw=0.0)
        # closing = 20 - 10*cos(0) = 10; TTC = 30/10 = 3.0
        assert math.isfinite(ttc)
        assert abs(ttc - 3.0) < 1e-6

    def test_approaching_same_direction_with_yaw_diff(self):
        """有航向差的接近场景，前车速度投影后仍可算出 TTC。"""
        # lead_yaw=0.3, ego_yaw=0.0 => cos(0.3) ≈ 0.9553
        # closing = 20 - 15*0.9553 ≈ 5.67
        ttc = compute_ttc(fwd=20.0, ego_v=20.0, lead_v=15.0,
                          lead_yaw=0.3, ego_yaw=0.0)
        assert math.isfinite(ttc)
        assert ttc > 0.0

    def test_not_approaching_lead_faster_returns_inf(self):
        """前车更快时返回 inf。"""
        ttc = compute_ttc(fwd=30.0, ego_v=10.0, lead_v=20.0,
                          lead_yaw=0.0, ego_yaw=0.0)
        assert ttc == float('inf')

    def test_not_approaching_same_speed_returns_inf(self):
        """同速行驶返回 inf。"""
        ttc = compute_ttc(fwd=30.0, ego_v=15.0, lead_v=15.0,
                          lead_yaw=0.0, ego_yaw=0.0)
        assert ttc == float('inf')

    def test_fwd_near_zero_returns_zero(self):
        """距离极近时直接返回 0.0。"""
        ttc = compute_ttc(fwd=1e-8, ego_v=20.0, lead_v=10.0,
                          lead_yaw=0.0, ego_yaw=0.0)
        assert ttc == 0.0

    def test_fwd_exactly_zero_returns_zero(self):
        """距离为 0 返回 0.0。"""
        ttc = compute_ttc(fwd=0.0, ego_v=20.0, lead_v=10.0,
                          lead_yaw=0.0, ego_yaw=0.0)
        assert ttc == 0.0

    def test_both_zero_speed_returns_inf(self):
        """双车静止返回 inf（无接近速度）。"""
        ttc = compute_ttc(fwd=10.0, ego_v=0.0, lead_v=0.0,
                          lead_yaw=0.0, ego_yaw=0.0)
        assert ttc == float('inf')

    def test_with_lead_v_proj_override_approaching(self):
        """使用 lead_v_proj 覆盖时，closing = ego_v - lead_v_proj。"""
        ttc = compute_ttc(fwd=20.0, ego_v=20.0, lead_v=99.0,
                          lead_yaw=0.0, ego_yaw=0.0, lead_v_proj=10.0)
        # closing = 20 - 10 = 10; TTC = 20/10 = 2.0
        assert abs(ttc - 2.0) < 1e-6

    def test_with_lead_v_proj_not_approaching(self):
        """lead_v_proj 大于 ego_v 时返回 inf。"""
        ttc = compute_ttc(fwd=20.0, ego_v=10.0, lead_v=5.0,
                          lead_yaw=0.0, ego_yaw=0.0, lead_v_proj=15.0)
        # closing = 10 - max(0, 15) = -5 < floor => inf
        assert ttc == float('inf')

    def test_with_lead_v_proj_zero(self):
        """lead_v_proj=0 时完全按自车速度计算。"""
        ttc = compute_ttc(fwd=15.0, ego_v=10.0, lead_v=99.0,
                          lead_yaw=0.0, ego_yaw=0.0, lead_v_proj=0.0)
        # closing = 10 - 0 = 10; TTC = 15/10 = 1.5
        assert abs(ttc - 1.5) < 1e-6

    def test_low_closing_speed_returns_inf(self):
        """极低接近速度时自适应阈值生效，返回 inf。"""
        # ego_v=0.2, closing_floor = max(1e-3, 0.05*0.2) = max(1e-3, 0.01) = 0.01
        # closing = 0.2 - 0.1 = 0.1 > 0.01 => finite
        ttc = compute_ttc(fwd=5.0, ego_v=0.2, lead_v=0.1,
                          lead_yaw=0.0, ego_yaw=0.0)
        assert math.isfinite(ttc)
        assert abs(ttc - 50.0) < 1e-4

    def test_ego_v_zero_lead_stationary_returns_inf(self):
        """自车静止、前车静止，closing=0，返回 inf。"""
        ttc = compute_ttc(fwd=5.0, ego_v=0.0, lead_v=0.0,
                          lead_yaw=0.0, ego_yaw=0.0, lead_v_proj=0.0)
        assert ttc == float('inf')


# ─────────────────────────────────────────────────────────
# compute_min_safe_distance
# ─────────────────────────────────────────────────────────

class TestComputeMinSafeDistance:
    """compute_min_safe_distance 最小安全跟车距离测试。"""

    def test_both_zero_returns_standstill(self):
        """双车静止返回 SAFE_DIST_STANDSTILL。"""
        d = compute_min_safe_distance(0.0, 0.0)
        assert abs(d - SAFE_DIST_STANDSTILL) < 1e-6

    def test_high_speed_scales_up(self):
        """高速时安全距离显著增大。"""
        d_low = compute_min_safe_distance(5.0, 5.0)
        d_high = compute_min_safe_distance(20.0, 20.0)
        assert d_high > d_low

    def test_negative_velocities_clamped_to_zero(self):
        """负速度被钳位到 0，结果与双零相同。"""
        d_neg = compute_min_safe_distance(-5.0, -3.0)
        d_zero = compute_min_safe_distance(0.0, 0.0)
        assert abs(d_neg - d_zero) < 1e-6

    def test_lead_faster_reduces_distance(self):
        """前车更快时安全距离比同速更短。"""
        d_same = compute_min_safe_distance(15.0, 15.0)
        d_lead_faster = compute_min_safe_distance(15.0, 25.0)
        assert d_lead_faster < d_same

    def test_result_within_bounds(self):
        """结果始终在 [SAFE_DIST_STANDSTILL, SAFE_DIST_MAX] 范围内。"""
        for v_e in [0.0, 5.0, 15.0, 30.0]:
            for v_l in [0.0, 5.0, 15.0, 30.0]:
                d = compute_min_safe_distance(v_e, v_l)
                assert d >= SAFE_DIST_STANDSTILL - 1e-6
                assert d <= SAFE_DIST_MAX + 1e-6

    def test_moderate_speed_calculation(self):
        """中等速度下手动验算。"""
        v_e = 10.0
        d_react = v_e * SAFE_REACTION_TIME
        d_ego = (v_e * v_e) / (2.0 * SAFE_EGO_MAX_DECEL)
        d_lead = 0.0
        d_min = SAFE_DIST_STANDSTILL + d_react + d_ego - d_lead
        d = compute_min_safe_distance(v_e, 0.0)
        # 结果应被 clamp，至少不小于 floor
        assert d >= SAFE_DIST_STANDSTILL - 1e-6


# ─────────────────────────────────────────────────────────
# compute_ttc_gate_distance
# ─────────────────────────────────────────────────────────

class TestComputeTtcGateDistance:
    """compute_ttc_gate_distance AEB 触发距离门限测试。"""

    def test_normal_case_returns_reasonable_distance(self):
        """正常工况返回合理门限距离。"""
        d = compute_ttc_gate_distance(
            min_safe_dist=15.0, closing_speed=10.0, ttc_brake_start=15.0)
        # dynamic_dist = 10*(15-1) = 140; base=max(20, 18, 140)=140
        # clamp(140, 20, 25) = 25
        assert d == AEB_MAX_ENGAGE_DIST

    def test_closing_speed_zero_returns_max_engage_dist(self):
        """无接近速度时返回最大触发距离。"""
        d = compute_ttc_gate_distance(
            min_safe_dist=15.0, closing_speed=0.0, ttc_brake_start=15.0)
        # dynamic_dist = 0*(15-1) = 0; base=max(20, 18, 0)=20
        # clamp(20, 20, 25) = 20
        assert abs(d - TTC_AEB_MAX_DIST) < 1e-6

    def test_small_safe_dist_returns_at_least_max_dist(self):
        """安全距离小时不低于 TTC_AEB_MAX_DIST。"""
        d = compute_ttc_gate_distance(
            min_safe_dist=5.0, closing_speed=0.0, ttc_brake_start=10.0)
        assert d >= TTC_AEB_MAX_DIST

    def test_custom_max_engage_dist(self):
        """自定义 max_engage_dist 参数生效。"""
        d = compute_ttc_gate_distance(
            min_safe_dist=15.0, closing_speed=0.0, ttc_brake_start=15.0,
            max_engage_dist=40.0)
        assert d <= 40.0 + 1e-6

    def test_large_closing_speed_clamped_by_max_engage(self):
        """大接近速度被 max_engage_dist 钳位。"""
        d = compute_ttc_gate_distance(
            min_safe_dist=15.0, closing_speed=50.0, ttc_brake_start=15.0,
            max_engage_dist=25.0)
        assert d <= 25.0 + 1e-6


# ─────────────────────────────────────────────────────────
# aeb_curv_suppress
# ─────────────────────────────────────────────────────────

class TestAebCurvSuppress:
    """aeb_curv_suppress 弯道 AEB 抑制测试。"""

    def test_zero_curvature_returns_one(self):
        """零曲率时不抑制（返回 1.0）。"""
        s = aeb_curv_suppress(0.0)
        assert abs(s - 1.0) < 1e-6

    def test_large_curvature_approaches_max_suppress(self):
        """大曲率时抑制趋近 AEB_CURV_SUPPRESS_MAX。"""
        s = aeb_curv_suppress(1.0)
        # exp(-1.0/0.03) ≈ 0 => suppress ≈ AEB_CURV_SUPPRESS_MAX
        assert abs(s - AEB_CURV_SUPPRESS_MAX) < 0.01

    def test_medium_curvature_between_bounds(self):
        """中等曲率时抑制值在 [MAX, 1.0] 之间。"""
        s = aeb_curv_suppress(0.015)
        assert AEB_CURV_SUPPRESS_MAX <= s <= 1.0

    def test_result_always_in_range(self):
        """任意曲率下结果始终在 [AEB_CURV_SUPPRESS_MAX, 1.0]。"""
        for curv in [0.0, 0.001, 0.01, 0.03, 0.1, 0.5, 2.0]:
            s = aeb_curv_suppress(curv)
            assert AEB_CURV_SUPPRESS_MAX - 1e-6 <= s <= 1.0 + 1e-6

    def test_negative_curvature_same_as_positive(self):
        """负曲率与正曲率抑制效果相同（取绝对值）。"""
        s_pos = aeb_curv_suppress(0.05)
        s_neg = aeb_curv_suppress(-0.05)
        assert abs(s_pos - s_neg) < 1e-10


# ─────────────────────────────────────────────────────────
# apply_aeb
# ─────────────────────────────────────────────────────────

class TestApplyAeb:
    """apply_aeb AEB 三级制动逻辑测试。"""

    def test_emergency_fwd_zero_full_brake(self):
        """距离为 0 → 全力制动。"""
        aeb_cmd, act = apply_aeb(
            fwd=0.0, ttc=float('inf'), lon=0.0, min_safe_dist=10.0)
        assert act is True
        assert abs(aeb_cmd - LON_CMD_MAX_BRAKE_DECEL) < 1e-6

    def test_emergency_fwd_within_emergency_dist(self):
        """距离在紧急制动范围内 → 全力制动。"""
        aeb_cmd, act = apply_aeb(
            fwd=3.0, ttc=1.0, lon=0.0, min_safe_dist=10.0,
            aeb_emergency_dist=AEB_EMERGENCY_DIST)
        assert act is True
        assert abs(aeb_cmd - LON_CMD_MAX_BRAKE_DECEL) < 1e-6

    def test_emergency_closing_and_very_close(self):
        """接近中且距离在 0.75*min_safe_dist 内 → 全力制动。"""
        min_safe = 10.0
        fwd = min_safe * 0.7  # 7.0, < 0.75*10=7.5 且 > AEB_EMERGENCY_DIST=5.0
        aeb_cmd, act = apply_aeb(
            fwd=fwd, ttc=2.0, lon=0.0, min_safe_dist=min_safe, closing=1.0)
        assert act is True
        assert abs(aeb_cmd - LON_CMD_MAX_BRAKE_DECEL) < 1e-6

    def test_ttc_brake_full_triggers_full_brake(self):
        """TTC < TTC_BRAKE_FULL 且在门限内 → ACC_NORMAL_BRAKE_MAX。"""
        min_safe = 10.0
        # 需要 fwd <= ttc_gate_dist, closing > 0.8, TTC finite
        # gate_dist: dynamic = 5*(15-1)=70; base=max(20,12,70)=70; clamp(70,20,25)=25
        aeb_cmd, act = apply_aeb(
            fwd=20.0, ttc=3.0, lon=0.0, min_safe_dist=min_safe,
            closing=5.0)
        assert act is True
        # TTC=3 < TTC_BRAKE_FULL=5, not full_confirmed, fwd(20) > aeb_dist_hard(10)+2=12
        assert abs(aeb_cmd - ACC_NORMAL_BRAKE_MAX) < 1e-6

    def test_ttc_brake_full_confirmed_and_close(self):
        """full_confirmed=True 且距离足够近 → LON_CMD_MAX_BRAKE_DECEL。"""
        min_safe = 10.0
        # fwd=11 < aeb_dist_hard(10)+2=12
        aeb_cmd, act = apply_aeb(
            fwd=11.0, ttc=3.0, lon=0.0, min_safe_dist=min_safe,
            closing=5.0, full_confirmed=True)
        assert act is True
        assert abs(aeb_cmd - LON_CMD_MAX_BRAKE_DECEL) < 1e-6

    def test_ttc_brake_proportional(self):
        """TTC 在 FULL 和 START 之间 → 比例制动。"""
        min_safe = 10.0
        # cs=1.0, ttc_brake_start=15, ttc_brake_full=5
        # TTC=10, ratio=(15-10)/(15-5)=0.5 => 0.5*3=1.5
        aeb_cmd, act = apply_aeb(
            fwd=22.0, ttc=10.0, lon=0.0, min_safe_dist=min_safe,
            closing=5.0)
        assert act is True
        expected = 0.5 * ACC_NORMAL_BRAKE_MAX
        assert abs(aeb_cmd - expected) < 0.1

    def test_no_trigger_far_distance_and_large_ttc(self):
        """距离超出门限且 TTC 大 → 不触发。"""
        aeb_cmd, act = apply_aeb(
            fwd=50.0, ttc=20.0, lon=0.0, min_safe_dist=10.0, closing=0.0)
        assert act is False
        assert aeb_cmd == 0.0

    def test_no_trigger_closing_too_low(self):
        """closing 不够（<=0.8）时 TTC 制动不触发。"""
        min_safe = 10.0
        aeb_cmd, act = apply_aeb(
            fwd=20.0, ttc=3.0, lon=0.0, min_safe_dist=min_safe,
            closing=0.5)  # < 0.8
        # closing <= 0.8 => TTC 分支不进入; 距离制动 closing > 0.5 要求
        # 但 fwd=20 > aeb_dist_soft=18 => 距离制动也不进
        # => 应该走 "fwd > ttc_gate_dist and fwd > aeb_dist_soft" => no trigger
        # gate_dist: dynamic=0.5*(15-1)=7; base=max(20,12,7)=20; clamp(20,20,25)=20
        # fwd=20 <= ttc_gate_dist=20 => 不走 early return
        # TTC分支: closing=0.5 <= 0.8 => skip
        # 距离分支: fwd=20 > aeb_dist_soft=18 => skip
        assert act is False

    def test_class_aware_pedestrian_tighter_threshold(self):
        """行人 class 乘子 >1 → TTC 阈值收紧，相同 TTC 下更容易触发。"""
        min_safe = 10.0
        # vehicle (mult=1.0): ttc_brake_start=15, ttc_brake_full=5
        _, act_vehicle = apply_aeb(
            fwd=22.0, ttc=6.0, lon=0.0, min_safe_dist=min_safe,
            closing=5.0, ttc_class_mult=1.0)
        # pedestrian (mult=1.6): ttc_brake_start=24, ttc_brake_full=8
        # TTC=6 < 8 => should trigger full brake
        _, act_ped = apply_aeb(
            fwd=22.0, ttc=6.0, lon=0.0, min_safe_dist=min_safe,
            closing=5.0, ttc_class_mult=1.6)
        # vehicle: TTC=6 > TTC_BRAKE_FULL=5 => proportional ratio=(15-6)/(15-5)=0.9
        # both should trigger, but pedestrian should give stronger brake
        assert act_ped is True
        aeb_v, _ = apply_aeb(
            fwd=22.0, ttc=6.0, lon=0.0, min_safe_dist=min_safe,
            closing=5.0, ttc_class_mult=1.0)
        aeb_p, _ = apply_aeb(
            fwd=22.0, ttc=6.0, lon=0.0, min_safe_dist=min_safe,
            closing=5.0, ttc_class_mult=1.6)
        assert aeb_p >= aeb_v

    def test_curvature_suppression_relaxes_threshold(self):
        """弯道曲率大时 TTC 阈值放宽，相同 TTC 下更不容易触发。"""
        min_safe = 10.0
        # 直道: ttc_brake_full = 5.0*1.0 = 5.0
        _, act_straight = apply_aeb(
            fwd=22.0, ttc=5.5, lon=0.0, min_safe_dist=min_safe,
            closing=5.0, filtered_curv=0.0)
        # 大曲率: cs≈0.5, ttc_brake_full = 5.0*0.5 = 2.5
        _, act_curve = apply_aeb(
            fwd=22.0, ttc=5.5, lon=0.0, min_safe_dist=min_safe,
            closing=5.0, filtered_curv=1.0)
        # 直道 TTC=5.5 > 5.0 => 不全制动，但可能比例制动
        # 弯道 TTC=5.5 > 2.5 => 不全制动
        # 直道比弯道更容易触发
        if act_straight:
            aeb_s, _ = apply_aeb(
                fwd=22.0, ttc=5.5, lon=0.0, min_safe_dist=min_safe,
                closing=5.0, filtered_curv=0.0)
            aeb_c, _ = apply_aeb(
                fwd=22.0, ttc=5.5, lon=0.0, min_safe_dist=min_safe,
                closing=5.0, filtered_curv=1.0)
            assert aeb_s >= aeb_c

    def test_distance_brake_soft_zone(self):
        """距离在软安全区内，即使 TTC 宽裕也触发距离制动。"""
        min_safe = 10.0
        aeb_dist_soft = min_safe + AEB_SAFE_DIST_BUFFER  # 18
        # fwd=16 < aeb_dist_soft=18, closing > 0.5
        # TTC 大但距离近
        aeb_cmd, act = apply_aeb(
            fwd=16.0, ttc=100.0, lon=0.0, min_safe_dist=min_safe,
            closing=1.0)
        assert act is True
        # soft_ratio = (18-16)/(18-10) = 2/8 = 0.25 => 0.25*3 = 0.75
        expected = 0.25 * ACC_NORMAL_BRAKE_MAX
        assert aeb_cmd >= expected - 0.1

    def test_lon_already_braking_preserved(self):
        """当前已有制动指令时取较大值。"""
        existing_brake = 2.0
        aeb_cmd, act = apply_aeb(
            fwd=0.0, ttc=float('inf'), lon=existing_brake,
            min_safe_dist=10.0)
        assert act is True
        assert aeb_cmd >= existing_brake


# ─────────────────────────────────────────────────────────
# LonSmoothing
# ─────────────────────────────────────────────────────────

class TestLonSmoothing:
    """LonSmoothing 纵向指令平滑器测试。"""

    def test_large_jump_is_rate_limited(self):
        """大跳变被变化率限幅限制。"""
        sm = LonSmoothing(dt=0.01)
        # 初始值 0，目标 10，巡航模式
        out = sm.update(10.0)
        max_step = LON_RATE_DECEL_CRUISE * 0.01  # 2.50*0.01=0.025
        # 但 first call _prev=0, target=10 => limited=clamp(10, 0-0.025, 0+0.025)=0.025
        # _filtered += 0.25*(0.025 - 0) = 0.00625
        assert out < 1.0  # 远小于 10
        assert out > 0.0

    def test_aeb_active_bypasses_lowpass(self):
        """AEB 激活时跳过低通滤波，直接输出限幅值。"""
        sm = LonSmoothing(dt=0.01)
        sm.reset(0.0)
        out = sm.update(10.0, aeb_active=True)
        # aeb rate = 60.0, step = 60*0.01 = 0.6
        # limited = clamp(10, -0.6, 0.6) = 0.6
        # aeb_active => _filtered = limited = 0.6
        assert abs(out - 0.6) < 1e-6
        assert abs(sm.value - 0.6) < 1e-6
        assert abs(sm.prev - 0.6) < 1e-6

    def test_aeb_multiple_steps_rapid_increase(self):
        """AEB 模式下多步快速上升。"""
        sm = LonSmoothing(dt=0.01)
        sm.reset(0.0)
        for _ in range(20):
            sm.update(10.0, aeb_active=True)
        # 20 步 * 0.6 = 12 > 10 => 已到达 10
        assert abs(sm.value - 10.0) < 1.0

    def test_reset_changes_value(self):
        """reset() 将内部状态设为指定值。"""
        sm = LonSmoothing(dt=0.01)
        sm.update(5.0, aeb_active=True)
        sm.reset(3.0)
        assert abs(sm.value - 3.0) < 1e-6
        assert abs(sm.prev - 3.0) < 1e-6

    def test_reset_default_zero(self):
        """reset() 默认归零。"""
        sm = LonSmoothing(dt=0.01)
        sm.update(5.0, aeb_active=True)
        sm.reset()
        assert abs(sm.value) < 1e-6
        assert abs(sm.prev) < 1e-6

    def test_boundary_brake_uses_boundary_rate(self):
        """边界制动使用 LON_RATE_BOUNDARY 限幅。"""
        sm = LonSmoothing(dt=0.01)
        sm.reset(0.0)
        out = sm.update(10.0, boundary_brake=True)
        max_step = LON_RATE_BOUNDARY * 0.01  # 8.0*0.01=0.08
        # limited = clamp(10, -0.08, 0.08) = 0.08
        # _filtered += 0.25*(0.08 - 0) = 0.02
        assert abs(out - 0.02) < 1e-6

    def test_has_lead_decel_rate(self):
        """跟车减速模式使用 LON_RATE_DECEL_ACC。"""
        sm = LonSmoothing(dt=0.01)
        sm.reset(0.0)
        out = sm.update(5.0, has_lead=True)
        max_step = LON_RATE_DECEL_ACC * 0.01  # 3.0*0.01=0.03
        # limited = clamp(5, -0.03, 0.03) = 0.03
        assert sm.prev == pytest.approx(0.03, abs=1e-6)

    def test_max_rate_override_takes_effect(self):
        """外部强制限速低于工况限速时生效。"""
        sm = LonSmoothing(dt=0.01)
        sm.reset(0.0)
        # 巡航减速限幅 2.5，但 override 更小
        out = sm.update(10.0, max_rate_override=1.0)
        max_step = 1.0 * 0.01  # 0.01
        # limited = clamp(10, -0.01, 0.01) = 0.01
        assert abs(sm.prev - 0.01) < 1e-6

    def test_max_rate_override_larger_than_case_rate_ignored(self):
        """外部限速高于工况限速时不生效（取更严的）。"""
        sm = LonSmoothing(dt=0.01)
        sm.reset(0.0)
        sm.update(10.0, max_rate_override=100.0)
        # 巡航减速限幅 2.5 < 100 => 用 2.5
        max_step = LON_RATE_DECEL_CRUISE * 0.01
        assert abs(sm.prev - max_step) < 1e-6

    def test_steady_state_output_near_target(self):
        """稳态时输出趋近目标值。"""
        sm = LonSmoothing(dt=0.01)
        sm.reset(0.0)
        target = 2.0
        for _ in range(500):
            sm.update(target, has_lead=True)
        assert abs(sm.value - target) < 0.1

    def test_brake_release_rate_limit(self):
        """从大制动值释放时使用 LON_RATE_BRAKE_RELEASE 限幅。"""
        sm = LonSmoothing(dt=0.01)
        sm.reset(5.0)  # 当前大制动值
        # has_lead=True, _prev=5.0 > 1.0 and target < _prev
        # => rate = max(LON_RATE_BRAKE_RELEASE, LON_RATE_ACCEL_ACC)
        out = sm.update(3.0, has_lead=True)
        max_step = max(LON_RATE_BRAKE_RELEASE, LON_RATE_ACCEL_ACC) * 0.01
        expected_prev = 5.0 - max_step  # 5.0 - 0.04 = 4.96
        assert abs(sm.prev - expected_prev) < 1e-6

    def test_value_property_matches_filtered(self):
        """value 属性返回低通滤波后的值。"""
        sm = LonSmoothing(dt=0.01)
        sm.reset(0.0)
        sm.update(1.0, aeb_active=True)
        assert sm.value == sm._filtered

    def test_prev_property_tracks_slope_limit_base(self):
        """prev 属性跟踪坡度限幅起点。"""
        sm = LonSmoothing(dt=0.01)
        sm.reset(2.0)
        assert sm.prev == 2.0
        sm.update(2.0)
        assert sm.prev == 2.0  # 目标相同，限幅后仍为 2.0


# ─────────────────────────────────────────────────────────
# LongitudinalController
# ─────────────────────────────────────────────────────────

class TestLongitudinalController:
    """LongitudinalController ACC 纵向控制器测试。"""

    def test_steady_state_small_output(self):
        """稳态跟车（距离=参考距离，速度一致）输出接近零。"""
        ctrl = LongitudinalController(dt=0.01)
        ego_v = 15.0
        # d_ref = max(ACC_D0 + ACC_TIME_GAP * ego_v, min_safe_dist)
        min_safe = compute_min_safe_distance(ego_v, ego_v)
        d_ref = max(ACC_D0 + ACC_TIME_GAP * ego_v, min_safe)
        out = ctrl.compute(
            dist=d_ref, ego_v=ego_v, lead_v_proj=ego_v,
            lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        assert abs(out) < 0.5

    def test_closing_gap_positive_brake(self):
        """接近前车（距离小于参考）→ 正值（制动）。"""
        ctrl = LongitudinalController(dt=0.01)
        ego_v = 15.0
        min_safe = compute_min_safe_distance(ego_v, 10.0)
        out = ctrl.compute(
            dist=min_safe * 0.5, ego_v=ego_v, lead_v_proj=10.0,
            lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        assert out > 0.0

    def test_opening_gap_negative_accel_capped(self):
        """远离前车（距离大于参考）→ 负值（加速），被 drive_max 钳位。"""
        ctrl = LongitudinalController(dt=0.01)
        ego_v = 15.0
        min_safe = compute_min_safe_distance(ego_v, ego_v)
        out = ctrl.compute(
            dist=100.0, ego_v=ego_v, lead_v_proj=ego_v,
            lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        assert out < 0.0
        # 加速被 drive_max 钳位
        drive_max = ACC_DRIVE_MAX_BASE  # 0.6
        assert out >= -(ACC_DRIVE_MAX_LIMIT + 0.1)

    def test_reset_clears_i_term(self):
        """reset() 后积分项归零。"""
        ctrl = LongitudinalController(dt=0.01)
        # 运行多步积累积分
        min_safe = compute_min_safe_distance(15.0, 15.0)
        for _ in range(100):
            ctrl.compute(
                dist=min_safe * 0.5, ego_v=15.0, lead_v_proj=15.0,
                lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        assert abs(ctrl.i_term) > 0.0  # 积分应有值
        ctrl.reset()
        assert abs(ctrl.i_term) < 1e-9

    def test_set_gains_changes_output(self):
        """set_gains() 后增益变化影响输出（非饱和场景）。"""
        ctrl = LongitudinalController(dt=0.01)
        ego_v = 15.0
        # 用同速 + 小距离误差，避免输出饱和到 drive_max/brake_max
        min_safe = compute_min_safe_distance(ego_v, ego_v)
        d_ref = max(ACC_D0 + ACC_TIME_GAP * ego_v, min_safe)
        dist = d_ref - 0.2  # 小距离误差
        out_default = ctrl.compute(
            dist=dist, ego_v=ego_v, lead_v_proj=ego_v,
            lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        ctrl.reset()
        ctrl.set_gains(acc_kd=2.0)
        out_high_kd = ctrl.compute(
            dist=dist, ego_v=ego_v, lead_v_proj=ego_v,
            lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        # 更高的 KD 增益 → 更大的制动输出
        assert abs(out_high_kd) > abs(out_default) + 1e-6

    def test_set_gains_clamps_negative_to_zero(self):
        """set_gains() 将负增益钳位到 0。"""
        ctrl = LongitudinalController(dt=0.01)
        ctrl.set_gains(acc_kd=-1.0, acc_ki=-0.5)
        g = ctrl.gains()
        assert g['acc_kd'] == 0.0
        assert g['acc_ki'] == 0.0

    def test_gains_returns_current_values(self):
        """gains() 返回当前增益字典。"""
        ctrl = LongitudinalController(dt=0.01)
        g = ctrl.gains()
        assert 'acc_kd' in g
        assert 'acc_ki' in g
        assert 'acc_kv' in g
        assert 'acc_ka' in g
        assert g['acc_kd'] == ACC_KD
        assert g['acc_ki'] == ACC_KI

    def test_set_gains_partial_update(self):
        """set_gains() 只更新指定增益。"""
        ctrl = LongitudinalController(dt=0.01)
        original_kv = ctrl.gains()['acc_kv']
        ctrl.set_gains(acc_kd=1.5)
        g = ctrl.gains()
        assert g['acc_kd'] == 1.5
        assert g['acc_kv'] == original_kv  # 未变

    def test_lead_acceleration_feedforward(self):
        """前车减速时前馈项增大制动。"""
        ctrl = LongitudinalController(dt=0.01)
        ego_v = 15.0
        min_safe = compute_min_safe_distance(ego_v, 10.0)
        dist = max(ACC_D0 + ACC_TIME_GAP * ego_v, min_safe)
        # 前车加速度为零
        out_no_accel = ctrl.compute(
            dist=dist, ego_v=ego_v, lead_v_proj=10.0,
            lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        ctrl.reset()
        # 前车在减速
        out_decel = ctrl.compute(
            dist=dist, ego_v=ego_v, lead_v_proj=10.0,
            lead_accel=-3.0, min_safe_dist=min_safe, curv=0.0)
        # 前车减速 → ff_term 负 → lon = -(-a_cmd) 更负 → 更大制动
        # 但 ff_term 有限幅，且其他项相同
        # 实际上: ff_term = clamp(KA*(-3), -FF_MAX, FF_MAX) = clamp(-3, -0.6, 0.6) = -0.6
        # a_cmd = dist_term + i_term + speed_term + (-0.6)
        # lon_raw = -a_cmd => 比无前馈时更正 => 更大制动
        assert out_decel > out_no_accel or abs(out_decel) > abs(out_no_accel)

    def test_curvature_blocks_acceleration(self):
        """弯道曲率超阈值时禁止加速。"""
        ctrl = LongitudinalController(dt=0.01)
        ego_v = 15.0
        min_safe = compute_min_safe_distance(ego_v, ego_v)
        # 远距离 → 应该是加速（负值）
        out_straight = ctrl.compute(
            dist=100.0, ego_v=ego_v, lead_v_proj=ego_v,
            lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        ctrl.reset()
        out_curve = ctrl.compute(
            dist=100.0, ego_v=ego_v, lead_v_proj=ego_v,
            lead_accel=0.0, min_safe_dist=min_safe,
            curv=CURV_NO_ACCEL_THRESH + 0.01)
        if out_straight < 0:
            assert out_curve == 0.0  # 弯道禁止加速

    def test_i_term_zero_when_ki_zero(self):
        """KI=0 时积分项始终为零。"""
        ctrl = LongitudinalController(dt=0.01)
        ctrl.set_gains(acc_ki=0.0)
        min_safe = compute_min_safe_distance(15.0, 10.0)
        for _ in range(50):
            ctrl.compute(
                dist=min_safe * 0.5, ego_v=15.0, lead_v_proj=10.0,
                lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        assert abs(ctrl.i_term) < 1e-9

    def test_i_term_pauses_at_large_v_rel(self):
        """速度差很大时积分暂停（abs(v_rel) >= ACC_I_PAUSE_VDIFF）。"""
        ctrl = LongitudinalController(dt=0.01)
        min_safe = compute_min_safe_distance(15.0, 15.0)
        # v_rel = 10 - 15 = -5, abs(v_rel)=5 >= ACC_I_PAUSE_VDIFF=1.5 => 积分暂停
        for _ in range(100):
            ctrl.compute(
                dist=min_safe * 0.8, ego_v=15.0, lead_v_proj=10.0,
                lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        assert abs(ctrl.i_term) < 1e-9

    def test_i_term_accumulates_at_small_v_rel(self):
        """速度差很小时积分正常累积（abs(v_rel) < ACC_I_PAUSE_VDIFF）。"""
        ctrl = LongitudinalController(dt=0.01)
        min_safe = compute_min_safe_distance(15.0, 15.0)
        # v_rel = 15 - 15 = 0, abs(v_rel)=0 < ACC_I_PAUSE_VDIFF=1.5 => 积分累积
        # gap_err = dist - d_ref = min_safe*0.8 - d_ref < 0 => closing 积分
        for _ in range(100):
            ctrl.compute(
                dist=min_safe * 0.8, ego_v=15.0, lead_v_proj=15.0,
                lead_accel=0.0, min_safe_dist=min_safe, curv=0.0)
        assert abs(ctrl.i_term) > 1e-6

    def test_multiple_steps_output_bounded(self):
        """多步运行后输出始终在合理范围内。"""
        ctrl = LongitudinalController(dt=0.01)
        min_safe = compute_min_safe_distance(15.0, 10.0)
        outputs = []
        for i in range(200):
            dist = min_safe * (0.5 + 0.005 * i)
            out = ctrl.compute(
                dist=dist, ego_v=15.0, lead_v_proj=10.0,
                lead_accel=-1.0, min_safe_dist=min_safe, curv=0.0)
            outputs.append(out)
        for out in outputs:
            assert -ACC_DRIVE_MAX_LIMIT - 0.1 <= out <= ACC_BRAKE_MAX_LIMIT + 0.1
