#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""common.py 单元测试。

覆盖 clamp / apply_deadband / soft_deadband / wrap_angle / is_finite /
quaternion_to_yaw / parse_tagged_lines。
"""

import math

import pytest

from common import (
    _TAGGED_LINE_MAX_LEN,
    apply_deadband,
    clamp,
    is_finite,
    parse_tagged_lines,
    quaternion_to_yaw,
    soft_deadband,
    wrap_angle,
)


# ── clamp ────────────────────────────────────────────────────────────────────


class TestClamp:
    def test_inside_range(self):
        assert clamp(5, 0, 10) == 5

    def test_below_lo(self):
        assert clamp(-1, 0, 10) == 0

    def test_above_hi(self):
        assert clamp(15, 0, 10) == 10

    def test_at_lo(self):
        assert clamp(0, 0, 10) == 0

    def test_at_hi(self):
        assert clamp(10, 0, 10) == 10

    def test_inverted_bounds_lo_wins(self):
        # lo > hi: max(lo, min(hi, v)) → min(5, 10)=10, max(10, 10)=10 ... 实际 min(hi=5,v=1)=1, max(lo=10,1)=10
        # 具体行为：max(lo=10, min(hi=5, v=1)) → max(10, 1) = 10
        assert clamp(1, 10, 5) == 10

    def test_inverted_bounds_hi_wins(self):
        # max(lo=10, min(hi=5, v=12)) → max(10, 5) = 10
        assert clamp(12, 10, 5) == 10

    def test_nan(self):
        # Python 3.x: min(hi, nan) 返回 hi（非 NaN 优先），所以 clamp 返回 hi
        assert clamp(float('nan'), 0, 10) == 10

    def test_inf(self):
        assert clamp(float('inf'), 0, 10) == 10

    def test_neg_inf(self):
        assert clamp(float('-inf'), 0, 10) == 0

    def test_float_bounds(self):
        assert clamp(0.5, -1.0, 1.0) == 0.5

    def test_negative_range(self):
        assert clamp(0, -10, -1) == -1


# ── apply_deadband ───────────────────────────────────────────────────────────


class TestApplyDeadband:
    def test_inside_band_positive(self):
        assert apply_deadband(0.005, 0.01) == 0.0

    def test_inside_band_negative(self):
        assert apply_deadband(-0.005, 0.01) == 0.0

    def test_on_boundary_positive(self):
        assert apply_deadband(0.01, 0.01) == 0.0

    def test_on_boundary_negative(self):
        assert apply_deadband(-0.01, 0.01) == 0.0

    def test_outside_band_positive(self):
        assert apply_deadband(0.02, 0.01) == 0.02

    def test_outside_band_negative(self):
        assert apply_deadband(-0.02, 0.01) == -0.02

    def test_zero_input(self):
        assert apply_deadband(0.0, 0.01) == 0.0

    def test_zero_band(self):
        # band=0: 只有 v=0 通过 abs(v)<=0
        assert apply_deadband(0.0, 0.0) == 0.0
        assert apply_deadband(1e-9, 0.0) == 1e-9


# ── soft_deadband ────────────────────────────────────────────────────────────


class TestSoftDeadband:
    def test_inside_band_positive(self):
        # |v| <= band → v * 0.25
        assert soft_deadband(0.005, 0.01) == pytest.approx(0.005 * 0.25)

    def test_inside_band_negative(self):
        assert soft_deadband(-0.005, 0.01) == pytest.approx(-0.005 * 0.25)

    def test_at_band_boundary(self):
        # av == band → still inside → 25%
        assert soft_deadband(0.01, 0.01) == pytest.approx(0.01 * 0.25)

    def test_at_two_band(self):
        # av == 2*band → scale = (2b - b)/b = 1.0 → 25% + 75% = 100%
        assert soft_deadband(0.02, 0.01) == pytest.approx(0.02)

    def test_above_two_band(self):
        assert soft_deadband(0.05, 0.01) == pytest.approx(0.05)

    def test_mid_transition(self):
        # av = 1.5*band → scale = (1.5b - b)/b = 0.5
        # factor = 0.25 + 0.75*0.5 = 0.625
        v = 0.015
        band = 0.01
        expected = v * (0.25 + 0.75 * 0.5)
        assert soft_deadband(v, band) == pytest.approx(expected)

    def test_sign_preserved_negative(self):
        result = soft_deadband(-0.015, 0.01)
        assert result < 0

    def test_zero_band_passthrough(self):
        # band <= 1e-6 → return v directly
        assert soft_deadband(0.5, 0.0) == 0.5
        assert soft_deadband(0.5, 1e-7) == 0.5

    def test_zero_input(self):
        assert soft_deadband(0.0, 0.01) == pytest.approx(0.0)


# ── wrap_angle ───────────────────────────────────────────────────────────────


class TestWrapAngle:
    def test_zero(self):
        assert wrap_angle(0.0) == pytest.approx(0.0)

    def test_pi(self):
        assert wrap_angle(math.pi) == pytest.approx(math.pi, abs=1e-10)

    def test_neg_pi(self):
        assert wrap_angle(-math.pi) == pytest.approx(-math.pi, abs=1e-10)

    def test_two_pi_wraps_to_zero(self):
        assert wrap_angle(2 * math.pi) == pytest.approx(0.0, abs=1e-10)

    def test_large_positive(self):
        # 100*pi → wraps near 0
        result = wrap_angle(100 * math.pi)
        assert -math.pi <= result <= math.pi

    def test_large_negative(self):
        result = wrap_angle(-100 * math.pi)
        assert -math.pi <= result <= math.pi

    def test_half_pi(self):
        assert wrap_angle(math.pi / 2) == pytest.approx(math.pi / 2)

    def test_three_halves_pi(self):
        # 3pi/2 → -pi/2
        assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2, abs=1e-10)


# ── is_finite ────────────────────────────────────────────────────────────────


class TestIsFinite:
    def test_normal(self):
        assert is_finite(1.0) is True

    def test_zero(self):
        assert is_finite(0.0) is True

    def test_nan(self):
        assert is_finite(float('nan')) is False

    def test_inf(self):
        assert is_finite(float('inf')) is False

    def test_neg_inf(self):
        assert is_finite(float('-inf')) is False

    def test_large_but_finite(self):
        assert is_finite(1e308) is True

    def test_small_but_finite(self):
        assert is_finite(1e-308) is True


# ── quaternion_to_yaw ────────────────────────────────────────────────────────


class TestQuaternionToYaw:
    def test_identity(self):
        # (0,0,0,1) → yaw=0
        assert quaternion_to_yaw(0, 0, 0, 1) == pytest.approx(0.0)

    def test_90_deg_yaw(self):
        # 绕 Z 轴转 90°: q = (0, 0, sin(45°), cos(45°))
        angle = math.pi / 2
        qz = math.sin(angle / 2)
        qw = math.cos(angle / 2)
        assert quaternion_to_yaw(0, 0, qz, qw) == pytest.approx(math.pi / 2)

    def test_180_deg_yaw(self):
        # 绕 Z 轴转 180°: q = (0, 0, 1, 0)
        assert quaternion_to_yaw(0, 0, 1, 0) == pytest.approx(math.pi)

    def test_neg_90_deg_yaw(self):
        angle = -math.pi / 2
        qz = math.sin(angle / 2)
        qw = math.cos(angle / 2)
        assert quaternion_to_yaw(0, 0, qz, qw) == pytest.approx(-math.pi / 2)

    def test_all_zero_quaternion(self):
        # 所有分量为 0 → atan2(0, 1) = 0
        assert quaternion_to_yaw(0, 0, 0, 0) == pytest.approx(0.0)


# ── parse_tagged_lines ───────────────────────────────────────────────────────


class TestParseTaggedLines:
    def test_empty_buffer(self):
        buf = bytearray()
        assert parse_tagged_lines(buf) == {}
        assert len(buf) == 0

    def test_single_line(self):
        buf = bytearray(b'SPEED:12.50\n')
        result = parse_tagged_lines(buf)
        assert result == {'SPEED': 12.50}
        assert len(buf) == 0  # consumed

    def test_multiple_lines(self):
        buf = bytearray(b'TTC:8.00\nDIST:15.50\nPSI:0.1234\n')
        result = parse_tagged_lines(buf)
        assert result == {'TTC': 8.00, 'DIST': 15.50, 'PSI': 0.1234}
        assert len(buf) == 0

    def test_partial_line_stays_in_buffer(self):
        buf = bytearray(b'SPEED:10.0\nPARTIAL:5.')
        result = parse_tagged_lines(buf)
        assert result == {'SPEED': 10.0}
        assert bytes(buf) == b'PARTIAL:5.'

    def test_line_exceeding_max_len_skipped(self):
        # 一行超过 _TAGGED_LINE_MAX_LEN 字节 → 被跳过
        long_line = b'A:' + b'x' * _TAGGED_LINE_MAX_LEN + b'\n'
        buf = bytearray(long_line + b'SPEED:1.0\n')
        result = parse_tagged_lines(buf)
        assert 'A' not in result
        assert result == {'SPEED': 1.0}

    def test_malformed_no_colon(self):
        buf = bytearray(b'NOCOLON\n')
        result = parse_tagged_lines(buf)
        assert result == {}

    def test_malformed_non_numeric_value(self):
        buf = bytearray(b'TAG:abc\n')
        result = parse_tagged_lines(buf)
        assert result == {}

    def test_whitespace_handling(self):
        buf = bytearray(b'  TTC : 8.00 \n')
        result = parse_tagged_lines(buf)
        assert result == {'TTC': 8.00}

    def test_consume_only_complete_lines(self):
        # 两行完整 + 一个不完整的尾部
        buf = bytearray(b'A:1.0\nB:2.0\nC:3')
        result = parse_tagged_lines(buf)
        assert result == {'A': 1.0, 'B': 2.0}
        assert bytes(buf) == b'C:3'

    def test_long_buffer_without_newline_truncated(self):
        # 超过 _TAGGED_LINE_MAX_LEN 但无换行 → 截断到尾部
        buf = bytearray(b'x' * (_TAGGED_LINE_MAX_LEN + 100))
        parse_tagged_lines(buf)
        assert len(buf) == _TAGGED_LINE_MAX_LEN
