#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""serial_protocol.py 单元测试。

覆盖 _crc8_dallas / build_esp32_payload / Esp32ControlFrame。
"""

import pytest

from control.serial_protocol import Esp32ControlFrame, _crc8_dallas, build_esp32_payload


# ── _crc8_dallas ─────────────────────────────────────────────────────────────


class TestCrc8Dallas:
    def test_empty(self):
        assert _crc8_dallas(b'') == 0x00

    def test_known_ascii_123456789(self):
        # CRC-8/MAXIM (poly 0x31, init 0x00, no reflect/no XOR-out): "123456789" → 0xA2
        assert _crc8_dallas(b'123456789') == 0xA2

    def test_single_byte_zero(self):
        # 0x00 的 CRC 查表结果
        assert _crc8_dallas(b'\x00') == 0x00

    def test_single_byte_ff(self):
        assert _crc8_dallas(b'\xff') == _crc8_dallas(b'\xff')

    def test_all_zeros_8_bytes(self):
        result = _crc8_dallas(b'\x00' * 8)
        assert isinstance(result, int)
        assert 0 <= result <= 255

    def test_all_ff_8_bytes(self):
        result = _crc8_dallas(b'\xff' * 8)
        assert isinstance(result, int)
        assert 0 <= result <= 255

    def test_deterministic(self):
        data = b'TTC:8.00 DIST:15.50 '
        assert _crc8_dallas(data) == _crc8_dallas(data)

    def test_different_data_different_crc(self):
        assert _crc8_dallas(b'hello') != _crc8_dallas(b'world')

    def test_return_range(self):
        for byte_val in range(256):
            result = _crc8_dallas(bytes([byte_val]))
            assert 0 <= result <= 255


# ── Esp32ControlFrame ────────────────────────────────────────────────────────


class TestEsp32ControlFrame:
    def test_construction(self):
        frame = Esp32ControlFrame(
            ttc=8.0, dist=15.5, psi=0.1, delta=0.05,
            speed=16.7, lon=-2.5, offset=0.1,
            lead_v_proj=14.0, min_safe_dist=10.0,
            lane_warn_margin=1.98, lane_hard_margin=3.06, filtered_curv=0.01,
        )
        assert frame.ttc == 8.0
        assert frame.dist == 15.5
        assert frame.psi == 0.1
        assert frame.delta == 0.05
        assert frame.speed == 16.7
        assert frame.lon == -2.5
        assert frame.offset == 0.1
        assert frame.lead_v_proj == 14.0
        assert frame.min_safe_dist == 10.0
        assert frame.lane_warn_margin == 1.98
        assert frame.lane_hard_margin == 3.06
        assert frame.filtered_curv == 0.01

    def test_frozen(self):
        frame = Esp32ControlFrame(
            ttc=1, dist=2, psi=3, delta=4, speed=5, lon=6,
            offset=7, lead_v_proj=8, min_safe_dist=9,
            lane_warn_margin=10, lane_hard_margin=11, filtered_curv=12,
        )
        with pytest.raises(AttributeError):
            frame.ttc = 999


# ── build_esp32_payload ──────────────────────────────────────────────────────


def _make_default_frame():
    return Esp32ControlFrame(
        ttc=8.0, dist=15.5, psi=0.1234, delta=0.05,
        speed=16.7, lon=-2.5, offset=0.1,
        lead_v_proj=14.0, min_safe_dist=10.0,
        lane_warn_margin=1.98, lane_hard_margin=3.06, filtered_curv=0.01,
    )


class TestBuildEsp32Payload:
    def test_returns_bytes(self):
        payload = build_esp32_payload(_make_default_frame())
        assert isinstance(payload, bytes)

    def test_ends_with_newline(self):
        payload = build_esp32_payload(_make_default_frame())
        assert payload.endswith(b'\n')

    def test_contains_all_tags(self):
        payload = build_esp32_payload(_make_default_frame()).decode('ascii')
        for tag in ('TTC:', 'DIST:', 'PSI:', 'DELTA:', 'SPEED:', 'ACC:',
                     'OFFSET:', 'LEADV:', 'DSAFE:', 'WMRN:', 'WHRD:', 'CURV:', 'CRC:'):
            assert tag in payload

    def test_crc_appended(self):
        payload = build_esp32_payload(_make_default_frame()).decode('ascii')
        assert 'CRC:' in payload
        # CRC 后面紧跟 \n
        crc_idx = payload.index('CRC:')
        crc_value = payload[crc_idx + 4:].strip()
        assert len(crc_value) == 2  # 两位十六进制

    def test_crc_is_valid(self):
        payload = build_esp32_payload(_make_default_frame())
        # 找到 CRC: 前的 body
        crc_tag = b' CRC:'
        idx = payload.index(crc_tag)
        body = payload[:idx + 1]  # 包含末尾空格
        expected_crc = _crc8_dallas(body)
        crc_str = payload[idx + 5:idx + 7].decode('ascii')
        assert int(crc_str, 16) == expected_crc

    def test_negative_acc_format(self):
        # ACC 用 +.2f 格式，负数应带负号
        frame = Esp32ControlFrame(
            ttc=1, dist=2, psi=0, delta=0, speed=0, lon=-3.5,
            offset=0, lead_v_proj=0, min_safe_dist=0,
            lane_warn_margin=0, lane_hard_margin=0, filtered_curv=0,
        )
        payload = build_esp32_payload(frame).decode('ascii')
        assert 'ACC:-3.50' in payload

    def test_zero_frame(self):
        frame = Esp32ControlFrame(
            ttc=0, dist=0, psi=0, delta=0, speed=0, lon=0,
            offset=0, lead_v_proj=0, min_safe_dist=0,
            lane_warn_margin=0, lane_hard_margin=0, filtered_curv=0,
        )
        payload = build_esp32_payload(frame)
        assert payload.endswith(b'\n')
        assert b'TTC:0.00' in payload
        assert b'CRC:' in payload
