# -*- coding: utf-8 -*-
"""CommandGate 回归测试。

锁定两条契约：
  1. gate 的 clamp / NaN 守护 / 安全回退帧与原 ADAS.py 内联逻辑**字节级一致**
     （引入 gate 不得改变任何下发数值）；
  2. reason / severity 分类正确（结构化诊断）。
"""

import math

from common import clamp, is_finite
from config import (
    LANE_DEFAULT_WIDTH,
    LON_CMD_MAX_BRAKE_DECEL,
    LON_CMD_MAX_DRIVE_ACCEL,
)
from control.serial_protocol import Esp32ControlFrame, build_esp32_payload
from control.command_gate import (
    CommandGate,
    GATE_AEB_OVERRIDE,
    GATE_BACKUP_TAKEOVER,
    GATE_NAN_BLOCK,
    GATE_NORMAL,
    SEVERITY_CRITICAL,
    SEVERITY_NORMAL,
    SEVERITY_WARN,
)

NAN = float("nan")
INF = float("inf")

# 全字段有限、可下发的一组基准入参（顺序对齐 evaluate 形参）
BASE = dict(
    upd_psi=0.3, delta=0.05, cur_off=0.1, ego_v=12.0, lon_cmd=-0.5, ttc=8.0,
    dist=20.0, lead_v_proj=10.0, min_safe_dist=9.0, cur_lane_width=3.5,
    lane_warn_margin=1.0, lane_hard_margin=0.7, filtered_curv=0.01,
)


def _orig_clamp(upd_psi, delta, ego_v, lon_cmd, ttc, dist):
    """原 ADAS.py 1109-1115 内联 clamp 公式。"""
    return (
        ttc if is_finite(ttc) else 999.99,
        clamp(dist, 0.0, 999.99),
        clamp(upd_psi, -9.9999, 9.9999),
        clamp(delta, -9.9999, 9.9999),
        clamp(ego_v, -99.99, 99.99),
        clamp(lon_cmd, -LON_CMD_MAX_DRIVE_ACCEL, LON_CMD_MAX_BRAKE_DECEL),
    )


def _orig_safe_payload(lon_cmd):
    """原 _build_safe_fallback_payload。"""
    return build_esp32_payload(Esp32ControlFrame(
        ttc=999.99, dist=999.99, psi=0.0, delta=0.0, speed=0.0, lon=float(lon_cmd),
        offset=0.0, lead_v_proj=0.0, min_safe_dist=0.0,
        lane_warn_margin=LANE_DEFAULT_WIDTH * 0.5 * 0.6,
        lane_hard_margin=LANE_DEFAULT_WIDTH * 0.5 * 0.4, filtered_curv=0.0))


def test_clamp_matches_inline_over_range():
    g = CommandGate()
    # 仅有限值（非有限值会被 NaN 守护拦截，另有专门测试覆盖）
    probes = [0.0, 0.5, -0.5, 12.3, -7.0, 1000.0, -1000.0, 99.999, 9.99995, 1e-9]
    for v in probes:
        kw = dict(BASE)
        kw.update(upd_psi=v, delta=v, ego_v=v, lon_cmd=v, ttc=v, dist=v)
        d = g.evaluate(**kw)
        assert not d.blocked
        got = (d.ttc, d.dist, d.psi, d.delta, d.speed, d.lon)
        exp = _orig_clamp(v, v, v, v, v, v)
        assert got == exp, (v, got, exp)


def test_nonfinite_blocks_and_reports_each_field():
    g = CommandGate()
    fields = ["upd_psi", "delta", "cur_off", "ego_v", "lon_cmd", "dist",
              "lead_v_proj", "min_safe_dist", "cur_lane_width",
              "lane_warn_margin", "lane_hard_margin", "filtered_curv"]
    for f in fields:
        for bad in (NAN, INF, -INF):
            kw = dict(BASE)
            kw[f] = bad
            d = g.evaluate(**kw)
            assert d.blocked
            assert d.reason == GATE_NAN_BLOCK
            assert d.severity == SEVERITY_CRITICAL
            assert f in d.bad_fields


def test_ttc_inf_is_not_blocked():
    # ttc=inf 是“无前车”正常哨兵，不应触发 NaN 拦截，应落到 999.99
    g = CommandGate()
    kw = dict(BASE)
    kw["ttc"] = INF
    d = g.evaluate(**kw)
    assert not d.blocked
    assert d.ttc == 999.99


def test_reason_classification():
    g = CommandGate()
    assert g.evaluate(**BASE).reason == GATE_NORMAL
    assert g.evaluate(**BASE).severity == SEVERITY_NORMAL
    d_aeb = g.evaluate(aeb_active=True, **BASE)
    assert d_aeb.reason == GATE_AEB_OVERRIDE and d_aeb.severity == SEVERITY_WARN
    d_to = g.evaluate(takeover_active=True, **BASE)
    assert d_to.reason == GATE_BACKUP_TAKEOVER and d_to.severity == SEVERITY_WARN
    # AEB 优先于接管
    d_both = g.evaluate(aeb_active=True, takeover_active=True, **BASE)
    assert d_both.reason == GATE_AEB_OVERRIDE


def test_safe_fallback_payload_byte_identical():
    g = CommandGate()
    for lon in (LON_CMD_MAX_BRAKE_DECEL, 0.0, 1.5, 2.5):
        assert g.build_safe_fallback_payload(lon) == _orig_safe_payload(lon)


def test_nonfinite_fields_helper_matches_candidates():
    g = CommandGate()
    # 全有限 → 空
    assert g.nonfinite_fields(0, 0, 0, 1, 0, 1, 0, 0, 3.5, 1, 1, 0) == []
    # 多个非有限 → 全部列出且保持声明顺序
    got = g.nonfinite_fields(NAN, 0, 0, INF, 0, 1, 0, 0, 3.5, 1, 1, NAN)
    assert got == ["upd_psi", "ego_v", "filtered_curv"]
