# -*- coding: utf-8 -*-
"""★6 TelemetryReporter 回归测试。

关键：reporter 产出的遥测行必须覆盖 telemetry.FIELDS 全部列（搬移漏字段会被抓住），
且 ★5 接口列 target_offset/target_speed 正确。
"""

from types import SimpleNamespace

from control.telemetry_reporter import TelemetryReporter


class FakeTelemetry(object):
    def __init__(self):
        self.rows = []

    def record(self, row):
        self.rows.append(row)


class FakeVehicle(object):
    def read_feedback(self):
        return SimpleNamespace(psi=0.1, delta=0.02, brake=1.5, stale=False)


class FakePeer(object):
    def is_failover_available(self):
        return True


def _node():
    memory = SimpleNamespace(
        cycle_count=42, filtered_road_psi=0.1, filtered_cte=0.2, filtered_curv=0.01,
        psi_i_term=0.0, lane_safe_margin=1.5, lane_warn_margin=1.0, lane_hard_margin=0.7,
        target_lane_offset=0.3)
    signals = SimpleNamespace(
        ego_x=1.0, ego_y=2.0, ego_yaw=0.1, ego_v=12.0, lead_x=30.0, lead_y=0.0,
        lead_v=10.0, road_psi=0.05, lead_cls=1, driver_set_speed=13.9)
    lon_ctrl = SimpleNamespace(i_term=0.5)
    return SimpleNamespace(
        vehicle=FakeVehicle(), peer_hb=FakePeer(), memory=memory, signals=signals,
        lon_ctrl=lon_ctrl, _last_gate_reason="normal", _last_gate_severity=0,
        _last_lead_cls_stale=False, _takeover_seed_cls=0, _takeover_guard_until=0.0)


def _ctxs():
    lateral_ctx = SimpleNamespace(
        raw_cte=0.1, raw_curv=0.01, curv_guard=0.012, in_curve=False, delta=0.03,
        delta_cte=0.01, delta_ff=0.005, boundary_delta=0.0, upd_psi=0.05,
        boundary_brake=False, boundary_warn=False)
    lon_ctx = SimpleNamespace(
        ttc=8.0, dist=30.0, lead_v_proj=10.0, min_safe_dist=9.0, closing_speed=2.0,
        aeb_active=False)
    lead_ctx = SimpleNamespace(acc_has_lead=True, lead_detected=True)
    return lateral_ctx, lon_ctx, lead_ctx


def test_reporter_covers_all_fields():
    from telemetry import FIELDS
    tel = FakeTelemetry()
    rep = TelemetryReporter(tel)
    lat, lon, lead = _ctxs()
    rep.record(_node(), 1.23, lat, lon, lead, False, -0.5, -0.3,
               0.05, 0.03, 12.0, -0.5, 3.5)
    assert len(tel.rows) == 1
    row = tel.rows[0]
    missing = [f for f in FIELDS if f not in row]
    assert missing == [], "reporter 缺失 FIELDS 列: %r" % missing


def test_reporter_key_values():
    tel = FakeTelemetry()
    rep = TelemetryReporter(tel)
    lat, lon, lead = _ctxs()
    rep.record(_node(), 1.23, lat, lon, lead, False, -0.5, -0.3,
               0.05, 0.03, 12.0, -0.5, 3.5)
    row = tel.rows[0]
    assert row['cycle'] == 42
    assert row['ego_v'] == 12.0
    assert row['esp_delta'] == 0.02
    assert row['gate_reason'] == "normal"
    assert row['aeb_level'] in ("safe", "warning", "emergency")
    # ★5 接口列
    assert row['target_offset'] == 0.3
    assert row['target_speed'] == 13.9
