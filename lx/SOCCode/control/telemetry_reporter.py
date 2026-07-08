#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""遥测行组织（TelemetryReporter）—— 从 AdasNode 抽出（★6 主循环瘦身，
openpilot 式 publish / diagnostics 分离）。

持有 telemetry sink；`record()` 接收 `AdasNode` 与本拍上下文，构造遥测行并非阻塞投递。
与原 ADAS.py `_record_telemetry` **字节级一致**（同字段同值，仅 `self.`→`node.`），
另追加 ★5 planning→control 接口的只读投影列（target_offset / target_speed）。

Python 3.6 兼容（JetPack 4.x）。中文注释，遵循 SOCCode 约定。
"""

import time

from control.aeb_overlay import summarize_aeb
from control.safety_status import aggregate_safety_status
from control.trajectory import summarize_target


class TelemetryReporter:
    """每周期遥测行的组织器（无控制逻辑，仅读 node 状态 + 非阻塞投递）。"""

    def __init__(self, telemetry):
        self._telemetry = telemetry

    def record(self, node, now, lateral_ctx, lon_ctx, lead_ctx, in_curve_hold,
               lon_cmd, lon_raw_cmd, psi_tx, delta_tx, speed_tx, lon_tx,
               cur_lane_width, fb=None):
        """组织本周期遥测行并非阻塞投递（写盘在后台线程，零阻塞控制环）。

        fb 可由调用方传入（本拍已读的底盘回传），避免每拍重复读 read_feedback。
        """
        _fb = fb if fb is not None else node.vehicle.read_feedback()
        # AEB 结构化诊断（只读投影；engaged 直取 lon_ctx.aeb_active，与控制一致不漂移）
        _aeb = summarize_aeb(lon_ctx.ttc, lon_ctx.dist, node.signals.ego_v,
                             lon_ctx.lead_v_proj, lon_ctx.aeb_active)
        # 统一安全状态聚合（只读，把分散安全信号聚成整机安全态；不改任何控制）
        try:
            _failover = node.peer_hb.is_failover_available()
        except Exception:
            _failover = True
        _safety = aggregate_safety_status(
            gate_reason=node._last_gate_reason, gate_severity=node._last_gate_severity,
            aeb_level=_aeb.level, failover_available=_failover, esp32_stale=_fb.stale)
        # ★5 planning→control 接口（只读投影：横向期望偏移/曲率 + 纵向期望速度）
        _path, _speed = summarize_target(
            getattr(node.memory, 'target_lane_offset', 0.0) or 0.0,
            lateral_ctx.curv_guard,
            getattr(node.signals, 'driver_set_speed', 0.0) or 0.0)
        self._telemetry.record({
            't_wall': time.time(),
            't_mono': now,
            'cycle': node.memory.cycle_count,
            'ego_x': node.signals.ego_x,
            'ego_y': node.signals.ego_y,
            'ego_yaw': node.signals.ego_yaw,
            'ego_v': node.signals.ego_v,
            'lead_x': node.signals.lead_x,
            'lead_y': node.signals.lead_y,
            'lead_v': node.signals.lead_v,
            'road_psi': node.signals.road_psi,
            'filtered_road_psi': node.memory.filtered_road_psi,
            'raw_cte': lateral_ctx.raw_cte,
            'filtered_cte': node.memory.filtered_cte,
            'raw_curv': lateral_ctx.raw_curv,
            'filtered_curv': node.memory.filtered_curv,
            'curv_guard': lateral_ctx.curv_guard,
            'in_curve': lateral_ctx.in_curve,
            'delta': lateral_ctx.delta,
            'delta_cte': lateral_ctx.delta_cte,
            'delta_ff': lateral_ctx.delta_ff,
            'boundary_delta': lateral_ctx.boundary_delta,
            'psi_i_term': node.memory.psi_i_term,
            'upd_psi': lateral_ctx.upd_psi,
            'lon_raw_cmd': lon_raw_cmd,
            'lon_cmd': lon_cmd,
            'acc_i_term': node.lon_ctrl.i_term,
            'aeb_active': lon_ctx.aeb_active,
            'lead_cls': node.signals.lead_cls,
            'in_curve_hold': in_curve_hold,
            'dist': lon_ctx.dist,
            'ttc': lon_ctx.ttc,
            'lead_v_proj': lon_ctx.lead_v_proj,
            'min_safe_dist': lon_ctx.min_safe_dist,
            'closing_speed': lon_ctx.closing_speed,
            'acc_has_lead': lead_ctx.acc_has_lead,
            'lead_detected': lead_ctx.lead_detected,
            'cur_lane_width': cur_lane_width,
            'lane_safe_margin': node.memory.lane_safe_margin,
            'lane_warn_margin': node.memory.lane_warn_margin,
            'lane_hard_margin': node.memory.lane_hard_margin,
            'boundary_brake': lateral_ctx.boundary_brake,
            'boundary_warn': lateral_ctx.boundary_warn,
            'psi_tx': psi_tx,
            'delta_tx': delta_tx,
            'speed_tx': speed_tx,
            'lon_tx': lon_tx,
            'esp_psi': _fb.psi,
            'esp_delta': _fb.delta,
            'esp_brake': _fb.brake,
            # 结构化诊断：本帧命令门控 reason/severity（CommandGate）
            'gate_reason': node._last_gate_reason,
            'gate_severity': node._last_gate_severity,
            # AEB 结构化诊断 overlay（对标 Autoware AEB node）
            'aeb_level': _aeb.level,
            'aeb_drac': _aeb.drac,
            # 统一安全状态聚合（对标 openpilot selfdriveState）
            'safety_level': _safety.level,
            'safety_state': _safety.state,
            # ★5 planning→control 接口（只读投影）
            'target_offset': _path.lateral_offset_target,
            'target_speed': _speed.target_speed,
            # class-aware AEB / 接管 cls 冗余诊断字段
            'lead_cls_stale': bool(getattr(node, '_last_lead_cls_stale', False)),
            'takeover_seed_cls': (node._takeover_seed_cls
                                  if now < node._takeover_guard_until
                                  else 0),
            # AEB 预测路径碰撞检查诊断（control/aeb_path_check.py；未启用时恒默认值）
            'aeb_path_risk': bool(getattr(lon_ctx, 'aeb_path_risk', False)),
            'aeb_path_tcol': getattr(lon_ctx, 'aeb_path_tcol', float('inf')),
            'aeb_rss_dist': getattr(lon_ctx, 'aeb_rss_dist', 0.0),
        })
