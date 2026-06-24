#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""控制管线纯计算内核。

把 ADAS._control_loop_impl 中"感知→横向→前车→弯道保持→AEB→纵向→平滑→
非AEB限幅"这一段**纯计算**抽出来（不含 ROS I/O、串口、心跳、NaN 防护、
clamp、telemetry），使其能脱离 rclpy / Simulink 被离线驱动：
  - 在线：ADAS.py 调用本函数，行为与抽取前逐行等价（同样的调用顺序与参数）。
  - 离线：replay.py / 评测框架用 CSV 重建 signals 序列后直接驱动本函数，
    复用真实控制器与跨周期状态，做秒级回归对比。

约定与全工程一致：Python 3.6 兼容、单线程、所有状态就地写入
signals / memory / managers（与在线完全相同的副作用顺序）。
"""

import copy
from dataclasses import dataclass

from lateral import lane_margins_from_width
from control.lateral_controller import compute_lateral_command
from control.longitudinal_policy import (
    compute_longitudinal_policy,
    evaluate_lead_context,
    limit_non_aeb_lon,
    update_aeb_alert,
    update_curve_hold,
)
from control.safety import apply_safety_supervisor


# ml_result 入参哨兵：区分"未传入（内核自行调 ml_bridge）"与"显式传入 None（无 ML）"。
# 锁步影子核传入主核已算出的 ml_result，保证两遍计算确定性一致，且影子侧不触碰
# 真实 ml_bridge（异步线程 / ONNX 会话，不可深拷贝）。
_UNSET = object()


@dataclass
class PipelineResult:
    """一个控制周期纯计算的全部产出（clamp / NaN 防护之前）。"""
    cur_lane_width: float
    lateral_ctx: object
    lead_ctx: object
    lon_ctx: object
    in_curve_hold: bool
    lon_cmd: float          # 平滑 + 非AEB限幅后的最终纵向指令
    lon_raw_cmd: float      # 平滑前（含边界制动取大）的纵向指令
    ml_result: object = None  # 本拍使用的 ML 推理结果（供锁步影子核复用，确定性）


def update_lane_state(now, signals, memory, lane_est):
    """车道宽估计 + 三级余量刷新（等价于原 AdasNode._update_lane_state）。"""
    # 双车道：超车期间车辆中心相对路径有 +OVT_LANE_OFFSET_M 偏移，原始 lane_offset
    # 不再代表"在车道内的横向偏移"。把偏移先减去当前目标车道中心，再喂给估计器，
    # 保证估计的仍是"目标车道宽度"而不是双车道整体宽度。
    target_off = float(getattr(memory, 'target_lane_offset', 0.0) or 0.0)
    if signals.lane_offset_received:
        raw_offset_for_est = signals.lane_offset - target_off
    else:
        raw_offset_for_est = None
    cur_lane_width = lane_est.update(
        raw_offset_for_est,
        now,
        filtered_curv=memory.filtered_curv,
        ego_v=signals.ego_v,
        lane_offset_last_rx=signals.lane_offset_last_rx,
    )
    (memory.lane_safe_margin,
     memory.lane_warn_margin,
     memory.lane_hard_margin) = lane_margins_from_width(cur_lane_width)
    return cur_lane_width


def run_pure_pipeline(now, signals, memory, managers, takeover_rate=None,
                      ml_result=_UNSET):
    """执行单周期纯控制计算，返回 PipelineResult。

    与原 _control_loop_impl 中对应段落逐行等价：
      lane → lateral → lead → curve_hold → aeb_alert → longitudinal
      → 边界制动取大 → lon_smooth → 非AEB限幅
    takeover_rate 由调用方传入（在线=接管窗内的强制限速 / None；离线=None）。
    ml_result 缺省（_UNSET）时由本内核自行调 ml_bridge；锁步影子核可显式传入主核
    本拍的 ml_result 复用，保证两遍计算确定性一致且不触碰真实 ml_bridge。
    """
    # ── 0. 超车状态机（双车道）：在最早执行，依据上一拍 LeadTracker 状态决定
    # 本拍 target_lane_offset / suppress_lead_for_overtake。
    # target_lane_offset 由超车状态机按 OVT_LANE_SHIFT_RATE_M_S 缓慢爬升，
    # 是慢变量——无需清横向缓存强制 100Hz 重算（那会重新引入帧门控本要规避的
    # 微分脉冲串）；横向控制器在下一道路帧（≤50ms）自然取到最新目标偏移。
    overtake = managers.overtake
    if overtake is not None:
        overtake.update(now, signals, memory, managers.lead_tracker.state)

    cur_lane_width = update_lane_state(now, signals, memory, managers.lane_est)
    lateral_ctx = compute_lateral_command(
        now, signals, memory, getattr(managers, 'lateral_model', None))

    # 超车 ACTIVE/PASSING 阶段抑制前车：不能用 _replace 修改 lead_ctx，
    # 因为 lead_tracker 内部状态会基于"原始检测结果"前进，导致它每拍都误判为
    # "刚重新获取前车"，触发 ACC reset 风暴和 lead_acquire 保护，把驱动指令钳零。
    # 改为在 evaluate_lead_context 之前用浅拷贝覆盖 lead_received / lead_last_rx_time，
    # 让 lead_tracker 走"无前车"分支：raw_has_lead=False, acc_has_lead=False，
    # 后续巡航分支才会真正放开油门。
    # 使用 copy.copy() 避免原地修改 signals，消除 R-03 时间耦合风险。
    suppress = bool(getattr(memory, 'suppress_lead_for_overtake', False))
    if suppress:
        signals_for_lead = copy.copy(signals)
        signals_for_lead.lead_received = False
        # 让 lead_fresh 判定也直接失败，避免 raw_has_lead 复活
        signals_for_lead.lead_last_rx_time = -1e9
    else:
        signals_for_lead = signals

    lead_ctx = evaluate_lead_context(
        now, signals_for_lead, memory, managers, cur_lane_width, lateral_ctx,
    )
    in_curve_hold = update_curve_hold(
        now, signals, memory, managers, lead_ctx,
    )
    update_aeb_alert(now, signals, managers, lead_ctx)

    # ML 推理（可选，受 config.ML_ENABLED 控制）。锁步影子核传入主核结果时跳过，
    # 既保证确定性一致，又不触碰真实 ml_bridge（异步线程 / ONNX 会话）。
    if ml_result is _UNSET:
        ml_result = None
        if managers.ml_bridge is not None:
            ml_result = managers.ml_bridge.update(now, signals, lead_ctx)

    lon_ctx = compute_longitudinal_policy(
        now, signals, lead_ctx, lateral_ctx, memory, managers, in_curve_hold,
        ml_result=ml_result,
    )

    # 边界制动优先级高于常规纵向输出，避免接近车道边缘仍给油。
    lon_cmd = lon_ctx.lon_cmd
    if lateral_ctx.boundary_brake > 0.0:
        lon_cmd = max(lon_cmd, lateral_ctx.boundary_brake)
    safety = apply_safety_supervisor(
        now, lon_cmd, signals, lead_ctx, lateral_ctx, lon_ctx,
    )
    lon_cmd = safety.lon_cmd
    lon_raw_cmd = lon_cmd

    comfort_layer = getattr(managers, 'comfort_layer', None)
    if comfort_layer is not None:
        lon_cmd = comfort_layer.update(
            target=lon_cmd,
            aeb_active=(lon_ctx.aeb_active or safety.active),
            boundary_brake=(lateral_ctx.boundary_brake > 0.0),
        )

    lon_cmd = managers.lon_smooth.update(
        lon_cmd,
        aeb_active=(lon_ctx.aeb_active or safety.active),
        has_lead=lead_ctx.acc_has_lead,
        boundary_brake=(lateral_ctx.boundary_brake > 0.0),
        max_rate_override=takeover_rate,
    )
    if not (lon_ctx.aeb_active or safety.active):
        lon_cmd = limit_non_aeb_lon(
            lon_cmd,
            lon_ctx,
            lead_ctx.acc_has_lead,
            signals.ego_v,
            boundary_brake_active=(lateral_ctx.boundary_brake > 0.0),
            raw_lead_v_proj=lead_ctx.raw_lead_v_proj,
        )

    return PipelineResult(
        cur_lane_width=cur_lane_width,
        lateral_ctx=lateral_ctx,
        lead_ctx=lead_ctx,
        lon_ctx=lon_ctx,
        in_curve_hold=in_curve_hold,
        lon_cmd=lon_cmd,
        lon_raw_cmd=lon_raw_cmd,
        ml_result=ml_result,
    )
