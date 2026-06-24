#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""纵向策略编排：单周期纵向控制指令计算。

负责从感知和横向输出综合计算纵向加速度指令，包含：
  1. 前车上下文评估（委托 LeadTracker）
  2. 弯道保持状态更新（委托 CurveHoldManager）
  3. AEB 告警状态更新（委托 AebAlertManager）
  4. 三种纵向模式的切换与计算：
     - ACC 跟车模式（有前车）
     - 弯道保持模式（弯道中丢失前车）
     - 巡航模式（无前车无弯道保持）
  5. AEB 制动叠加与纵向指令平滑限制
"""

import logging
import math
from typing import Optional

from common import clamp, is_finite
from config import *
from longitudinal import (
    apply_aeb,
    aeb_curv_suppress,
    compute_min_safe_distance,
    compute_ttc,
    compute_ttc_gate_distance,
)

from control.context import ControlManagers, ControlMemory, LateralContext, VehicleSignals
from control.state import LeadContext, LeadTrackingInputs, LongitudinalContext


def evaluate_lead_context(now: float,
                          signals: VehicleSignals,
                          memory: ControlMemory,
                          managers: ControlManagers,
                          cur_lane_width: float,
                          lateral_ctx: LateralContext) -> LeadContext:
    """构建前车跟踪输入并执行评估，返回完整前车上下文。

    如果本周期前车重获，则重置 ACC 控制器。
    """
    inputs = LeadTrackingInputs(
        ego_x=signals.ego_x,
        ego_y=signals.ego_y,
        ego_yaw=signals.ego_yaw,
        ego_v=signals.ego_v,
        lead_x=signals.lead_x,
        lead_y=signals.lead_y,
        lead_yaw=signals.lead_yaw,
        lead_v=signals.lead_v,
        lead_cls=signals.lead_cls,
        lead_received=signals.lead_received,
        lead_last_rx_time=signals.lead_last_rx_time,
        filtered_curv=memory.filtered_curv,
        cur_lane_width=cur_lane_width,
        lane_locked=managers.lane_est.is_locked,
        last_acc_has_lead=memory.last_acc_has_lead,
        filtered_lead_v_proj=memory.filtered_lead_v_proj,
        last_lead_v_proj=memory.last_lead_v_proj,
        last_lead_reacq_t=memory.last_lead_reacq_t,
        last_curve_t=memory.last_curve_t,
    )
    lead_ctx = managers.lead_tracker.evaluate(now, inputs, lateral_ctx.in_curve)
    # 前车重获时重置纵向控制器和加速度估计
    if lead_ctx.lead_acquired:
        acquired_lead_v = (
            float(lead_ctx.raw_lead_v_proj)
            if is_finite(lead_ctx.raw_lead_v_proj)
            else max(0.0, float(signals.lead_v))
        )
        acquired_lead_v = max(0.0, acquired_lead_v)
        memory.filtered_lead_v_proj = acquired_lead_v
        memory.last_lead_v_proj = acquired_lead_v
        # last_lead_v_time 置 0 而非 now：下一拍进入加速度估计时
        # dt_lead = now - 0 会很大，导致 raw_lead_accel ≈ 0（分子接近 0，分母很大），
        # 不会产生虚假加速度。若置为 now，则第一拍 dt_lead≈dt(10ms)，
        # 而 lead_v_proj 因低通滤波(α=0.04)还未收敛，差值非零，会产生虚假大加速度。
        memory.last_lead_v_time = 0.0
        memory.filtered_v_tgt = acquired_lead_v
        memory.last_lead_reacq_t = now
        memory.acc_acquire_ff_clamp_logged = False
        memory.aeb_full_confirm_count = 0
        managers.lon_ctrl.reset()
        memory.filtered_lead_accel = 0.0
        if getattr(managers, 'lead_estimator', None) is not None:
            managers.lead_estimator.reset(acquired_lead_v)
    return lead_ctx


def update_curve_hold(now: float,
                      signals: VehicleSignals,
                      memory: ControlMemory,
                      managers: ControlManagers,
                      lead_ctx: LeadContext) -> bool:
    """更新弯道保持状态机，返回是否处于弯道保持模式。"""
    return managers.curve_hold.update(
        now,
        has_lead=lead_ctx.has_lead,
        raw_has_lead=lead_ctx.raw_has_lead,
        filtered_curv=memory.filtered_curv,
        ego_v=signals.ego_v,
    )


def update_aeb_alert(now: float,
                     signals: VehicleSignals,
                     managers: ControlManagers,
                     lead_ctx: LeadContext):
    """更新 AEB 告警状态机。"""
    managers.aeb_alert.update(now, signals.ego_v, lead_ctx)


def limit_non_aeb_lon(lon_cmd: float,
                      lon_ctx: LongitudinalContext,
                      acc_has_lead: bool,
                      ego_v: float,
                      boundary_brake_active: bool = False,
                      raw_lead_v_proj: Optional[float] = None) -> float:
    """Apply final non-AEB longitudinal output limits.

    Boundary braking is a safety correction, so it must not be reduced by
    regular ACC speed-match/no-brake comfort caps.
    """
    lon_cmd = clamp(lon_cmd, -LON_CMD_MAX_DRIVE_ACCEL, ACC_NORMAL_BRAKE_MAX)
    if boundary_brake_active:
        return lon_cmd

    if acc_has_lead:
        match_brake_cap = clamp(
            lon_ctx.closing_speed / max(ACC_MATCH_TAU_S, 0.2) + ACC_MATCH_BRAKE_MARGIN,
            0.0,
            ACC_NORMAL_BRAKE_MAX,
        )
        lon_cmd = min(lon_cmd, match_brake_cap)
        if (
            lon_ctx.dist > (lon_ctx.min_safe_dist + ACC_NO_BRAKE_DIST_MARGIN)
            and ego_v <= ((raw_lead_v_proj if raw_lead_v_proj is not None else lon_ctx.lead_v_proj) + 0.8)
        ):
            lon_cmd = min(lon_cmd, 0.2)
    elif ego_v >= (min(DRIVER_SET_SPEED, SYSTEM_MAX_CRUISE, ROAD_LIMIT_SPEED) - 0.2):
        lon_cmd = max(lon_cmd, 0.0)
    return lon_cmd


def compute_longitudinal_policy(now: float,
                                signals: VehicleSignals,
                                lead_ctx: LeadContext,
                                lateral_ctx: LateralContext,
                                memory: ControlMemory,
                                managers: ControlManagers,
                                in_curve_hold: bool,
                                ml_result=None) -> LongitudinalContext:
    """计算单周期纵向控制指令。

    根据前车状态和弯道保持状态选择不同的纵向策略：
      - ACC 模式：跟车控制，使用距离误差 PI + 速度差 + 前车加速度前馈
      - 弯道保持模式：PI 速度控制器保持弯道入口速度
      - 巡航模式：速度跟踪 + AEB 告警保持
    """
    lead_state = managers.lead_tracker.state
    alert_state = managers.aeb_alert.state
    curve_hold_state = managers.curve_hold.state
    aeb_active = False
    dist = 999.99
    ttc = float('inf')
    lead_v_proj = 0.0
    min_safe_dist = 0.0
    lead_acquire_grace_active = False
    acc_ff_before = 0.0
    acc_ff_after = 0.0
    closing_speed = 0.0

    # ── 无前车时重置 AEB 全制动确认计数 ──
    if not lead_ctx.acc_has_lead:
        memory.aeb_full_confirm_count = 0

    # ══════════════════════════════════════════════════════
    # 模式 1：ACC 跟车模式（有前车）
    # ══════════════════════════════════════════════════════
    if lead_ctx.acc_has_lead:
        x_rel = lead_ctx.x_rel
        y_rel = lead_ctx.y_rel
        # 数据不新鲜时使用记忆位置
        if (not lead_ctx.lead_fresh) and (now - lead_state.last_confirmed_lead_t) <= LEAD_MEMORY_S:
            x_rel = lead_state.last_lead_x_rel
            y_rel = lead_state.last_lead_y_rel
        dist = x_rel

        # ── 前车投影速度：直接使用 LeadTracker 已滤波的输出，避免双重滤波 ──
        raw_lead_v_proj = lead_ctx.raw_lead_v_proj
        memory.filtered_lead_v_proj = lead_ctx.predicted_lead_v_proj
        lead_v_proj = memory.filtered_lead_v_proj

        # ── 前车加速度估计（两阶段差分） ──
        lead_estimator = getattr(managers, 'lead_estimator', None)
        if lead_estimator is not None:
            dt_lead = (
                now - memory.last_lead_v_time
                if memory.last_lead_v_time > 0
                else memory.dt
            )
            lead_v_proj, memory.filtered_lead_accel = lead_estimator.update(
                lead_v_proj, dt_lead)
            memory.filtered_lead_v_proj = lead_v_proj
        elif memory.last_lead_v_time > 0:
            dt_lead = now - memory.last_lead_v_time
            if dt_lead > 0.001:
                raw_lead_accel = (lead_v_proj - memory.last_lead_v_proj) / dt_lead
                raw_lead_accel = clamp(raw_lead_accel, -LEAD_ACCEL_MAX, LEAD_ACCEL_MAX)
                memory.filtered_lead_accel += LEAD_ACCEL_FILTER_ALPHA * (
                    raw_lead_accel - memory.filtered_lead_accel
                )
        memory.last_lead_v_proj = lead_v_proj
        memory.last_lead_v_time = now

        lead_accel = memory.filtered_lead_accel
        # 前车重获保护期内，限制前馈加速度
        if lead_ctx.recent_reacq:
            lead_accel = clamp(lead_accel, -ACC_REACQ_FF_MAX, ACC_REACQ_FF_MAX)

        # ── 前车重获制动保护（防止突然松油门） ──
        lead_acquire_grace_active = (now - memory.last_lead_reacq_t) <= ACC_LEAD_ACQUIRE_GRACE_S
        acc_ff_before = clamp(ACC_KA * lead_accel, -ACC_FF_MAX, ACC_FF_MAX)
        if lead_acquire_grace_active and acc_ff_before < ACC_LEAD_ACQUIRE_MAX_BRAKE:
            clamped_lead_accel = ACC_LEAD_ACQUIRE_MAX_BRAKE / max(ACC_KA, 1e-6)
            lead_accel = max(lead_accel, clamped_lead_accel)
            acc_ff_after = clamp(ACC_KA * lead_accel, -ACC_FF_MAX, ACC_FF_MAX)
            if not memory.acc_acquire_ff_clamp_logged:
                logging.info(
                    '[ACC_LEAD] acquire brake clamp: ff_before=%.3f ff_after=%.3f grace=%.2f dist=%.2f ttc=%.2f ego_v=%.2f lead_v=%.2f',
                    acc_ff_before,
                    acc_ff_after,
                    now - memory.last_lead_reacq_t,
                    dist,
                    lead_ctx.acc_ttc,
                    signals.ego_v,
                    lead_v_proj,
                )
                memory.acc_acquire_ff_clamp_logged = True
        else:
            acc_ff_after = acc_ff_before

        # ── TTC 和安全距离 ──
        ttc = compute_ttc(
            dist,
            signals.ego_v,
            signals.lead_v,
            signals.lead_yaw,
            signals.ego_yaw,
            lead_v_proj,
        )
        min_safe_dist = compute_min_safe_distance(signals.ego_v, lead_v_proj)

        # ── 目标速度滤波 ──
        memory.filtered_v_tgt += VTGT_FILTER_ALPHA * (lead_v_proj - memory.filtered_v_tgt)

        # ── ACC 控制器计算 ──
        lon_cmd = managers.lon_ctrl.compute(
            dist=dist,
            ego_v=signals.ego_v,
            lead_v_proj=lead_v_proj,
            lead_accel=lead_accel,
            min_safe_dist=min_safe_dist,
            curv=lateral_ctx.curv_guard,
        )

        # ── 前车重获时额外限制 ──
        d_ref = max(ACC_D0 + ACC_TIME_GAP * max(signals.ego_v, 0.0), min_safe_dist)
        if lead_ctx.recent_reacq and dist > (d_ref + max(8.0, signals.ego_v * 0.3)):
            lon_cmd = min(lon_cmd, 0.0)
        if lead_ctx.recent_reacq:
            lon_cmd = clamp(lon_cmd, -ACC_REACQ_DRIVE_MAX, ACC_REACQ_BRAKE_MAX)

        # ── AEB 制动叠加 ──
        # class-aware：行人/障碍取更宽的乘子、更远的触发距离，并旁路 min lead_v 网关
        # （它们的 lead_v_proj 长期≈0，常规网关会卡掉所有可制动机会）。
        # UNKNOWN/VEHICLE 取乘子 1.0 + 旁路 False = 与改造前完全一致。
        cls_mult = AEB_CLASS_TTC_MULT.get(lead_ctx.lead_cls, 1.0)
        cls_engage_dist = AEB_CLASS_ENGAGE_DIST.get(lead_ctx.lead_cls, AEB_MAX_ENGAGE_DIST)
        cls_bypass_minv = AEB_CLASS_BYPASS_MIN_LEAD_V.get(lead_ctx.lead_cls, False)

        aeb_allowed = abs(y_rel) <= lead_ctx.lead_lat_gate
        closing_speed = max(0.0, signals.ego_v - max(0.0, lead_v_proj))
        ttc_cs = aeb_curv_suppress(memory.filtered_curv)
        ttc_brake_start = TTC_BRAKE_START * ttc_cs * cls_mult
        ttc_full_thr = TTC_BRAKE_FULL * ttc_cs * cls_mult
        # 把 cls_engage_dist 同步传给 ttc_gate_dist 的 clamp 上界，
        # 否则行人 40m 触发距只会在外层 aeb_target_valid 网关生效，
        # 内层 ttc_gate_dist 仍被 AEB_MAX_ENGAGE_DIST(25) 夹住，服务/全制动进不去。
        ttc_gate_dist = compute_ttc_gate_distance(
            min_safe_dist, closing_speed, ttc_brake_start,
            max_engage_dist=cls_engage_dist)

        # AEB 全制动确认计数
        aeb_target_valid = (
            aeb_allowed
            and is_finite(ttc)
            and dist <= cls_engage_dist
            and lead_state.lead_confirm_count >= LEAD_CONFIRM_CYCLES
            and (
                cls_bypass_minv
                or lead_v_proj >= ACC_MIN_VALID_LEAD_V
                or dist <= ACC_CLOSE_SLOW_LEAD_DIST
            )
        )
        # 全制动确认阈值按 class 收紧：行人 2 / 障碍 3 / 车辆 5。
        # 退出步长仍 -2 不分 class，保护误识别后能快速解除。
        cls_confirm = AEB_CLASS_FULL_CONFIRM_CYCLES.get(
            lead_ctx.lead_cls, AEB_FULL_CONFIRM_CYCLES)
        # 滞回（M-02）：已确认全制动后，TTC 略超阈值（15% 容差）时维持
        # 计数不增不减，防止阈值边界处 count 在 cls_confirm 和 cls_confirm-1
        # 之间振荡导致全制动↔服务制动反复切换（制动执行器抖动）。
        _aeb_full_active = (memory.aeb_full_confirm_count >= cls_confirm)
        if (
            aeb_target_valid and closing_speed > 0.8
            and dist <= ttc_gate_dist and ttc <= ttc_full_thr
        ):
            memory.aeb_full_confirm_count = min(
                memory.aeb_full_confirm_count + 1,
                cls_confirm,
            )
        elif (_aeb_full_active
              and aeb_target_valid and closing_speed > 0.8
              and dist <= ttc_gate_dist and ttc <= ttc_full_thr * 1.15):
            # 已确认全制动 + TTC 在阈值 115% 内：维持计数，不增不减
            pass
        else:
            # 非对称退出：触发条件不再满足时一次衰减 2 拍，避免边界抖动锁定 AEB
            memory.aeb_full_confirm_count = max(memory.aeb_full_confirm_count - 2, 0)

        # 服务制动：TTC 在阈值内渐进步进
        if (
            aeb_target_valid and closing_speed > 0.8
            and dist <= ttc_gate_dist and ttc < ttc_brake_start
        ):
            service_ratio = clamp(
                (ttc_brake_start - ttc) / (ttc_brake_start - ttc_full_thr + 1e-6),
                0.0,
                1.0,
            )
            lon_cmd = max(lon_cmd, service_ratio * ACC_NORMAL_BRAKE_MAX)

        # 完整 AEB 判定
        if aeb_target_valid:
            lon_cmd, aeb_active = apply_aeb(
                dist,
                ttc,
                lon_cmd,
                min_safe_dist,
                memory.filtered_curv,
                closing=closing_speed,
                full_confirmed=(memory.aeb_full_confirm_count >= cls_confirm),
                ttc_class_mult=cls_mult,
                max_engage_dist=cls_engage_dist,
            )

        # ── ML 辅助（可选） ──
        # ML 预测的加速度作为 PID 输出的 soft blend，不替代规则层。
        # 混合权重 0.15：ML 占比小，主要起"提前预警"作用。
        if ml_result is not None and not aeb_active:
            ml_lon = ml_result.acc_pred
            if is_finite(ml_lon):
                blend = 0.15
                lon_cmd = lon_cmd * (1.0 - blend) + ml_lon * blend
            # ML AEB emergency 且规则层尚未全制动 → 加速确认计数
            if ml_result.aeb_class >= 2 and memory.aeb_full_confirm_count < cls_confirm:
                memory.aeb_full_confirm_count = min(
                    memory.aeb_full_confirm_count + 1,
                    cls_confirm,
                )

        # ── 非 AEB 时的纵向限制 ──
        if not aeb_active:
            lon_cmd = limit_non_aeb_lon(
                lon_cmd,
                LongitudinalContext(
                    dist=dist,
                    min_safe_dist=min_safe_dist,
                    lead_v_proj=lead_v_proj,
                    closing_speed=closing_speed,
                ),
                acc_has_lead=True,
                ego_v=signals.ego_v,
                raw_lead_v_proj=raw_lead_v_proj,
            )

    # ══════════════════════════════════════════════════════
    # 模式 2：弯道保持模式（弯道中丢失前车）
    # ══════════════════════════════════════════════════════
    elif in_curve_hold:
        memory.filtered_lead_accel = 0.0
        managers.lon_ctrl.reset()
        if getattr(managers, 'comfort_layer', None) is not None:
            managers.comfort_layer.reset(memory.filtered_lon)
        memory.filtered_v_tgt += VTGT_FILTER_ALPHA * (signals.ego_v - memory.filtered_v_tgt)
        v_hold = curve_hold_state.v_target
        v_err = signals.ego_v - v_hold
        p_term = CURVE_HOLD_SPEED_KP * v_err
        curve_hold_state.v_i = clamp(
            curve_hold_state.v_i + v_err * memory.dt,
            -CURVE_HOLD_I_MAX / max(CURVE_HOLD_SPEED_KI, 1e-6),
            CURVE_HOLD_I_MAX / max(CURVE_HOLD_SPEED_KI, 1e-6),
        )
        i_term = CURVE_HOLD_SPEED_KI * curve_hold_state.v_i
        lon_cmd = clamp(p_term + i_term, -0.5, LON_CMD_MAX_BRAKE_DECEL)
        memory.filtered_lon = lon_cmd

    # ══════════════════════════════════════════════════════
    # 模式 3：巡航模式（无前车、无弯道保持）
    # ══════════════════════════════════════════════════════
    else:
        memory.filtered_lead_accel = 0.0
        managers.lon_ctrl.reset()
        if getattr(managers, 'comfort_layer', None) is not None:
            managers.comfort_layer.reset(memory.filtered_lon)
        memory.filtered_v_tgt += VTGT_FILTER_ALPHA * (signals.ego_v - memory.filtered_v_tgt)

        # 前车丢失后的保护期
        acc_lead_loss_guard = (now - lead_state.last_acc_lead_valid_t) <= LEAD_LOSS_COAST_S
        cruise_drive_guard = acc_lead_loss_guard or lateral_ctx.in_curve or lead_ctx.recent_curve_exit

        # 目标速度取驾驶设定速度、系统上限和道路限速的较小值
        v_tgt = min(DRIVER_SET_SPEED, SYSTEM_MAX_CRUISE, ROAD_LIMIT_SPEED)
        # 弯道时根据侧向加速度限制目标速度
        v_curve_max = math.sqrt(CORNERING_MAX_LAT_ACCEL / max(lateral_ctx.curv_guard, 1e-6))
        v_tgt = min(v_tgt, v_curve_max)

        # ── 超车抑制旁路 ──
        # 上层把 lead_ctx.acc_has_lead 改成了 False（pipeline 里的 _replace），
        # 但巡航分支保留了多重"前车刚丢失/前车记忆未过期"保护，会把 raw_lon 钳零
        # 甚至轻微制动，导致车在停止前车前 10m 怎么也起不来。检测到"超车主动
        # 抑制"就完全跳过这些保护，直接用纯巡航 P 控制，保证有油门可加速绕过。
        if getattr(memory, 'suppress_lead_for_overtake', False):
            raw_lon = clamp(
                CRUISE_KP * (signals.ego_v - OVT_CRUISE_TARGET_V),
                -OVT_CRUISE_DRIVE_ACCEL,
                ACC_COMFORT_DECEL,
            )
            memory.filtered_lon += LON_FILTER_ALPHA * (raw_lon - memory.filtered_lon)
            return LongitudinalContext(
                lon_cmd=memory.filtered_lon,
                aeb_active=False,
                dist=999.99,
                ttc=float('inf'),
                lead_v_proj=0.0,
                min_safe_dist=0.0,
            )

        # ── AEB 告警保持模式 ──
        if alert_state.active:
            if signals.ego_v < 0.3 and now >= alert_state.stop_hold_until:
                logging.info('[AEB_ALERT] exit: standstill hold done, resume cruise')
                alert_state.active = False
                alert_state.has_lead = False
                alert_state.hold_speed = 0.0
                alert_state.cooldown_until = now + 2.0
                alert_state.stop_hold_until = 0.0
            if alert_state.active:
                v_hold = max(alert_state.hold_speed, 0.0)
                v_tgt = min(v_tgt, v_hold)
                raw_lon = clamp(
                    CRUISE_KP * (signals.ego_v - v_tgt),
                    -ACC_COMFORT_ACCEL * 0.2,
                    ACC_COMFORT_DECEL,
                )
                if cruise_drive_guard:
                    raw_lon = max(raw_lon, 0.0)
            else:
                raw_lon = clamp(
                    CRUISE_KP * (signals.ego_v - v_tgt),
                    -ACC_COMFORT_ACCEL,
                    ACC_COMFORT_DECEL,
                )
                if cruise_drive_guard:
                    raw_lon = max(raw_lon, 0.0)
        else:
            # ── 正常巡航 ──
            if (now - lead_state.last_confirmed_lead_t) <= LEAD_LOSS_COAST_S:
                # 前车刚丢失后的过渡期：限制制动力度
                raw_lon = clamp(
                    CRUISE_KP * (signals.ego_v - v_tgt),
                    -ACC_COMFORT_ACCEL * 0.3,
                    LON_CMD_MAX_BRAKE_DECEL,
                )
                raw_lon = clamp(raw_lon, -ACC_COMFORT_ACCEL * 0.3, ACC_COMFORT_DECEL)
            else:
                raw_lon = clamp(
                    CRUISE_KP * (signals.ego_v - v_tgt),
                    -LON_CMD_MAX_DRIVE_ACCEL,
                    LON_CMD_MAX_BRAKE_DECEL,
                )
                raw_lon = clamp(raw_lon, -ACC_COMFORT_ACCEL, ACC_COMFORT_DECEL)
            # 保护期内不产生负向加速度
            if cruise_drive_guard:
                raw_lon = max(raw_lon, 0.0)

        # ── 本周期刚丢失前车时重置平滑器 ──
        if lead_ctx.acc_lost_this_cycle and cruise_drive_guard:
            cleared_lon = max(raw_lon, 0.0)
            memory.filtered_lon = cleared_lon
            managers.lon_smooth.reset(value=cleared_lon)
            if getattr(managers, 'comfort_layer', None) is not None:
                managers.comfort_layer.reset(cleared_lon)

        # 低通滤波
        memory.filtered_lon += LON_FILTER_ALPHA * (raw_lon - memory.filtered_lon)
        lon_cmd = memory.filtered_lon

    return LongitudinalContext(
        lon_cmd=lon_cmd,
        aeb_active=aeb_active,
        dist=dist,
        ttc=ttc,
        lead_v_proj=lead_v_proj,
        min_safe_dist=min_safe_dist,
        lead_acquire_grace_active=lead_acquire_grace_active,
        acc_ff_before=acc_ff_before,
        acc_ff_after=acc_ff_after,
        closing_speed=closing_speed,
    )
