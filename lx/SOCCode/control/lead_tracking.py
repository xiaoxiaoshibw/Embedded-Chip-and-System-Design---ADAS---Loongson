#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""前车检测与 ACC 门控策略。

负责：
  1. 将前车全局坐标投影到自车坐标系，得到纵向/横向相对位置
  2. 对相对位置做低通滤波降噪
  3. 根据曲率和车道宽度动态调整前车横向检测窗口
  4. 多级确认机制：原始检测 → 确认计数 → ACC 可用
  5. 弯道记忆与保活：短暂丢失时保留前车状态
  6. ACC 释放条件判定：偏出车道、远离、速度骤降等
"""

import logging
import math

from common import is_finite
from config import *
from lateral import (
    compute_curve_hold_window,
    compute_lead_lateral_window,
    compute_relative_in_ego_frame,
)
from longitudinal import compute_ttc

from control.state import LeadContext, LeadTrackerState, LeadTrackingInputs


class LeadTracker:
    """管理前车跟踪状态并评估 ACC 前车可用性。"""

    def __init__(self):
        self.state = LeadTrackerState()

    def reset_on_lead_swap(self):
        """多目标航迹切换主前车时调用：清除与"上一个目标"绑定的内部状态。

        必须清的：
          - rel_filter_primed：让下一拍走 priming 分支用新目标首帧初始化滤波器，
            避免把旧目标的滤波器残值串接到新目标，造成 0.2~0.5s 虚假趋势；
          - lead_confirm_count：重新走确认流程，否则旧目标的"已确认"会被错给新目标；
          - acc_lane_out_release_count / acc_dist_opening_release_count：
            释放判定的"连续计数"对新目标无意义；
          - prev_abs_y_rel / cutin_lat_rate：切入横向速率与旧目标轨迹绑定。

        不动 filtered_x_rel / filtered_y_rel / filtered_v_proj：它们会被
        priming 分支在下一拍直接覆盖；这里清不清都等价。
        """
        s = self.state
        s.rel_filter_primed = False
        s.lead_confirm_count = 0
        s.acc_lane_out_release_count = 0
        s.acc_dist_opening_release_count = 0
        s.prev_abs_y_rel = -1.0
        s.prev_y_rel_t = -1e9
        s.cutin_lat_rate = 0.0
        s.prev_acc_eval_dist = None
        s.prev_acc_eval_lead_v_proj = 0.0

    def evaluate(self, now: float, inputs: LeadTrackingInputs, in_curve: bool) -> LeadContext:
        """每周期调用，综合判断前车是否可用并返回完整上下文。

        参数:
            now: 单调时钟
            inputs: 前车跟踪所需输入数据
            in_curve: 是否在弯道中

        返回:
            LeadContext 包含前车位置、确认状态、ACC 门控结果等
        """
        state = self.state
        x_rel = y_rel = 0.0

        # ── 1. 前车数据新鲜度检查 ──
        lead_fresh = (
            inputs.lead_received
            and (now - inputs.lead_last_rx_time) <= LEAD_TIMEOUT_S
        )
        # 首次检测到前车（或超时丢失后重获）时，相对位置/投影速度滤波器直接以
        # 首帧测量值初始化，避免从 0 低通爬升造成数拍内 dist 偏小、v_proj 偏低，
        # 进而被 AEB 误判为"高接近率 + 距离不足"而虚假触发。
        priming = lead_fresh and not state.rel_filter_primed
        if not lead_fresh:
            state.rel_filter_primed = False
        if lead_fresh:
            # 将全局坐标投影到自车坐标系
            x_rel, y_rel = compute_relative_in_ego_frame(
                inputs.lead_x, inputs.lead_y, inputs.ego_x, inputs.ego_y, inputs.ego_yaw
            )
            if priming:
                state.filtered_x_rel = x_rel
                state.filtered_y_rel = y_rel
            else:
                # 对相对位置做低通滤波降噪
                state.filtered_x_rel += LEAD_REL_FILTER_ALPHA * (x_rel - state.filtered_x_rel)
                state.filtered_y_rel += LEAD_REL_FILTER_ALPHA * (y_rel - state.filtered_y_rel)
            state.rel_filter_primed = True
            x_rel = state.filtered_x_rel
            y_rel = state.filtered_y_rel

        # ── 2. 根据曲率和车道宽度动态调整横向检测窗口 ──
        lead_lat_max, lead_lat_straight, lead_lat_curve = compute_lead_lateral_window(
            inputs.filtered_curv, inputs.cur_lane_width
        )
        lead_lat_gate = lead_lat_max * TTC_AEB_MAX_LAT_RATIO
        if abs(inputs.filtered_curv) > CURV_LEAD_THRESH:
            lead_lat_gate = max(
                lead_lat_gate,
                min(inputs.cur_lane_width * 0.5, LEAD_LAT_MAX_STRAIGHT_MAX) * 0.85,
            )
        # 按 class 放宽 AEB 横向门：行人横穿/障碍在车道边缘时车辆门会拒掉 AEB。
        # 只乘到 lead_lat_gate 上，下方 in_lane / cutin_predicted 用的是
        # lead_lat_max / corridor，与 ACC 跟车判定无关，避免误干扰正常跟车。
        # 默认 UNKNOWN/VEHICLE 乘子 1.0，行为与改造前逐字节一致。
        lead_lat_gate *= AEB_CLASS_LAT_GATE_MULT.get(inputs.lead_cls, 1.0)

        # ── 2.5 切入预判 ──
        # 相邻走廊内、横向持续向本车道逼近、且预测时域内会进入横向窗口的目标，
        # 提前视为"在道内候选"，让 ACC 早一步接管减速。AEB 的 lead_lat_gate
        # 不受此影响——只有目标真正进入窄门限后 AEB 才会介入，保持保守。
        cutin_predicted = False
        if lead_fresh:
            cur_abs_y = abs(y_rel)
            dt_y = now - state.prev_y_rel_t
            if state.prev_abs_y_rel >= 0.0 and 1e-3 < dt_y < 1.0:
                raw_lat_rate = (state.prev_abs_y_rel - cur_abs_y) / dt_y
                state.cutin_lat_rate += CUTIN_LAT_RATE_ALPHA * (
                    raw_lat_rate - state.cutin_lat_rate)
            state.prev_abs_y_rel = cur_abs_y
            state.prev_y_rel_t = now
            if (x_rel > 0.0 and x_rel <= LEAD_MAX_TRACK_DIST
                    and cur_abs_y > lead_lat_max):
                corridor = lead_lat_max * CUTIN_CORRIDOR_RATIO
                predicted_abs_y = cur_abs_y - state.cutin_lat_rate * CUTIN_HORIZON_S
                cutin_predicted = (
                    cur_abs_y <= corridor
                    and state.cutin_lat_rate >= CUTIN_MIN_LAT_RATE
                    and predicted_abs_y <= lead_lat_max
                )
        else:
            state.prev_abs_y_rel = -1.0
            state.cutin_lat_rate = 0.0

        # ── 3. 原始前车判定：前方有物体且在横向窗口内（或预判切入） ──
        raw_has_lead = (
            lead_fresh
            and x_rel > 0.0
            and x_rel <= LEAD_MAX_TRACK_DIST
            and (abs(y_rel) <= lead_lat_max or cutin_predicted)
        )
        # 连续确认计数：检测到加 1，丢失减 1
        if raw_has_lead:
            state.lead_confirm_count = min(state.lead_confirm_count + 1, LEAD_CONFIRM_CYCLES)
            state.last_confirmed_lead_t = now
            state.last_lead_x_rel = x_rel
            state.last_lead_y_rel = y_rel
        else:
            state.lead_confirm_count = max(state.lead_confirm_count - 1, 0)

        # ── 4. 弯道记忆前车：最近确认过且仍在弯道横向窗口内 ──
        curve_hold_lat = compute_curve_hold_window(inputs.cur_lane_width)
        curve_memory_has_lead = (
            in_curve
            and (now - state.last_confirmed_lead_t) <= LEAD_MEMORY_S
            and state.last_lead_x_rel > 0.0
            and state.last_lead_x_rel <= LEAD_MAX_TRACK_DIST
            and abs(state.last_lead_y_rel) <= curve_hold_lat
        )
        # 弯道跟踪：当前检测或记忆中的前车
        curve_track_has_lead = (
            in_curve
            and (
                (
                    lead_fresh
                    and x_rel > 0.0
                    and x_rel <= LEAD_MAX_TRACK_DIST
                    and abs(y_rel) <= curve_hold_lat
                )
                or curve_memory_has_lead
            )
            and (now - state.last_confirmed_lead_t) <= LEAD_CURVE_LOSS_HOLD_S
        )

        # ── 5. 最终前车存在判定 ──
        has_lead = (
            (raw_has_lead and state.lead_confirm_count >= LEAD_CONFIRM_CYCLES)
            or curve_track_has_lead
        )
        # 短暂丢失保活
        if (not has_lead) and ((now - state.last_confirmed_lead_t) <= LEAD_KEEPALIVE_S):
            has_lead = True

        # ── 6. AEB 告警和 ACC 的前车有效性 ──
        # 提前计算 acc_eval_dist 和 lead_in_lane_for_acc，
        # 因为下面 slow_lead_acc_ok 的判定要用它们。
        acc_eval_dist = x_rel if raw_has_lead else state.last_lead_x_rel
        # 切入预判命中时，ACC 车道内判定一并放宽（AEB 的 lead_lat_gate 不放宽）
        lead_in_lane_for_acc = raw_has_lead and (
            abs(y_rel) <= lead_lat_gate or cutin_predicted)

        lead_speed_valid_for_alert = lead_fresh and inputs.lead_v >= AEB_ALERT_ARM_MIN_LEAD_V
        lead_speed_invalid_for_alert = lead_fresh and inputs.lead_v < AEB_ALERT_INVALID_LEAD_V
        lead_valid_for_alert = (
            raw_has_lead
            and lead_speed_valid_for_alert
            and x_rel > 0.0
            and x_rel <= LEAD_MAX_TRACK_DIST
            and state.lead_confirm_count >= AEB_ALERT_ARM_CONFIRM_CYCLES
        )
        lead_speed_valid_for_acc = (
            lead_fresh
            and inputs.lead_v >= AEB_ALERT_INVALID_LEAD_V
            and (inputs.lead_v >= ACC_MIN_VALID_LEAD_V or x_rel <= ACC_CLOSE_SLOW_LEAD_DIST)
        )
        # 近距离已确认前车即使速度极低（接近停止）也保持 ACC 跟踪，
        # 避免前车停下后 ACC 释放导致自车继续前行。
        slow_lead_acc_ok = (
            raw_has_lead
            and lead_fresh
            and inputs.lead_v < AEB_ALERT_INVALID_LEAD_V
            and state.lead_confirm_count >= LEAD_CONFIRM_CYCLES
            and acc_eval_dist <= ACC_CLOSE_SLOW_LEAD_DIST
            and lead_in_lane_for_acc
        )
        base_acc_lead_ok = (
            raw_has_lead
            and (lead_speed_valid_for_acc or slow_lead_acc_ok)
            and state.lead_confirm_count >= LEAD_CONFIRM_CYCLES
        )

        # ── 7. 前车投影速度计算与异常修正 ──
        acc_reject_reason = ''
        acc_release_reason = ''
        acc_lead_valid = False
        raw_lead_v_proj = (
            max(0.0, inputs.lead_v * math.cos(inputs.lead_yaw - inputs.ego_yaw))
            if lead_fresh else inputs.last_lead_v_proj
        )
        recent_curve_exit = (now - inputs.last_curve_t) <= CURVE_EXIT_GRACE_S
        recent_reacq = (now - inputs.last_lead_reacq_t) <= LEAD_REACQ_PROTECT_S

        # 弯道/重获前车保护期内，保证投影速度不低于阈值
        if lead_fresh and (in_curve or recent_curve_exit or recent_reacq):
            raw_lead_v_proj = max(
                raw_lead_v_proj,
                inputs.lead_v * LEAD_REACQ_MIN_PROJ_RATIO,
                inputs.last_lead_v_proj * LEAD_REACQ_MIN_LAST_RATIO,
            )
        # 速度跳变检测：远处前车速度突然大幅下降时做混合修正
        if (
            acc_eval_dist > LEAD_DROP_GLITCH_DIST
            and raw_lead_v_proj < inputs.last_lead_v_proj * LEAD_DROP_GLITCH_RATIO
            and inputs.last_lead_v_proj > 1.0
        ):
            raw_lead_v_proj = inputs.last_lead_v_proj * LEAD_DROP_GLITCH_BLEND

        # 滤波后的前车投影速度（滤波器状态由跟踪器自持，首帧直接以测量值初始化）
        if priming:
            state.filtered_v_proj = raw_lead_v_proj
        else:
            state.filtered_v_proj += LEAD_V_PROJ_FILTER_ALPHA * (
                raw_lead_v_proj - state.filtered_v_proj)
        predicted_lead_v_proj = state.filtered_v_proj

        # ── 8. TTC 计算 ──
        # acc_eval_dist 为 0 时（last_lead_x_rel 初始值）不计算 TTC，
        # 避免 compute_ttc 返回 0.0 被误判为碰撞。
        acc_ttc = (
            compute_ttc(
                acc_eval_dist,
                inputs.ego_v,
                inputs.lead_v,
                inputs.lead_yaw,
                inputs.ego_yaw,
                predicted_lead_v_proj,
            )
            if acc_eval_dist > 0.5 else float('inf')
        )

        # ── 9. ACC 车道内判定（已在第 6 步提前计算 lead_in_lane_for_acc） ──

        # ── 10. 前车远离 / 速度骤降检测 ──
        prev_acc_eval_dist = state.prev_acc_eval_dist
        prev_acc_eval_lead_v_proj = state.prev_acc_eval_lead_v_proj
        dist_increasing = (
            prev_acc_eval_dist is not None
            and raw_has_lead
            and acc_eval_dist >= ACC_LEAD_OPENING_MIN_DIST
            and acc_eval_dist > (prev_acc_eval_dist + ACC_LEAD_OPENING_DELTA_M)
        )
        lead_speed_collapsing = (
            raw_has_lead
            and predicted_lead_v_proj < max(0.5, prev_acc_eval_lead_v_proj * ACC_LEAD_COLLAPSE_RATIO)
            and predicted_lead_v_proj < max(0.0, inputs.ego_v - ACC_LEAD_COLLAPSE_EGO_GAP)
        )
        opening_and_collapsing = dist_increasing and lead_speed_collapsing
        ttc_too_large = (
            raw_has_lead
            and acc_eval_dist > ACC_CLOSE_SLOW_LEAD_DIST
            and is_finite(acc_ttc)
            and acc_ttc > ACC_MAX_VALID_TTC_S
        )

        # 更新上一拍距离和速度记录
        if raw_has_lead:
            state.prev_acc_eval_dist = acc_eval_dist
            state.prev_acc_eval_lead_v_proj = predicted_lead_v_proj
        else:
            state.prev_acc_eval_dist = None
            state.prev_acc_eval_lead_v_proj = 0.0

        # ── 11. ACC 偏出车道 / 远离释放计数 ──
        if inputs.last_acc_has_lead:
            if raw_has_lead and not lead_in_lane_for_acc:
                state.acc_lane_out_release_count = min(
                    state.acc_lane_out_release_count + 1,
                    ACC_LEAD_RELEASE_LANE_OUT_CYCLES,
                )
            else:
                state.acc_lane_out_release_count = 0
            if raw_has_lead and dist_increasing:
                state.acc_dist_opening_release_count = min(
                    state.acc_dist_opening_release_count + 1,
                    ACC_LEAD_RELEASE_OPENING_CYCLES,
                )
            else:
                state.acc_dist_opening_release_count = 0
        else:
            state.acc_lane_out_release_count = 0
            state.acc_dist_opening_release_count = 0

        lane_out_release = (
            inputs.last_acc_has_lead
            and state.acc_lane_out_release_count >= ACC_LEAD_RELEASE_LANE_OUT_CYCLES
        )
        dist_opening_release = (
            inputs.last_acc_has_lead
            and state.acc_dist_opening_release_count >= ACC_LEAD_RELEASE_OPENING_CYCLES
        )

        # ── 12. ACC 前车有效性门控 ──
        if not inputs.last_acc_has_lead:
            # 上周期无前车：需要更严格条件
            if not raw_has_lead:
                acc_reject_reason = 'lead_missing'
            elif state.lead_confirm_count < LEAD_CONFIRM_CYCLES:
                acc_reject_reason = 'confirm_pending'
            elif not lead_in_lane_for_acc:
                acc_reject_reason = 'lane_out'
            elif ttc_too_large:
                acc_reject_reason = 'ttc_far'
            elif dist_increasing:
                acc_reject_reason = 'dist_opening'
            elif not (lead_speed_valid_for_acc or slow_lead_acc_ok):
                acc_reject_reason = 'lead_speed_low'
            else:
                acc_lead_valid = base_acc_lead_ok
        else:
            # 上周期有前车：允许更大容差，但检测特定释放条件
            if not raw_has_lead:
                acc_reject_reason = 'lead_missing'
            elif opening_and_collapsing:
                acc_reject_reason = 'speed_collapse_release'
                acc_release_reason = acc_reject_reason
            elif lane_out_release:
                acc_reject_reason = 'lane_out'
                acc_release_reason = acc_reject_reason
            elif dist_opening_release:
                acc_reject_reason = 'dist_opening'
                acc_release_reason = acc_reject_reason
            elif not (lead_speed_valid_for_acc or slow_lead_acc_ok):
                acc_reject_reason = 'lead_speed_low'
            else:
                acc_lead_valid = base_acc_lead_ok

        # 弯道内的近距离低速前车回退判定
        curve_fallback_acc_ok = (
            curve_track_has_lead
            and not acc_reject_reason
            and state.last_lead_x_rel > 0.0
            and state.last_lead_x_rel <= ACC_CLOSE_SLOW_LEAD_DIST
            and inputs.last_lead_v_proj >= ACC_MIN_VALID_LEAD_V
        )
        if (not acc_lead_valid) and curve_fallback_acc_ok:
            acc_lead_valid = True

        # 前车保活：刚丢失短时间内仍然保持 ACC
        if acc_lead_valid:
            state.last_acc_lead_valid_t = now
        elif (
            inputs.last_acc_has_lead
            and acc_reject_reason in ('lead_missing', 'confirm_pending')
            and (now - state.last_acc_lead_valid_t) <= ACC_LEAD_KEEPALIVE_S
        ):
            acc_lead_valid = True

        logging.debug(
            '[ACC_LEAD_FINAL] base_ok=%s final_ok=%s reject=%s release=%s raw_has_lead=%s lk=%s dist=%.2f ttc=%.2f lv=%.2f',
            base_acc_lead_ok,
            acc_lead_valid,
            acc_reject_reason or 'none',
            acc_release_reason or 'none',
            raw_has_lead,
            inputs.lane_locked,
            acc_eval_dist,
            acc_ttc,
            predicted_lead_v_proj,
        )

        acc_has_lead = acc_lead_valid
        acc_lost_this_cycle = inputs.last_acc_has_lead and not acc_has_lead

        # 记录拒绝/释放原因日志
        if acc_lead_valid:
            state.last_acc_reject_reason = ''
            state.last_acc_release_reason = ''
        else:
            if acc_reject_reason and acc_reject_reason != state.last_acc_reject_reason:
                logging.info(
                    '[ACC_LEAD] reject: reason=%s dist=%.2f y=%.2f ttc=%.2f ego_v=%.2f lead_v=%.2f detected=%s',
                    acc_reject_reason, acc_eval_dist, y_rel, acc_ttc,
                    inputs.ego_v, predicted_lead_v_proj, has_lead
                )
                state.last_acc_reject_reason = acc_reject_reason
            if (
                inputs.last_acc_has_lead
                and acc_reject_reason
                and acc_reject_reason != state.last_acc_release_reason
            ):
                logging.warning(
                    '[ACC_LEAD] release: reason=%s dist=%.2f d_prev=%.2f ttc=%.2f ego_v=%.2f lead_v=%.2f lane_out_cnt=%d dist_open_cnt=%d',
                    acc_release_reason or acc_reject_reason,
                    acc_eval_dist,
                    prev_acc_eval_dist if prev_acc_eval_dist is not None else -1.0,
                    acc_ttc,
                    inputs.ego_v,
                    predicted_lead_v_proj,
                    state.acc_lane_out_release_count,
                    state.acc_dist_opening_release_count,
                )
                state.last_acc_release_reason = acc_reject_reason

        # 前车重获日志
        lead_acquired = acc_has_lead and not inputs.last_acc_has_lead
        if lead_acquired:
            logging.info(
                '[ACC_LEAD] acquire: dist=%.2f y=%.2f ttc=%.2f ego_v=%.2f lead_v=%.2f confirm=%d',
                acc_eval_dist, y_rel, acc_ttc, inputs.ego_v, predicted_lead_v_proj,
                state.lead_confirm_count
            )

        return LeadContext(
            x_rel=x_rel,
            y_rel=y_rel,
            lead_fresh=lead_fresh,
            lead_lat_max=lead_lat_max,
            lead_lat_straight=lead_lat_straight,
            lead_lat_curve=lead_lat_curve,
            lead_lat_gate=lead_lat_gate,
            raw_has_lead=raw_has_lead,
            has_lead=has_lead,
            lead_detected=has_lead,
            lead_valid_for_alert=lead_valid_for_alert,
            lead_speed_invalid_for_alert=lead_speed_invalid_for_alert,
            acc_has_lead=acc_has_lead,
            acc_lead_valid=acc_lead_valid,
            acc_lost_this_cycle=acc_lost_this_cycle,
            acc_reject_reason=acc_reject_reason,
            lead_in_lane_for_acc=lead_in_lane_for_acc,
            predicted_lead_v_proj=predicted_lead_v_proj,
            lane_out_release=lane_out_release,
            dist_opening_release=dist_opening_release,
            acc_ff_before=0.0,
            acc_ff_after=0.0,
            acc_eval_dist=acc_eval_dist,
            acc_ttc=acc_ttc,
            raw_lead_v_proj=raw_lead_v_proj,
            recent_curve_exit=recent_curve_exit,
            recent_reacq=recent_reacq,
            lead_acquired=lead_acquired,
            lead_cls=inputs.lead_cls,
        )
