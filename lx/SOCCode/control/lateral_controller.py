#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""横向控制器：单周期横向（方向盘）指令计算。

数据流：
  1. 对道路航向做低通滤波，得到滤波航向与转向率
  2. 计算预览航向、曲率、前馈转角
  3. 对 CTE 做低通滤波与微分，计算 CTE 修正转角
  4. 航向 PID 控制 + CTE 修正 + 曲率前馈 → 原始转角
  5. 转角变化率限制
  6. 边界修正叠加
  7. 返回 LateralContext
"""

import math
from dataclasses import replace

from common import clamp, wrap_angle
from config import (
    BOUNDARY_DELTA_RATE_MULT,
    CORNERING_RRATE_THRESH,
    CTE_EFFECTIVE_LIMIT,
    CTE_FILTER_ALPHA,
    CTRL_DT_MAX,
    CTRL_DT_MIN,
    CURV_FILTER_ALPHA,
    CURVE_CTE_BOOST_MAX,
    CURVE_CTE_BOOST_SCALE,
    CURVE_FF_ATTEN_MAX,
    CURVE_FF_ATTEN_SCALE,
    CURV_NO_ACCEL_THRESH,
    I_DECAY_IN_CORNER,
    K_CTE,
    K_CTE_D,
    K_DELTA,
    K_FF_CURV,
    K_PREVIEW_GAIN,
    K_PSI_D,
    K_PSI_I,
    K_PSI_P,
    MAX_CTE_CORR,
    MAX_CTE_DOT,
    MAX_DELTA,
    MAX_DELTA_RATE,
    MAX_FF_DELTA,
    MAX_PSI_D,
    MAX_PSI_ERR,
    MAX_PSI_I,
    PSI_I_LOW_SPEED_DECAY,
    PSI_I_LOW_SPEED_GATE,
    ROAD_PSI_FILTER_ALPHA,
    STEER_SIGN,
    WHEEL_BASE,
)
from lateral import adaptive_preview_time, compute_boundary_correction

from control.context import ControlMemory, LateralContext, VehicleSignals


def _apply_boundary(base_ctx: LateralContext,
                    base_delta: float,
                    dt: float,
                    signals: VehicleSignals,
                    memory: ControlMemory) -> LateralContext:
    """在"未叠加边界"的 base_ctx 上叠加车道边界修正，返回完整 LateralContext。

    边界修正只依赖当前 lane_offset 与车道余量（compute_boundary_correction
    本身无状态），因此可脱离 road_psi 帧门控，按最新 lane_offset 单独刷新。
    会就地写入 memory.last_delta。dt 沿用所属道路帧的真实 dt，保证边界限幅
    与 upd_psi 的时间基准和叠加前一致。
    """
    target_off = float(getattr(memory, 'target_lane_offset', 0.0) or 0.0)
    cur_off = signals.lane_offset if signals.lane_offset_received else 0.0
    # 双车道：边界余量按"相对目标车道中心"计算（见原步骤 8 说明）。
    boundary_off = cur_off - target_off
    in_transition = (
        abs(target_off) > 0.1
        or abs(cur_off) > max(memory.lane_warn_margin, 0.6)
    )
    if in_transition:
        boundary_delta = 0.0
        boundary_brake = 0.0
        boundary_warn = False
    else:
        boundary_delta, boundary_brake, boundary_warn = compute_boundary_correction(
            boundary_off,
            base_delta,
            signals.ego_v,
            memory.lane_safe_margin,
            memory.lane_warn_margin,
            memory.lane_hard_margin,
        )
    # 给边界修正一个独立的、更宽松的 rate limit，避免瞬间打满方向盘
    boundary_max_step = MAX_DELTA_RATE * BOUNDARY_DELTA_RATE_MULT * dt
    boundary_delta = clamp(boundary_delta, -boundary_max_step, boundary_max_step)
    delta = clamp(base_delta + boundary_delta, -MAX_DELTA, MAX_DELTA)
    memory.last_delta = delta

    # 预测下一拍航向（用于 ESP32 回读校验）
    yaw_rate = signals.ego_v / WHEEL_BASE * math.tan(delta)
    upd_psi = wrap_angle(signals.ego_yaw + yaw_rate * dt)

    return replace(
        base_ctx,
        delta=delta,
        boundary_delta=boundary_delta,
        boundary_brake=boundary_brake,
        boundary_warn=boundary_warn,
        cur_off=cur_off,
        upd_psi=upd_psi,
    )


def compute_lateral_command(now: float,
                            signals: VehicleSignals,
                            memory: ControlMemory,
                            lateral_model=None) -> LateralContext:
    """计算单周期横向控制指令。

    参数:
        now: 单调时钟
        signals: 本周期感知信号
        memory: 跨周期记忆状态（会被就地更新）

    返回:
        LateralContext 包含所有横向相关输出
    """
    # ── 0. 帧门控 + 真实 dt ──
    # 仿真端以 20Hz 发布感知数据，而本循环以 100Hz 运行：若每拍都推进滤波/微分/
    # 积分，会把 50ms 的信号跳变当成 10ms 处理，使微分项放大 5 倍并产生脉冲串，
    # 进而引发航向过冲与弯道发散。因此仅在道路航向有新帧时推进有状态计算，
    # 其余拍直接沿用上一帧结果（仿真端本就以 20Hz 读取 delta）。
    if (memory.lat_cached_ctx is not None
            and signals.road_last_rx == memory.lat_last_road_rx):
        # 道路航向无新帧：滤波/PID 等有状态计算沿用上一帧。
        if signals.lane_offset_last_rx == memory.lat_last_lane_rx:
            return memory.lat_cached_ctx
        # 但车道偏移有新帧 → 仅用最新 lane_offset 重算边界修正，
        # 不让边界制动/OFFSET 字段被 road_psi 帧率拖慢。
        final_ctx = _apply_boundary(
            memory.lat_base_ctx, memory.lat_base_delta,
            memory.lat_frame_dt, signals, memory,
        )
        memory.lat_cached_ctx = final_ctx
        memory.lat_last_lane_rx = signals.lane_offset_last_rx
        return final_ctx

    first_update = memory.lat_last_update_t < 0.0
    if first_update:
        dt = clamp(memory.dt, CTRL_DT_MIN, CTRL_DT_MAX)
    else:
        dt = clamp(now - memory.lat_last_update_t, CTRL_DT_MIN, CTRL_DT_MAX)
    # 硬下限保护（H-04）：CTRL_DT_MIN 可通过 params.yaml 覆盖（H-03 白名单
    # 不包含它，因为正常降频到 50Hz 需要改 dt 下界）。但无论如何 dt 不能
    # 到达 0，否则后续除法 (rrate, cte_dot, psi_d) 会除零产生 Inf/NaN。
    # 1e-6s = 1μs，远低于任何合理的控制周期，仅作为最后防线。
    dt = max(dt, 1e-6)
    memory.lat_last_update_t = now
    memory.lat_last_road_rx = signals.road_last_rx

    # 低通系数按实际帧间隔重标定，保持与 100Hz 设计一致的时间常数（速率无关）：
    # 单次以 alpha_eff 滤波 == 以原 alpha 连续滤波 (dt/memory.dt) 次。
    rate_n = dt / memory.dt
    road_psi_alpha = 1.0 - (1.0 - ROAD_PSI_FILTER_ALPHA) ** rate_n
    cte_alpha = 1.0 - (1.0 - CTE_FILTER_ALPHA) ** rate_n
    curv_alpha = 1.0 - (1.0 - CURV_FILTER_ALPHA) ** rate_n

    # ── 1. 道路航向低通滤波 ──
    memory.filtered_road_psi = wrap_angle(
        memory.filtered_road_psi
        + road_psi_alpha * wrap_angle(signals.road_psi - memory.filtered_road_psi)
    )

    # ── 2. 自适应预览时间和转向率 ──
    dyn_prev = adaptive_preview_time(signals.ego_v, memory.filtered_curv)
    # 转向率 = 滤波航向的变化率
    # 首次更新时 prev_road_psi 尚未初始化，强制 rrate=0，避免虚假大曲率。
    if first_update:
        rrate = 0.0
        memory.prev_road_psi = memory.filtered_road_psi
    else:
        rrate = wrap_angle(memory.filtered_road_psi - memory.prev_road_psi) / dt
        memory.prev_road_psi = memory.filtered_road_psi

    # 预览航向 = 滤波航向 + 增益 × 转向率 × 预览时间
    prev_psi = wrap_angle(memory.filtered_road_psi + K_PREVIEW_GAIN * rrate * dyn_prev)

    # ── 3. 曲率估计与滤波 ──
    # 低速时曲率估计噪声被放大，增加速度衰减因子抑制虚假曲率
    ego_v_abs = abs(signals.ego_v)
    if ego_v_abs > 0.3:
        raw_curv = rrate / signals.ego_v
        # 低速区间 (0.3~2.0 m/s) 对原始曲率做线性衰减，防止噪声放大
        low_speed_atten = clamp((ego_v_abs - 0.3) / 1.7, 0.0, 1.0)
        raw_curv *= low_speed_atten
    else:
        raw_curv = 0.0
    memory.filtered_curv += curv_alpha * (raw_curv - memory.filtered_curv)
    # 取原始与滤波的较大绝对值作为保护值
    curv_guard = max(abs(memory.filtered_curv), abs(raw_curv))
    in_curve = curv_guard > CURV_NO_ACCEL_THRESH
    if in_curve:
        memory.last_curve_t = now

    # ── 4. 曲率前馈转角 ──
    if abs(signals.ego_v) > 0.3:
        ff_ratio = clamp(abs(memory.filtered_curv) / CURVE_FF_ATTEN_SCALE, 0.0, 1.0)
        ff_gain = K_FF_CURV * (1.0 - CURVE_FF_ATTEN_MAX * ff_ratio)
        delta_ff = clamp(
            ff_gain * math.atan(WHEEL_BASE * memory.filtered_curv),
            -MAX_FF_DELTA,
            MAX_FF_DELTA,
        )
    else:
        delta_ff = 0.0

    # ── 5. CTE 横向偏移修正 ──
    raw_cte = signals.lane_offset if signals.lane_offset_received else 0.0
    memory.filtered_cte += cte_alpha * (raw_cte - memory.filtered_cte)
    # 双车道目标偏移：超车状态机写入 memory.target_lane_offset 时，
    # 期望车辆跟踪的车道中心从右车道（target=0）平移到左车道（target>0）。
    # 把追踪误差改为 (filtered_cte - target_lane_offset)，控制器其它环节不变。
    target_off = float(getattr(memory, 'target_lane_offset', 0.0) or 0.0)
    cte_track_err = memory.filtered_cte - target_off
    cte_track_dot_ref = memory.cte_prev - target_off
    # 微分在未饱和值上做，避免饱和时 D 项假死；对称限幅防止 dt 极小时爆冲
    cte_dot = clamp((cte_track_err - cte_track_dot_ref) / dt, -MAX_CTE_DOT, MAX_CTE_DOT)
    memory.cte_prev = memory.filtered_cte
    # 限幅仅用于 P 项，避免过大修正
    cte_ctrl = clamp(cte_track_err, -CTE_EFFECTIVE_LIMIT, CTE_EFFECTIVE_LIMIT)
    # 低速时 CTE 修正增益按速度增大
    cte_speed_factor = clamp(abs(signals.ego_v) / 1.0, 0.0, 1.0)
    # 弯道内 CTE 增益增强
    cte_boost = 1.0 + CURVE_CTE_BOOST_MAX * clamp(
        abs(memory.filtered_curv) / CURVE_CTE_BOOST_SCALE,
        0.0,
        1.0,
    )
    # CTE 修正符号恒为 -1：仿真端 cte 在道路切线局部系内计算（已绕 psi_ref 旋转），
    # 正值恒表示车在路径左侧、与弯道方向无关，故修正符号不随曲率翻转。
    # 此前按曲率把符号翻成 +1 会在左弯把 CTE 修正变成正反馈，导致横向发散。
    cte_sign = -1.0
    delta_cte = clamp(
        cte_speed_factor * cte_boost * (cte_sign * K_CTE * cte_ctrl - cte_sign * K_CTE_D * cte_dot),
        -MAX_CTE_CORR,
        MAX_CTE_CORR,
    )

    # ── 6. 航向 PID 控制 ──
    gains = memory.gains
    psi_err = clamp(wrap_angle(prev_psi - signals.ego_yaw), -MAX_PSI_ERR, MAX_PSI_ERR)
    # 弯道中衰减 I 项防止积分累积过大
    if abs(rrate) > CORNERING_RRATE_THRESH:
        memory.psi_i_term *= I_DECAY_IN_CORNER
    # δ-饱和 anti-windup：若上一拍方向盘已贴近物理上限且当前误差仍朝同方向推，
    # 继续积分会让 I 项越界，脱离饱和后产生反向过冲。此种工况下跳过积分。
    saturated_prev = abs(memory.last_delta) >= MAX_DELTA * 0.98
    same_sign_drive = (memory.last_delta * STEER_SIGN * psi_err) > 0.0
    delta_windup = saturated_prev and same_sign_drive
    # 方向性条件积分：当误差方向与积分方向相反时（系统已在回正），
    # 加速释放积分，避免弯道→直道过渡时 I 项残留导致方向盘偏转。
    if abs(signals.ego_v) < PSI_I_LOW_SPEED_GATE:
        memory.psi_i_term *= PSI_I_LOW_SPEED_DECAY
    elif delta_windup:
        # 不仅冻结积分，还轻微衰减，把已经累积的部分慢慢放掉
        memory.psi_i_term *= I_DECAY_IN_CORNER
    else:
        # 误差方向与积分方向相反 → 快速释放积分（加速回正）
        if (psi_err * memory.psi_i_term) < 0.0:
            memory.psi_i_term *= 0.82
        memory.psi_i_term = clamp(memory.psi_i_term + psi_err * dt, -MAX_PSI_I, MAX_PSI_I)
    psi_d = clamp((psi_err - memory.psi_prev_err) / dt, -MAX_PSI_D, MAX_PSI_D)
    memory.psi_prev_err = psi_err
    # 目标航向 = 当前航向 + PID 输出
    psi_tgt = wrap_angle(
        signals.ego_yaw
        + gains.lat_kp * psi_err
        + gains.lat_ki * memory.psi_i_term
        + gains.lat_kd * psi_d
    )

    # ── 7. 转角合成与变化率限制 ──
    # 航向误差 → 转角
    dr = STEER_SIGN * clamp(K_DELTA * wrap_angle(psi_tgt - signals.ego_yaw), -MAX_DELTA, MAX_DELTA)
    # 叠加曲率前馈
    dr = clamp(dr + STEER_SIGN * delta_ff, -MAX_DELTA, MAX_DELTA)
    # 叠加 CTE 修正
    dr = clamp(dr + STEER_SIGN * delta_cte, -MAX_DELTA, MAX_DELTA)
    if lateral_model is not None:
        model_dr = lateral_model.compute(
            psi_err=psi_err,
            cte_err=cte_track_err,
            ego_v=signals.ego_v,
            filtered_curv=memory.filtered_curv,
            delta_ff=delta_ff,
        )
        if model_dr is not None:
            dr = model_dr
            # 模型控制器没有积分项；缓慢释放 PID 记忆，便于运行时切回 PID。
            memory.psi_i_term *= PSI_I_LOW_SPEED_DECAY
    # 转角变化率限制（得到"叠加边界前"的基准转角 base_delta）
    max_step = MAX_DELTA_RATE * dt
    base_delta = clamp(
        memory.last_delta + clamp(dr - memory.last_delta, -max_step, max_step),
        -MAX_DELTA,
        MAX_DELTA,
    )

    # ── 8. base_ctx：本帧"未叠加边界修正"的结果 ──
    # 边界修正只依赖 lane_offset，交由 _apply_boundary 叠加，使其能脱离
    # road_psi 帧门控、随 lane_offset 新帧刷新（见函数开头的帧门控分支）。
    base_ctx = LateralContext(
        dyn_prev=dyn_prev,
        rrate=rrate,
        prev_psi=prev_psi,
        raw_curv=raw_curv,
        curv_guard=curv_guard,
        in_curve=in_curve,
        delta=base_delta,
        delta_ff=delta_ff,
        delta_cte=delta_cte,
        boundary_delta=0.0,
        boundary_brake=0.0,
        boundary_warn=False,
        raw_cte=raw_cte,
        cur_off=0.0,
        upd_psi=0.0,
    )
    memory.lat_base_ctx = base_ctx
    memory.lat_base_delta = base_delta
    memory.lat_frame_dt = dt
    memory.lat_last_lane_rx = signals.lane_offset_last_rx

    # ── 9. 叠加边界修正，得到完整 LateralContext 并缓存 ──
    ctx = _apply_boundary(base_ctx, base_delta, dt, signals, memory)
    memory.lat_cached_ctx = ctx
    return ctx
