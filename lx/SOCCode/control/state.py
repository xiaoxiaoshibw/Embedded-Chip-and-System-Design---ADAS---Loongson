#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""轻量级控制数据模型。

定义前车上下文 (LeadContext)、纵向上下文 (LongitudinalContext)、
前车跟踪输入/状态、AEB 告警状态及弯道保持状态等数据类，
用于在控制子模块间传递计算结论。
"""

from dataclasses import dataclass
from typing import Optional


@dataclass
class LeadContext:
    """前车跟踪与 ACC 门控结果。"""
    x_rel: float = 0.0                     # 前车相对于自车的纵向距离 (m)
    y_rel: float = 0.0                     # 前车相对于自车的横向距离 (m)
    lead_fresh: bool = False                # 前车数据是否在超时内
    lead_lat_max: float = 0.0               # 当前横向检测窗口 (m)
    lead_lat_straight: float = 0.0          # 直道横向窗口 (m)
    lead_lat_curve: float = 0.0             # 弯道横向窗口 (m)
    lead_lat_gate: float = 0.0              # AEB 用的更窄横向门限 (m)
    raw_has_lead: bool = False              # 原始（未确认）前车存在标志
    has_lead: bool = False                  # 经过确认/记忆机制的前车标志
    lead_detected: bool = False             # 同 has_lead，语义别名
    lead_valid_for_alert: bool = False       # 是否满足 AEB 告警触发条件
    lead_speed_invalid_for_alert: bool = False  # 前车速度太低不适合告警
    acc_has_lead: bool = False               # ACC 是否允许使用前车
    acc_lead_valid: bool = False             # ACC 前车最终有效性
    acc_lost_this_cycle: bool = False         # 本周期 ACC 刚丢失前车
    acc_reject_reason: str = ''               # ACC 拒绝前车的理由
    lead_in_lane_for_acc: bool = False        # 前车是否在 ACC 车道门限内
    predicted_lead_v_proj: float = 0.0       # 滤波后的前车投影速度 (m/s)
    lane_out_release: bool = False            # 因前车偏出车道释放 ACC
    dist_opening_release: bool = False         # 因前车远离释放 ACC
    acc_ff_before: float = 0.0                # 前馈夹紧前的 FF 值
    acc_ff_after: float = 0.0                 # 前馈夹紧后的 FF 值
    acc_eval_dist: float = 0.0                 # ACC 评估用的纵向距离 (m)
    acc_ttc: float = float('inf')              # ACC 评估用的 TTC (s)
    raw_lead_v_proj: float = 0.0                # 原始前车投影速度 (m/s)
    recent_curve_exit: bool = False              # 最近退出弯道（还在保护期）
    recent_reacq: bool = False                  # 最近重新获取前车（在保护期内）
    lead_acquired: bool = False                  # 本周期新获取到前车
    lead_cls: int = 0                            # 主前车 actor class（透传给 AEB 选阈值）


@dataclass
class LongitudinalContext:
    """纵向控制单周期计算结果。"""
    lon_cmd: float = 0.0                    # 最终纵向加速度指令 (m/s², 正=减速)
    aeb_active: bool = False                # 是否 AEB 激活
    dist: float = 999.99                    # 前车距离 (m)
    ttc: float = float('inf')               # 碰撞时间 (s)
    lead_v_proj: float = 0.0                 # 前车投影速度 (m/s)
    min_safe_dist: float = 0.0               # 最小安全距离 (m)
    lead_acquire_grace_active: bool = False   # 前车获取保护期是否活跃
    acc_ff_before: float = 0.0                # 前车加速度 FF 限幅前
    acc_ff_after: float = 0.0                 # 前车加速度 FF 限幅后
    closing_speed: float = 0.0                # 接近速度 (m/s)


@dataclass
class LeadTrackingInputs:
    """前车跟踪模块的输入数据。"""
    ego_x: float = 0.0
    ego_y: float = 0.0
    ego_yaw: float = 0.0
    ego_v: float = 0.0
    lead_x: float = 0.0
    lead_y: float = 0.0
    lead_yaw: float = 0.0
    lead_v: float = 0.0
    lead_cls: int = 0
    lead_received: bool = False
    lead_last_rx_time: float = 0.0
    filtered_curv: float = 0.0
    cur_lane_width: float = 0.0
    lane_locked: bool = False
    last_acc_has_lead: bool = False
    filtered_lead_v_proj: float = 0.0
    last_lead_v_proj: float = 0.0
    last_lead_reacq_t: float = -1e9
    last_curve_t: float = -1e9   # 最后一次检测到弯道的时间（来自 ControlMemory）


@dataclass
class LeadTrackerState:
    """前车跟踪模块需跨周期保持的状态。"""
    filtered_x_rel: float = 0.0             # 滤波后的纵向相对距离
    filtered_y_rel: float = 0.0             # 滤波后的横向相对距离
    filtered_v_proj: float = 0.0            # 滤波后的前车投影速度
    rel_filter_primed: bool = False         # 相对位置/速度滤波器是否已用首帧测量值初始化
    prev_abs_y_rel: float = -1.0            # 上一拍 |y_rel|（切入横向逼近速率，-1=未初始化）
    prev_y_rel_t: float = -1e9              # 上一拍 y_rel 时间戳
    cutin_lat_rate: float = 0.0             # 低通后的横向逼近速率 (m/s)
    last_confirmed_lead_t: float = -1e9     # 上次确认前车时间
    last_lead_x_rel: float = 0.0           # 上次确认的纵向距离
    last_lead_y_rel: float = 0.0           # 上次确认的横向距离
    lead_confirm_count: int = 0              # 连续确认计数
    prev_acc_eval_dist: Optional[float] = None  # 上一次 ACC 评估距离
    prev_acc_eval_lead_v_proj: float = 0.0   # 上一次前车投影速度
    last_acc_lead_valid_t: float = -1e9      # 上次 ACC 有效时间
    last_acc_reject_reason: str = ''          # 上次 ACC 拒绝理由
    last_acc_release_reason: str = ''          # 上次 ACC 释放理由
    acc_lane_out_release_count: int = 0        # 前车偏出车道连续计数
    acc_dist_opening_release_count: int = 0    # 前车远离连续计数


@dataclass
class AebAlertState:
    """AEB 告警状态机。"""
    active: bool = False                    # 告警是否激活
    start_t: float = 0.0                    # 激活起始时间
    has_lead: bool = False                  # 是否有前车（用于告警退出判断）
    armed: bool = False                     # 是否已就绪（见过有效前车）
    last_lead_time: float = 0.0             # 上次见到有效前车的时间
    hold_speed: float = 0.0                 # 告警时自车速度（用于保持）
    cooldown_until: float = 0.0             # 冷却截止时间
    stop_hold_until: float = 0.0            # 停车保持截止时间


@dataclass
class CurveHoldState:
    """弯道保持状态机。"""
    active: bool = False                    # 是否处于弯道保持模式
    v_target: float = 0.0                   # 保持的目标速度 (m/s)
    start_t: float = 0.0                    # 激活起始时间
    v_i: float = 0.0                         # 速度积分项
    prev_has_lead: bool = False             # 上周期是否有前车
    prev_raw_has_lead: bool = False         # 上周期原始前车标志
    loss_since: float = -1e9                # 前车丢失起始时间
    reacq_since: float = -1e9               # 前车重新获取起始时间
