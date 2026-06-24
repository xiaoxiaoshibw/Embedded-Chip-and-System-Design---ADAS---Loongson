#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ADAS 纵向控制相关纯算法与小类。

包含 ACC 控制器、TTC 计算、安全距离计算、AEB 制动逻辑以及纵向指令平滑器。
这些函数仅依赖 config 参数，不依赖 ROS 或主循环状态。
"""

import math
from typing import Dict, Optional

from common import clamp
from config import *


def _soft_deadband(v, band):
    """软死区：小信号区按 25% 衰减，过渡区线性放大到 100%。

    用于 ACC 稳态时消除小范围抖动。
    """
    if band <= 1e-6:
        return v
    av = abs(v)
    if av <= band:
        return v * 0.25
    if av >= band * 2.0:
        return v
    scale = (av - band) / band
    return math.copysign(av * (0.25 + 0.75 * scale), v)


def _is_finite(v):
    """判断 v 既不是 inf 也不是 nan。"""
    return (not math.isinf(v)) and (not math.isnan(v))


class LongitudinalController:
    """ACC 纵向控制器。

    采用距离误差 D + 积分 I + 速度差 V + 前车加速度前馈 A 的结构，
    输出纵向加速度指令 (正值=减速, 负值=加速)。

    I 项在速度差很小时暂停积分，在控制量饱和时衰减，稳态时额外衰减。
    """

    def __init__(self, dt: float):
        self._dt = dt                     # 控制周期 (s)
        self._dist_i = 0.0                # 距离误差积分
        self._v_diff_filt = 0.0           # 速度差低通滤波值
        self._acc_kd = ACC_KD             # 距离误差增益
        self._acc_ki = ACC_KI             # 积分增益
        self._acc_kv = ACC_KV             # 速度差增益
        self._acc_ka = ACC_KA             # 前车加速度前馈增益

    def reset(self):
        """重置控制器内部状态（前车丢失或重获时调用）。"""
        self._dist_i = 0.0
        self._v_diff_filt = 0.0

    def set_gains(self, acc_kd: Optional[float] = None, acc_ki: Optional[float] = None,
                  acc_kv: Optional[float] = None, acc_ka: Optional[float] = None):
        """运行时更新 ACC 控制增益。"""
        if acc_kd is not None:
            self._acc_kd = max(0.0, float(acc_kd))
        if acc_ki is not None:
            self._acc_ki = max(0.0, float(acc_ki))
        if acc_kv is not None:
            self._acc_kv = max(0.0, float(acc_kv))
        if acc_ka is not None:
            self._acc_ka = max(0.0, float(acc_ka))

    def gains(self) -> Dict[str, float]:
        """返回当前 ACC 控制增益。"""
        return {
            "acc_kd": self._acc_kd,
            "acc_ki": self._acc_ki,
            "acc_kv": self._acc_kv,
            "acc_ka": self._acc_ka,
        }

    def compute(self, dist: float, ego_v: float, lead_v_proj: float,
                lead_accel: float, min_safe_dist: float, curv: float) -> float:
        """单步 ACC 控制。

        参数:
            dist: 自车到前车纵向距离 (m)
            ego_v: 自车速度 (m/s)
            lead_v_proj: 前车投影到自车方向的速度 (m/s)
            lead_accel: 前车加速度估计 (m/s²)
            min_safe_dist: 最小安全距离 (m)
            curv: 滤波曲率绝对值 (1/m)

        返回:
            纵向加速度指令 (正=减速, 负=加速)
        """
        # 参考距离 = 基础间距 + 时距 × 自车速度（不低于安全距离）
        d_ref = max(ACC_D0 + ACC_TIME_GAP * max(ego_v, 0.0), min_safe_dist)
        gap_err = dist - d_ref

        # 速度差（正值表示前车更慢→接近中）
        v_rel = lead_v_proj - ego_v
        closing_on_lead = v_rel < 0.0

        # 距离误差做限幅；接近时制动侧保留更多余量
        gap_err_ctrl = clamp(gap_err, -ACC_GAP_ERR_BRAKE_CAP, ACC_GAP_ERR_DRIVE_CAP)
        gap_err_ctrl_for_drive = min(gap_err_ctrl, 0.0) if closing_on_lead else gap_err_ctrl

        # D 项：距离误差比例
        dist_term = self._acc_kd * gap_err_ctrl_for_drive

        # V 项：速度差非对称低通后比例。
        # 朝"接近"方向（v_rel 减小）→ 用大 α 快响应，让 ACC 提前跟上前车减速，
        # 减少把工况推给 AEB 的概率；反向用小 α 保留原稳态滤波强度。
        v_diff_delta = v_rel - self._v_diff_filt
        v_diff_alpha = (ACC_VDIFF_ALPHA_CLOSING
                        if v_diff_delta < 0.0
                        else ACC_VDIFF_ALPHA_OPENING)
        self._v_diff_filt += v_diff_alpha * v_diff_delta
        speed_term = self._acc_kv * self._v_diff_filt

        # A 项：前车加速度前馈（限幅）
        ff_term = clamp(self._acc_ka * lead_accel, -ACC_FF_MAX, ACC_FF_MAX)

        # 动态加速/制动上限：根据速度差和距离误差自适应
        drive_gap_err = min(gap_err, 0.0) if closing_on_lead else gap_err
        drive_max = clamp(
            ACC_DRIVE_MAX_BASE
            + ACC_DRIVE_MAX_GAIN_V * max(v_rel, 0.0)
            + ACC_DRIVE_MAX_GAIN_D * max(drive_gap_err, 0.0),
            ACC_DRIVE_MAX_BASE,
            ACC_DRIVE_MAX_LIMIT
        )
        brake_max = clamp(
            ACC_BRAKE_MAX_BASE
            + ACC_BRAKE_MAX_GAIN_V * max(-v_rel, 0.0)
            + ACC_BRAKE_MAX_GAIN_D * max(-gap_err, 0.0),
            ACC_BRAKE_MAX_BASE,
            ACC_BRAKE_MAX_LIMIT
        )

        # I 项：速度差小才积分；饱和时衰减积分防止 windup
        # ki 为 0 时积分项失去物理意义，且 1e-6 兜底会让钳位上限爆到 1e6，
        # 后续切回正常 ki 时产生爆冲；这里直接清零。
        if self._acc_ki <= 1e-9:
            self._dist_i = 0.0
            i_candidate = 0.0
        else:
            i_candidate = self._dist_i
            if abs(v_rel) < ACC_I_PAUSE_VDIFF:
                i_candidate += gap_err_ctrl_for_drive * self._dt
            i_bound = ACC_I_MAX / self._acc_ki
            i_candidate = clamp(i_candidate, -i_bound, i_bound)
        a_candidate = dist_term + self._acc_ki * i_candidate + speed_term + ff_term
        lon_candidate = -a_candidate                          # 取反：正值=减速
        sat_drive = (lon_candidate < -drive_max) and (gap_err_ctrl_for_drive > 0.0)
        sat_brake = (lon_candidate > brake_max) and (gap_err_ctrl_for_drive < 0.0)
        if sat_drive or sat_brake:
            self._dist_i *= ACC_I_DECAY_SAT                  # 饱和时积分衰减
        else:
            self._dist_i = i_candidate
        i_term = self._acc_ki * self._dist_i

        # 总加速度指令
        a_cmd = dist_term + i_term + speed_term + ff_term
        lon_raw = -a_cmd
        lon_cmd = clamp(lon_raw, -drive_max, brake_max)

        # 稳态附近施加软死区，抑制微幅抖动
        if abs(gap_err) < ACC_STEADY_GAP_BAND and abs(v_rel) < ACC_STEADY_VREL_BAND:
            self._dist_i *= ACC_I_DECAY_STEADY
            lon_cmd = _soft_deadband(lon_cmd, 0.08)

        # 弯道中禁止加速
        if curv > CURV_NO_ACCEL_THRESH and lon_cmd < 0:
            lon_cmd = 0.0

        return lon_cmd

    @property
    def i_term(self) -> float:
        """当前 I 项输出值。"""
        return self._dist_i * self._acc_ki


def compute_ttc(fwd, ego_v, lead_v, lead_yaw, ego_yaw, lead_v_proj=None):
    """计算碰撞时间 (TTC)。

    参数:
        fwd: 纵向距离 (m)
        ego_v: 自车速度 (m/s)
        lead_v: 前车速度 (m/s)
        lead_yaw: 前车航向 (rad)
        ego_yaw: 自车航向 (rad)
        lead_v_proj: 前车投影速度（若已计算则直接使用）

    返回:
        TTC (s)，若不接近则返回 inf
    """
    if fwd < 1e-6:
        return 0.0
    if lead_v_proj is None:
        closing = ego_v - lead_v * math.cos(lead_yaw - ego_yaw)
    else:
        closing = ego_v - max(0.0, lead_v_proj)
    # 自适应"非接近"阈值：高速时绝对阈值偏小会让 TTC 在低 closing 处发散，
    # 低速时绝对阈值偏大又会把真实接近误判为安全。按自车速度比例放缩，
    # 同时保留 1e-3 防止除零。
    closing_floor = max(1e-3, 0.05 * max(ego_v, 0.0))
    if closing <= closing_floor:
        return float('inf')
    return fwd / closing


def compute_min_safe_distance(v_ego, v_lead):
    """计算最小安全跟车距离。

    基于：反应距离 + 自车制动距离 - 前车制动距离 + 静止距离。
    """
    v_e = max(v_ego, 0.0)
    v_l = max(v_lead, 0.0)
    d_react = v_e * SAFE_REACTION_TIME                                 # 反应距离
    d_ego = (v_e * v_e) / (2.0 * max(SAFE_EGO_MAX_DECEL, 0.1))       # 自车制动距离
    d_lead = (v_l * v_l) / (2.0 * max(SAFE_LEAD_MAX_DECEL, 0.1))     # 前车制动距离
    d_min = SAFE_DIST_STANDSTILL + d_react + d_ego - d_lead
    # 低速时安全距离用线性过渡，避免过度保守
    min_floor = SAFE_DIST_STANDSTILL + (10.0 - SAFE_DIST_STANDSTILL) * clamp(
        v_e / max(SAFE_DIST_LOW_SPEED_REF, 0.1), 0.0, 1.0
    )
    return clamp(d_min, min_floor, SAFE_DIST_MAX)


def compute_ttc_gate_distance(min_safe_dist, closing_speed, ttc_brake_start,
                              max_engage_dist=AEB_MAX_ENGAGE_DIST):
    """计算 AEB 触发的距离门限。

    综合最小安全距离、动态距离（基于 TTC）和固定最大距离三者取最大值。
    max_engage_dist 由调用方按 class 决定（行人/障碍传 cls_engage_dist，
    其余传默认 AEB_MAX_ENGAGE_DIST 保持原行为）。
    """
    dynamic_dist = closing_speed * max(ttc_brake_start - 1.0, 0.0)
    base_dist = max(TTC_AEB_MAX_DIST, min_safe_dist * 1.2, dynamic_dist)
    return clamp(base_dist, TTC_AEB_MAX_DIST, max_engage_dist)


def aeb_curv_suppress(filtered_curv):
    """弯道内抑制 AEB 灵敏度：曲率越大抑制越多。

    返回值在 [AEB_CURV_SUPPRESS_MAX, 1.0] 之间，乘到 TTC 阈值上。
    """
    suppress = AEB_CURV_SUPPRESS_MAX + (1.0 - AEB_CURV_SUPPRESS_MAX) * math.exp(
        -abs(filtered_curv) / AEB_CURV_SCALE
    )
    return clamp(suppress, AEB_CURV_SUPPRESS_MAX, 1.0)


def apply_aeb(fwd, ttc, lon, min_safe_dist, filtered_curv=0.0,
              closing=0.0, full_confirmed=False,
              aeb_emergency_dist=AEB_EMERGENCY_DIST,
              ttc_class_mult=1.0,
              max_engage_dist=AEB_MAX_ENGAGE_DIST):
    """AEB 制动逻辑判断。

    三级制动策略：
      1. 紧急制动（距离极近）
      2. TTC 制动（碰撞时间短）
      3. 距离制动（距离安全边界太近）

    参数:
        fwd: 纵向距离 (m)
        ttc: 碰撞时间 (s)
        lon: 当前纵向指令
        min_safe_dist: 最小安全距离 (m)
        filtered_curv: 滤波曲率
        closing: 接近速度 (m/s)
        full_confirmed: 是否已确认全制动

    返回:
        (调整后纵向指令, 是否触发AEB)
    """
    cs = aeb_curv_suppress(filtered_curv)   # 弯道抑制系数
    # class 乘子由调用方决定（lead_ctx.lead_cls 在 longitudinal_policy 里查表）；
    # 默认 1.0 = 与改造前完全一致。
    ttc_brake_start = TTC_BRAKE_START * cs * ttc_class_mult
    ttc_brake_full = TTC_BRAKE_FULL * cs * ttc_class_mult
    aeb_dist_hard = min_safe_dist
    aeb_dist_soft = min_safe_dist + AEB_SAFE_DIST_BUFFER
    closing = max(0.0, closing)

    # 1. 紧急制动：距离极近直接最大制动
    if fwd <= aeb_emergency_dist:
        return LON_CMD_MAX_BRAKE_DECEL, True
    if closing > 0.8 and fwd <= max(aeb_emergency_dist, min_safe_dist * 0.75):
        return LON_CMD_MAX_BRAKE_DECEL, True

    ttc_gate_dist = compute_ttc_gate_distance(
        min_safe_dist, closing, ttc_brake_start, max_engage_dist=max_engage_dist)
    # 距离超出门限且超出软安全距离则不触发
    if fwd > ttc_gate_dist and fwd > aeb_dist_soft:
        return lon, False

    aeb, act = lon, False
    # 2. TTC 制动
    if _is_finite(ttc) and closing > 0.8 and fwd <= ttc_gate_dist:
        if ttc <= ttc_brake_full:
            if full_confirmed and fwd <= (aeb_dist_hard + 2.0):
                aeb = max(aeb, LON_CMD_MAX_BRAKE_DECEL)
            else:
                aeb = max(aeb, ACC_NORMAL_BRAKE_MAX)
            act = True
        elif ttc < ttc_brake_start:
            # 线性渐进制动
            ratio = clamp(
                (ttc_brake_start - ttc) / (ttc_brake_start - ttc_brake_full + 1e-6),
                0.0, 1.0
            )
            aeb = max(aeb, ratio * ACC_NORMAL_BRAKE_MAX)
            act = True

    # 3. 距离制动：虽 TTC 宽裕但距离安全边界内
    if fwd < aeb_dist_soft and closing > 0.5:
        soft_ratio = clamp(
            (aeb_dist_soft - fwd) / (aeb_dist_soft - aeb_dist_hard + 1e-6),
            0.0, 1.0,
        )
        aeb = max(aeb, soft_ratio * ACC_NORMAL_BRAKE_MAX)
        act = True

    return (max(lon, aeb), True) if act else (lon, False)


class LonSmoothing:
    """纵向加速度指令变化率限制器 + 一阶低通滤波。

    根据不同工况（AEB、ACC 跟车、巡航、边界制动）使用不同的变化率限幅，
    确保控制量变化平滑且安全。
    """

    def __init__(self, dt: float):
        self._dt = dt                      # 控制周期 (s)
        self._prev = 0.0                    # 上一步限幅后输出
        self._filtered = 0.0                # 低通滤波后输出

    def update(self, target: float,
               aeb_active: bool = False,
               has_lead: bool = False,
               boundary_brake: bool = False,
               max_rate_override: Optional[float] = None) -> float:
        """对纵向指令做坡度限制和低通滤波。

        参数:
            target: 原始纵向指令
            aeb_active: 是否处于 AEB 状态
            has_lead: 是否有前车
            boundary_brake: 是否边界制动
            max_rate_override: 可选的外部强制变化率上限 (m/s³)。
                如果设置，最终采用 min(工况 rate, override)，用于主备接管
                保护窗等需要在常规策略上叠加更严限幅的场景，避免事后用
                lon_smooth.reset() 覆盖内部状态。

        返回:
            平滑后的纵向指令
        """
        # 根据工况选择变化率限幅
        if aeb_active:
            rate = LON_RATE_AEB                              # AEB：急速制动
        elif boundary_brake:
            rate = LON_RATE_BOUNDARY                         # 边界制动：较快
        elif has_lead:
            if self._prev > 1.0 and target < self._prev:
                # 跟车且从大制动值释放：限制释放速度
                rate = max(LON_RATE_BRAKE_RELEASE, LON_RATE_ACCEL_ACC)
            else:
                rate = LON_RATE_DECEL_ACC if target > self._prev else LON_RATE_ACCEL_ACC
        else:
            if self._prev > 1.0 and target < self._prev:
                rate = max(LON_RATE_BRAKE_RELEASE, LON_RATE_ACCEL_CRUISE)
            else:
                rate = LON_RATE_DECEL_CRUISE if target > self._prev else LON_RATE_ACCEL_CRUISE

        # 外部强制限速优先（仅取更严的）
        if max_rate_override is not None and max_rate_override < rate:
            rate = max_rate_override

        # 坡度限制：限制每周期最大变化量
        max_step = rate * self._dt
        limited = clamp(target, self._prev - max_step, self._prev + max_step)
        self._prev = limited

        if aeb_active:
            # AEB 分支同时把 _prev 对齐到 limited：避免下一拍从 AEB 切回 ACC 时
            # _filtered 与 _prev 不同源，导致第一帧实际跳跃比预期大。
            self._prev = limited
            self._filtered = limited
            return self._filtered

        # 一阶低通滤波：进一步平滑输出
        self._filtered += LON_OUTPUT_ALPHA * (limited - self._filtered)

        return self._filtered

    @property
    def value(self) -> float:
        """当前平滑器输出值（外部诊断/低通后值）。"""
        return self._filtered

    @property
    def prev(self) -> float:
        """限幅起点（坡度限速所用的 _prev）。

        接管 flapping cooldown 分支必须用这个而不是 value：cooldown 时
        lon_smooth 内部状态未被 reset，下一拍 update() 仍以 _prev 为限幅起点；
        若 _takeover_prev_lon 取 _filtered 会与限幅起点错位，保护窗第一帧
        引入预期外阶跃。
        """
        return self._prev

    def reset(self, value: float = 0.0):
        """重置平滑器到指定值（前车重获/接管时使用）。

        线程安全说明：本类仅在 ROS2 timer callback（单线程 executor）中访问，
        不存在真正的并发写入。若未来切换到多线程 executor，需在调用方加锁后再调用。
        """
        self._prev = value
        self._filtered = value
