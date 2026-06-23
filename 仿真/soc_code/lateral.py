#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""横向控制与边界相关算法。

包含车道宽度估计器、坐标系变换、车道安全余量计算、前车横向窗口、
自适应预览时间以及边界修正（软/硬约束 + 制动）。
"""

import collections
import logging
import math
import time
from typing import Tuple

from config import *
from common import clamp, is_finite


class LaneWidthEstimator:
    """基于左右偏移样本估计车道宽度。

    工作原理：
      1. 收集左右偏移绝对值样本到单独缓冲区。
      2. 取 95 分位数作为单侧偏移估计。
      3. 两侧相加得到车道宽度估计，再用一阶低通滤波平滑。
      4. 弯道时采用更短的窗口，并对偏移做横向补偿修正。
    """

    def __init__(self, loop_hz: float):
        self._hz = loop_hz
        # 冷启动保守策略：样本不足时使用 LANE_WIDTH_MIN 而非 LANE_DEFAULT_WIDTH。
        # LANE_DEFAULT_WIDTH(3.8m) 偏宽，会让边界余量偏大，导致边界制动延迟触发。
        # 用下限值保守估计，样本积累到 LANE_EST_MIN_SAMPLES 后再切换到实测值。
        self._width = LANE_WIDTH_MIN
        self._prev_width = LANE_WIDTH_MIN
        self._warmup_done = False           # 是否已完成冷启动（样本足够后置 True）
        self._locked = False    # 超时时锁定宽度，防止数据缺失导致估计漂移
        _init_len = max(int(LANE_EST_WINDOW_STRAIGHT * loop_hz), LANE_EST_MIN_SAMPLES * 2)
        self._buf_l = collections.deque(maxlen=_init_len)  # 左偏移样本
        self._buf_r = collections.deque(maxlen=_init_len)  # 右偏移样本
        self._samples_dirty = False
        self._last_recompute_t = 0.0
        self._last_buf_sizes = (0, 0)
        self._recompute_interval_s = max(0.1, 4.0 / max(loop_hz, 1.0))

    def _window_size(self, filtered_curv: float) -> int:
        """根据曲率选择统计窗口长度：弯道用短窗口，直道用长窗口。"""
        secs = (
            LANE_EST_WINDOW_CURVE
            if abs(filtered_curv) > LANE_EST_CURV_THRESH
            else LANE_EST_WINDOW_STRAIGHT
        )
        return max(int(secs * self._hz), LANE_EST_MIN_SAMPLES * 2)

    def _resize_bufs(self, maxlen: int):
        """动态调整缓冲区大小，保留已有数据。"""
        if self._buf_l.maxlen != maxlen:
            self._buf_l = collections.deque(self._buf_l, maxlen=maxlen)
            self._buf_r = collections.deque(self._buf_r, maxlen=maxlen)
            self._samples_dirty = True

    def _should_recompute(self, now: float) -> bool:
        """检查样本数是否足够且距离上次计算是否超过最小间隔。"""
        if len(self._buf_l) < LANE_EST_MIN_SAMPLES or len(self._buf_r) < LANE_EST_MIN_SAMPLES:
            return False
        buf_sizes = (len(self._buf_l), len(self._buf_r))
        if buf_sizes != self._last_buf_sizes:
            self._last_buf_sizes = buf_sizes
            self._samples_dirty = True
        if not self._samples_dirty:
            return False
        return (now - self._last_recompute_t) >= self._recompute_interval_s

    @staticmethod
    def _percentile(buf, pct: float) -> float:
        """线性插值计算指定百分位数。"""
        if not buf:
            return 0.0
        s = sorted(buf)
        k = (len(s) - 1) * pct / 100.0
        lo = int(k)
        hi = min(lo + 1, len(s) - 1)
        return s[lo] + (k - lo) * (s[hi] - s[lo])

    @staticmethod
    def _correct_offset(raw_offset: float, filtered_curv: float, ego_v: float) -> float:
        """高速弯道下对横向偏移做曲率补偿：弯道内侧偏移会被车体向心力放大。"""
        curv_mag = abs(filtered_curv)
        if curv_mag < 1e-4 or abs(ego_v) < 0.5:
            return raw_offset
        correction = clamp(K_LAT_COMP * (ego_v ** 2) * curv_mag, 0.0, 1.5)
        sign = 1.0 if raw_offset >= 0.0 else -1.0
        return raw_offset + sign * correction

    def update(self, raw_offset, now: float,
               filtered_curv: float = 0.0,
               ego_v: float = 0.0,
               lane_offset_last_rx: float = 0.0) -> float:
        """每周期调用一次，返回当前车道宽度估计值。

        参数:
            raw_offset: 原始车道横向偏移 (m)
            now: 单调时钟时间
            filtered_curv: 滤波后曲率
            ego_v: 自车速度 (m/s)
            lane_offset_last_rx: 最近一次偏移数据接收时刻
        """
        # 数据超时时锁定宽度，不再更新
        if (now - lane_offset_last_rx) > LANE_EST_TIMEOUT_S:
            self._locked = True
            return self._width
        self._locked = False

        if raw_offset is None or not is_finite(raw_offset):
            return self._width
        # 过滤明显的离群偏移：绝对值过大或弯道大曲率时偏移过大
        if abs(raw_offset) > 4.0 or (abs(filtered_curv) > 0.03 and abs(raw_offset) > 1.5):
            return self._width

        self._resize_bufs(self._window_size(filtered_curv))
        corrected = self._correct_offset(raw_offset, filtered_curv, ego_v)
        if abs(corrected) > 4.5:
            return self._width

        # 正偏移放入左缓冲，负偏移取绝对值放入右缓冲
        if corrected > 0.05:
            self._buf_l.append(corrected)
            self._samples_dirty = True
        elif corrected < -0.05:
            self._buf_r.append(abs(corrected))
            self._samples_dirty = True

        if not self._should_recompute(now):
            return self._width

        # 计算两侧的 P95 偏移，求和得到车道宽度估计
        p95_l = self._percentile(self._buf_l, LANE_EST_PERCENTILE)
        p95_r = self._percentile(self._buf_r, LANE_EST_PERCENTILE)
        estimated = clamp(p95_l + p95_r, LANE_WIDTH_MIN, LANE_WIDTH_MAX)

        # 冷启动完成：首次有足够样本时从保守值平滑过渡到实测值
        if not self._warmup_done:
            self._warmup_done = True
            self._width = estimated          # 直接跳到实测值，跳过低通滤波的慢爬升
            self._prev_width = estimated
            logging.info('[LANE_EST] warmup done: estimated=%.2fm', estimated)
            self._last_recompute_t = now
            self._samples_dirty = False
            return self._width

        # 一阶低通滤波平滑宽度估计
        self._width += LANE_WIDTH_FILTER_ALPHA * (estimated - self._width)
        # 限制宽度每周期最大变化量，防抖
        delta = self._width - self._prev_width
        if abs(delta) > LANE_WIDTH_MAX_RATE:
            self._width = self._prev_width + math.copysign(LANE_WIDTH_MAX_RATE, delta)
        self._prev_width = self._width
        self._last_recompute_t = now
        self._samples_dirty = False
        return self._width

    @property
    def is_locked(self) -> bool:
        """宽度是否被锁定（数据超时时锁定）。"""
        return self._locked

    @property
    def width(self) -> float:
        """当前车道宽度估计值。"""
        return self._width

    @property
    def sample_counts(self) -> Tuple[int, int]:
        """返回 (左侧样本数, 右侧样本数)，用于日志/监控，避免外部直接访问私有缓冲。"""
        return len(self._buf_l), len(self._buf_r)


def compute_relative_in_ego_frame(lead_x, lead_y, ego_x, ego_y, ego_yaw):
    """将前车全局坐标投影到自车坐标系，得到纵向(x_rel)和横向(y_rel)距离。

    返回: (x_rel, y_rel)，x_rel 为前方距离，y_rel 正值表示前车在自车右侧。
    """
    dx, dy = lead_x - ego_x, lead_y - ego_y
    c, s = math.cos(ego_yaw), math.sin(ego_yaw)
    return c * dx + s * dy, -s * dx + c * dy


def lane_margins_from_width(width_m):
    """由车道宽度计算三级余量：safe > warn > hard。

    返回: (lane_safe_margin, lane_warn_margin, lane_hard_margin)
    """
    half_w = 0.5 * width_m
    lane_safe_margin = max(half_w - VEHICLE_HALF_WIDTH, MIN_LANE_SAFE_MARGIN)
    lane_warn_margin = lane_safe_margin * LANE_WARN_RATIO
    lane_hard_margin = lane_safe_margin * LANE_HARD_RATIO
    return lane_safe_margin, lane_warn_margin, lane_hard_margin


def compute_lead_lateral_window(filtered_curv, lane_width_m):
    """根据曲率和车道宽度计算前车横向检测窗口，弯道时收窄。

    返回: (lead_lat_max, lat_straight, lat_curve)
    """
    lat_straight = clamp(
        lane_width_m * LEAD_LAT_STRAIGHT_RATIO,
        LEAD_LAT_MAX_STRAIGHT_MIN,
        LEAD_LAT_MAX_STRAIGHT_MAX,
    )
    lat_curve = clamp(
        lane_width_m * LEAD_LAT_CURVE_RATIO,
        LEAD_LAT_MAX_CURVE_MIN,
        LEAD_LAT_MAX_CURVE_MAX,
    )
    curv_ratio = clamp(
        (abs(filtered_curv) - CURV_LEAD_THRESH) / (AEB_CURV_SCALE - CURV_LEAD_THRESH + 1e-6),
        0.0,
        1.0,
    )
    curve_corridor = clamp(
        lane_width_m * 0.5,
        compute_curve_hold_window(lane_width_m),
        LEAD_LAT_MAX_STRAIGHT_MAX,
    )
    # A lead vehicle on the same curved lane can appear laterally offset in the
    # ego frame before both vehicles align with the local tangent. Keep enough
    # corridor in curves so AEB/ACC can acquire it before the gap becomes tiny.
    lead_lat_max = lat_straight + curv_ratio * (curve_corridor - lat_straight)
    return lead_lat_max, lat_straight, lat_curve


def compute_curve_hold_window(lane_width_m):
    """弯道保持模式下使用的前车横向窗口。"""
    return clamp(
        lane_width_m * LEAD_CURVE_HOLD_LAT_RATIO,
        LEAD_LAT_MAX_CURVE_MIN,
        LEAD_LAT_MAX_CURVE_MAX,
    )


def adaptive_preview_time(v, filtered_curv=0.0):
    """自适应预览时间：高速时增加预览距离，弯道时衰减。"""
    base = PREVIEW_TIME_MIN + (PREVIEW_TIME_MAX - PREVIEW_TIME_MIN) * clamp(
        v / PREVIEW_SPEED_REF, 0.0, 1.0
    )
    curv_ratio = clamp(abs(filtered_curv) / CURVE_PREVIEW_ATTEN_SCALE, 0.0, 1.0)
    return base * (1.0 - CURVE_PREVIEW_ATTEN_MAX * curv_ratio)


_boundary_last_log_time = 0.0
BOUNDARY_LOG_INTERVAL_S = 1.0


def compute_boundary_correction(offset, delta, v,
                                lane_safe_margin, lane_warn_margin, lane_hard_margin):
    """计算车道边界修正量（转角修正 + 制动修正）。

    当车辆偏移超过 hard_margin 时，线性增长修正力度；
    同时根据车速缩放制动量。

    返回: (delta_correction, brake_correction, boundary_warning)
    """
    global _boundary_last_log_time
    ao = abs(offset)
    if ao <= lane_hard_margin:
        return 0.0, 0.0, False

    # 修正符号恒定：offset>0（车在路径左侧）需负转角拉回，offset<0 需正转角。
    # 符号在道路切线局部系内已确定，不随曲率翻转（翻转会在弯道里变成正反馈）。
    sign = -1.0 if offset > 0.0 else 1.0
    ratio = clamp(
        (ao - lane_hard_margin) / (lane_safe_margin - lane_hard_margin + 1e-6),
        0.0,
        1.0,
    )
    # 转角修正：hard 边界以上按比例增加
    dc = clamp(
        sign * K_LATERAL_HARD * (ao - lane_hard_margin) * (0.35 + 0.65 * ratio),
        -MAX_DELTA * 0.18,
        MAX_DELTA * 0.18,
    )
    # 制动修正：与偏移比例和车速正相关
    bk = clamp(BOUNDARY_BRAKE_EXTRA * ratio * clamp(v / 10.0, 0.4, 1.5), 0.0, 2.0)

    now = time.monotonic()
    if now - _boundary_last_log_time >= BOUNDARY_LOG_INTERVAL_S:
        logging.warning('BOUNDARY HARD: offset=%.3fm corr=%.4frad brake=%.2f', offset, dc, bk)
        _boundary_last_log_time = now
    return dc, bk, True


class LateralSmoothing:
    """方向盘转角变化率限幅 + 一阶低通滤波。

    对称于 LonSmoothing：在 ESP32 发送链路的最末端做"防跳变兜底"。
    lateral_controller 内部已有 MAX_DELTA_RATE 等硬限位，本类做最外层
    平滑，避免任何感知抖动经 boundary/cte 直传到执行器。

    takeover guard 期通过 max_rate_override 收紧速率（与 LonSmoothing 一致），
    无需 reset 内部状态即可临时叠加更严限幅。
    """

    def __init__(self, dt: float):
        self._dt = dt
        self._prev = 0.0
        self._filtered = 0.0

    def update(self, target: float, max_rate_override=None) -> float:
        """对方向盘转角做坡度限制 + 一阶低通。

        参数:
            target: 原始方向盘转角 (rad)
            max_rate_override: 可选的外部强制速率上限 (rad/s)，用于
                takeover guard 临时收紧；最终采用 min(常规, override)。

        返回:
            平滑后的方向盘转角 (rad)
        """
        rate = LAT_RATE_NORMAL
        if max_rate_override is not None and max_rate_override < rate:
            rate = max_rate_override
        max_step = rate * self._dt
        limited = clamp(target, self._prev - max_step, self._prev + max_step)
        self._prev = limited
        self._filtered += LAT_OUTPUT_ALPHA * (limited - self._filtered)
        return self._filtered

    @property
    def value(self) -> float:
        """当前低通后输出值。"""
        return self._filtered

    @property
    def prev(self) -> float:
        """限幅起点（坡度限速所用的 _prev）。"""
        return self._prev

    def reset(self, value: float = 0.0):
        """重置平滑器到指定值（接管 / 异常恢复时调用）。"""
        self._prev = value
        self._filtered = value
