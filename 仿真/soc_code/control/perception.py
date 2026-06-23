#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""感知层：把所有车系话题（car2 / car3..carN，含未来行人）当成毫米波雷达数据源
持续监听，每周期产出一份共享 PerceptionFrame，含所有 fresh 目标的相对位置 +
主前车选举结果。

设计哲学
========
- **始终启用**：与 MULTI_TARGET_COUNT 无关（单目标部署也走这条路径，只是
  ``_targets`` 里只有 car2 一条记录），下游不再需要分支判断"多目标是否打开"。
- **单一权威**：所有"相对位置 / 横向窗口 / cut-in 预判 / 主前车选举"在本模块
  集中计算，下游模块（OvertakeManager、未来的避障/AEB 监控）从 PerceptionFrame
  读，避免散落派生。
- **byte-equivalent 兼容**：``MULTI_TARGET_COUNT==1`` 时本模块不写回
  ``signals.lead_*``（仍由 ROS callback 直接写），仅作为下游/遥测的额外视图，
  与改造前现状字节级一致。``>1`` 时承担原 ``_select_primary_lead`` 的写回职责。
- **不接管 LeadTracker 内部滤波**：LeadTracker 仍对 ``signals.lead_x/y`` 做
  ``LEAD_REL_FILTER_ALPHA`` 低通——本模块对 ``primary_track`` 又过一次同 alpha
  的低通只是为了给非 LeadTracker 的消费者（overtake、避障）提供同源数值。

非线程安全
==========
默认 rclpy SingleThreadedExecutor 下回调串行，``_get()`` 的 dict.get+赋值在
单线程下不会竞争。若未来切到 MultiThreadedExecutor，需在 ``_targets`` 上加锁。

Python 3.6 兼容；不使用 PEP 604/585 语法。
"""

import math
from dataclasses import dataclass, field
from typing import Dict, Optional

from config import (
    ACTOR_CLASS_UNKNOWN,
    CUTIN_CORRIDOR_RATIO,
    CUTIN_HORIZON_S,
    CUTIN_LAT_RATE_ALPHA,
    CUTIN_MIN_LAT_RATE,
    LEAD_REL_FILTER_ALPHA,
    LEAD_TIMEOUT_S,
    LEAD_V_PROJ_FILTER_ALPHA,
    MULTI_TARGET_FWD_MAX,
    MULTI_TARGET_FWD_MIN,
)
from lateral import compute_lead_lateral_window, compute_relative_in_ego_frame

# car2 始终是 id=2 基准前车，沿用 multi_target 时期约定。
CAR2_ID = 2


def _finite(*vals):
    for v in vals:
        if v is None:
            return False
        try:
            if math.isinf(v) or math.isnan(v):
                return False
        except TypeError:
            return False
    return True


@dataclass
class _Track:
    """单目标航迹仓库条目，跨周期保留滤波/差分所需的最少状态。"""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    cls: int = ACTOR_CLASS_UNKNOWN
    last_rx: float = -1e9
    # 跨周期：相对位置低通滤波（与 LeadTracker.filtered_x_rel 同 alpha）
    filtered_x_rel: float = 0.0
    filtered_y_rel: float = 0.0
    rel_primed: bool = False
    # 跨周期：投影接近速度（x_rel 的负向变化率），LEAD_V_PROJ_FILTER_ALPHA 低通
    filtered_v_proj: float = 0.0
    prev_x_rel: float = 0.0
    prev_x_rel_t: float = -1e9
    v_proj_primed: bool = False
    # 跨周期：横向逼近速率（|y_rel| 减小为正），CUTIN_LAT_RATE_ALPHA 低通
    prev_abs_y_rel: float = -1.0
    prev_rel_t: float = -1e9
    cutin_lat_rate: float = 0.0


@dataclass(frozen=True)
class TrackRel:
    """一个目标在某一拍的相对状态快照（PerceptionFrame.tracks 的值类型）。"""
    tid: int                     # 目标 id（car2 总是 2）
    cls: int                     # ACTOR_CLASS_*
    x_rel: float                 # 前向距离（ego frame，m，>0 表示在前方）
    y_rel: float                 # 横向偏移（左正右负，m）
    v_world: float               # /car{N}_v 原始标量速度（m/s）
    v_proj: float                # x_rel 负向变化率经低通滤波（m/s，>0 表示接近）
    lat_rate: float              # |y_rel| 减小的速率经低通（m/s，>0 表示向本道靠拢）
    last_rx: float               # 位姿最近接收时刻
    fresh: bool                  # (now - last_rx) <= LEAD_TIMEOUT_S
    in_lane: bool                # |y_rel| <= lead_lat_max 且位于前向窗口
    cutin: bool                  # cut-in 预判（相邻走廊内、即将进入本车道）


@dataclass(frozen=True)
class PerceptionFrame:
    """单周期感知输出。"""
    now: float
    ego_x: float
    ego_y: float
    ego_yaw: float
    lane_width: float
    filtered_curv: float
    lead_lat_max: float
    tracks: Dict[int, TrackRel] = field(default_factory=dict)
    primary_tid: Optional[int] = None      # 选中的主前车 id；选不出 = None
    primary_via_cutin: bool = False        # 主前车是否经 cut-in 预判选入
    n_fresh: int = 0                        # 本拍 fresh 的目标数（含 car2）


class PerceptionLayer:
    """目标航迹仓库 + 每周期 PerceptionFrame 构建器。

    使用：
      - ROS 话题 callback 收到 pose/v/class 时调用 ``ingest_*``。
      - 控制循环每拍调用一次 ``build_frame(...)``，返回 PerceptionFrame。
      - 主前车的写回（signals.lead_*）由调用方判断是否需要（多目标模式下需要，
        单目标模式下 callback 已直接写过）。
    """

    def __init__(self):
        self._targets = {}        # type: Dict[int, _Track]

    # ── ingestion ──
    def _get(self, tid):
        t = self._targets.get(tid)
        if t is None:
            t = _Track()
            self._targets[tid] = t
        return t

    def ingest_pose(self, tid, x, y, yaw, now):
        """位姿话题回调：更新位置/朝向 + 时间戳。无效值整条丢弃。"""
        if not _finite(x, y, yaw, now):
            return
        t = self._get(tid)
        t.x = x
        t.y = y
        t.yaw = yaw
        t.last_rx = now

    def ingest_v(self, tid, v):
        """速度话题回调：仅更新 v（不刷新 last_rx，位姿才代表航迹新鲜度）。"""
        if not _finite(v):
            return
        self._get(tid).v = v

    def ingest_cls(self, tid, cls):
        """分类话题回调：仅更新 cls。无效值整条丢弃。"""
        try:
            c = int(cls)
        except (TypeError, ValueError):
            return
        if c < 0 or c > 255:
            return
        self._get(tid).cls = c

    def get_cls(self, tid):
        """供调用方在选主失败时回填 signals.lead_cls，避免上一拍 cls 残留。"""
        t = self._targets.get(tid)
        return t.cls if t is not None else ACTOR_CLASS_UNKNOWN

    def has_target(self, tid):
        return tid in self._targets

    # ── per-tick frame build ──
    def build_frame(self, ego_x, ego_y, ego_yaw, lane_width, filtered_curv, now):
        """构建本拍 PerceptionFrame：投影 + cut-in 判别 + 选主。

        参数:
            ego_x/y/yaw: 自车位姿（世界系）
            lane_width:  当前车道宽估计
            filtered_curv: 当前滤波曲率
            now:         单调时钟
        """
        lane_win, _lat_straight, _lat_curve = compute_lead_lateral_window(
            filtered_curv, lane_width)

        tracks = {}
        n_fresh = 0
        # 候选 (x_rel, tid, TrackRel, via_cutin) 取前向最近
        best = None
        car2_fallback = None
        for tid, t in self._targets.items():
            if not _finite(t.x, t.y, t.yaw):
                continue
            fresh = (now - t.last_rx) <= LEAD_TIMEOUT_S
            # 即便不 fresh 也参与构建（fresh=False 标记给下游决策），但选主只在 fresh 上做
            raw_x_rel, raw_y_rel = compute_relative_in_ego_frame(
                t.x, t.y, ego_x, ego_y, ego_yaw)

            # 相对位置低通：与 LeadTracker 同 alpha；首拍直接 priming。
            # 这条滤波只服务 perception 的外部消费者；LeadTracker 仍对
            # signals.lead_* 独立做同样滤波，二者状态相互独立。
            if not fresh:
                t.rel_primed = False
                t.v_proj_primed = False
                t.prev_abs_y_rel = -1.0
            if fresh:
                if not t.rel_primed:
                    t.filtered_x_rel = raw_x_rel
                    t.filtered_y_rel = raw_y_rel
                    t.rel_primed = True
                else:
                    t.filtered_x_rel += LEAD_REL_FILTER_ALPHA * (
                        raw_x_rel - t.filtered_x_rel)
                    t.filtered_y_rel += LEAD_REL_FILTER_ALPHA * (
                        raw_y_rel - t.filtered_y_rel)

            x_rel = t.filtered_x_rel if fresh else raw_x_rel
            y_rel = t.filtered_y_rel if fresh else raw_y_rel

            # 接近速度（v_proj）：以 x_rel 的负变化率为正接近，按 alpha 低通
            v_proj = t.filtered_v_proj
            if fresh:
                if not t.v_proj_primed:
                    t.prev_x_rel = x_rel
                    t.prev_x_rel_t = now
                    t.filtered_v_proj = 0.0
                    t.v_proj_primed = True
                    v_proj = 0.0
                else:
                    dt = now - t.prev_x_rel_t
                    if 1e-3 < dt < 1.0:
                        raw_v_proj = -(x_rel - t.prev_x_rel) / dt
                        t.filtered_v_proj += LEAD_V_PROJ_FILTER_ALPHA * (
                            raw_v_proj - t.filtered_v_proj)
                        v_proj = t.filtered_v_proj
                    t.prev_x_rel = x_rel
                    t.prev_x_rel_t = now

            # 横向逼近速率（cut-in 预判）
            forward_ok = (MULTI_TARGET_FWD_MIN <= x_rel <= MULTI_TARGET_FWD_MAX)
            cur_abs_y = abs(y_rel)
            if fresh:
                if t.prev_abs_y_rel < 0.0:
                    t.cutin_lat_rate = 0.0
                else:
                    dt_y = now - t.prev_rel_t
                    if 1e-3 < dt_y < 1.0:
                        raw_rate = (t.prev_abs_y_rel - cur_abs_y) / dt_y
                        t.cutin_lat_rate += CUTIN_LAT_RATE_ALPHA * (
                            raw_rate - t.cutin_lat_rate)
                t.prev_abs_y_rel = cur_abs_y
                t.prev_rel_t = now
            in_lane = bool(fresh and forward_ok and cur_abs_y <= lane_win)
            cutin = False
            if fresh and forward_ok and not in_lane:
                corridor = lane_win * CUTIN_CORRIDOR_RATIO
                predicted_abs = max(0.0, cur_abs_y - t.cutin_lat_rate * CUTIN_HORIZON_S)
                cutin = (
                    cur_abs_y <= corridor
                    and t.cutin_lat_rate >= CUTIN_MIN_LAT_RATE
                    and predicted_abs <= lane_win
                )

            tr = TrackRel(
                tid=tid,
                cls=t.cls,
                x_rel=x_rel,
                y_rel=y_rel,
                v_world=t.v,
                v_proj=v_proj,
                lat_rate=t.cutin_lat_rate if fresh else 0.0,
                last_rx=t.last_rx,
                fresh=fresh,
                in_lane=in_lane,
                cutin=cutin,
            )
            tracks[tid] = tr

            if not fresh:
                continue
            n_fresh += 1
            # 选主备选：car2 是兜底候选（fresh + 前向），其它必须 in_lane 或 cutin
            if tid == CAR2_ID and forward_ok:
                car2_fallback = tr
            if forward_ok and (in_lane or cutin):
                if best is None or x_rel < best.x_rel:
                    best = tr

        primary = best if best is not None else car2_fallback
        primary_tid = primary.tid if primary is not None else None
        via_cutin = bool(best is not None and best.cutin)

        return PerceptionFrame(
            now=now,
            ego_x=ego_x,
            ego_y=ego_y,
            ego_yaw=ego_yaw,
            lane_width=lane_width,
            filtered_curv=filtered_curv,
            lead_lat_max=lane_win,
            tracks=tracks,
            primary_tid=primary_tid,
            primary_via_cutin=via_cutin,
            n_fresh=n_fresh,
        )
