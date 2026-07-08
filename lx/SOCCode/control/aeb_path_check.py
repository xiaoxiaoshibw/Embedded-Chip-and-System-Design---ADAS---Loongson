#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AEB 预测路径碰撞检查（对标 Autoware autonomous_emergency_braking）。

与现有 AEB（TTC/距离判据，只盯选举出的主前车）互补：
  1. 用自车当前速度 + 转角做运动学 bicycle 积分，生成 AEB_PATH_HORIZON_S 的
     预测 footprint 路径（对齐 Autoware "IMU path"，我们用转角代替 IMU ω，
     见其 README 中 steering angle 方案的讨论）；
  2. 对感知帧**全部** fresh 目标（含横穿行人、未被选举为主前车的目标）做
     恒速外推 + footprint 相交检查，得到最早碰撞时间 t_col；
  3. 对"当前已在走廊内"的目标做 RSS 最小安全距离判据
     d_rss = v_ego·ρ + v_ego²/(2·a_ego) − v_obs²/(2·a_obs)；
  4. 连续 AEB_PATH_CONFIRM_CYCLES 拍命中才触发（消抖，退出 -2 非对称衰减，
     与主 AEB 的 full_confirm 计数同风格）。

融合约定（见 pipeline.run_pure_pipeline）：
  - **只增不减**：仅通过 max() 抬高 lon_cmd，从不减小已有制动量；
  - 触发时置 lon_ctx.aeb_active=True，下游平滑/门控/心跳走既有 AEB 通路；
  - AEB_PATH_CHECK_ENABLED=0（默认）时 managers.aeb_path 不实例化，
    本模块不参与任何计算——与原行为字节级一致。

确定性（锁步）约定：
  evaluate() 是纯函数 + 单计数器状态，不读时钟/不做 I/O。障碍列表
  path_obstacles 是 plain tuple 列表（pickle 安全），由主核构建并原样
  传给锁步影子核（同 ml_result 模式），保证两遍计算逐位一致。

超车联动：
  超车状态机压制的主目标（suppress_lead_for_overtake）由调用方在构建
  障碍列表时剔除（node 侧剔 primary_tid；离线侧整条跳过），否则路径
  检查会在借道接近段对被超目标全力制动，与超车"受控接近"打架。

Python 3.6 兼容；注释中文。
"""

import math
from dataclasses import dataclass, replace

from common import clamp
from config import (
    ACC_NORMAL_BRAKE_MAX,
    ACTOR_CLASS_VEHICLE,
    AEB_PATH_CONFIRM_CYCLES,
    AEB_PATH_HORIZON_S,
    AEB_PATH_LAT_MARGIN,
    AEB_PATH_LON_MARGIN,
    AEB_PATH_MIN_EGO_V,
    AEB_PATH_OBS_HALF_WIDTH,
    AEB_PATH_PREFILTER_LAT,
    AEB_PATH_STEP_S,
    AEB_PATH_TCOL_FULL_S,
    AEB_PATH_TCOL_START_S,
    AEB_PATH_YAWRATE_SIGN,
    AEB_RSS_EGO_DECEL,
    AEB_RSS_MIN_DIST,
    AEB_RSS_OBS_DECEL,
    AEB_RSS_REACT_S,
    LON_CMD_MAX_BRAKE_DECEL,
    MAX_DELTA,
    VEHICLE_HALF_WIDTH,
    WHEEL_BASE,
)

# 障碍元组下标约定（plain tuple：pickle/锁步线格式友好，勿改顺序）：
# (tid, cls, x_rel, y_rel, v_proj, lat_rate)
#   x_rel    前向距离 (m, >0 在前方)
#   y_rel    横向偏移 (m, 符号与感知层 ego frame 一致)
#   v_proj   x_rel 负向变化率 (m/s, >0 接近)
#   lat_rate |y_rel| 减小速率 (m/s, >0 向本车道靠拢)
OBS_TID = 0
OBS_CLS = 1
OBS_X = 2
OBS_Y = 3
OBS_VPROJ = 4
OBS_LATRATE = 5


def _finite(v):
    try:
        return not (math.isnan(v) or math.isinf(v))
    except TypeError:
        return False


@dataclass(frozen=True)
class AebPathDecision:
    """单拍路径检查结果（只读）。"""
    engaged: bool = False            # 消抖确认后触发
    risk: bool = False               # 本拍原始命中（未消抖），供遥测观察
    brake_cmd: float = 0.0           # 建议制动量 (m/s², 正=减速)
    t_col: float = float('inf')     # 最早预测碰撞时间 (s)
    obstacle_tid: int = -1           # 触发目标 id；-1 = 无
    rss_dist: float = 0.0            # 触发目标的 RSS 最小安全距离 (m)
    rss_violated: bool = False       # 是否 RSS 距离不足


_INACTIVE = AebPathDecision()


def obstacles_from_frame(frame, exclude_tid=None):
    """从 PerceptionFrame 构建障碍元组列表（node 侧，每拍调用）。

    exclude_tid：超车压制中的主目标 id（剔除，避免与超车受控接近打架）。
    只取 fresh 目标；数值经有限性过滤。
    """
    obstacles = []
    if frame is None:
        return obstacles
    for tid, tr in frame.tracks.items():
        if not tr.fresh:
            continue
        if exclude_tid is not None and tid == exclude_tid:
            continue
        if not (_finite(tr.x_rel) and _finite(tr.y_rel)
                and _finite(tr.v_proj) and _finite(tr.lat_rate)):
            continue
        obstacles.append(
            (int(tid), int(tr.cls), float(tr.x_rel), float(tr.y_rel),
             float(tr.v_proj), float(tr.lat_rate)))
    return obstacles


def obstacles_from_lead_ctx(lead_ctx):
    """离线回退：无感知帧时从 lead_ctx 合成单目标列表（replay/run_scenario）。

    调用方保证超车压制时 lead_ctx 已走"无前车"分支（acc_has_lead=False），
    故此处只看 lead_detected/acc_has_lead。
    """
    if lead_ctx is None:
        return []
    if not (lead_ctx.lead_detected or lead_ctx.acc_has_lead):
        return []
    x_rel = float(lead_ctx.x_rel)
    y_rel = float(lead_ctx.y_rel)
    v_proj = float(lead_ctx.raw_lead_v_proj)
    if not (_finite(x_rel) and _finite(y_rel) and _finite(v_proj)):
        return []
    cls = int(getattr(lead_ctx, 'lead_cls', ACTOR_CLASS_VEHICLE) or 0)
    return [(2, cls, x_rel, y_rel, v_proj, 0.0)]


def _predict_ego_path(ego_v, delta, n_steps, dt):
    """运动学 bicycle 积分：返回 [(x, y, theta), ...]（初始 ego frame）。

    ω = SIGN · v·tan(δ)/L。|δ| 极小时退化为直线（快速路径，符号无关）。
    """
    delta = clamp(float(delta), -MAX_DELTA, MAX_DELTA)
    if abs(delta) < 1e-3:
        return [(ego_v * dt * (k + 1), 0.0, 0.0) for k in range(n_steps)]
    omega = AEB_PATH_YAWRATE_SIGN * ego_v * math.tan(delta) / max(WHEEL_BASE, 1e-6)
    pts = []
    x = 0.0
    y = 0.0
    theta = 0.0
    for _ in range(n_steps):
        x += ego_v * math.cos(theta) * dt
        y += ego_v * math.sin(theta) * dt
        theta += omega * dt
        pts.append((x, y, theta))
    return pts


def _obstacle_at(obs, t, ego_v):
    """障碍恒速外推：返回 t 时刻障碍在**初始 ego frame** 中的绝对位置。

    v_proj 是相对接近速度（x_rel 负向变化率），障碍自身纵向速度还原为
    v_obs = ego_v − v_proj，故 x_o(t) = x_rel + (ego_v − v_proj)·t
    （静止目标 v_proj=ego_v → x_o 恒为 x_rel ✓；同速前车 v_proj=0 →
    x_o 与自车同步前进、纵向差恒定 ✓）。
    |y| 沿横向逼近速率线性推进，允许穿越 0 到另一侧（自然建模"行人
    横穿通过后离开走廊"）。
    """
    x_o = obs[OBS_X] + (ego_v - obs[OBS_VPROJ]) * t
    y0 = obs[OBS_Y]
    if y0 >= 0.0:
        y_o = y0 - obs[OBS_LATRATE] * t
    else:
        y_o = y0 + obs[OBS_LATRATE] * t
    return x_o, y_o


def _rss_distance(ego_v, v_proj):
    """RSS 纵向最小安全距离。v_obs 由 v_ego − closing 还原（负值截 0）。"""
    v_obs = max(ego_v - max(v_proj, 0.0), 0.0)
    d = (ego_v * AEB_RSS_REACT_S
         + ego_v * ego_v / (2.0 * AEB_RSS_EGO_DECEL)
         - v_obs * v_obs / (2.0 * AEB_RSS_OBS_DECEL))
    return max(d, AEB_RSS_MIN_DIST)


class AebPathChecker:
    """有状态消抖外壳：连续命中 AEB_PATH_CONFIRM_CYCLES 拍才 engage。

    仅持一个整型计数器——deepcopy/pickle 皆廉价（锁步影子核快照友好）。
    """

    def __init__(self):
        self.confirm_count = 0

    def evaluate(self, ego_v, delta, obstacles):
        """单拍评估。纯计算：不读时钟、不做 I/O、无日志。

        参数:
            ego_v:     自车速度 (m/s)
            delta:     当前转角指令 (rad, lateral_ctx.delta)
            obstacles: 障碍元组列表（见模块头约定）
        """
        if ego_v < AEB_PATH_MIN_EGO_V or not obstacles:
            self.confirm_count = max(self.confirm_count - 2, 0)
            return _INACTIVE

        n_steps = int(AEB_PATH_HORIZON_S / AEB_PATH_STEP_S)
        path = None                      # 懒生成：无候选目标时零开销
        best_tcol = float('inf')
        best_tid = -1
        best_rss = 0.0
        rss_violated = False

        for obs in obstacles:
            x0 = obs[OBS_X]
            y0 = obs[OBS_Y]
            # 预过滤：后方目标、横向过远且无逼近趋势的目标不进入检查
            if x0 < -1.0:
                continue
            if abs(y0) > AEB_PATH_PREFILTER_LAT and obs[OBS_LATRATE] <= 0.0:
                continue

            half_w = (VEHICLE_HALF_WIDTH
                      + AEB_PATH_OBS_HALF_WIDTH.get(obs[OBS_CLS],
                                                    AEB_PATH_OBS_HALF_WIDTH[ACTOR_CLASS_VEHICLE])
                      + AEB_PATH_LAT_MARGIN)

            # ── RSS 判据：仅对"当前已在走廊内"的目标 ──
            if abs(y0) <= half_w and x0 > 0.0:
                d_rss = _rss_distance(ego_v, obs[OBS_VPROJ])
                if x0 < d_rss:
                    rss_violated = True
                    if best_tid < 0:
                        best_tid = obs[OBS_TID]
                        best_rss = d_rss

            # ── footprint 路径相交：逐步比较自车/障碍预测位置 ──
            if path is None:
                path = _predict_ego_path(ego_v, delta, n_steps, AEB_PATH_STEP_S)
            prev_dlon = None
            prev_dlat = None
            for k in range(n_steps):
                t_k = (k + 1) * AEB_PATH_STEP_S
                ex, ey, eth = path[k]
                ox, oy = _obstacle_at(obs, t_k, ego_v)
                dx = ox - ex
                dy = oy - ey
                c = math.cos(eth)
                s = math.sin(eth)
                dlon = c * dx + s * dy       # 自车航向系纵向差
                dlat = -s * dx + c * dy      # 自车航向系横向差
                hit = abs(dlon) <= AEB_PATH_LON_MARGIN and abs(dlat) <= half_w
                # 步进跨越保护：相邻两步 dlon 变号（高闭合速度一步跨过碰撞盒）
                crossed = (prev_dlon is not None and prev_dlon > 0.0 and dlon < 0.0
                           and min(abs(prev_dlat), abs(dlat)) <= half_w)
                if hit or crossed:
                    if t_k < best_tcol:
                        best_tcol = t_k
                        best_tid = obs[OBS_TID]
                        best_rss = _rss_distance(ego_v, obs[OBS_VPROJ])
                    break
                prev_dlon = dlon
                prev_dlat = dlat

        risk = rss_violated or best_tcol < float('inf')
        if risk:
            self.confirm_count = min(self.confirm_count + 1,
                                     AEB_PATH_CONFIRM_CYCLES)
        else:
            # 非对称退出（与主 AEB full_confirm 同风格）：一次衰减 2 拍防边界抖动
            self.confirm_count = max(self.confirm_count - 2, 0)

        engaged = risk and self.confirm_count >= AEB_PATH_CONFIRM_CYCLES
        brake = 0.0
        if engaged:
            if best_tcol <= AEB_PATH_TCOL_FULL_S:
                brake = LON_CMD_MAX_BRAKE_DECEL
            elif best_tcol <= AEB_PATH_TCOL_START_S:
                ratio = clamp(
                    (AEB_PATH_TCOL_START_S - best_tcol)
                    / (AEB_PATH_TCOL_START_S - AEB_PATH_TCOL_FULL_S + 1e-6),
                    0.0, 1.0)
                brake = max(brake, ratio * ACC_NORMAL_BRAKE_MAX)
            if rss_violated:
                # RSS 距离不足：至少常规制动力（距离亏空越大越接近上限，由
                # t_col 分支自然接管更高值）
                brake = max(brake, ACC_NORMAL_BRAKE_MAX * 0.5)

        return AebPathDecision(
            engaged=engaged,
            risk=risk,
            brake_cmd=brake,
            t_col=best_tcol,
            obstacle_tid=best_tid,
            rss_dist=best_rss,
            rss_violated=rss_violated,
        )


def fuse_path_decision(lon_ctx, decision):
    """把路径检查结果融合进 LongitudinalContext（只增不减）。

    engaged：lon_cmd = max(原值, brake_cmd)，aeb_active 置 True——下游平滑
    / CommandGate / 心跳 AEB 标志走既有 AEB 通路，无需任何新分支。
    未触发时仅回填诊断字段（aeb_path_risk/tcol/rss），控制量原样。
    """
    if decision is None:
        return lon_ctx
    if decision.engaged:
        return replace(
            lon_ctx,
            lon_cmd=max(lon_ctx.lon_cmd, decision.brake_cmd),
            aeb_active=True,
            aeb_path_risk=True,
            aeb_path_tcol=decision.t_col,
            aeb_rss_dist=decision.rss_dist,
        )
    return replace(
        lon_ctx,
        aeb_path_risk=decision.risk,
        aeb_path_tcol=decision.t_col,
        aeb_rss_dist=decision.rss_dist,
    )
