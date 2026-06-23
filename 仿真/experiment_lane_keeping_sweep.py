#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车道保持速度边界扫描（纯 Python SIL，真实 SOCCode 控制内核）。

复现并刷新报告第三部分的「车道保持速度稳定边界」数据：在同一条高速弯道
赛道上，对一组目标车速做扫描，分别在「关闭弯道前瞻限速（探横向控制器开环
极限）」与「启用弯道前瞻限速（出厂态）」两种配置下，各做 N 次带随机扰动的
重复试验取平均，统计稳态横向误差 RMS / 峰值 / 是否发散，并插值经验临界速度
V_crit。

控制内核为 soc_code/pipeline.run_pure_pipeline（本地副本，与 lx/SOCCode/ 同步），
世界端用自行车模型 + 弧长投影感知（与 仿真/sim_track.py 同源）。

用法:
    python experiment_lane_keeping_sweep.py                 # 全扫描 + 出 CSV
    python experiment_lane_keeping_sweep.py --quick         # 少量速度/试验，自检
"""
import argparse
import csv
import math
import os
import random
import sys
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, 'soc_code'))

from control.context import ControlManagers, ControlMemory, VehicleSignals
from pipeline import run_pure_pipeline
from lateral import LaneWidthEstimator
from control.lead_tracking import LeadTracker
from control.aeb_alert import AebAlertManager
from control.curve_hold import CurveHoldManager
from longitudinal import LongitudinalController, LonSmoothing
import control.longitudinal_policy as lonpol
import control.safety as safetymod

# 复用 sim_track 的路径插值与车辆模型
import sim_track as ST

OUT_DATA = os.path.normpath(os.path.join(_HERE, '..', '成果', '数据'))


# ══════════════════════════════════════════════════════════════
#  扫描专用赛道：缓和高速弯道（接近 Town04 高速环路）
# ══════════════════════════════════════════════════════════════
def make_sweep_track(radius=200.0, arc_deg=130.0, warmup=90.0):
    """直道 warmup + 单一缓弯（半径 radius，弧度 arc_deg）。"""
    pts = []
    for i in range(int(warmup) + 1):
        pts.append((float(i), 0.0))
    x0, y0 = pts[-1]
    arc = math.radians(arc_deg)
    n = int(radius * arc / 1.0)  # 约 1m 一个点
    for i in range(1, n + 1):
        th = arc * i / n
        x = x0 + radius * math.sin(th)
        y = y0 + radius * (1.0 - math.cos(th))
        pts.append((x, y))
    return pts, 'R=%.0fm 高速弯道' % radius


# ══════════════════════════════════════════════════════════════
#  单次试验
# ══════════════════════════════════════════════════════════════
def _set_speed_and_mode(target_v, lookahead_on):
    lonpol.DRIVER_SET_SPEED = target_v
    lonpol.SYSTEM_MAX_CRUISE = target_v + 2.0
    lonpol.ROAD_LIMIT_SPEED = target_v
    if lookahead_on:
        lonpol.CORNERING_MAX_LAT_ACCEL = 2.2
        safetymod.SAFETY_CURVE_LAT_ACCEL = 1.2
    else:
        lonpol.CORNERING_MAX_LAT_ACCEL = 1e9
        safetymod.SAFETY_CURVE_LAT_ACCEL = 1e9


def run_trial(road, target_v, lookahead_on, seed, warmup_s=9.0, hold_s=18.0, dt=0.01,
              collect_series=False):
    rng = random.Random(seed)
    _set_speed_and_mode(target_v, lookahead_on)

    # 起点 + 随机初始扰动
    sx0, sy0, syaw0, _ = road.interpolate(0.0)
    off0 = rng.uniform(-0.35, 0.35)       # 初始横向偏移
    yaw_err0 = math.radians(rng.uniform(-2.5, 2.5))
    ego = ST.VehicleModel(sx0 - math.sin(syaw0) * off0,
                          sy0 + math.cos(syaw0) * off0,
                          syaw0 + yaw_err0,
                          v=target_v * 0.85)

    memory = ControlMemory(dt=dt)
    memory.filtered_v_tgt = target_v
    managers = ControlManagers(
        lane_est=LaneWidthEstimator(loop_hz=100),
        lead_tracker=LeadTracker(),
        aeb_alert=AebAlertManager(),
        curve_hold=CurveHoldManager(),
        lon_ctrl=LongitudinalController(dt=dt),
        lon_smooth=LonSmoothing(dt=dt),
    )

    t = 0.0
    ego_s = 0.0
    warm_n = int(warmup_s / dt)
    max_off = 0.0
    sum_sq = 0.0
    cnt = 0
    sum_v = 0.0
    diverged = False
    series = [] if collect_series else None
    traj = [] if collect_series else None
    i = 0
    total_n = int((warmup_s + hold_s) / dt)

    while i < total_n and ego_s < road.total_s - 1.5:
        rx, ry, road_yaw, road_curv = road.interpolate(ego_s)
        dx = ego.x - rx
        dy = ego.y - ry
        c, s = math.cos(road_yaw), math.sin(road_yaw)
        lane_offset = -s * dx + c * dy
        meas_off = lane_offset + rng.gauss(0.0, 0.02)  # 感知噪声

        signals = VehicleSignals()
        signals.ego_x = ego.x; signals.ego_y = ego.y
        signals.ego_yaw = ego.yaw; signals.ego_v = ego.v
        signals.ego_received = True; signals.ego_psi_received = True
        signals.road_psi = road_yaw; signals.road_received = True
        signals.road_last_rx = t; signals.ego_last_rx = t
        signals.lane_offset = meas_off; signals.lane_offset_received = True
        signals.lane_offset_last_rx = t

        snap = signals.snapshot()
        result = run_pure_pipeline(t, snap, memory, managers)
        delta = result.lateral_ctx.delta
        a_lon = -result.lon_cmd
        ego.step(delta, a_lon, dt)

        # 重新投影到路径
        best_ds = ego_s; best_dist = 1e9
        for ds_off in range(-3, 9):
            ss = ego_s + ds_off * 1.0
            if ss < 0 or ss > road.total_s:
                continue
            px, py, _, _ = road.interpolate(ss)
            d = math.hypot(ego.x - px, ego.y - py)
            if d < best_dist:
                best_dist = d; best_ds = ss
        ego_s = best_ds

        if i >= warm_n:
            a = abs(lane_offset)
            max_off = max(max_off, a)
            sum_sq += a * a; cnt += 1; sum_v += ego.v
            if a > 1.75:
                diverged = True
        if collect_series:
            series.append((t, ego.v, lane_offset))
            traj.append((ego.x, ego.y))
        t += dt; i += 1

    rms = math.sqrt(sum_sq / cnt) if cnt else 0.0
    mean_v = sum_v / cnt if cnt else 0.0
    return dict(rms=rms, max_off=max_off, mean_v=mean_v, diverged=diverged,
                series=series, traj=traj)


def sweep(road, speeds_kmh, lookahead_on, n_trials=6):
    rows = []
    for vk in speeds_kmh:
        tv = vk / 3.6
        rmss = []; maxs = []; means = []; divs = 0
        for k in range(n_trials):
            r = run_trial(road, tv, lookahead_on, seed=1000 * int(vk) + k)
            rmss.append(r['rms']); maxs.append(r['max_off'])
            means.append(r['mean_v']); divs += 1 if r['diverged'] else 0
        rms_mean = sum(rmss) / len(rmss)
        rms_std = (sum((x - rms_mean) ** 2 for x in rmss) / len(rmss)) ** 0.5
        row = dict(target_kmh=vk, target_v=round(tv, 2),
                   rms_mean=round(rms_mean, 4), rms_std=round(rms_std, 4),
                   max_off=round(max(maxs), 3),
                   mean_v=round(sum(means) / len(means), 2),
                   mean_v_kmh=round(sum(means) / len(means) * 3.6, 1),
                   diverge_rate=round(divs / n_trials, 2), n=n_trials)
        rows.append(row)
        print('  v=%3.0f km/h  RMS=%.3f±%.3f  |off|max=%.2f  实测均速=%.1f km/h  发散率=%.0f%%' % (
            vk, rms_mean, rms_std, row['max_off'], row['mean_v_kmh'], 100 * divs / n_trials))
    return rows


def v_crit_interp(rows, thresh=0.5):
    """RMS 穿越 thresh 的线性插值临界速度 (km/h)。"""
    for a, b in zip(rows, rows[1:]):
        if a['rms_mean'] <= thresh < b['rms_mean']:
            f = (thresh - a['rms_mean']) / (b['rms_mean'] - a['rms_mean'])
            return a['target_kmh'] + f * (b['target_kmh'] - a['target_kmh'])
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quick', action='store_true')
    ap.add_argument('--trials', type=int, default=6)
    ap.add_argument('--radius', type=float, default=200.0)
    args = ap.parse_args()

    pts, desc = make_sweep_track(radius=args.radius)
    road = ST.RoadPath(pts)
    print('扫描赛道: %s  总长 %.0fm' % (desc, road.total_s))

    if args.quick:
        speeds = [36, 72, 96, 120]
        trials = 2
    else:
        speeds = [36, 48, 60, 72, 84, 96, 108, 120, 132, 144]
        trials = args.trials

    print('\n===== 关闭弯道前瞻限速（横向控制器开环极限） =====')
    raw = sweep(road, speeds, lookahead_on=False, n_trials=trials)
    print('\n===== 启用弯道前瞻限速（出厂态） =====')
    look = sweep(road, speeds, lookahead_on=True, n_trials=trials)

    vc = v_crit_interp(raw)
    print('\n横向控制器开环经验临界速度 V_crit ≈ %.1f km/h' % (vc or -1))
    # 出厂态最高有界速度
    bounded = [r['target_kmh'] for r in look if r['rms_mean'] <= 0.5 and r['diverge_rate'] == 0]
    print('出厂态（前瞻限速 ON）全程有界最高目标车速 ≥ %d km/h' % (max(bounded) if bounded else -1))

    if not args.quick:
        os.makedirs(OUT_DATA, exist_ok=True)
        cols = ['target_kmh', 'target_v', 'rms_mean', 'rms_std', 'max_off',
                'mean_v', 'mean_v_kmh', 'diverge_rate', 'n']
        for tag, rows in (('raw', raw), ('lookahead', look)):
            p = os.path.join(OUT_DATA, 'lane_keeping_sweep_%s.csv' % tag)
            with open(p, 'w', newline='', encoding='utf-8') as fh:
                w = csv.DictWriter(fh, fieldnames=cols); w.writeheader(); w.writerows(rows)
            print('CSV: %s' % p)


if __name__ == '__main__':
    main()
