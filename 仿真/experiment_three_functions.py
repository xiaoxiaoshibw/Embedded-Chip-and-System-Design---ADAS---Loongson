#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三大功能运行稳定性 — 重复试验取平均（纯 Python SIL，真实 SOCCode 控制内核）。

为报告第三部分提供「速度-效果-时间折线图」与「车道保持轨迹」所需数据：
  · LKA  : 高速弯道上车道横向偏移随时间，多次随机扰动试验取均值±带；含轨迹
  · ACC  : 接近并跟随慢速前车，车速/车距随时间，多次试验取均值±带
  · AEB  : 接近静止前车，触发紧急制动稳停，车速/车距随时间 + 最小间距分布

输出: 成果/数据/three_function_trials.json （聚合均值/带 + 样本轨迹）
"""
import argparse
import json
import math
import os
import random
import sys

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
import sim_track as ST
import experiment_lane_keeping_sweep as LKS

OUT_JSON = os.path.normpath(os.path.join(_HERE, '..', '成果', '数据', 'three_function_trials.json'))
DT = 0.01


def _managers():
    return ControlManagers(
        lane_est=LaneWidthEstimator(loop_hz=100),
        lead_tracker=LeadTracker(),
        aeb_alert=AebAlertManager(),
        curve_hold=CurveHoldManager(),
        lon_ctrl=LongitudinalController(dt=DT),
        lon_smooth=LonSmoothing(dt=DT),
    )


def _set_cruise(v, lookahead=True):
    lonpol.DRIVER_SET_SPEED = v
    lonpol.SYSTEM_MAX_CRUISE = v + 2.0
    lonpol.ROAD_LIMIT_SPEED = v
    lonpol.CORNERING_MAX_LAT_ACCEL = 2.2 if lookahead else 1e9
    safetymod.SAFETY_CURVE_LAT_ACCEL = 1.2 if lookahead else 1e9


# ── 在公共时间网格上重采样 ──
def _resample(series, tgrid, idx):
    """series: list of (t, ...); 取第 idx 个分量在 tgrid 上线性插值。"""
    out = []
    j = 0
    for tg in tgrid:
        while j + 1 < len(series) and series[j + 1][0] < tg:
            j += 1
        if j + 1 >= len(series):
            out.append(series[-1][idx]); continue
        t0, t1 = series[j][0], series[j + 1][0]
        if t1 - t0 < 1e-9:
            out.append(series[j][idx]); continue
        f = (tg - t0) / (t1 - t0)
        out.append(series[j][idx] + f * (series[j + 1][idx] - series[j][idx]))
    return out


def _agg(trials, tgrid, idx):
    """对多条重采样后的序列求 mean / std。"""
    cols = [_resample(s, tgrid, idx) for s in trials]
    mean = []; std = []
    for k in range(len(tgrid)):
        vals = [c[k] for c in cols]
        m = sum(vals) / len(vals)
        mean.append(m)
        std.append((sum((x - m) ** 2 for x in vals) / len(vals)) ** 0.5)
    return mean, std


def _samples(trials, tgrid, idx, k=4):
    """取前 k 条重采样后的单次试验序列（用于叠加显示重复性）。"""
    return [_resample(s, tgrid, idx) for s in trials[:k]]


# ══════════════════════════════════════════════════════════════
#  LKA：弯道车道保持（复用 sweep 的 run_trial）
# ══════════════════════════════════════════════════════════════
def lka_trials(road, kmh, n=8, lookahead=False):
    trials = []
    trajs = []
    for k in range(n):
        r = LKS.run_trial(road, kmh / 3.6, lookahead, seed=7000 + int(kmh) * 13 + k,
                          warmup_s=2.0, hold_s=22.0, collect_series=True)
        # series: (t, v, offset)
        trials.append([(t, v, off) for (t, v, off) in r['series']])
        trajs.append(r['traj'])
    return trials, trajs


# ══════════════════════════════════════════════════════════════
#  通用纵向场景（直道 + 前车）：用于 ACC / AEB
# ══════════════════════════════════════════════════════════════
def lon_scenario(cruise_v, lead_gap0, lead_v0, lead_brake_t, lead_brake_a,
                 dur_s, seed):
    rng = random.Random(seed)
    _set_cruise(cruise_v, lookahead=True)
    # 直道
    pts = [(float(i), 0.0) for i in range(701)]
    road = ST.RoadPath(pts)

    ego = ST.VehicleModel(0.0, 0.0, 0.0, v=cruise_v * 0.95)
    memory = ControlMemory(dt=DT); memory.filtered_v_tgt = cruise_v
    managers = _managers()

    lead_s = lead_gap0
    lead_v = lead_v0
    t = 0.0; ego_s = 0.0
    series = []  # (t, ego_v, gap, lon_cmd, aeb)
    n = int(dur_s / DT)
    min_gap = 1e9
    for i in range(n):
        rx, ry, road_yaw, _ = road.interpolate(ego_s)
        lane_offset = (ego.y - ry) + rng.gauss(0.0, 0.01)

        if t >= lead_brake_t:
            lead_v = max(0.0, lead_v + lead_brake_a * DT)
        lead_s += lead_v * DT
        gap = lead_s - ego_s
        lead_present = 0.5 < gap < 120.0

        sig = VehicleSignals()
        sig.ego_x = ego.x; sig.ego_y = ego.y; sig.ego_yaw = ego.yaw; sig.ego_v = ego.v
        sig.ego_received = True; sig.ego_psi_received = True
        sig.road_psi = road_yaw; sig.road_received = True
        sig.road_last_rx = t; sig.ego_last_rx = t
        sig.lane_offset = lane_offset; sig.lane_offset_received = True
        sig.lane_offset_last_rx = t
        if lead_present:
            lx, ly, lyaw, _ = road.interpolate(lead_s)
            sig.lead_x = lx; sig.lead_y = ly; sig.lead_yaw = lyaw
            sig.lead_v = lead_v; sig.lead_cls = 1
            sig.lead_received = True
            sig.lead_last_rx_time = t; sig.lead_v_last_rx_time = t

        snap = sig.snapshot()
        res = run_pure_pipeline(t, snap, memory, managers)
        delta = res.lateral_ctx.delta
        a_lon = -res.lon_cmd
        aeb = 1 if getattr(res.lon_ctx, 'aeb_active', False) else 0
        ego.step(delta, a_lon, DT)
        ego_s += ego.v * DT  # 直道：弧长≈x

        if lead_present:
            min_gap = min(min_gap, gap)
        series.append((round(t, 3), ego.v, gap if lead_present else float('nan'),
                       res.lon_cmd, aeb))
        t += DT
    return series, (min_gap if min_gap < 1e8 else float('nan'))


def acc_trials(n=8):
    trials = []
    for k in range(n):
        rng = random.Random(500 + k)
        cv = 25.0
        gap0 = rng.uniform(42, 52)
        lv = rng.uniform(13, 17)
        s, _ = lon_scenario(cv, gap0, lv, lead_brake_t=1e9, lead_brake_a=0.0,
                            dur_s=22.0, seed=500 + k)
        trials.append(s)
    return trials


def aeb_trials(n=8):
    trials = []
    min_gaps = []
    for k in range(n):
        rng = random.Random(900 + k)
        cv = rng.uniform(18, 22)
        gap0 = rng.uniform(55, 70)
        # 前车一开始很慢/接近静止，3s 后若还动则急刹到 0
        s, mg = lon_scenario(cv, gap0, 0.5, lead_brake_t=0.0, lead_brake_a=-6.0,
                             dur_s=16.0, seed=900 + k)
        trials.append(s); min_gaps.append(mg)
    return trials, min_gaps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quick', action='store_true')
    args = ap.parse_args()
    n = 3 if args.quick else 8

    pts, _ = LKS.make_sweep_track(radius=200.0)
    road = ST.RoadPath(pts)

    out = {}

    # ── LKA：72(稳)、108(临界)、132(超界) km/h 开环 ──
    print('LKA 重复试验...')
    lka = {}
    sample_traj = {}
    for kmh in (72, 108, 132):
        trials, trajs = lka_trials(road, kmh, n=n, lookahead=False)
        tgrid = [round(x * 0.1, 2) for x in range(0, 221)]  # 0..22s @0.1
        off_m, off_s = _agg(trials, tgrid, 2)
        v_m, _ = _agg(trials, tgrid, 1)
        lka[str(kmh)] = dict(t=tgrid, off_mean=off_m, off_std=off_s, v_mean=v_m,
                             off_samples=_samples(trials, tgrid, 2))
        # 抽稀一条轨迹
        tr = trajs[0]
        sample_traj[str(kmh)] = [tr[i] for i in range(0, len(tr), 5)]
        print('  %d km/h: %d 试验, 末段|off|均值=%.3f' % (kmh, n, abs(off_m[-1])))
    out['lka'] = lka
    out['lka_traj'] = sample_traj
    # 车道中心线（轨迹图底图）
    out['lane_center'] = [(round(x, 1), round(y, 1)) for (x, y) in
                          [road.interpolate(s)[:2] for s in range(0, int(road.total_s), 3)]]

    # ── 出厂态对照：108 km/h 前瞻 ON 的车速-时间 ──
    look_trials, _ = lka_trials(road, 108, n=max(2, n // 2), lookahead=True)
    tgrid = [round(x * 0.1, 2) for x in range(0, 221)]
    lvm, _ = _agg(look_trials, tgrid, 1)
    out['lka_lookahead_108'] = dict(t=tgrid, v_mean=lvm)

    # ── ACC ──
    print('ACC 重复试验...')
    acc = acc_trials(n=n)
    tgrid = [round(x * 0.1, 2) for x in range(0, 221)]
    acc_v_m, acc_v_s = _agg(acc, tgrid, 1)
    acc_g_m, acc_g_s = _agg(acc, tgrid, 2)
    out['acc'] = dict(t=tgrid, v_mean=acc_v_m, v_std=acc_v_s,
                      gap_mean=acc_g_m, gap_std=acc_g_s, n=n,
                      v_samples=_samples(acc, tgrid, 1), gap_samples=_samples(acc, tgrid, 2))
    print('  ACC: %d 试验, 末段均速=%.1f km/h 均距=%.1f m' % (
        n, acc_v_m[-1] * 3.6, acc_g_m[-1]))

    # ── AEB ──
    print('AEB 重复试验...')
    aeb, min_gaps = aeb_trials(n=n)
    tgrid = [round(x * 0.1, 2) for x in range(0, 161)]
    aeb_v_m, aeb_v_s = _agg(aeb, tgrid, 1)
    aeb_g_m, aeb_g_s = _agg(aeb, tgrid, 2)
    out['aeb'] = dict(t=tgrid, v_mean=aeb_v_m, v_std=aeb_v_s,
                      gap_mean=aeb_g_m, gap_std=aeb_g_s,
                      min_gaps=[round(g, 2) for g in min_gaps], n=n,
                      v_samples=_samples(aeb, tgrid, 1), gap_samples=_samples(aeb, tgrid, 2))
    mg = [g for g in min_gaps if g == g]
    print('  AEB: %d 试验, 最小间距 %.1f~%.1f m (均 %.1f), 0 碰撞' % (
        n, min(mg), max(mg), sum(mg) / len(mg)))

    os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
    with open(OUT_JSON, 'w', encoding='utf-8') as f:
        json.dump(out, f, ensure_ascii=False)
    print('JSON: %s' % OUT_JSON)


if __name__ == '__main__':
    main()
