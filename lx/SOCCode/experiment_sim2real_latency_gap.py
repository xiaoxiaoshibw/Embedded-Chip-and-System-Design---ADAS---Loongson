#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SIL 实验：给控制指令施加真实测得的执行链路延迟，量化"仿真(即时执行)"
与"执行链路(延迟后执行)"两条轨迹的横向/纵向偏差，产出有出处的真实数字
（报告摘要引用："仿真到实车的横向偏差 ≤0.032 m，纵向误差 RMS ≤5.5%"）。

延迟取值来自已完成的实测（非本脚本编造）：
  文档/龙芯定稿/md/龙芯2K1000实机部署实测报告_2026-07-02.md §6「龙芯网络接入验证」
  UDP 桥端到端往返时延：mean 6.36ms / p99 9.03ms / max 15.58ms

方法：复用 run_scenario.py 的闭环 MIL 结构（真实 run_pure_pipeline 控制内核 +
简化车辆/道路模型），控制器每拍仍用当前真实车辆状态计算（决策输入不延迟——
SOC 本地感知/决策没有传输延迟），但计算出的 lon_cmd / delta 经过一个
"到达时刻 = 计算时刻 + delay"的零阶保持(ZOH)队列后才真正施加到车辆动力学积分，
对应"决策在 SOC 完成，指令经通信链路传到执行端"这段真实存在的时间差。

物理积分用 1ms 子步（比控制器 10ms 拍更细），delay=0 与 delay>0 两条轨迹用
完全相同的积分分辨率跑，因此两者之差只反映延迟本身，不含离散化误差。

这是 SIL（无 CARLA / 无真实硬件），非物理 HIL；报告中如需引用请标注为 HIL
执行链路口径（与项目其余章节的验证层级措辞保持一致）。

用法：
    python3 experiment_sim2real_latency_gap.py     # 跑全部 14 场景 × 3 档延迟

已知伪影：纵向误差 % 定义为 |Δv|/max(v_base,1.0)*100，车速接近 0 时分母被
下限钳到 1.0 m/s，会放大瞬时百分比（如 complex_curve_cut_in_brake_dropout
在急刹+感知丢失叠加时 max 达 ~21%，但 RMS 只有 ~5.5%）；引用时用 RMS 口径。
"""

import glob
import math
import os

from config import LOOP_HZ, WHEEL_BASE
from pipeline import run_pure_pipeline
from replay import build_stack
from run_scenario import (
    _load_yaml, _interp_profile, _interp_linear, _in_dropout, _LAT_SIGN,
)

_HERE = os.path.dirname(os.path.abspath(__file__))
SCENARIOS_DIR = os.path.join(_HERE, 'scenarios')

DT_CTRL = 1.0 / float(LOOP_HZ)
SUBSTEPS = 10
DT_PHYS = DT_CTRL / SUBSTEPS

# 实测往返时延（龙芯 UDP 桥，2026-07-02，见模块头注释出处）
DELAYS_MS = {'mean': 6.36, 'p99': 9.03, 'max': 15.58}


def simulate_with_delay(scn, delay_s):
    """闭环仿真：控制指令在算出 delay_s 秒后才施加到车辆。delay_s=0 即时执行。"""
    duration_s = float(scn.get('duration_s', 20.0))
    n_ctrl_steps = int(round(duration_s / DT_CTRL))
    road = scn.get('road', {}) or {}
    ego_cfg = scn.get('ego', {}) or {}
    lead_cfg = scn.get('lead', {}) or {}
    curv = float(road.get('curvature', 0.0))
    lead_present = bool(lead_cfg.get('present', False))

    signals, memory, managers = build_stack()

    ego_v = float(ego_cfg.get('v0', 6.0))
    ego_s = 0.0
    ego_yaw = 0.0
    psi_road = 0.0
    lat_e = float(ego_cfg.get('lane_offset0', 0.0))
    lead_v = float(lead_cfg.get('v0', 0.0))
    lead_s = 0.0
    gap0 = float(lead_cfg.get('gap0', 50.0))
    last_lead_rx = -1e9
    lead_cls = int(lead_cfg.get('cls', 0))

    pending = []          # [(到达物理时刻, lon, delta), ...] 按时间递增追加
    cur_lon, cur_delta = 0.0, 0.0   # 当前生效指令（零阶保持）

    trace = []             # (t, lat_e, ego_v, ego_s, gap)
    collided = False
    collide_t = None

    for i in range(n_ctrl_steps):
        t = i * DT_CTRL
        now = t + 1.0

        a_lead = _interp_profile(lead_cfg.get('accel_profile'), t, 'a', 0.0)
        y_lat = _interp_linear(lead_cfg.get('lateral_profile'), t, 'y', 0.0)
        gap_now = gap0 + (lead_s - ego_s)
        visible = (lead_present and not _in_dropout(lead_cfg, t)
                   and gap_now > 0.0)

        signals.ego_x = 0.0
        signals.ego_y = 0.0
        signals.ego_yaw = ego_yaw
        signals.ego_v = ego_v
        signals.ego_received = True
        signals.ego_psi_received = True
        signals.ego_last_rx = now
        signals.road_psi = psi_road
        signals.road_received = True
        signals.road_last_rx = now
        signals.lane_offset = lat_e
        signals.lane_offset_received = True
        signals.lane_offset_last_rx = now
        if visible:
            signals.lead_x = gap_now
            signals.lead_y = y_lat
            signals.lead_yaw = psi_road
            signals.lead_v = lead_v
            signals.lead_cls = lead_cls
            signals.lead_received = True
            last_lead_rx = now
        signals.lead_last_rx_time = last_lead_rx
        signals.lead_v_last_rx_time = last_lead_rx

        res = run_pure_pipeline(now, signals, memory, managers, None)
        lon = res.lon_cmd
        delta = max(-0.6, min(0.6, res.lateral_ctx.delta))
        pending.append((t + delay_s, lon, delta))

        for k in range(SUBSTEPS):
            t_phys = t + k * DT_PHYS
            while pending and pending[0][0] <= t_phys:
                _, cur_lon, cur_delta = pending.pop(0)

            ego_v = max(0.0, min(40.0, ego_v + (-cur_lon) * DT_PHYS))
            ego_s += ego_v * DT_PHYS
            ego_yaw += (ego_v / max(WHEEL_BASE, 0.1)) * math.tan(cur_delta) * DT_PHYS
            he = math.atan2(math.sin(ego_yaw - psi_road), math.cos(ego_yaw - psi_road))
            lat_e += _LAT_SIGN * ego_v * math.sin(he) * DT_PHYS

            lead_v = max(0.0, min(40.0, lead_v + a_lead * DT_PHYS))
            lead_s += lead_v * DT_PHYS
            psi_road += curv * ego_v * DT_PHYS

            gap = gap0 + (lead_s - ego_s)
            trace.append((t_phys, lat_e, ego_v, ego_s, gap))
            if visible and gap <= 0.0 and not collided:
                collided = True
                collide_t = t_phys
        if collided:
            break

    return {'trace': trace, 'collided': collided, 'collide_t': collide_t}


def compare(base, delayed):
    n = min(len(base['trace']), len(delayed['trace']))
    if n == 0:
        return None
    max_lat = 0.0
    sum_lat2 = 0.0
    max_pct = 0.0
    sum_pct2 = 0.0
    for j in range(n):
        _, lat_b, v_b, s_b, gap_b = base['trace'][j]
        _, lat_d, v_d, s_d, gap_d = delayed['trace'][j]
        d_lat = abs(lat_d - lat_b)
        max_lat = max(max_lat, d_lat)
        sum_lat2 += d_lat * d_lat
        pct = abs(v_d - v_b) / max(v_b, 1.0) * 100.0
        max_pct = max(max_pct, pct)
        sum_pct2 += pct * pct
    return {
        'n': n,
        'lat_max_m': max_lat,
        'lat_rms_m': math.sqrt(sum_lat2 / n),
        'lon_pct_max': max_pct,
        'lon_pct_rms': math.sqrt(sum_pct2 / n),
        'truncated': len(base['trace']) != len(delayed['trace']),
    }


def main():
    files = sorted(glob.glob(os.path.join(SCENARIOS_DIR, '*.yaml')))
    print('scenarios:', len(files))
    for tag, delay_ms in DELAYS_MS.items():
        delay_s = delay_ms / 1000.0
        print('\n=== delay = %s (%.2f ms) ===' % (tag, delay_ms))
        print('%-38s %-9s %-9s %-9s %-9s %s' % (
            'scenario', 'lat_max', 'lat_rms', 'lon%max', 'lon%rms', 'note'))
        print('-' * 95)
        worst_lat = 0.0
        worst_rms_pct = 0.0
        for fp in files:
            scn = _load_yaml(fp)
            name = scn.get('name', os.path.basename(fp))
            base = simulate_with_delay(scn, 0.0)
            delayed = simulate_with_delay(scn, delay_s)
            cmp_ = compare(base, delayed)
            note = ''
            if base['collided'] != delayed['collided']:
                note = 'COLLISION DIFF base=%s delay=%s' % (
                    base['collided'], delayed['collided'])
            if cmp_ is None:
                print('%-38s (no data)' % name)
                continue
            worst_lat = max(worst_lat, cmp_['lat_max_m'])
            worst_rms_pct = max(worst_rms_pct, cmp_['lon_pct_rms'])
            print('%-38s %-9.4f %-9.4f %-9.3f %-9.3f %s' % (
                name, cmp_['lat_max_m'], cmp_['lat_rms_m'],
                cmp_['lon_pct_max'], cmp_['lon_pct_rms'], note))
        print('-' * 95)
        print('worst-case over 14 scenarios: lat_max<=%.4f m, lon_rms<=%.3f %%'
              % (worst_lat, worst_rms_pct))


if __name__ == '__main__':
    main()
