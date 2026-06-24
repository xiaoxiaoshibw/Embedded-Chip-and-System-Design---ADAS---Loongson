#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""声明式场景 + KPI 评测框架。

读取 scenarios/*.yaml，用一个**闭环**迷你仿真驱动 pipeline.run_pure_pipeline
（ego 真正响应控制器的 lon_cmd / delta），跑完算 KPI 并按场景内 expect 阈值
判通过/失败。一条命令跑全部场景，任一失败进程退出码非 0，适合提交前回归。

被测对象是真实控制器栈（与 ADAS 在线一致），仅"车辆 + 道路 + 前车"是模型。

简化说明（足够覆盖 ACC/AEB/LKA 的安全与舒适 KPI）：
  - 纵向：单质点，ego_accel = -lon_cmd（lon_cmd 正=减速）。
  - 横向：运动学单车模型，前车/道路用弧长 + 恒曲率近似。
  - 这是模型在环（MIL），不替代 Simulink HIL，用于算法回归对比。

依赖 PyYAML（见 requirements.txt）。Python 3.6 兼容。
"""

import argparse
import glob
import math
import os
import sys

from config import LOOP_HZ, WHEEL_BASE
from control.context import VehicleSignals
from pipeline import run_pure_pipeline
from replay import build_stack

# 横向模型与控制器约定的耦合符号：经 straight_cruise（初始偏移 0.3m）验证，
# 取 +1.0 时偏移被 LKA 收敛（0.3→~0.086）；取 -1.0 会发散。
# 若改了控制器横向符号需用 straight_cruise 重新复核此符号。
_LAT_SIGN = 1.0


def _load_yaml(path):
    try:
        import yaml
    except Exception:
        raise SystemExit(
            'run_scenario 需要 PyYAML：python3 -m pip install PyYAML')
    # 显式 utf-8：场景文件含中文注释，Windows 默认 gbk 会解码失败
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _interp_profile(profile, t, key, default=0.0):
    """分段阶梯取值：返回 t 时刻最后一个 time<=t 的段的值。"""
    if not profile:
        return default
    val = profile[0].get(key, default)
    for seg in profile:
        if t >= seg.get('t', 0.0):
            val = seg.get(key, val)
        else:
            break
    return val


def _interp_linear(profile, t, key, default=0.0):
    """分段线性插值（用于横向 y 平滑切入）。"""
    if not profile:
        return default
    pts = [(s.get('t', 0.0), s.get(key, default)) for s in profile]
    if t <= pts[0][0]:
        return pts[0][1]
    for i in range(1, len(pts)):
        t0, y0 = pts[i - 1]
        t1, y1 = pts[i]
        if t <= t1:
            if t1 <= t0:
                return y1
            r = (t - t0) / (t1 - t0)
            return y0 + r * (y1 - y0)
    return pts[-1][1]


def _in_dropout(lead_cfg, t):
    for d in lead_cfg.get('dropout', []) or []:
        if d.get('t_start', -1) <= t < d.get('t_end', -1):
            return True
    return False


def simulate(scn):
    dt = 1.0 / float(LOOP_HZ)
    n_steps = int(round(scn.get('duration_s', 20.0) / dt))
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
    # 前车 actor class（0=UNKNOWN, 1=VEHICLE, 2=OBSTACLE, 3=PEDESTRIAN）
    # 用于测试 class-aware AEB 参数差异化
    lead_cls = int(lead_cfg.get('cls', 0))

    min_gap = float('inf')
    min_ttc = float('inf')
    sum_e2 = 0.0
    sum_jerk2 = 0.0
    n = 0
    n_jerk = 0
    prev_lon = None
    aeb_edges = 0
    prev_aeb = False
    collided = False

    for i in range(n_steps):
        t = i * dt
        now = t + 1.0

        # 前车纵向（分段加速度积分）
        a_lead = _interp_profile(lead_cfg.get('accel_profile'), t, 'a', 0.0)
        lead_v = max(0.0, min(40.0, lead_v + a_lead * dt))
        lead_s += lead_v * dt
        gap = gap0 + (lead_s - ego_s)
        y_lat = _interp_linear(lead_cfg.get('lateral_profile'), t, 'y', 0.0)
        visible = (lead_present and not _in_dropout(lead_cfg, t)
                   and gap > 0.0)

        # 道路航向（恒曲率）
        psi_road += curv * ego_v * dt

        # 组织感知信号
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
            signals.lead_x = gap
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
        delta = res.lateral_ctx.delta

        # ── 闭环：ego 响应控制 ──
        ego_v = max(0.0, min(40.0, ego_v + (-lon) * dt))
        ego_s += ego_v * dt
        steer = max(-0.6, min(0.6, delta))
        ego_yaw += (ego_v / max(WHEEL_BASE, 0.1)) * math.tan(steer) * dt
        he = math.atan2(math.sin(ego_yaw - psi_road),
                        math.cos(ego_yaw - psi_road))
        lat_e += _LAT_SIGN * ego_v * math.sin(he) * dt

        # ── KPI 累计 ──
        if visible:
            if math.isfinite(gap) and gap < 200.0:
                min_gap = min(min_gap, gap)
            ttc = res.lon_ctx.ttc
            if math.isfinite(ttc) and ttc > 0.0:
                min_ttc = min(min_ttc, ttc)
            if gap <= 0.0:
                collided = True
        if math.isfinite(lat_e):
            sum_e2 += lat_e * lat_e
            n += 1
        if prev_lon is not None and math.isfinite(lon):
            j = (lon - prev_lon) / dt
            sum_jerk2 += j * j
            n_jerk += 1
        prev_lon = lon
        aeb = bool(res.lon_ctx.aeb_active)
        if aeb and not prev_aeb:
            aeb_edges += 1
        prev_aeb = aeb
        if collided:
            break

    return {
        'collision': collided,
        'min_gap_m': None if min_gap == float('inf') else round(min_gap, 3),
        'min_ttc_s': None if min_ttc == float('inf') else round(min_ttc, 3),
        'lat_cte_rms_m': round(math.sqrt(sum_e2 / n), 4) if n else None,
        'lon_jerk_rms': round(math.sqrt(sum_jerk2 / n_jerk), 4) if n_jerk else None,
        'aeb_activations': aeb_edges,
    }


def check(kpi, expect):
    """按 expect 中存在的键判定，返回 (passed, [失败原因])。"""
    fails = []
    if 'collision' in expect and bool(kpi['collision']) != bool(expect['collision']):
        fails.append('collision=%s expected %s'
                     % (kpi['collision'], expect['collision']))
    if 'min_gap_min' in expect:
        g = kpi['min_gap_m']
        if g is None or g < expect['min_gap_min']:
            fails.append('min_gap=%s < %s' % (g, expect['min_gap_min']))
    if 'min_ttc_min' in expect:
        v = kpi['min_ttc_s']
        if v is None or v < expect['min_ttc_min']:
            fails.append('min_ttc=%s < %s' % (v, expect['min_ttc_min']))
    if 'lat_cte_rms_max' in expect:
        v = kpi['lat_cte_rms_m']
        if v is None or v > expect['lat_cte_rms_max']:
            fails.append('lat_rms=%s > %s' % (v, expect['lat_cte_rms_max']))
    if 'jerk_rms_max' in expect:
        v = kpi['lon_jerk_rms']
        if v is not None and v > expect['jerk_rms_max']:
            fails.append('jerk_rms=%s > %s' % (v, expect['jerk_rms_max']))
    if 'aeb_activations_min' in expect:
        if kpi['aeb_activations'] < expect['aeb_activations_min']:
            fails.append('aeb=%d < %d'
                         % (kpi['aeb_activations'], expect['aeb_activations_min']))
    if 'aeb_activations_max' in expect:
        if kpi['aeb_activations'] > expect['aeb_activations_max']:
            fails.append('aeb=%d > %d'
                         % (kpi['aeb_activations'], expect['aeb_activations_max']))
    return (len(fails) == 0), fails


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description='ADAS 场景 KPI 评测')
    parser.add_argument('path', nargs='?',
                        default=os.path.join(here, 'scenarios'),
                        help='场景 yaml 或目录（默认 scenarios/）')
    args = parser.parse_args(argv)

    if os.path.isdir(args.path):
        files = sorted(glob.glob(os.path.join(args.path, '*.yaml')))
    else:
        files = [args.path]
    if not files:
        raise SystemExit('未找到场景文件: %s' % args.path)

    n_fail = 0
    print('%-22s %-7s %-9s %-9s %-9s %-9s %-4s  %s' % (
        'scenario', 'result', 'min_gap', 'min_ttc', 'lat_rms',
        'jerk_rms', 'aeb', 'detail'))
    print('-' * 100)
    for fp in files:
        scn = _load_yaml(fp)
        name = scn.get('name', os.path.basename(fp))
        kpi = simulate(scn)
        passed, fails = check(kpi, scn.get('expect', {}) or {})
        if not passed:
            n_fail += 1
        print('%-22s %-7s %-9s %-9s %-9s %-9s %-4s  %s' % (
            name,
            'PASS' if passed else 'FAIL',
            kpi['min_gap_m'], kpi['min_ttc_s'], kpi['lat_cte_rms_m'],
            kpi['lon_jerk_rms'], kpi['aeb_activations'],
            '' if passed else '; '.join(fails)))

    print('-' * 100)
    print('%d/%d passed' % (len(files) - n_fail, len(files)))
    return 1 if n_fail else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
