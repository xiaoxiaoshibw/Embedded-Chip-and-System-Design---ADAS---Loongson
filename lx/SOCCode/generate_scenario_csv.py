#!/usr/bin/env python3
"""Run scenarios and emit 55-column telemetry CSVs.

Mirrors run_scenario.py's simulation loop exactly, but also writes CSV rows.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import LOOP_HZ, WHEEL_BASE
from pipeline import run_pure_pipeline
from replay import build_stack
from telemetry import FIELDS

import yaml


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _interp_profile(profile, t, key, default=0.0):
    if not profile:
        return default
    for seg in profile:
        if seg.get('t', -1) > t:
            break
        default = seg.get(key, default)
    return default


def _interp_linear(profile, t, key, default=0.0):
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


def simulate_and_write_csv(scn, out_path):
    """Run one scenario exactly like run_scenario.py, write CSV, return KPIs."""
    dt = 1.0 / float(LOOP_HZ)
    n_steps = int(round(scn.get('duration_s', 20.0) / dt))
    road = scn.get('road', {}) or {}
    ego_cfg = scn.get('ego', {}) or {}
    lead_cfg = scn.get('lead', {}) or {}
    curv = float(road.get('curvature', 0.0))
    lead_present = bool(lead_cfg.get('present', False))
    lead_cls = int(lead_cfg.get('cls', 0))

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

    min_gap = float('inf')
    min_ttc = float('inf')
    sum_e2 = 0.0
    sum_jerk2 = 0.0
    n_jerk = 0
    prev_lon = None
    aeb_count = 0
    prev_aeb = False
    collided = False
    n = 0

    rows = []
    _LAT_SIGN = 1.0

    for i in range(n_steps):
        t = i * dt
        now = t + 1.0

        # Lead dynamics
        a_lead = _interp_profile(lead_cfg.get('accel_profile'), t, 'a', 0.0)
        lead_v = max(0.0, min(40.0, lead_v + a_lead * dt))
        lead_s += lead_v * dt
        gap = gap0 + (lead_s - ego_s)
        y_lat = _interp_linear(lead_cfg.get('lateral_profile'), t, 'y', 0.0)
        visible = (lead_present and not _in_dropout(lead_cfg, t) and gap > 0.0)

        # Road heading
        psi_road += curv * ego_v * dt

        # Signals (same as run_scenario.py)
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

        # Pipeline
        res = run_pure_pipeline(now, signals, memory, managers, None)
        lon = res.lon_cmd
        delta = res.lateral_ctx.delta

        # Closed-loop ego dynamics
        ego_v = max(0.0, min(40.0, ego_v + (-lon) * dt))
        ego_s += ego_v * dt
        steer = max(-0.6, min(0.6, delta))
        ego_yaw += (ego_v / max(WHEEL_BASE, 0.1)) * math.tan(steer) * dt
        he = math.atan2(math.sin(ego_yaw - psi_road), math.cos(ego_yaw - psi_road))
        lat_e += _LAT_SIGN * ego_v * math.sin(he) * dt

        # KPIs
        if visible and math.isfinite(gap) and gap < 200.0:
            min_gap = min(min_gap, gap)
        ttc = res.lon_ctx.ttc
        if visible and math.isfinite(ttc) and ttc > 0.0:
            min_ttc = min(min_ttc, ttc)
        if visible and gap <= 0.0:
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
            aeb_count += 1
        prev_aeb = aeb

        # CSV row (55 columns)
        row = {f: '' for f in FIELDS}
        row['t_wall'] = repr(now)
        row['t_mono'] = repr(now)
        row['cycle'] = str(i)
        row['ego_x'] = repr(ego_s)
        row['ego_y'] = repr(lat_e)
        row['ego_yaw'] = repr(ego_yaw)
        row['ego_v'] = repr(ego_v)
        if visible:
            row['lead_x'] = repr(gap)
            row['lead_y'] = repr(y_lat)
            row['lead_v'] = repr(lead_v)
        row['road_psi'] = repr(psi_road)
        row['filtered_road_psi'] = repr(memory.filtered_road_psi)
        row['raw_cte'] = repr(lat_e)
        row['filtered_cte'] = repr(memory.filtered_cte)
        row['raw_curv'] = repr(curv)
        row['filtered_curv'] = repr(memory.filtered_curv)
        row['delta'] = repr(delta)
        row['delta_cte'] = repr(res.lateral_ctx.delta_cte)
        row['delta_ff'] = repr(res.lateral_ctx.delta_ff)
        row['boundary_delta'] = repr(res.lateral_ctx.boundary_delta)
        row['psi_i_term'] = repr(memory.psi_i_term)
        row['lon_cmd'] = repr(lon)
        row['lon_raw_cmd'] = repr(res.lon_raw_cmd)
        row['acc_i_term'] = repr(0.0)
        row['aeb_active'] = '1' if aeb else '0'
        row['in_curve_hold'] = '1' if res.in_curve_hold else '0'
        if visible:
            row['dist'] = repr(gap)
            if math.isfinite(ttc) and ttc > 0:
                row['ttc'] = repr(ttc)
            row['lead_v_proj'] = repr(lead_v)
            row['min_safe_dist'] = repr(res.lon_ctx.min_safe_dist)
            closing = ego_v - lead_v
            row['closing_speed'] = repr(closing)
        row['acc_has_lead'] = '1' if res.lead_ctx.acc_has_lead else '0'
        row['lead_detected'] = '1' if res.lead_ctx.lead_detected else '0'
        row['cur_lane_width'] = repr(res.cur_lane_width)
        row['lane_safe_margin'] = repr(memory.lane_safe_margin)
        row['lane_warn_margin'] = repr(memory.lane_warn_margin)
        row['lane_hard_margin'] = repr(memory.lane_hard_margin)
        row['boundary_brake'] = repr(res.lateral_ctx.boundary_brake)
        row['boundary_warn'] = '1' if res.lateral_ctx.boundary_warn else '0'
        row['lead_cls'] = str(lead_cls)
        rows.append(row)

    # Write CSV
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    kpi = {
        'step_count': n_steps,
        'min_gap': min_gap if min_gap != float('inf') else None,
        'min_ttc': min_ttc if min_ttc != float('inf') else None,
        'lat_cte_rms': math.sqrt(sum_e2 / max(n, 1)),
        'jerk_rms': math.sqrt(sum_jerk2 / max(n_jerk, 1)),
        'aeb_activations': aeb_count,
        'collision': collided,
    }
    return kpi


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', help='Single scenario name')
    parser.add_argument('--out-dir', default='csv_output')
    args = parser.parse_args()

    scn_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scenarios')
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), args.out_dir)

    if args.scenario:
        files = [os.path.join(scn_dir, args.scenario + '.yaml')]
    else:
        files = sorted([
            os.path.join(scn_dir, f)
            for f in os.listdir(scn_dir) if f.endswith('.yaml')
        ])

    results = {}
    for path in files:
        name = os.path.splitext(os.path.basename(path))[0]
        scn = _load_yaml(path)
        out_path = os.path.join(out_dir, f'{name}.csv')
        print(f'Running {name}...', end=' ', flush=True)
        kpi = simulate_and_write_csv(scn, out_path)
        results[name] = kpi
        print(f'{kpi["step_count"]} steps, {os.path.getsize(out_path)} bytes')

    summary_path = os.path.join(out_dir, 'summary.json')
    with open(summary_path, 'w', encoding='utf-8') as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    print(f'\n{"Scenario":<25} {"MinGap":>8} {"MinTTC":>8} {"CTE_RMS":>8} {"Jerk":>8} {"AEB":>4} {"Coll":>5}')
    print('-' * 72)
    for name, k in results.items():
        gap = f'{k["min_gap"]:.2f}' if k["min_gap"] is not None else 'N/A'
        ttc = f'{k["min_ttc"]:.2f}' if k["min_ttc"] is not None else 'N/A'
        print(f'{name:<25} {gap:>8} {ttc:>8} {k["lat_cte_rms"]:>8.4f} {k["jerk_rms"]:>8.4f} {k["aeb_activations"]:>4} {"YES" if k["collision"] else "no":>5}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
