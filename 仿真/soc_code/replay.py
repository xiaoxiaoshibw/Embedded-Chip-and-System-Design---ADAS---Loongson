#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""离线回放测试台。

用 telemetry.py 产出的 CSV 重建 VehicleSignals 时间序列，**不启动 rclpy**，
直接驱动 pipeline.run_pure_pipeline（与在线完全相同的控制器与跨周期状态），
逐周期重算控制量并汇总 KPI。用于改算法后秒级回归对比。

用法：
    python3 replay.py [CSV路径] [--out 重算结果.csv]
不带路径时取 /tmp 下最新 adas_*_telemetry_*.csv（TELEMETRY_DIR 可改目录）。

仅依赖标准库 + 本工程模块（不需要 numpy）。Python 3.6 兼容。

rosbag 输入：本工程不引入 ROS 依赖。需回放 rosbag 时，先用
`ros2 bag` 侧脚本把话题导出成本 CSV 格式（列见 telemetry.FIELDS），再喂给本脚本。
"""

import argparse
import csv
import glob
import math
import os
import sys

from config import LANE_DEFAULT_WIDTH, LOOP_HZ
from lateral import LaneWidthEstimator, lane_margins_from_width
from longitudinal import LonSmoothing
from control.aeb_alert import AebAlertManager
from control.context import ControlManagers, ControlMemory, VehicleSignals
from control.curve_hold import CurveHoldManager
from control.comfort import make_comfort_layer
from control.lead_estimator import make_lead_estimator
from control.lead_tracking import LeadTracker
from control.model_lateral import make_lateral_model_controller
from control.mpc_longitudinal import make_lon_controller
from control.overtake import OvertakeManager
from pipeline import run_pure_pipeline


def _latest_csv():
    out_dir = os.environ.get('TELEMETRY_DIR', '/tmp')
    files = glob.glob(os.path.join(out_dir, 'adas_*_telemetry_*.csv'))
    return max(files, key=os.path.getmtime) if files else None


def _f(row, key, default=0.0):
    """从 CSV 行取 float；缺列/空串/解析失败回退 default。"""
    v = row.get(key, '')
    if v == '' or v is None:
        return default
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def build_stack():
    """构造与 AdasNode.__init__ 等价的控制器栈（无 ROS）。"""
    dt = 1.0 / float(LOOP_HZ)
    memory = ControlMemory(dt=dt)
    (memory.lane_safe_margin,
     memory.lane_warn_margin,
     memory.lane_hard_margin) = lane_margins_from_width(LANE_DEFAULT_WIDTH)
    managers = ControlManagers(
        lane_est=LaneWidthEstimator(LOOP_HZ),
        lead_tracker=LeadTracker(),
        aeb_alert=AebAlertManager(),
        curve_hold=CurveHoldManager(),
        lon_ctrl=make_lon_controller(dt=dt),
        lon_smooth=LonSmoothing(dt=dt),
        overtake=OvertakeManager(),
        lateral_model=make_lateral_model_controller(),
        comfort_layer=make_comfort_layer(dt=dt),
        lead_estimator=make_lead_estimator(),
    )
    return VehicleSignals(), memory, managers


def _apply_row(signals, row, now):
    """把一行 CSV 还原成"本周期刚收到的新鲜感知"。"""
    signals.ego_x = _f(row, 'ego_x')
    signals.ego_y = _f(row, 'ego_y')
    signals.ego_yaw = _f(row, 'ego_yaw')
    signals.ego_v = _f(row, 'ego_v')
    signals.lead_x = _f(row, 'lead_x')
    signals.lead_y = _f(row, 'lead_y')
    signals.lead_v = _f(row, 'lead_v')
    signals.road_psi = _f(row, 'road_psi')
    signals.lane_offset = _f(row, 'raw_cte')
    signals.ego_received = True
    signals.ego_psi_received = True
    signals.road_received = True
    signals.lead_received = True
    signals.lane_offset_received = True
    # 回放时所有数据都视为"当拍刚到"，使新鲜度/超时判定通过
    signals.ego_last_rx = now
    signals.road_last_rx = now
    signals.lead_last_rx_time = now
    signals.lead_v_last_rx_time = now
    signals.lane_offset_last_rx = now


def replay(path, out_csv=None):
    with open(path, 'r') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit('CSV 为空: %s' % path)

    signals, memory, managers = build_stack()
    dt = 1.0 / float(LOOP_HZ)

    # t_mono 列归零做时间基准；缺失/异常则用 index*dt 兜底
    try:
        t0 = float(rows[0].get('t_mono', '') or 0.0)
    except (TypeError, ValueError):
        t0 = 0.0

    writer = None
    out_fh = None
    if out_csv:
        out_fh = open(out_csv, 'w', newline='')
        writer = csv.writer(out_fh)
        writer.writerow(['t', 'lon_cmd', 'delta', 'dist', 'ttc',
                         'aeb_active', 'acc_has_lead', 'cur_lane_width',
                         'filtered_cte'])

    min_ttc = float('inf')
    min_gap = float('inf')
    sum_cte2 = 0.0
    sum_jerk2 = 0.0
    n = 0
    n_jerk = 0
    aeb_edges = 0
    prev_aeb = False
    prev_lon = None

    for i, row in enumerate(rows):
        try:
            tm = float(row.get('t_mono', '') or (t0 + i * dt))
        except (TypeError, ValueError):
            tm = t0 + i * dt
        now = tm - t0 + 1.0  # +1 避免 now=0 与各模块 -1e9 初值边界纠缠

        _apply_row(signals, row, now)
        res = run_pure_pipeline(now, signals, memory, managers, None)

        lon = res.lon_cmd
        ttc = res.lon_ctx.ttc
        dist = res.lon_ctx.dist
        aeb = bool(res.lon_ctx.aeb_active)
        has_lead = bool(res.lead_ctx.acc_has_lead)

        # KPI 累计
        if has_lead:
            if math.isfinite(ttc) and ttc > 0.0:
                min_ttc = min(min_ttc, ttc)
            if math.isfinite(dist) and dist < 900.0:
                min_gap = min(min_gap, dist)
        if math.isfinite(memory.filtered_cte):
            sum_cte2 += memory.filtered_cte ** 2
            n += 1
        if prev_lon is not None and math.isfinite(lon) and math.isfinite(prev_lon):
            jerk = (lon - prev_lon) / dt
            sum_jerk2 += jerk * jerk
            n_jerk += 1
        prev_lon = lon
        if aeb and not prev_aeb:
            aeb_edges += 1
        prev_aeb = aeb

        if writer is not None:
            writer.writerow([
                '%.4f' % now, repr(lon), repr(res.lateral_ctx.delta),
                repr(dist), repr(ttc), 1 if aeb else 0,
                1 if has_lead else 0, repr(res.cur_lane_width),
                repr(memory.filtered_cte),
            ])

    if out_fh is not None:
        out_fh.close()

    kpi = {
        'rows': len(rows),
        'min_ttc_s': None if min_ttc == float('inf') else round(min_ttc, 3),
        'min_gap_m': None if min_gap == float('inf') else round(min_gap, 3),
        'lat_cte_rms_m': round(math.sqrt(sum_cte2 / n), 4) if n else None,
        'lon_jerk_rms': round(math.sqrt(sum_jerk2 / n_jerk), 4) if n_jerk else None,
        'aeb_activations': aeb_edges,
    }
    return kpi


def main(argv=None):
    parser = argparse.ArgumentParser(description='ADAS 离线回放 / KPI')
    parser.add_argument('csv', nargs='?', help='telemetry CSV 路径')
    parser.add_argument('--out', help='把逐周期重算结果另存为 CSV（便于 diff）')
    args = parser.parse_args(argv)

    path = args.csv or _latest_csv()
    if not path or not os.path.exists(path):
        raise SystemExit('找不到输入 CSV，请显式指定路径')
    print('replaying %s' % path)
    kpi = replay(path, args.out)
    print('---- KPI ----')
    for k, v in kpi.items():
        print('  %-16s %s' % (k, v))
    if args.out:
        print('per-step 重算结果已写入 %s' % args.out)


if __name__ == '__main__':
    main(sys.argv[1:])
