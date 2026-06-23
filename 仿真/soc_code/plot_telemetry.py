#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""telemetry.py 产出的 CSV 离线可视化。

用法：
    python3 plot_telemetry.py [CSV路径] [--save out.png]

不带路径时自动取 /tmp 下最新的 adas_*_telemetry_*.csv。
仅依赖 numpy + matplotlib（见 requirements.txt 注释，仅离线分析用）。
Python 3.6 兼容。
"""

import argparse
import glob
import os
import sys

import numpy as np

import matplotlib
if os.environ.get('MPL_HEADLESS') == '1':
    matplotlib.use('Agg')
import matplotlib.pyplot as plt


def _latest_csv():
    files = glob.glob('/tmp/adas_*_telemetry_*.csv')
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def _load(path):
    # names=True 用首行做列名；nan/inf 字符串 genfromtxt 原生可解析
    data = np.genfromtxt(path, delimiter=',', names=True)
    if data.size == 0:
        raise SystemExit('telemetry CSV 为空: %s' % path)
    return data


def _t(data):
    """相对时间轴（s），从单调时钟起点归零。"""
    tm = data['t_mono']
    return tm - tm[0]


def main(argv=None):
    parser = argparse.ArgumentParser(description='ADAS telemetry 可视化')
    parser.add_argument('csv', nargs='?', help='telemetry CSV 路径')
    parser.add_argument('--save', help='保存为图片而非弹窗显示')
    args = parser.parse_args(argv)

    path = args.csv or _latest_csv()
    if not path or not os.path.exists(path):
        raise SystemExit('找不到 telemetry CSV，请显式指定路径')
    print('loading %s' % path)
    d = _load(path)
    t = _t(d)

    fig, axes = plt.subplots(3, 2, figsize=(16, 11))
    fig.suptitle(os.path.basename(path))

    # 1. 俯视轨迹
    ax = axes[0, 0]
    ax.plot(d['ego_x'], d['ego_y'], '-', label='ego', lw=1.2)
    ax.plot(d['lead_x'], d['lead_y'], '--', label='lead', lw=1.0)
    ax.set_title('trajectory (x-y)')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.axis('equal'); ax.legend(); ax.grid(True, alpha=0.3)

    # 2. 跟车距离 vs 安全距离
    ax = axes[0, 1]
    ax.plot(t, d['dist'], label='dist', lw=1.0)
    ax.plot(t, d['min_safe_dist'], '--', label='min_safe_dist', lw=1.0)
    ax.set_title('following distance')
    ax.set_xlabel('t [s]'); ax.set_ylabel('[m]')
    ax.set_ylim(0, np.nanpercentile(d['dist'][np.isfinite(d['dist'])], 99) + 5
                if np.any(np.isfinite(d['dist'])) else 100)
    ax.legend(); ax.grid(True, alpha=0.3)

    # 3. 纵向指令 + AEB 阴影
    ax = axes[1, 0]
    ax.plot(t, d['lon_raw_cmd'], label='lon_raw', lw=0.8, alpha=0.6)
    ax.plot(t, d['lon_cmd'], label='lon_cmd', lw=1.0)
    ax.plot(t, d['lon_tx'], ':', label='lon_tx', lw=1.0)
    aeb = d['aeb_active'] > 0.5
    ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1], where=aeb,
                    color='red', alpha=0.12, label='AEB')
    ax.set_title('longitudinal cmd (+ = brake)')
    ax.set_xlabel('t [s]'); ax.set_ylabel('[m/s^2]')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 4. 横向偏移 + 车道余量
    ax = axes[1, 1]
    ax.plot(t, d['raw_cte'], label='raw_cte', lw=0.8, alpha=0.6)
    ax.plot(t, d['filtered_cte'], label='filtered_cte', lw=1.0)
    ax.plot(t, d['lane_warn_margin'], '--', color='orange', lw=0.8,
            label='warn')
    ax.plot(t, -d['lane_warn_margin'], '--', color='orange', lw=0.8)
    ax.plot(t, d['lane_hard_margin'], '--', color='red', lw=0.8, label='hard')
    ax.plot(t, -d['lane_hard_margin'], '--', color='red', lw=0.8)
    ax.set_title('lane offset & margins')
    ax.set_xlabel('t [s]'); ax.set_ylabel('[m]')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 5. TTC（截断到可读范围）
    ax = axes[2, 0]
    ttc = np.array(d['ttc'], dtype=float)
    ttc_disp = np.clip(np.where(np.isfinite(ttc), ttc, np.nan), 0, 30)
    ax.plot(t, ttc_disp, label='ttc (clip 30s)', lw=1.0)
    ax.set_title('time-to-collision')
    ax.set_xlabel('t [s]'); ax.set_ylabel('[s]')
    ax.legend(); ax.grid(True, alpha=0.3)

    # 6. 方向盘转角分量
    ax = axes[2, 1]
    for k in ('delta', 'delta_cte', 'delta_ff', 'boundary_delta'):
        ax.plot(t, np.degrees(d[k]), label=k, lw=0.9)
    ax.set_title('steering components')
    ax.set_xlabel('t [s]'); ax.set_ylabel('[deg]')
    ax.legend(); ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    if args.save:
        fig.savefig(args.save, dpi=110)
        print('saved %s' % args.save)
    else:
        plt.show()


if __name__ == '__main__':
    main(sys.argv[1:])
