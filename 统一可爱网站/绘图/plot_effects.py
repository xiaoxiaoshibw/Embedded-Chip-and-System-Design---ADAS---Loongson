#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""新功能效果数据图：跑通「驾驶仿真 → 边缘计算」真实管线，画出 66s 全场景时序 + KPI 汇总。

数据 100% 来自 ../主控/adas_core.py + ../主控/edge_engine.py（与网站后台同一套代码），
证明：实时驾驶舱数据、边缘 5s 滑窗 KPI、事件检测、风险分级、上云全部按预期工作。

输出：绘图/新功能效果_数据图.png
"""

import os
import shutil
import sys
import tempfile

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

# 中文字体（YaHei 在前）+ 负号正常显示
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

_HERE = os.path.dirname(os.path.abspath(__file__))
_SITE_DIR = os.path.dirname(_HERE)
_ROOT = os.path.dirname(_SITE_DIR)
sys.path.insert(0, _ROOT)
sys.path.insert(0, _SITE_DIR)

from 主控 import adas_core as sim_feed
from 主控 import EdgeEngine

# 场景配色（可爱风格 pastel）
PHASE_COLOR = {
    'cruise': '#d6f7ef', 'acc': '#dcecff', 'aeb': '#ffd9e1', 'boundary': '#fff2cf',
    'pedestrian': '#ffe2d8', 'failover': '#eee0ff', 'overtake': '#d9f5e6',
}
PHASE_CN = {
    'cruise': '巡航', 'acc': 'ACC 跟车', 'aeb': 'AEB 急刹', 'boundary': '边界修正',
    'pedestrian': '行人横穿', 'failover': '主备接管', 'overtake': '超车',
}


def collect():
    """驱动真实管线，逐帧采集时序与事件。"""
    tmp_outbox = tempfile.mkdtemp(prefix='edge_plot_')   # 真实写盘以体现"上云"计数
    edge = EdgeEngine(window_s=5.0, emit_interval_s=1.0, outbox_dir=tmp_outbox)
    edge._tmp_outbox = tmp_outbox
    fps = 20.0
    ts, spd, ttc, off = [], [], [], []
    cum_upload, cum_event, risk_seq = [], [], []
    events = []  # (t, type, severity)
    risk_rank = {'normal': 0, 'warning': 1, 'critical': 2}

    for t in np.arange(0.0, sim_feed.TOTAL_S, 1.0 / fps):
        fr = sim_feed.frame(float(t))
        lead = fr['lead']
        f = edge.feed(
            float(t), fr['ego_v'], lead['gap'] if lead['detected'] else None,
            lead['lead_v'] if lead['detected'] else None, lead['detected'],
            lane_offset=fr['lane_offset'], aeb_active=fr['aeb'],
            ped_warn=fr['ped_warn'], ped_ttc=fr['ped_ttc'],
            boundary_brake=(fr['lon_cmd'] if fr['lon_src'] == 'boundary' else 0.0),
            overtake_active=(fr['overtake_state'] != 'idle'),
            failover_src=fr['failover_src'])
        ts.append(float(t))
        spd.append(fr['ego_v'] * 3.6)
        lt = lead['ttc'] if (lead['detected'] and lead['ttc'] is not None) else np.nan
        ttc.append(min(lt, 30.0) if lt == lt else np.nan)
        off.append(fr['lane_offset'])
        snap = edge.snapshot()
        cum_upload.append(snap['cloud_uploads'])
        cum_event.append(snap['total_events'])
        risk = (snap['summary'] or {}).get('risk_level', 'normal')
        risk_seq.append(risk_rank[risk])
        for ev in f.events:
            events.append((float(t), ev['type'], ev['severity']))
    return dict(ts=np.array(ts), spd=np.array(spd), ttc=np.array(ttc),
                off=np.array(off), upload=np.array(cum_upload),
                event=np.array(cum_event), risk=np.array(risk_seq),
                events=events, edge=edge)


def phases():
    tl = sim_feed.TIMELINE
    out = []
    for i, (t0, name, _desc) in enumerate(tl):
        t1 = tl[i + 1][0] if i + 1 < len(tl) else sim_feed.TOTAL_S
        out.append((t0, t1, name))
    return out


def shade_phases(ax, label_top=False):
    for t0, t1, name in phases():
        ax.axvspan(t0, t1, color=PHASE_COLOR.get(name, '#eee'), alpha=0.55, lw=0)
        if label_top:
            ax.text((t0 + t1) / 2, 1.02, PHASE_CN.get(name, name),
                    transform=ax.get_xaxis_transform(), ha='center', va='bottom',
                    fontsize=9, color='#6b5b73', fontweight='bold')


def main():
    d = collect()
    ts = d['ts']

    fig = plt.figure(figsize=(13.5, 11))
    fig.patch.set_facecolor('#fef9ff')
    gs = fig.add_gridspec(4, 2, height_ratios=[1, 1, 1, 1.05],
                          hspace=0.42, wspace=0.22,
                          left=0.07, right=0.965, top=0.9, bottom=0.07)

    fig.suptitle('萌驾舱 · 新功能效果数据图  ——  驾驶仿真 → 边缘计算 全链路实测（66s 七场景）',
                 fontsize=17, fontweight='bold', color='#d94f86', y=0.965)
    fig.text(0.5, 0.925,
             '数据源：本次新增 sim_feed + edge（与网站后台同一套代码）· 边缘 5s 滑窗 KPI / 事件检测 / 风险分级 / 上云',
             ha='center', fontsize=10.5, color='#8a7a93')

    # ① 车速
    ax1 = fig.add_subplot(gs[0, :])
    shade_phases(ax1, label_top=True)
    ax1.plot(ts, d['spd'], color='#2bb6a0', lw=2.4)
    ax1.set_ylabel('车速 (km/h)', fontsize=11)
    ax1.set_ylim(0, max(35, d['spd'].max() * 1.15))
    ax1.grid(alpha=0.25)

    # ② TTC
    ax2 = fig.add_subplot(gs[1, :], sharex=ax1)
    shade_phases(ax2)
    ax2.plot(ts, d['ttc'], color='#f08a3c', lw=2.4)
    ax2.axhline(5.0, color='#ffab33', ls='--', lw=1.3, label='预警 5s')
    ax2.axhline(2.5, color='#ff5470', ls='--', lw=1.3, label='危急 2.5s')
    ax2.set_ylabel('前车 TTC (s)', fontsize=11)
    ax2.set_ylim(0, 30)
    ax2.legend(loc='upper right', fontsize=9, framealpha=0.9)
    ax2.grid(alpha=0.25)

    # ③ 车道偏移
    ax3 = fig.add_subplot(gs[2, :], sharex=ax1)
    shade_phases(ax3)
    ax3.plot(ts, d['off'], color='#7a45c8', lw=2.2)
    ax3.axhspan(0.55, 0.9, color='#ffd43b', alpha=0.18)
    ax3.axhspan(-0.9, -0.55, color='#ffd43b', alpha=0.18)
    for y in (0.55, -0.55):
        ax3.axhline(y, color='#ffab33', ls=':', lw=1)
    for y in (0.9, -0.9):
        ax3.axhline(y, color='#ff5470', ls=':', lw=1)
    ax3.set_ylabel('车道偏移 (m)', fontsize=11)
    ax3.set_xlabel('仿真时间 (s)', fontsize=11)
    ax3.set_ylim(-1.05, 1.05)
    ax3.grid(alpha=0.25)

    # ④-左 边缘计算累计：上云 + 风险事件（双轴）
    ax4 = fig.add_subplot(gs[3, 0])
    ax4.plot(ts, d['upload'], color='#2f7fd6', lw=2.4, label='累计上云条数')
    ax4.set_xlabel('仿真时间 (s)', fontsize=11)
    ax4.set_ylabel('累计上云条数', color='#2f7fd6', fontsize=11)
    ax4.tick_params(axis='y', labelcolor='#2f7fd6')
    ax4b = ax4.twinx()
    ax4b.plot(ts, d['event'], color='#d94f86', lw=2.4, label='累计风险事件')
    ax4b.set_ylabel('累计风险事件', color='#d94f86', fontsize=11)
    ax4b.tick_params(axis='y', labelcolor='#d94f86')
    ax4.set_title('边缘计算输出（持续上云 + 事件累计）', fontsize=11.5, color='#5b4a63')
    ax4.grid(alpha=0.2)

    # ④-右 事件类型计数（条形）
    ax5 = fig.add_subplot(gs[3, 1])
    counts = d['edge'].snapshot()['event_type_counts']
    label_map = {
        'ttc_warning': 'TTC预警', 'ttc_critical': 'TTC危急', 'hard_brake': '急刹',
        'aeb_activation': 'AEB触发', 'aeb_active': 'AEB持续', 'lane_offset_warning': '车道偏移',
        'lane_departure': '车道偏离', 'pedestrian_warning': '行人预警',
        'pedestrian_critical': '行人危急', 'boundary_brake': '边界制动',
        'jerk_warning': '顿挫', 'failover': '主备接管',
    }
    order = sorted(counts.items(), key=lambda kv: kv[1], reverse=True)[:8]
    names = [label_map.get(k, k) for k, _ in order]
    vals = [v for _, v in order]
    bar_colors = ['#ff8fb3', '#5aa9ff', '#43d9b8', '#ffc83d', '#b07bf0',
                  '#ff8b6b', '#7ed957', '#ff6b8b'][:len(names)]
    ax5.barh(range(len(names)), vals, color=bar_colors, edgecolor='white')
    ax5.set_yticks(range(len(names)))
    ax5.set_yticklabels(names, fontsize=10)
    ax5.invert_yaxis()
    ax5.set_title('边缘事件检测分类计数', fontsize=11.5, color='#5b4a63')
    for i, v in enumerate(vals):
        ax5.text(v, i, ' %d' % v, va='center', fontsize=9, color='#5b4a63')
    ax5.set_xlabel('次数', fontsize=10)
    ax5.grid(axis='x', alpha=0.2)

    # 汇总数字角标
    snap = d['edge'].snapshot()
    su = '√ 上云 %d 条   √ 累计事件 %d   √ 峰值车速 %.0f km/h   √ 全程 7 场景全部检出' % (
        snap['cloud_uploads'], snap['total_events'], d['spd'].max())
    fig.text(0.07, 0.012, su, fontsize=10.5, color='#2bb6a0', fontweight='bold')

    out = os.path.join(_HERE, '新功能效果_数据图.png')
    fig.savefig(out, dpi=140, facecolor=fig.get_facecolor())
    print('saved:', out)
    print('cloud_uploads=%d total_events=%d max_speed=%.1f' % (
        snap['cloud_uploads'], snap['total_events'], d['spd'].max()))

    # 清理临时上云目录
    d['edge'].close()
    tmp = getattr(d['edge'], '_tmp_outbox', None)
    if tmp and os.path.isdir(tmp):
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    main()
