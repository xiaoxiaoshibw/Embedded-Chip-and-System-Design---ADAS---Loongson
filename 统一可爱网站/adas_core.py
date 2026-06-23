#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""驾驶仿真数据源（自包含，纯标准库）。

为统一网站的「实时驾驶舱」提供贴近真实趋势的运行数据。内置 66s 运动学脚本，
串起 巡航 / ACC / AEB / 边界 / 行人 / 主备接管 / 超车 全部功能与告警，循环播放。

不依赖 CARLA。可被 server.py 在后台线程驱动；也可改接真实 ADAS 后端（见 server.py
的 --adas-url 代理模式）或录制场景 CSV 回溯（--scenarios-dir）。

本文件原为统一网站从已删除的「主控」包内联自带，使网站自包含可独立运行。
"""

import math


# (起始 t, 段名, 中文简述)
TIMELINE = [
    (0,  'cruise',     'LKA 直线巡航'),
    (8,  'acc',        'ACC 自适应跟车'),
    (20, 'aeb',        '前车急刹 · AEB 紧急制动'),
    (30, 'boundary',   '车道偏移 · LKA 边界修正'),
    (38, 'pedestrian', '行人横穿 · 制动避让'),
    (48, 'failover',   '主控失效 · 备机无感接管'),
    (56, 'overtake',   '静止前车 · 变道超越'),
]
TOTAL_S = 66.0


def seg_at(t):
    name, desc = 'cruise', 'LKA 直线巡航'
    for ts, nm, ds in TIMELINE:
        if t >= ts:
            name, desc = nm, ds
    return name, desc


def frame(t):
    """生成第 t 秒（已对 TOTAL_S 取模）的运行数据 dict。"""
    seg, desc = seg_at(t)
    ego_v = 8.0
    lane_offset = 0.05 * math.sin(t * 0.6)
    lane_width = 3.8
    curvature = 0.004 * math.sin(t * 0.25)
    lead = {'detected': False, 'gap': None, 'ttc': None, 'rel_speed': None, 'lead_v': None}
    ped_warn, ped_ttc = False, None
    steer = (lane_offset * -0.4) + curvature * 8.0
    throttle, brake = 0.25, 0.0
    lon_cmd, lon_src = -0.4, 'cruise'
    mode, features = 'LKA', ['LKA']
    aeb = False
    overtake_state = 'idle'
    failover_src = 0

    if seg == 'acc':
        lead_v = 6.0 + 1.5 * math.sin((t - 8) * 0.5)
        ego_v = 7.5
        gap = max(10.0, 22 - (ego_v - lead_v) * (t - 8) * 0.2)
        rel = ego_v - lead_v
        lead = {'detected': True, 'gap': round(gap, 1),
                'ttc': round(gap / rel, 1) if rel > 0.1 else 99.0,
                'rel_speed': round(rel, 2), 'lead_v': round(lead_v, 2)}
        mode, features = 'ACC+LKA', ['LKA', 'ACC']
        lon_cmd, lon_src = 0.3, 'acc/cruise'
        throttle, brake = 0.12, 0.0

    elif seg == 'aeb':
        phase = t - 20
        lead_v = max(0.0, 6.0 - phase * 2.0)
        ego_v = max(0.5, 8.0 - phase * 1.6)
        gap = max(4.0, 14.0 - phase * 1.5)
        rel = ego_v - lead_v
        ttc = gap / rel if rel > 0.1 else 99.0
        lead = {'detected': True, 'gap': round(gap, 1), 'ttc': round(ttc, 1),
                'rel_speed': round(rel, 2), 'lead_v': round(lead_v, 2)}
        aeb = ttc < 3.0 or gap < 6.0
        mode = 'AEB' if aeb else 'ACC+LKA'
        features = ['LKA', 'ACC', 'AEB'] if aeb else ['LKA', 'ACC']
        lon_cmd = 6.5 if aeb else 2.0
        lon_src = 'aeb' if aeb else 'acc/cruise'
        throttle, brake = 0.0, (1.0 if aeb else 0.4)

    elif seg == 'boundary':
        ego_v = 7.0
        lane_offset = 0.7 * math.sin((t - 30) * 1.2)
        steer = -lane_offset * 0.8
        if abs(lane_offset) > 0.55:
            features = ['LKA', 'BOUNDARY']
            lon_cmd, lon_src = 1.2, 'boundary'
            brake = 0.15

    elif seg == 'pedestrian':
        ego_v = max(1.0, 7.0 - (t - 38) * 1.2)
        ped_ttc = max(0.5, 4.0 - (t - 38) * 0.5)
        ped_warn = ped_ttc < 4.0
        mode = 'AEB' if ped_ttc < 2.5 else 'LKA'
        features = ['LKA', 'PEDESTRIAN']
        aeb = ped_ttc < 2.5
        lon_cmd, lon_src = (5.0 if aeb else 2.0), 'pedestrian'
        throttle, brake = 0.0, (1.0 if aeb else 0.5)

    elif seg == 'failover':
        ego_v, lead_v, gap = 7.5, 6.5, 25.0
        rel = ego_v - lead_v
        lead = {'detected': True, 'gap': round(gap, 1),
                'ttc': round(gap / rel, 1) if rel > 0.1 else 99.0,
                'rel_speed': round(rel, 2), 'lead_v': round(lead_v, 2)}
        failover_src = 1 if (t - 48) < 5.0 else 0
        features = ['LKA', 'ACC'] + (['FAILOVER'] if failover_src == 1 else [])
        mode, lon_cmd, lon_src, throttle = 'ACC+LKA', 0.2, 'acc/cruise', 0.1

    elif seg == 'overtake':
        phase = t - 56
        overtake_state = ('approaching' if phase < 2 else
                          'shifting_left' if phase < 4 else
                          'passing' if phase < 7 else
                          'returning' if phase < 9 else 'idle')
        ego_v = 5.0 + phase * 0.3
        lane_offset = (-min(3.0, phase * 0.6) if overtake_state in ('shifting_left', 'passing')
                       else (-max(0.0, 3.0 - (phase - 7) * 1.5) if overtake_state == 'returning' else 0.0))
        lead = {'detected': overtake_state == 'approaching', 'gap': 8.0,
                'ttc': 99.0, 'rel_speed': 0.0, 'lead_v': 0.0}
        mode = 'OVERTAKE' if overtake_state != 'idle' else 'LKA'
        features = ['LKA', 'OVERTAKE'] if overtake_state != 'idle' else ['LKA']
        steer = -lane_offset * 0.3
        lon_cmd, lon_src, throttle = -1.0, 'overtake', 0.3

    steer = max(-1.0, min(1.0, steer))
    return dict(seg=seg, desc=desc, ego_v=ego_v, lane_offset=lane_offset,
                lane_width=lane_width, curvature=curvature, lead=lead,
                ped_warn=ped_warn, ped_ttc=ped_ttc, steer=steer,
                throttle=throttle, brake=brake, lon_cmd=lon_cmd, lon_src=lon_src,
                mode=mode, features=features, aeb=aeb,
                overtake_state=overtake_state, failover_src=failover_src)
