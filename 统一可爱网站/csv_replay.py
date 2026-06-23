#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""录制场景 CSV 回放器（纯标准库）。

把 ADAS 中央控制软件（run_adas.py）导出的运行 CSV 行，转换成统一网站
内置数据源 frame() 完全相同的 dict 结构，供「实时驾驶舱」回溯展示。

CSV 列（run_adas 写入）：
  t,ego_v,gap,lane_offset,mode,delta_cmd,lon_cmd,steer,throttle,brake,
  lead_v,lead_ttc,aeb_active,ovt_state,ped_warn,failover_src,ovt_off,ovt_tgt
无前车时 gap/lead_v/lead_ttc 写 '-'。
"""

import os
import csv
import glob


def _f(x, default=0.0):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def list_scenarios(directory):
    """扫描目录里的场景 CSV，返回 [{key, name, path}]（按文件名排序）。

    文件名约定 `NN_中文名.csv`，key=文件名(不含扩展名)，name=去掉序号前缀的中文名。
    以 `_` 开头的文件（如 _export.log）忽略。
    """
    out = []
    for p in sorted(glob.glob(os.path.join(directory, '*.csv'))):
        stem = os.path.splitext(os.path.basename(p))[0]
        if stem.startswith('_'):
            continue
        name = stem.split('_', 1)[1] if '_' in stem else stem
        out.append({'key': stem, 'name': name, 'path': p})
    return out


def load_rows(path):
    """读入 CSV 全部行（list of dict）。"""
    with open(path, encoding='utf-8') as f:
        return list(csv.DictReader(f))


def row_to_frame(row, seg, desc):
    """把一行 CSV 转成与 adas_core.frame() 一致的 dict。"""
    ego_v = _f(row.get('ego_v'))
    lane_offset = _f(row.get('lane_offset'))
    gap_raw = (row.get('gap') or '-').strip()
    detected = gap_raw not in ('-', '', 'None')
    aeb = (row.get('aeb_active') or '0').strip() == '1'
    ped_warn = (row.get('ped_warn') or '0').strip() == '1'
    ovt = (row.get('ovt_state') or 'idle').strip() or 'idle'
    failover_src = int(_f(row.get('failover_src')))

    if detected:
        gap = _f(gap_raw)
        lead_v = _f(row.get('lead_v'))
        ttc = _f(row.get('lead_ttc'), 99.0)
        rel = ego_v - lead_v
        lead = {'detected': True, 'gap': round(gap, 1), 'ttc': round(ttc, 1),
                'rel_speed': round(rel, 2), 'lead_v': round(lead_v, 2)}
    else:
        lead = {'detected': False, 'gap': None, 'ttc': None,
                'rel_speed': None, 'lead_v': None}

    feats = ['LKA']
    if detected:
        feats.append('ACC')
    if aeb:
        feats.append('AEB')
    if ovt != 'idle':
        feats.append('OVERTAKE')
    if ped_warn:
        feats.append('PEDESTRIAN')
    if failover_src == 1:
        feats.append('FAILOVER')

    lon_cmd = _f(row.get('lon_cmd'))
    if aeb:
        lon_src = 'aeb'
    elif ped_warn:
        lon_src = 'pedestrian'
    elif ovt != 'idle':
        lon_src = 'overtake'
    elif detected:
        lon_src = 'acc'
    else:
        lon_src = 'cruise'

    mode = (row.get('mode') or 'LKA').strip() or 'LKA'

    return dict(seg=seg, desc=desc, ego_v=ego_v, lane_offset=lane_offset,
                lane_width=3.8, curvature=0.0, lead=lead,
                ped_warn=ped_warn, ped_ttc=None, steer=_f(row.get('steer')),
                throttle=_f(row.get('throttle')), brake=_f(row.get('brake')),
                lon_cmd=lon_cmd, lon_src=lon_src, mode=mode, features=feats,
                aeb=aeb, overtake_state=ovt, failover_src=failover_src)
