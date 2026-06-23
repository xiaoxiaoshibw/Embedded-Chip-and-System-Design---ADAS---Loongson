#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""历史 CSV 数据仓库 —— 归类 + 总结(KPI 摘要/风险) + 索引生成（纯标准库）。

把仓库目录里的每个运行 CSV 自动：
  1. 归类（overtake/cutin/pedestrian/acc/highspeed/cruise；文件名优先，再按数据特征）；
  2. 总结全程 KPI（复用 edge_engine，window 设极大→不滑窗→整段聚合，口径与网站边缘计算一致）；
  3. 评风险等级（正常/注意/危险）；
  4. 写 index.json（按场景类型分组、组内按日期倒序），供网站浏览选择回放。

可作库被 server.py / archive_csv.py 导入，也可独立运行重建索引：
    python repo_index.py [仓库目录]
"""

import os
import csv
import glob
import json
import time

import edge_engine
import csv_replay

CATEGORY_ORDER = ['overtake', 'cutin', 'pedestrian', 'acc', 'highspeed', 'cruise', 'other']
CATEGORY_CN = {
    'overtake': '超车', 'cutin': '加塞 Cut-in', 'pedestrian': '行人横穿',
    'acc': 'ACC 跟车', 'highspeed': '超高速行驶', 'cruise': '巡航', 'other': '其他',
}
RISK_CN = {'normal': '正常', 'warning': '注意', 'critical': '危险'}
INDEX_NAME = 'index.json'


def _f(x, default=0.0):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def infer_category(fname, rows):
    """归类：文件名关键词优先，否则按运行数据特征推断。"""
    n = fname.lower()
    raw = fname
    if '超车' in raw or 'overtake' in n:
        return 'overtake'
    if '加塞' in raw or 'cutin' in n or 'cut-in' in n:
        return 'cutin'
    if '行人' in raw or 'pedestrian' in n or 'walker' in n:
        return 'pedestrian'
    if '超高速' in raw or 'highspeed' in n or 'high_speed' in n:
        return 'highspeed'
    if 'acc' in n or '跟车' in raw:
        return 'acc'
    # 数据特征兜底
    if any((r.get('ovt_state') or 'idle').strip() not in ('idle', '') for r in rows):
        return 'overtake'
    if any((r.get('ped_warn') or '0').strip() == '1' for r in rows):
        return 'pedestrian'
    if max((_f(r.get('ego_v')) for r in rows), default=0.0) > 25.0:
        return 'highspeed'
    if any((r.get('gap') or '-').strip() not in ('-', '', 'None') for r in rows):
        return 'acc'
    return 'cruise'


def _risk_of(kpi, evt, category=None):
    """风险分级（基于全程摘要 KPI + 事件计数），比"有1个critical就critical"更分级。"""
    min_ttc = kpi.get('min_ttc_s')
    min_gap = kpi.get('min_gap_m')
    # 超车场景的车道偏移是主动变道（预期行为），不计入"车道偏离"危险
    lane_dep = evt.get('lane_departure', 0) if category != 'overtake' else 0
    if ((min_ttc is not None and min_ttc < 1.5)
            or evt.get('aeb_activation', 0) > 0
            or evt.get('pedestrian_critical', 0) > 0
            or lane_dep > 0):
        return 'critical'
    if (kpi.get('hard_brake_count', 0) > 0
            or evt.get('pedestrian_warning', 0) > 0
            or (min_ttc is not None and min_ttc < 3.0)
            or (min_gap is not None and min_gap < 6.0)):
        return 'warning'
    return 'normal'


def summarize_csv(rows, category=None):
    """逐行喂 edge_engine(不滑窗)得到全程 KPI/事件统计，与网站边缘计算口径一致。"""
    eng = edge_engine.EdgeEngine(window_s=1e12, emit_interval_s=0.0, outbox_dir=None)
    hb_count = 0          # 急刹"次数"用上升沿统计（而非帧数，避免 1 次急刹算成 9 次）
    in_hb = False
    for i, row in enumerate(rows):
        fr = csv_replay.row_to_frame(row, '', '')
        ev = fr['ego_v']
        accel_in = 0.0 if ev < 0.3 else -fr['lon_cmd']
        is_hb = accel_in <= edge_engine.HARD_BRAKE_ACCEL
        if is_hb and not in_hb:
            hb_count += 1
        in_hb = is_hb
        lead = fr['lead']
        eng.feed(
            i * 0.05, ev, lead['gap'] if lead['detected'] else None,
            lead['lead_v'] if lead['detected'] else None, lead['detected'],
            lane_offset=fr['lane_offset'], accel=accel_in, aeb_active=fr['aeb'],
            ped_warn=fr['ped_warn'], ped_ttc=fr['ped_ttc'], boundary_brake=0.0,
            overtake_active=(fr['overtake_state'] != 'idle'),
            failover_src=fr['failover_src'])
    snap = eng.snapshot()
    summary = snap.get('summary') or {}
    kpis = summary.get('kpis') or {}
    evt = snap.get('event_type_counts') or {}
    risk = _risk_of(kpis, evt, category)
    # 精简对前端友好的摘要
    return {
        'max_kmh': kpis.get('max_speed_kmh'),
        'avg_kmh': kpis.get('avg_speed_kmh'),
        'min_gap_m': kpis.get('min_gap_m'),
        'min_ttc_s': kpis.get('min_ttc_s'),
        'cte_rms_m': kpis.get('cte_rms_m'),
        'jerk_rms': kpis.get('jerk_rms'),
        'hard_brake': hb_count,
        'aeb': evt.get('aeb_activation', 0),
        'ped': evt.get('pedestrian_warning', 0) + evt.get('pedestrian_critical', 0),
        'total_events': snap.get('total_events', 0),
    }, risk


def _name_from_stem(stem):
    """文件名去掉数字序号前缀作展示名。"""
    base = stem.split('_', 1)[1] if ('_' in stem and stem.split('_', 1)[0].isdigit()) else stem
    return base or stem


def build_record(repo_dir, path):
    rel = os.path.relpath(path, repo_dir).replace('\\', '/')
    stem = os.path.splitext(os.path.basename(path))[0]
    with open(path, encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    category = infer_category(os.path.basename(path), rows)
    kpi, risk = summarize_csv(rows, category)
    mtime = os.path.getmtime(path)
    return {
        'id': rel,
        'name': _name_from_stem(stem),
        'category': category,
        'category_cn': CATEGORY_CN.get(category, category),
        'date': mtime,
        'date_str': time.strftime('%Y-%m-%d %H:%M', time.localtime(mtime)),
        'duration_s': round(len(rows) * 0.05, 1),
        'rows': len(rows),
        'kpi': kpi,
        'risk': risk,
        'risk_cn': RISK_CN.get(risk, risk),
    }


def build_index(repo_dir):
    """扫描仓库所有 CSV，生成并写入 index.json，返回索引 dict。"""
    records = []
    for path in glob.glob(os.path.join(repo_dir, '**', '*.csv'), recursive=True):
        if os.path.basename(path).startswith('_'):
            continue
        try:
            rec = build_record(repo_dir, path)
            if rec:
                records.append(rec)
        except Exception as e:
            print('[repo] 跳过 %s：%s' % (path, e))
    # 排序：按类型顺序分组，组内按日期倒序（最新在前）
    def _sort_key(r):
        ci = CATEGORY_ORDER.index(r['category']) if r['category'] in CATEGORY_ORDER else 99
        return (ci, -r['date'])
    records.sort(key=_sort_key)
    index = {
        'generated': time.strftime('%Y-%m-%d %H:%M:%S'),
        'count': len(records),
        'category_order': CATEGORY_ORDER,
        'category_cn': CATEGORY_CN,
        'records': records,
    }
    with open(os.path.join(repo_dir, INDEX_NAME), 'w', encoding='utf-8') as f:
        json.dump(index, f, ensure_ascii=False, indent=2)
    return index


def load_index(repo_dir):
    """读 index.json（不存在则返回 None）。"""
    p = os.path.join(repo_dir, INDEX_NAME)
    if not os.path.isfile(p):
        return None
    try:
        with open(p, encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        return None


if __name__ == '__main__':
    import sys
    repo = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '数据仓库')
    idx = build_index(repo)
    print('[repo] 索引已重建：%s（%d 条）' % (
        os.path.join(repo, INDEX_NAME), idx['count']))
    for r in idx['records']:
        k = r['kpi']
        print('  [%-10s] %-14s %s  最高%.0fkm/h 车距%s TTC%s 急刹%d AEB%d  风险:%s' % (
            r['category'], r['name'], r['date_str'],
            k.get('max_kmh') or 0,
            ('%.1f' % k['min_gap_m']) if k.get('min_gap_m') is not None else '—',
            ('%.1f' % k['min_ttc_s']) if k.get('min_ttc_s') is not None else '—',
            k.get('hard_brake') or 0, k.get('aeb') or 0, r['risk_cn']))
