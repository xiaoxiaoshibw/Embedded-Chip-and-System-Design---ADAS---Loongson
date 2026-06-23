#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""主备健壮性实验（仪表化版 test_failover_sil）。

与 test_failover_sil.py 完全相同的真实 SIL 链路（真实 soc_worker 控制栈 +
虚拟 ESP32 仲裁/看门狗），但逐拍记录全部状态并支持多次重复试验，
输出 CSV/JSON 供 lx/图片 出标准实验图。

每次试验时间线（墙钟）：
  t=3   前车出现，ACC 跟车
  t=12  KILL 主控   → 期望: 备机接管 SRC 0→1
  t=24  重启主控    → 期望: 回切 SRC 1→0
  t=32  KILL 双控   → 期望: 看门狗 SRC 9，全力制动
  t=38  结束

用法：python experiment_failover.py [--trials 3] [--out DIR]
"""

import argparse
import csv
import json
import math
import os
import socket
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import paths  # noqa: E402 — adds HIL/carla_bridge/pc/ to sys.path for shared bridge modules

from bridge_config import (  # noqa: E402
    FAULT_PORT_BACKUP,
    FAULT_PORT_PRIMARY,
    SENSOR_PORT_BACKUP,
    SENSOR_PORT_PRIMARY,
)
from virtual_esp32 import VirtualEsp32  # noqa: E402

DT = 0.05            # 20Hz，与 CARLA 同步模式一致
WHEEL_BASE = 3.0
DURATION = 38.0

EVENTS = [(12.0, 'kill_primary'), (24.0, 'restore_primary'),
          (32.0, 'kill_both')]


def _first_to(transitions, dst, after, before=None):
    """注入时刻 after 之后（可选 before 之前）首次切到 dst 状态的时延。

    用切换事件列表而非 switch_t 字典：字典按 (from,to) 去重，同一跳变多次发生
    （如 kill_primary 与 kill_both 都产生 0→1）只会留最后一次，使时延计算串味。
    """
    if after is None:
        return None
    cands = [t - after for (_f, to, t) in transitions
             if to == dst and t >= after and (before is None or t < before)]
    return min(cands) if cands else None


def run_trial(trial_idx):
    esp32 = VirtualEsp32()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    procs = {}

    def start(role):
        procs[role] = subprocess.Popen(
            [sys.executable, os.path.join(_HERE, 'soc_worker.py'),
             '--role', role], cwd=_HERE,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def inject(role, cmd):
        port = FAULT_PORT_PRIMARY if role == 'primary' else FAULT_PORT_BACKUP
        sock.sendto(cmd.encode('ascii'), ('127.0.0.1', port))

    start('primary')
    start('backup')

    ego_v, ego_s, ego_yaw, psi_road, lat_e = 5.0, 0.0, 0.0, 0.0, 0.2
    lead_v, lead_s, gap0 = 6.0, 0.0, 45.0

    timeline = list(EVENTS)
    rows = []                 # 逐拍记录
    src_seq = []
    switch_t = {}             # (from,to) -> t（仅用于时序图标注，会按键去重）
    transitions = []          # [(from, to, t)] 全部跳变（指标计算用，不去重）
    inject_t = {}             # action -> t
    esp_events = []           # (t, text)
    collided = False
    t0 = time.monotonic()
    last_src = None

    while True:
        t = time.monotonic() - t0
        if t >= DURATION:
            break

        lead_v = max(0.0, lead_v)
        lead_s += lead_v * DT
        gap = gap0 + lead_s - ego_s
        lead_present = t >= 3.0 and gap > 0.0

        frame = {
            't': t,
            'ego_x': ego_s * math.cos(psi_road),
            'ego_y': ego_s * math.sin(psi_road),
            'ego_yaw': ego_yaw,
            'ego_v': ego_v,
            'road_psi': psi_road,
            'lane_offset': lat_e,
            'lead_present': bool(lead_present),
        }
        if lead_present:
            frame.update({
                'lead_x': (ego_s + gap) * math.cos(psi_road),
                'lead_y': (ego_s + gap) * math.sin(psi_road),
                'lead_yaw': psi_road,
                'lead_v': lead_v,
                'lead_cls': 1,
            })
        payload = json.dumps(frame).encode('utf-8')
        for port in (SENSOR_PORT_PRIMARY, SENSOR_PORT_BACKUP):
            sock.sendto(payload, ('127.0.0.1', port))

        if timeline and t >= timeline[0][0]:
            _, action = timeline.pop(0)
            inject_t[action] = t
            print('  [trial %d] t=%.2fs %s' % (trial_idx, t, action),
                  flush=True)
            if action == 'kill_primary':
                inject('primary', 'KILL')
            elif action == 'restore_primary':
                start('primary')
            elif action == 'kill_both':
                inject('primary', 'KILL')
                inject('backup', 'KILL')

        out = esp32.step()
        src = out['src']
        if src != last_src:
            src_seq.append(src)
            switch_t[(last_src, src)] = t
            transitions.append((last_src, src, t))
            print('  [trial %d] SRC %s -> %s @ t=%.2fs'
                  % (trial_idx, last_src, src, t), flush=True)
            last_src = src
        for ts, ev in esp32.drain_events():
            esp_events.append((t, ev))

        a_des = -float(out['a_brake'])
        delta = float(out['delta'])
        rows.append({
            't': round(t, 4), 'src': src,
            'a_des': round(a_des, 4), 'delta': round(delta, 5),
            'ego_v': round(ego_v, 4), 'lead_v': round(lead_v, 4),
            'gap': round(gap, 4),
            'watchdog': int(bool(out['watchdog'])),
            'aeb_floor': round(float(out['aeb_floor']), 4),
        })

        ego_v = max(0.0, min(40.0, ego_v + a_des * DT))
        ego_s += ego_v * DT
        steer = max(-0.6, min(0.6, delta))
        ego_yaw += (ego_v / WHEEL_BASE) * math.tan(steer) * DT
        he = math.atan2(math.sin(ego_yaw - psi_road),
                        math.cos(ego_yaw - psi_road))
        lat_e += ego_v * math.sin(he) * DT
        if lead_present and gap <= 0.0:
            collided = True
            break

        next_t = t0 + t + DT
        lag = next_t - time.monotonic()
        if lag > 0:
            time.sleep(lag)

    for p in procs.values():
        try:
            p.terminate()
            p.wait(timeout=3)
        except Exception:
            pass
    esp32.close()

    # ── 指标（用 transitions 列表 + 时间窗，避免同一跳变多次发生时串味）──
    kill_p = inject_t.get('kill_primary')
    restore_p = inject_t.get('restore_primary')
    kill_b = inject_t.get('kill_both')
    # 接管：kill_primary 之后、restore_primary 之前首次切到备控(SRC 1)
    takeover_lat = _first_to(transitions, 1, kill_p, restore_p)
    max_step = None
    if takeover_lat is not None and kill_p is not None:
        takeover_ts = kill_p + takeover_lat
        win = [(r['t'], r['a_des']) for r in rows
               if abs(r['t'] - takeover_ts) <= 1.0]
        max_step = max((abs(win[i + 1][1] - win[i][1])
                        for i in range(len(win) - 1)), default=0.0)

    metrics = {
        'trial': trial_idx,
        'src_seq': src_seq,
        'switch_t': {'%s->%s' % k: v for k, v in switch_t.items()},
        'inject_t': inject_t,
        'takeover_latency_s': takeover_lat,
        # 回切：restore_primary 之后、kill_both 之前首次切回主控(SRC 0)
        'switchback_latency_s': _first_to(transitions, 0, restore_p, kill_b),
        # 看门狗：双杀之后首次进入 SRC 9
        'watchdog_latency_s': _first_to(transitions, 9, kill_b),
        'max_accel_step': max_step,
        'min_gap': min((r['gap'] for r in rows if r['t'] >= 3.0),
                       default=None),
        'collided': collided,
    }
    return rows, metrics, esp_events


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=3)
    ap.add_argument('--out', default=os.path.join(
        _HERE, '..', 'lx', '图片', '10_主备健壮性实验', 'data'))
    args = ap.parse_args()
    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    all_metrics = []
    for i in range(1, args.trials + 1):
        print('=== Trial %d/%d ===' % (i, args.trials), flush=True)
        rows, metrics, esp_events = run_trial(i)
        all_metrics.append(metrics)

        csv_path = os.path.join(out_dir, 'trial_%d.csv' % i)
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        with open(os.path.join(out_dir, 'trial_%d_events.json' % i),
                  'w', encoding='utf-8') as f:
            json.dump([{'t': t, 'event': e} for t, e in esp_events],
                      f, ensure_ascii=False, indent=1)
        print('  saved %s (%d rows)' % (csv_path, len(rows)), flush=True)
        if i < args.trials:
            time.sleep(2.0)   # 等待端口释放

    with open(os.path.join(out_dir, 'metrics.json'), 'w',
              encoding='utf-8') as f:
        json.dump(all_metrics, f, ensure_ascii=False, indent=2)

    print('\n==== 汇总 ====')
    ok = True
    for m in all_metrics:
        line = ('trial %d: 接管=%.0fms 回切=%.2fs 看门狗=%.0fms '
                '最大阶跃=%.2f m/s^2 min_gap=%.1fm %s')
        try:
            print(line % (
                m['trial'],
                (m['takeover_latency_s'] or -1) * 1000,
                m['switchback_latency_s'] or -1,
                (m['watchdog_latency_s'] or -1) * 1000,
                m['max_accel_step'] if m['max_accel_step'] is not None else -1,
                m['min_gap'] if m['min_gap'] is not None else -1,
                'COLLIDED' if m['collided'] else 'OK'))
        except TypeError:
            print('trial %d: 指标缺失 %s' % (m['trial'], m))
        if (m['collided'] or m['takeover_latency_s'] is None
                or m['switchback_latency_s'] is None
                or m['watchdog_latency_s'] is None
                or (m['max_accel_step'] or 99) > 2.0):
            ok = False
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
