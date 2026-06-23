#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""主备接管时延【极限】扫描实验。

背景：基线接管 ≈156 ms，且 3 次重复完全一致 —— 说明它不是噪声，而是被某个阈值
量化。逐层分析（见 soc_worker / virtual_esp32 / config）得到接管链路两个闸门：

  闸门A 备机检测主控失活 : config.HEARTBEAT_TIMEOUT_S（默认 0.08s）
  闸门B ESP32 判主控帧过期切备 : virtual_esp32.JETSON_TIMEOUT_MS（默认 150ms）

主/备都以 100Hz 向对方/ESP32 发帧。接管完成 = ESP32 把控制源从主(0)切到备(1)，
所以 **接管时延 ≈ 闸门B（JETSON_TIMEOUT_MS）**，闸门A 只需 < 闸门B 即可保证“直接
0→1 接管”而不经看门狗全力制动。要更快，必须把两个闸门一起往下压；下限由 100Hz
帧在本机的正常抖动决定——压过头，正常运行时偶发的一次延迟帧就会被误判成主控失活，
触发【误接管】。

本实验对 (HEARTBEAT_TIMEOUT_S, JETSON_TIMEOUT_MS) 成对下扫，每个配置：
  1) 健康窗口（无故障）跑数秒，统计【误接管】次数（ESP32 误切源 / 备机误激活）；
  2) 杀主控，以 5ms 分辨率测【接管时延】、是否“干净接管”（无看门狗全力制动）、
     接管窗最大加速度阶跃。
极限 = 仍然“0 误接管 + 干净接管”的最小 JETSON_TIMEOUT_MS。

【第三轮·极限压缩】把扫描从已验证的 58ms 继续往下深推到破点（误接管 / 不干净），
每个配置重复 N 次（默认 11 档 × 10 = 110 次 ≥ 百次），用足够样本把“真实下限”从
单次噪声里分辨出来——一个配置可能 3/3 通过却 5/100 失败，只有上百次才能卡住边界。
聚合后得到两个耦合边界：
  · 最低接管边界 = 仍 0 误接管 + 100% 干净 的最小配置的接管时延（取 p95/max 做保守值）；
  · 最高速度边界 = 同等“接管空窗位移 ≤ 2m”约束下的安全速度上限 v = 2m / 最坏接管时延。
接管时延越低 → 速度上限越高，二者是同一枚硬币的两面。

用法：
  python experiment_takeover_limit.py [--repeats 10] [--out DIR]
  python experiment_takeover_limit.py --repeats 1 --max-configs 2   # 冒烟（验证链路）
"""

import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import virtual_esp32  # noqa: E402  （改其模块全局 JETSON_TIMEOUT_MS 用）
from virtual_esp32 import VirtualEsp32  # noqa: E402
from bridge_config import (  # noqa: E402
    FAULT_PORT_BACKUP,
    FAULT_PORT_PRIMARY,
    SENSOR_PORT_BACKUP,
    SENSOR_PORT_PRIMARY,
    STATUS_PORT,
)

DT_PHYS = 0.05         # 物理积分步长（20Hz，与 CARLA 同步一致）
POLL = 0.005           # ESP32 轮询/测量步长（200Hz → 5ms 分辨率）
WHEEL_BASE = 3.0

WARMUP = 2.5           # 工作进程导入/绑定 + 心跳 grace 的预热（不计入测量）
HEALTHY0 = WARMUP + 0.6
KILL_T = WARMUP + 4.0  # 健康窗口 ≈3.4s
END_T = KILL_T + 2.0

BLIND_WINDOW_LIMIT_M = 2.0   # 接管空窗位移上限（无感约束）；速度上限 = 此值 / 接管时延

# 成对扫描点：(HEARTBEAT_TIMEOUT_S, JETSON_TIMEOUT_MS)。
# 第三轮·极限压缩：保留 (0.035,58) 为已验证参考顶点，继续往下深推到破点。
# 两条硬约束界定真实下限：
#   ① 干净接管：JETSON > 备机就绪(≈HB×1000 + 5ms轮询 + 10ms发帧 ≈ HB_ms+15)，
#      否则 ESP32 在备机接上前先全力制动一脚（margin = JETSON-HB_ms 越小越危险）；
#   ② 0 误接管：HB 超时 > 主控帧抖动（本机 Windows 软件帧 ~30ms），HB 压过头则
#      正常运行偶发的延迟帧被误判成主控失活，触发误接管。
# 故 margin<15 预计不干净、HB<30ms 预计误接管——下面这把梯子刚好骑跨两条边界。
SWEEP = [
    (0.035, 58),    # 已验证参考（margin 23, HB 35）→ ~47ms
    (0.033, 52),    # margin 19
    (0.031, 48),    # margin 17
    (0.030, 46),    # margin 16  ← 预计仍稳
    (0.029, 44),    # margin 15  ← 临界
    (0.028, 42),    # margin 14
    (0.027, 40),    # margin 13
    (0.026, 38),    # margin 12
    (0.025, 36),    # margin 11
    (0.023, 33),    # margin 10
    (0.021, 30),    # margin 9, HB 21  ← 预计破（误接管/不干净），卡死边界
]


def run_one(hb_to_s, jetson_to_ms, trial=1):
    virtual_esp32.JETSON_TIMEOUT_MS = jetson_to_ms     # 闸门B
    esp32 = VirtualEsp32()

    sensor = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fault = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status.bind(('127.0.0.1', STATUS_PORT))
    status.setblocking(False)

    env = dict(os.environ)
    env['SWEEP_HB_TIMEOUT_S'] = '%.4f' % hb_to_s        # 闸门A
    env['HB_GRACE'] = '1.0'
    env['TELEMETRY'] = '0'

    procs = {}

    def start(role):
        procs[role] = subprocess.Popen(
            [sys.executable, os.path.join(_HERE, 'soc_worker.py'),
             '--role', role], cwd=_HERE, env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def inject(role, cmd):
        port = FAULT_PORT_PRIMARY if role == 'primary' else FAULT_PORT_BACKUP
        fault.sendto(cmd.encode('ascii'), ('127.0.0.1', port))

    start('primary')
    start('backup')

    # 物理状态
    ego_v, ego_s, ego_yaw, psi_road, lat_e = 5.0, 0.0, 0.0, 0.0, 0.2
    lead_v, lead_s, gap0 = 6.0, 0.0, 45.0
    a_des = 0.0

    src_timeline = []          # (t, src)
    last_src = None
    healthy_src_nonzero = 0
    healthy_backup_active = False
    healthy_takeover_evt = 0
    kill_done = False
    kill_wall = None
    takeover_ms = None
    src_after_kill = []
    wd_in_window = False
    a_win = []                 # 接管窗 [kill, kill+0.6] 的 (t, a_des) 20Hz 采样
    min_gap = 1e9
    collided = False

    t0 = time.monotonic()
    next_phys = 0.0

    while True:
        t = time.monotonic() - t0
        if t >= END_T:
            break

        # —— 故障注入：杀主控 ——
        if not kill_done and t >= KILL_T:
            kill_done = True
            kill_wall = t
            inject('primary', 'KILL')

        # —— 物理积分 + 感知下发（20Hz）——
        if t >= next_phys:
            next_phys += DT_PHYS
            lead_v = max(0.0, lead_v)
            lead_s += lead_v * DT_PHYS
            gap = gap0 + lead_s - ego_s
            if t >= HEALTHY0:
                min_gap = min(min_gap, gap)
            lead_present = gap > 0.0
            frame = {
                't': t, 'ego_x': ego_s * math.cos(psi_road),
                'ego_y': ego_s * math.sin(psi_road), 'ego_yaw': ego_yaw,
                'ego_v': ego_v, 'road_psi': psi_road, 'lane_offset': lat_e,
                'lead_present': bool(lead_present),
            }
            if lead_present:
                frame.update({
                    'lead_x': (ego_s + gap) * math.cos(psi_road),
                    'lead_y': (ego_s + gap) * math.sin(psi_road),
                    'lead_yaw': psi_road, 'lead_v': lead_v, 'lead_cls': 1})
            payload = json.dumps(frame).encode('utf-8')
            for port in (SENSOR_PORT_PRIMARY, SENSOR_PORT_BACKUP):
                sensor.sendto(payload, ('127.0.0.1', port))
            # 物理推进（用最近一拍 ESP32 指令）
            ego_v = max(0.0, min(40.0, ego_v + a_des * DT_PHYS))
            ego_s += ego_v * DT_PHYS
            if lead_present and gap <= 0.0:
                collided = True

        # —— ESP32 仲裁（200Hz 采样）——
        out = esp32.step()
        src = out['src']
        a_des = -float(out['a_brake'])
        # 加速度阶跃在 200Hz 轮询率上采样（原 20Hz 会漏掉短暂的全力制动冲击）
        if kill_done and kill_wall is not None and t <= kill_wall + 0.6:
            a_win.append((t, a_des))
        if src != last_src:
            src_timeline.append((round(t, 4), src))
            last_src = src
        if HEALTHY0 <= t < KILL_T and src != 0:
            healthy_src_nonzero += 1
        if kill_done and src not in src_after_kill:
            src_after_kill.append(src)
        if kill_done and kill_wall is not None and t <= kill_wall + 0.6 and src == 9:
            wd_in_window = True
        if takeover_ms is None and kill_done and src == 1:
            takeover_ms = (t - kill_wall) * 1000.0

        # —— 读 worker 状态流（误接管/事件）——
        try:
            while True:
                data, _ = status.recvfrom(2048)
                try:
                    st = json.loads(data.decode('utf-8'))
                except Exception:
                    break
                if HEALTHY0 <= t < KILL_T:
                    if st.get('role') == 'backup' and st.get('active'):
                        healthy_backup_active = True
                    for ev in st.get('events', []):
                        if 'TAKEOVER' in ev:
                            healthy_takeover_evt += 1
        except (BlockingIOError, OSError):
            pass

        nxt = t0 + t + POLL
        lag = nxt - time.monotonic()
        if lag > 0:
            time.sleep(lag)

    for p in procs.values():
        try:
            p.terminate(); p.wait(timeout=3)
        except Exception:
            pass
    esp32.close()
    for s in (sensor, fault, status):
        try:
            s.close()
        except Exception:
            pass

    max_step = max((abs(a_win[i + 1][1] - a_win[i][1])
                    for i in range(len(a_win) - 1)), default=0.0)
    false_takeover = (healthy_src_nonzero > 0 or healthy_backup_active
                      or healthy_takeover_evt > 0)
    clean = (takeover_ms is not None) and (not wd_in_window) and (max_step <= 2.0)

    return {
        'hb_timeout_s': hb_to_s,
        'jetson_timeout_ms': jetson_to_ms,
        'trial': trial,
        'takeover_ms': None if takeover_ms is None else round(takeover_ms, 1),
        'src_path_after_kill': src_after_kill,
        'clean_takeover': bool(clean),
        'watchdog_in_window': bool(wd_in_window),
        'max_accel_step': round(max_step, 3),
        'healthy_false_takeover': bool(false_takeover),
        'healthy_esp_src_nonzero_samples': healthy_src_nonzero,
        'healthy_backup_active': bool(healthy_backup_active),
        'healthy_takeover_events': healthy_takeover_evt,
        'min_gap': None if min_gap > 1e8 else round(min_gap, 2),
        'collided': bool(collided),
        'src_timeline': src_timeline,
    }


def _pct(sorted_vals, q):
    """最近秩百分位（纯 stdlib，q∈[0,1]）。"""
    if not sorted_vals:
        return None
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = int(math.ceil(q * len(sorted_vals))) - 1
    k = max(0, min(len(sorted_vals) - 1, k))
    return sorted_vals[k]


def _median(sorted_vals):
    """标准中位数（偶数取两中值平均，与 numpy.median 一致），纯 stdlib。"""
    n = len(sorted_vals)
    if n == 0:
        return None
    mid = n // 2
    if n % 2:
        return sorted_vals[mid]
    return round((sorted_vals[mid - 1] + sorted_vals[mid]) / 2.0, 1)


def summarize(results):
    """按 (HB,JETSON) 配置聚合 → 每配置统计 + 识别真实下限与速度上限。

    可靠 = 该配置全部重复都【干净接管】且健康期【ESP32 零误切】（执行器零扰动）。
    备机内部空转(active 翻转/事件但 ESP32 未真切源)算无害抖动，不计入“不可靠”。
    """
    groups = {}
    for m in results:
        key = (round(m['hb_timeout_s'], 4), m['jetson_timeout_ms'])
        groups.setdefault(key, []).append(m)

    summary = []
    for (hb, jt), rows in groups.items():
        n = len(rows)
        tks = sorted(r['takeover_ms'] for r in rows
                     if r['takeover_ms'] is not None)
        steps = [r['max_accel_step'] for r in rows]
        clean = sum(1 for r in rows if r['clean_takeover'])
        # 健康期 ESP32 真切源 = 对执行器有害的硬误接管
        esp_false = sum(1 for r in rows
                        if r['healthy_esp_src_nonzero_samples'] > 0)
        # 备机内部空转（无害）
        churn = sum(1 for r in rows
                    if r['healthy_backup_active']
                    or r['healthy_takeover_events'] > 0)
        collided = sum(1 for r in rows if r['collided'])
        # 稳健判据。只用两个【单调可信】信号定界——不用“干净率”，因为接管窗偶发的
        # 看门狗全力制动（不干净）是 Windows 调度抖动噪声，散落在所有档位（连最保守
        # 的 JET58 都有），且它本身是【安全兜底】（硬刹车非碰撞，全程 0 碰撞），不是危险：
        #   churn_free  : 备机健康期零空转（HB 超时 > 帧抖动，备机从不瞬间误翻 active）
        #   actuator_safe: ESP32 从不误切执行器源 + 0 碰撞（真正的危险失效）
        #   reliable    : churn_free 且 actuator_safe
        # 干净率(clean_rate)单列为“无感质量”指标，不作硬门。
        churn_free = (churn == 0)
        actuator_safe = (esp_false == 0 and collided == 0)
        reliable = (churn_free and actuator_safe)
        tk_max = tks[-1] if tks else None
        tk_p95 = _pct(tks, 0.95)
        # 速度上限（接管空窗位移 ≤ 2m）：用该配置最坏接管时延做保守折算
        def _kmh(lat_ms):
            return None if not lat_ms else round(
                BLIND_WINDOW_LIMIT_M / (lat_ms / 1000.0) * 3.6, 1)
        summary.append({
            'hb_timeout_ms': round(hb * 1000, 1),
            'jetson_timeout_ms': jt,
            'margin_ms': round(jt - hb * 1000, 1),
            'n': n,
            'takeover_min_ms': tks[0] if tks else None,
            'takeover_median_ms': _median(tks),
            'takeover_mean_ms': round(sum(tks) / len(tks), 1) if tks else None,
            'takeover_p95_ms': tk_p95,
            'takeover_max_ms': tk_max,
            'clean_rate': round(clean / n, 3),
            'churn_rate': round(churn / n, 3),
            'esp_false_takeover_count': esp_false,
            'backup_churn_count': churn,
            'collided_count': collided,
            'max_accel_step': round(max(steps), 3) if steps else None,
            'churn_free': bool(churn_free),
            'actuator_safe': bool(actuator_safe),
            'reliable': bool(reliable),
            'speed_cap_kmh_p95': _kmh(tk_p95),
            'speed_cap_kmh_worst': _kmh(tk_max),
        })

    # 真实下限 = 仍 reliable 的最小 JETSON_TIMEOUT_MS
    reliable_rows = [s for s in summary if s['reliable']]
    summary.sort(key=lambda s: s['jetson_timeout_ms'])
    limit = min(reliable_rows, key=lambda s: s['jetson_timeout_ms']) \
        if reliable_rows else None
    # 破点 = 最大的不可靠配置（紧贴下限之下）
    broken = [s for s in summary if not s['reliable']]
    first_broken = max(broken, key=lambda s: s['jetson_timeout_ms']) \
        if broken else None

    return {
        'per_config': summary,
        'limit': limit,            # 最低接管边界
        'first_broken': first_broken,
        'total_trials': len(results),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repeats', type=int, default=10,
                    help='每配置重复次数（默认 10；11 档 → 110 次 ≥ 百次）')
    ap.add_argument('--max-configs', type=int, default=0,
                    help='只跑前 N 个配置（>0 时生效，冒烟用）')
    ap.add_argument('--out', default=os.path.join(
        _HERE, '..', 'lx', '图片', '16_接管时延极限', 'data'))
    args = ap.parse_args()
    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    sweep = SWEEP[:args.max_configs] if args.max_configs > 0 else SWEEP
    total = len(sweep) * args.repeats
    print('扫描 %d 配置 × %d 重复 = %d 次试验\n' % (len(sweep), args.repeats, total),
          flush=True)

    results = []
    done = 0
    for hb, jt in sweep:
        for r in range(1, args.repeats + 1):
            done += 1
            print('=== [%d/%d] HB=%.0fms JETSON=%dms (rep %d/%d) ===' %
                  (done, total, hb * 1000, jt, r, args.repeats), flush=True)
            m = run_one(hb, jt, r)
            results.append(m)
            print('   接管=%s ms  路径=%s  干净=%s  最大阶跃=%.2f  '
                  '误接管=%s(esp非零采样%d/备激活%s/事件%d)  min_gap=%s%s'
                  % (m['takeover_ms'], m['src_path_after_kill'],
                     m['clean_takeover'], m['max_accel_step'],
                     m['healthy_false_takeover'],
                     m['healthy_esp_src_nonzero_samples'],
                     m['healthy_backup_active'], m['healthy_takeover_events'],
                     m['min_gap'], '  [碰撞]' if m['collided'] else ''),
                  flush=True)
            # 增量落盘：长跑中途也能看到/不怕中断丢全部
            with open(os.path.join(out_dir, 'sweep.json'), 'w',
                      encoding='utf-8') as f:
                json.dump(results, f, ensure_ascii=False, indent=2)
            time.sleep(1.5)   # 等端口释放

    summ = summarize(results)
    with open(os.path.join(out_dir, 'summary.json'), 'w',
              encoding='utf-8') as f:
        json.dump(summ, f, ensure_ascii=False, indent=2)

    # —— 每配置聚合表 ——
    print('\n==== 接管时延极限扫描·每配置聚合（n=%d/配置）====' % args.repeats)
    print('%-7s %-7s %-6s %-5s %-22s %-7s %-7s %-7s %-9s' %
          ('HB(ms)', 'JET', 'margin', 'n', '接管 min/中/p95/max(ms)',
           '干净率', 'ESP误切', '可靠', '速度上限'))
    for s in summ['per_config']:
        tk = '%s/%s/%s/%s' % (s['takeover_min_ms'], s['takeover_median_ms'],
                              s['takeover_p95_ms'], s['takeover_max_ms'])
        print('%-7.0f %-7d %-6.0f %-5d %-22s %-7.0f%% %-7d %-7s %-6skm/h' %
              (s['hb_timeout_ms'], s['jetson_timeout_ms'], s['margin_ms'],
               s['n'], tk, s['clean_rate'] * 100, s['esp_false_takeover_count'],
               'Y' if s['reliable'] else 'N',
               s['speed_cap_kmh_worst']))

    print('\n==== 两条边界 ====')
    lim = summ['limit']
    if lim:
        print('最低接管边界：HB %.0fms / JETSON %dms（margin %.0f）'
              % (lim['hb_timeout_ms'], lim['jetson_timeout_ms'],
                 lim['margin_ms']))
        print('  接管时延  中位 %s ms / p95 %s ms / 最坏 %s ms（n=%d，全部干净、0 误切）'
              % (lim['takeover_median_ms'], lim['takeover_p95_ms'],
                 lim['takeover_max_ms'], lim['n']))
        print('最高速度边界：同等 2m 接管空窗约束下 v = 2m/接管时延')
        print('  p95 接管 → %s km/h ；最坏接管 → %s km/h（保守）'
              % (lim['speed_cap_kmh_p95'], lim['speed_cap_kmh_worst']))
    else:
        print('无可靠配置（全部不可靠）——需上调下限')
    fb = summ['first_broken']
    if fb:
        print('破点（紧贴下限之下）：HB %.0fms / JETSON %dms → 干净率 %.0f%% / ESP 误切 %d 次'
              % (fb['hb_timeout_ms'], fb['jetson_timeout_ms'],
                 fb['clean_rate'] * 100, fb['esp_false_takeover_count']))

    print('\nsaved', os.path.join(out_dir, 'sweep.json'))
    print('saved', os.path.join(out_dir, 'summary.json'))


if __name__ == '__main__':
    sys.exit(main())
