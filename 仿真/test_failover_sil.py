#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""无 CARLA 的全链路 SIL 自检（回归测试）。

用 run_scenario.py 同款单车运动学模型替代 CARLA 世界端，但走与联合仿真
完全相同的进程间链路：

  本脚本(20Hz 模型) ─UDP 感知─→ 主/备 soc_worker(100Hz 真实控制栈)
                    ←─虚拟UART─ 真实 ESP32 帧(CRC8)
  虚拟 ESP32 仲裁/看门狗 → 模型执行

时间线（墙钟）：
  t=3   前车出现，ACC 跟车
  t=12  KILL 主控   → 期望: 备机接管，SRC 0→1
  t=24  重启主控    → 期望: SRC 1→0
  t=32  KILL 双控   → 期望: 看门狗 SRC 9，全力制动
  t=38  结束

断言：
  1. 出现过 SRC 0→1 / 1→0 / →9 三次切换；
  2. 0→1 切换前后 1s 窗口内执行减速度无 >2.0 m/s² 的单步阶跃（无感）；
  3. 全程未碰撞（gap > 0）。

退出码 0 = 全部通过。运行前不需要 CARLA，约 40s。
"""

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


def main():
    esp32 = VirtualEsp32()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    procs = {}

    def start(role):
        procs[role] = subprocess.Popen(
            [sys.executable, os.path.join(_HERE, 'soc_worker.py'),
             '--role', role], cwd=_HERE)

    def inject(role, cmd):
        port = FAULT_PORT_PRIMARY if role == 'primary' else FAULT_PORT_BACKUP
        sock.sendto(cmd.encode('ascii'), ('127.0.0.1', port))

    start('primary')
    start('backup')

    # ── 模型状态（与 run_scenario.simulate 等价的简化闭环）──
    ego_v, ego_s, ego_yaw, psi_road, lat_e = 5.0, 0.0, 0.0, 0.0, 0.2
    lead_v, lead_s, gap0 = 6.0, 0.0, 45.0

    KILL_PRIMARY_T, RESTORE_T, KILL_BOTH_T = 12.0, 24.0, 32.0
    timeline = [(KILL_PRIMARY_T, 'kill_primary'), (RESTORE_T, 'restore_primary'),
                (KILL_BOTH_T, 'kill_both')]
    src_seq = []
    a_applied_hist = []     # (t, a_des)
    switch_t = {}
    transitions = []        # [(t, from_src, to_src)] 全部切换，按时间
    collided = False
    t0 = time.monotonic()
    last_src = None
    duration = 38.0

    while True:
        t = time.monotonic() - t0
        if t >= duration:
            break

        # 前车运动
        lead_v = max(0.0, lead_v)
        lead_s += lead_v * DT
        gap = gap0 + lead_s - ego_s
        lead_present = t >= 3.0 and gap > 0.0

        # 感知帧（全局坐标：ego 沿弧长在直道上，前车在正前方 gap 处）
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

        # 故障时间线
        if timeline and t >= timeline[0][0]:
            _, action = timeline.pop(0)
            print('=== t=%.1fs %s ===' % (t, action), flush=True)
            if action == 'kill_primary':
                inject('primary', 'KILL')
            elif action == 'restore_primary':
                start('primary')
            elif action == 'kill_both':
                inject('primary', 'KILL')
                inject('backup', 'KILL')

        # 虚拟 ESP32 → 模型执行
        out = esp32.step()
        src = out['src']
        if src != last_src:
            src_seq.append(src)
            switch_t[(last_src, src)] = t
            transitions.append((t, last_src, src))
            print('  SRC %s -> %s @ t=%.2fs' % (last_src, src, t), flush=True)
            last_src = src
        for _, ev in esp32.drain_events():
            print('  [ESP32] %s' % ev, flush=True)

        a_des = -float(out['a_brake'])          # 正=加速
        delta = float(out['delta'])
        a_applied_hist.append((t, a_des))

        # 闭环积分（同 run_scenario：lon 正=减速 → a = -lon）
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

        # 实时节奏（worker 是墙钟 100Hz）
        next_t = t0 + t + DT
        lag = next_t - time.monotonic()
        if lag > 0:
            time.sleep(lag)

    for p in procs.values():
        try:
            p.terminate()
        except Exception:
            pass
    esp32.close()

    # ── 断言 ──
    fails = []

    # 主→备接管：接受直接 0→1 或经看门狗介入的 0→9→1 路径
    has_takeover = (0, 1) in switch_t or (
        (0, 9) in switch_t and (9, 1) in switch_t)
    if not has_takeover:
        fails.append('未发生主→备接管 (SRC 0→1 或 0→9→1)')
    if (1, 0) not in switch_t:
        fails.append('未发生备→主回切 (SRC 1→0)')
    if 9 not in src_seq:
        fails.append('双杀后看门狗未触发 (SRC 9)')
    if collided:
        fails.append('发生碰撞')

    # 无感性：只看 kill_primary 触发的"首次"接管（into src=1），且窗口必须落在
    # kill_both 之前——否则会把看门狗全力制动（设计行为）误当成接管阶跃。
    # Windows 调度抖动下主控 SEQ 可能瞬时停滞产生额外 0→1 flap，取首个即可。
    takeover_ts = next(
        (tt for tt, frm, to in transitions
         if to == 1 and KILL_PRIMARY_T <= tt < KILL_BOTH_T), None)
    if takeover_ts is not None:
        hi = min(takeover_ts + 1.0, KILL_BOTH_T - 0.2)
        win = [(tt, a) for tt, a in a_applied_hist
               if takeover_ts - 1.0 <= tt <= hi]
        max_step = max((abs(win[i + 1][1] - win[i][1])
                        for i in range(len(win) - 1)), default=0.0)
        print('takeover @%.2fs, 切换窗内最大减速度阶跃 = %.2f m/s^2/step'
              % (takeover_ts, max_step))
        if max_step > 2.0:
            fails.append('接管阶跃过大: %.2f m/s^2' % max_step)

    print('SRC 序列: %s' % src_seq)
    if fails:
        print('FAIL:')
        for f in fails:
            print('  - %s' % f)
        return 1
    print('PASS: 接管/回切/看门狗/无感性 全部通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
