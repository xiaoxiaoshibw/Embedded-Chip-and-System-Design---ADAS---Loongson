#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""无 CARLA 的双冗余 SIL 长跑演示（龙芯 / 任意 Linux 可部署）。

与 test_failover_sil.py 同款单车运动学模型 + 真实双 soc_worker 控制栈 +
虚拟 ESP32，但面向**现场演示与实机部署**：

  - 纯 Python 标准库链路（无 CARLA / 无 ROS / 无 numpy）→ 可直接跑在龙芯 LoongArch Linux；
  - 复用带安全联锁的双 SoC 控制台（dual_soc_console）：1/0 主备切换，s 状态面板；
  - 遥测落盘 CSV（供 边缘计算 / ollama 监控台 消费）；
  - --auto 走脚本化故障时间线，无人值守长跑；交互模式可手动注入。

用法：
  python sil_demo.py                 # 交互演示（默认无限跑，输入 ? 看指令）
  python sil_demo.py --auto          # 无人值守，循环故障时间线
  python sil_demo.py --duration 120  # 跑 120s 后退出
  python sil_demo.py --telemetry /var/lib/adas/sil.csv
"""

import argparse
import csv
import json
import math
import os
import socket
import sys
import time
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import paths  # noqa: E402 — adds HIL/carla_bridge/pc/ to sys.path for shared bridge modules

from bridge_config import (  # noqa: E402
    SENSOR_PORT_BACKUP,
    SENSOR_PORT_PRIMARY,
    LOG_DIR,
)
from virtual_esp32 import VirtualEsp32  # noqa: E402
from run_cosim import WorkerManager, KeyReader, SRC_NAMES  # noqa: E402
from dual_soc_console import DualSocConsole  # noqa: E402

DT = 0.05            # 20Hz 模型步长
WHEEL_BASE = 3.0

# 无人值守循环时间线（相对每轮起点的秒数, 动作）
AUTO_TIMELINE = [
    (12.0, 'kill_primary'),
    (24.0, 'restore_primary'),
    (32.0, 'kill_backup'),
    (40.0, 'restore_backup'),
]
AUTO_CYCLE_S = 50.0   # 一轮时间线长度，到点循环复位


def _default_telemetry_path():
    day_dir = os.path.join(LOG_DIR, datetime.now().strftime('%Y-%m-%d'))
    os.makedirs(day_dir, exist_ok=True)
    return os.path.join(day_dir, 'sil_%s.csv'
                        % datetime.now().strftime('%H%M%S'))


def run(duration=0.0, auto=False, telemetry=None, realtime=True):
    esp32 = VirtualEsp32()
    workers = WorkerManager()
    workers.start('primary')
    workers.start('backup')
    console = DualSocConsole(workers)
    keys = None if auto else KeyReader.instance()

    sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    tele_path = telemetry or _default_telemetry_path()
    os.makedirs(os.path.dirname(os.path.abspath(tele_path)), exist_ok=True)
    csv_fh = open(tele_path, 'w', newline='')
    writer = csv.writer(csv_fh)
    writer.writerow(['t', 'ego_v', 'gap', 'lane_offset', 'src', 'watchdog',
                     'delta_cmd', 'a_brake', 'pri_alive', 'bak_alive'])

    # ── 模型状态 ──
    ego_v, ego_s, ego_yaw, psi_road, lat_e = 5.0, 0.0, 0.0, 0.0, 0.2
    lead_v, lead_s, gap0 = 6.0, 0.0, 45.0

    pending = list(AUTO_TIMELINE)
    cycle_base = 0.0
    last_src = None
    last_print = -1.0
    armed = False        # 冷启动期 SoC 未上线、看门狗全力制动，不计入遥测/KPI
    t0 = time.monotonic()

    print('SIL 双冗余演示启动（无 CARLA）。遥测: %s' % tele_path, flush=True)
    if not auto:
        print('输入 ? 查看控制台指令（1/0 主备切换，安全联锁开启）', flush=True)

    try:
        while True:
            t = time.monotonic() - t0
            if duration > 0.0 and t >= duration:
                break

            # 前车运动 + 感知帧
            lead_s += max(0.0, lead_v) * DT
            gap = gap0 + lead_s - ego_s
            lead_present = t >= 3.0 and gap > 0.0
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
                    'lead_yaw': psi_road, 'lead_v': lead_v, 'lead_cls': 1,
                })
            payload = json.dumps(frame).encode('utf-8')
            for port in (SENSOR_PORT_PRIMARY, SENSOR_PORT_BACKUP):
                sensor_sock.sendto(payload, ('127.0.0.1', port))

            # 故障注入：无人值守走循环时间线；交互走控制台
            if auto:
                rel = t - cycle_base
                if rel >= AUTO_CYCLE_S:
                    cycle_base = t
                    pending = list(AUTO_TIMELINE)
                    rel = 0.0
                while pending and rel >= pending[0][0]:
                    _, action = pending.pop(0)
                    print('\n=== t=%.1fs 自动注入: %s ===' % (t, action),
                          flush=True)
                    _apply_auto(console, workers, action)
            console.update()
            if keys is not None:
                for k in keys.pop():
                    msgs, quit_now = console.handle_key(k, src=last_src)
                    for m in msgs:
                        print(m, flush=True)
                    if quit_now:
                        raise KeyboardInterrupt

            # 虚拟 ESP32 仲裁 → 模型执行
            out = esp32.step()
            src = out['src']
            if src != last_src:
                print('\n>>> 执行源切换: %s -> %s (t=%.2fs)'
                      % (SRC_NAMES.get(last_src, '-'),
                         SRC_NAMES.get(src, '?'), t), flush=True)
                last_src = src
            for _, ev in esp32.drain_events():
                print('  [ESP32] %s' % ev, flush=True)
            for _, ev in workers.drain_events():
                print('  [HB] %s' % ev, flush=True)

            a_des = -float(out['a_brake'])
            delta = float(out['delta'])
            ego_v = max(0.0, min(40.0, ego_v + a_des * DT))
            ego_s += ego_v * DT
            steer = max(-0.6, min(0.6, delta))
            ego_yaw += (ego_v / WHEEL_BASE) * math.tan(steer) * DT
            he = math.atan2(math.sin(ego_yaw - psi_road),
                            math.cos(ego_yaw - psi_road))
            lat_e += ego_v * math.sin(he) * DT

            # 只在某个 SoC 真正接管驾驶后才记遥测——避免冷启动看门狗瞬态污染边缘 KPI
            if not armed and src in (0, 1):
                armed = True
            if armed:
                writer.writerow(['%.2f' % t, '%.2f' % ego_v,
                                 '%.2f' % gap if lead_present else '',
                                 '%.3f' % lat_e, src, int(out['watchdog']),
                                 '%.4f' % out['delta'], '%.2f' % out['a_brake'],
                                 int(workers.alive('primary')),
                                 int(workers.alive('backup'))])

            if t - last_print >= 1.0:
                print(console.render_panel(sim_t=t, frame=frame, gap=gap,
                                           src=src, out=out), flush=True)
                last_print = t

            if realtime:
                lag = (t0 + t + DT) - time.monotonic()
                if lag > 0:
                    time.sleep(lag)
    except KeyboardInterrupt:
        print('\n(结束)')
    finally:
        csv_fh.close()
        workers.close()
        esp32.close()
        print('遥测已保存: %s' % tele_path)


def _apply_auto(console, workers, action):
    """无人值守动作：经控制台安全联锁（kill_both 会被自动拒绝）。"""
    key = {'kill_primary': '1', 'restore_primary': '0',
           'kill_backup': 'b', 'restore_backup': 'B'}.get(action)
    if key is None:
        return
    msgs, _ = console.handle_key(key)
    for m in msgs:
        print('  ' + m, flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='双冗余 SIL 长跑演示（无 CARLA，龙芯可部署）')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='运行秒数，0=无限（默认）')
    parser.add_argument('--auto', action='store_true',
                        help='无人值守：循环故障时间线，不读 stdin')
    parser.add_argument('--telemetry', default=None,
                        help='遥测 CSV 输出路径（默认 logs/<日期>/sil_*.csv）')
    args = parser.parse_args(argv)
    # 控制栈(soc_worker/心跳/虚拟ESP32)按墙钟运行，仿真必须实时推进，
    # 否则模型与控制栈解耦会发散——故不提供 --fast。
    run(duration=args.duration, auto=args.auto,
        telemetry=args.telemetry, realtime=True)


if __name__ == '__main__':
    main(sys.argv[1:])
