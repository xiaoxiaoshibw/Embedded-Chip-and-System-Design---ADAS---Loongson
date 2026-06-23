#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""联合仿真运行器：CARLA × SOCCode 双冗余 ADAS × 虚拟 ESP32。

被 cli.py 调用（推荐入口），也可独立运行：
  python run_cosim.py --scenario failover

链路：
  CARLA 真值感知 ─UDP JSON 20Hz─→ 主/备 SOC worker（100Hz 真实控制栈）
  worker ─真实 ESP32 帧(CRC8)/虚拟UART─→ 虚拟 ESP32（仲裁+AEB地板+看门狗）
  虚拟 ESP32 输出 ─→ CARLA 自车执行
  主备 worker 之间跑原版格式 UDP 心跳（接管种子 → 无感降级）
"""

import argparse
import csv
import json
import os
import socket
import subprocess
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import paths  # noqa: E402 — adds HIL/carla_bridge/pc/ to sys.path for shared bridge modules

from bridge_config import (  # noqa: E402
    CARLA_HOST,
    CARLA_PORT,
    FAULT_PORT_BACKUP,
    FAULT_PORT_PRIMARY,
    LOG_DIR,
    SENSOR_PORT_BACKUP,
    SENSOR_PORT_PRIMARY,
    STATUS_PORT,
    TOWN,
)
from scenarios import ORDER, SCENARIOS  # noqa: E402
from virtual_esp32 import VirtualEsp32  # noqa: E402
from dual_soc_console import DualSocConsole  # noqa: E402

SRC_NAMES = {0: 'PRIMARY', 1: 'BACKUP', 9: 'WATCHDOG'}


def import_carla():
    try:
        import carla
        return carla
    except ImportError:
        raise SystemExit(
            '无法导入 carla。请用 Python 3.12 并安装：\n'
            '  pip install "..\\CALRA\\PythonAPI\\carla\\dist\\'
            'carla-0.9.16-cp312-cp312-win_amd64.whl"')


class WorkerManager:
    """主/备 SOC worker 子进程的启动、故障注入与状态收集。"""

    def __init__(self):
        self.procs = {}
        self.status = {'primary': None, 'backup': None}
        # 最近一次收到状态帧的墙钟时刻（liveness 判据：worker 每 0.1s 上报一次，
        # 控制环卡死时整 run 循环阻塞 → 状态帧停发 → 时戳变陈旧）
        self.status_t = {'primary': 0.0, 'backup': 0.0}
        self.events = []
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._status_sock.bind(('127.0.0.1', STATUS_PORT))
        self._status_sock.settimeout(0.05)
        self._running = True
        threading.Thread(target=self._status_loop, daemon=True).start()

    def start(self, role):
        if role in self.procs and self.procs[role].poll() is None:
            return
        p = subprocess.Popen(
            [sys.executable, os.path.join(_HERE, 'soc_worker.py'),
             '--role', role],
            cwd=_HERE)
        self.procs[role] = p
        self.events.append((time.monotonic(), '%s worker started (pid=%d)'
                            % (role, p.pid)))

    def inject(self, role, cmd):
        port = FAULT_PORT_PRIMARY if role == 'primary' else FAULT_PORT_BACKUP
        try:
            self._sock.sendto(cmd.encode('ascii'), ('127.0.0.1', port))
        except Exception:
            pass
        self.events.append((time.monotonic(), 'FAULT %s -> %s' % (cmd, role)))

    def alive(self, role):
        p = self.procs.get(role)
        return p is not None and p.poll() is None

    def last_status_age(self, role):
        """距上次收到该 role 状态帧的秒数；从未收到返回 None。"""
        ts = self.status_t.get(role, 0.0)
        if ts <= 0.0:
            return None
        return time.monotonic() - ts

    def _status_loop(self):
        while self._running:
            try:
                data, _ = self._status_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                msg = json.loads(data.decode('utf-8'))
            except Exception:
                continue
            role = msg.get('role')
            if role in self.status:
                self.status[role] = msg
                self.status_t[role] = time.monotonic()
                for ev in msg.get('events', []):
                    self.events.append((time.monotonic(), '[%s] %s' % (role, ev)))

    def drain_events(self):
        ev, self.events = self.events, []
        return ev

    def close(self):
        self._running = False
        for p in self.procs.values():
            try:
                p.terminate()
            except Exception:
                pass
        try:
            self._status_sock.close()
        except Exception:
            pass


class KeyReader:
    """控制台输入单例：唯一的 stdin 读取者。

    - 场景运行中：pop() 把已输入的行拆成字符（故障注入按键）；
    - 菜单模式：read_line() 阻塞等待一整行。
    必须单例——stdin 只能有一个消费者，否则菜单 input() 会被抢行。
    """

    _inst = None

    @classmethod
    def instance(cls):
        if cls._inst is None:
            cls._inst = cls()
        return cls._inst

    def __init__(self):
        self._lines = []
        self._cond = threading.Condition()
        self._eof = False
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def _loop(self):
        while True:
            try:
                line = sys.stdin.readline()
            except Exception:
                line = ''
            with self._cond:
                if not line:
                    self._eof = True
                    self._cond.notify_all()
                    return
                self._lines.append(line.rstrip('\n'))
                self._cond.notify_all()

    def pop(self):
        """取走所有已输入行并拆成字符序列（运行中故障注入）。"""
        with self._cond:
            lines, self._lines = self._lines, []
        return [ch for ln in lines for ch in ln.strip()]

    def read_line(self, prompt=''):
        """阻塞读取一整行（菜单）。EOF 返回 None。"""
        if prompt:
            print(prompt, end='', flush=True)
        with self._cond:
            while not self._lines and not self._eof:
                self._cond.wait(0.2)
            if self._lines:
                return self._lines.pop(0)
            return None


def _do_action(workers, action):
    if action == 'kill_primary':
        workers.inject('primary', 'KILL')
    elif action == 'restore_primary':
        workers.start('primary')
    elif action == 'kill_backup':
        workers.inject('backup', 'KILL')
    elif action == 'restore_backup':
        workers.start('backup')
    elif action == 'kill_both':
        workers.inject('primary', 'KILL')
        workers.inject('backup', 'KILL')
    elif action == 'restore_both':
        workers.start('primary')
        workers.start('backup')
    elif action == 'hang_primary':
        workers.inject('primary', 'HANG')


def run_scenario(scn_key, host=CARLA_HOST, port=CARLA_PORT, town=TOWN,
                 no_rendering=False, spawn_index=None, realtime=True,
                 keys=None):
    """运行一个场景，返回 (切换事件列表, CSV 日志路径)。

    keys: KeyReader 或 None；None 时仍允许时间线注入，但无手动按键。
    """
    scn = SCENARIOS[scn_key]
    carla = import_carla()
    from carla_link import CarlaLink

    from datetime import datetime
    now = datetime.now()
    day_dir = os.path.join(LOG_DIR, now.strftime('%Y-%m-%d'))
    os.makedirs(day_dir, exist_ok=True)
    log_path = os.path.join(day_dir, 'cosim_%s_%s.csv'
                            % (scn_key, now.strftime('%H%M%S')))

    print('\n━━━ 场景: %s ━━━' % scn['name'])
    for line in scn.get('notes', []):
        print('  · %s' % line)
    print('connecting CARLA %s:%d ...' % (host, port), flush=True)

    link = CarlaLink(carla, host, port, scn, town=town,
                     no_rendering=no_rendering, spawn_index=spawn_index)
    print('ego=%s lead=%s map=%s' % (
        link.ego.type_id,
        link.lead.type_id if link.lead is not None else '(无)',
        link.map.name), flush=True)

    esp32 = VirtualEsp32()
    workers = WorkerManager()
    workers.start('primary')
    workers.start('backup')
    console = DualSocConsole(workers)
    if keys is not None:
        print('双 SoC 控制台已就绪，输入 ? 查看指令（1/0 主备切换，安全联锁开启）',
              flush=True)

    sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    timeline = sorted(scn.get('timeline', []), key=lambda x: x[0])
    pending = list(timeline)
    duration = float(scn.get('duration', 0.0))

    last_src = None
    last_print = 0.0
    wall_t0 = time.monotonic()
    switches = []

    csv_fh = open(log_path, 'w', newline='')
    writer = csv.writer(csv_fh)
    writer.writerow(['t', 'ego_v', 'gap', 'lane_offset', 'src', 'watchdog',
                     'delta_cmd', 'a_brake', 'steer', 'throttle', 'brake',
                     'pri_alive', 'bak_alive', 'aeb_floor'])

    try:
        while True:
            sim_t = link.tick()
            if duration > 0.0 and sim_t >= duration:
                break

            # 1) 感知 → 双 worker
            frame, gap = link.sense(sim_t)
            payload = json.dumps(frame).encode('utf-8')
            for sport in (SENSOR_PORT_PRIMARY, SENSOR_PORT_BACKUP):
                sensor_sock.sendto(payload, ('127.0.0.1', sport))

            # 2) 前车脚本
            link.drive_lead(sim_t)

            # 3) 虚拟 ESP32 仲裁 → 自车执行
            out = esp32.step()
            steer, throttle, brake = link.apply_ego(out)

            # 4) 场景时间线 / 手动按键
            while pending and sim_t >= pending[0][0]:
                _, action = pending.pop(0)
                print('\n=== t=%.1fs 注入: %s ===' % (sim_t, action), flush=True)
                _do_action(workers, action)
            console.update()
            if keys is not None:
                for k in keys.pop():
                    msgs, quit_now = console.handle_key(k, src=out['src'])
                    for m in msgs:
                        print(m, flush=True)
                    if quit_now:
                        raise KeyboardInterrupt

            # 5) 事件与状态输出
            src = out['src']
            if src != last_src:
                print('\n>>> 执行源切换: %s -> %s (t=%.2fs)'
                      % (SRC_NAMES.get(last_src, '-'),
                         SRC_NAMES.get(src, '?'), sim_t), flush=True)
                switches.append((sim_t, last_src, src))
                last_src = src
            for _, ev in esp32.drain_events():
                print('  [ESP32] %s' % ev, flush=True)
            for _, ev in workers.drain_events():
                print('  [HB] %s' % ev, flush=True)

            link.update_spectator()

            gap_s = '%.2f' % gap if gap != float('inf') else ''
            writer.writerow(['%.2f' % sim_t, '%.2f' % frame['ego_v'],
                             gap_s, '%.3f' % frame['lane_offset'],
                             src, int(out['watchdog']),
                             '%.4f' % out['delta'], '%.2f' % out['a_brake'],
                             '%.3f' % steer, '%.2f' % throttle,
                             '%.2f' % brake,
                             int(workers.alive('primary')),
                             int(workers.alive('backup')),
                             '%.2f' % out['aeb_floor']])
            csv_fh.flush()   # 每帧刷盘，便于监控台 tail 模式实时跟踪（CARLA 在线时显示实时数据）

            if sim_t - last_print >= 1.0:
                print(console.render_panel(sim_t=sim_t, frame=frame, gap=gap,
                                           src=src, out=out), flush=True)
                last_print = sim_t

            # 6) 实时节奏（worker 与心跳按墙钟运行，必须保持实时性）
            if realtime:
                target = wall_t0 + sim_t
                lag = target - time.monotonic()
                if lag > 0:
                    time.sleep(lag)
                elif lag < -2.0:
                    wall_t0 = time.monotonic() - sim_t
    except KeyboardInterrupt:
        print('\n(中断)')
    finally:
        csv_fh.close()
        workers.close()
        esp32.close()
        link.close()
        print('日志: %s' % log_path)
    return switches, log_path


def main(argv=None):
    parser = argparse.ArgumentParser(description='CARLA × SOC 双冗余联合仿真')
    parser.add_argument('--scenario', default='failover', choices=ORDER)
    parser.add_argument('--host', default=CARLA_HOST)
    parser.add_argument('--port', default=CARLA_PORT, type=int)
    parser.add_argument('--town', default=TOWN)
    parser.add_argument('--no-rendering', action='store_true')
    parser.add_argument('--spawn-index', default=None, type=int)
    parser.add_argument('--fast', dest='realtime', action='store_false',
                        default=True, help='不限速推进（离屏回归用）')
    args = parser.parse_args(argv)

    keys = KeyReader.instance()
    run_scenario(args.scenario, host=args.host, port=args.port,
                 town=args.town, no_rendering=args.no_rendering,
                 spawn_index=args.spawn_index, realtime=args.realtime,
                 keys=keys)


if __name__ == '__main__':
    main(sys.argv[1:])
