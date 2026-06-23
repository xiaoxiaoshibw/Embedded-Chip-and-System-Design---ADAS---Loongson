#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ADAS 联合仿真演示控制台（CLI，一站式入口）。

集成全部功能：
  - 场景演示：LKA / ACC / AEB / 静止前车超车 / 安全无感降级 / 自由交互
  - CARLA 仿真器：一键启动（等待就绪）/ 关闭 / 连接检测
  - SIL 链路自检：无需 CARLA 的接管/回切/看门狗/无感性回归（~40s）
  - KPI 报告：解析最近一次运行的 CSV 遥测（车速/最小车距/切换/无感性）
  - 运行参数设置：host / town / 出生点 / 渲染 / 画质

两种运行模式（同一 CLI）：
  A 本机自跑：CARLA + 本机真实 SOC 控制栈 + 虚拟 ESP32，单机闭环（场景 lka/acc/aeb/...）
  B 连真实 Nano：CARLA 当前端，感知/控制经 UDP 桥到真实双 Nano(.125/.124)+边缘盒(.123)

用法：
  python cli.py                 # 交互菜单（推荐，A/B 都在菜单里）
  python cli.py aeb             # 模式 A：直接运行指定场景
  python cli.py nano            # 模式 B：CARLA × 真实双 Nano 闭环
  python cli.py nano --edge-host 192.168.3.123
  python cli.py sil             # 直接跑 SIL 自检
  python cli.py report          # 直接打印最近一次 KPI 报告
  python cli.py start-carla / stop-carla
  python cli.py --list          # 列出场景
"""

import argparse
import csv
import glob
import os
import socket
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import paths  # noqa: E402 — adds HIL/carla_bridge/pc/ to sys.path for shared bridge modules

from bridge_config import CARLA_HOST, CARLA_PORT, LOG_DIR, TOWN  # noqa: E402
from scenarios import ORDER, SCENARIOS  # noqa: E402

CARLA_EXE = os.path.normpath(os.path.join(_HERE, '..', 'CALRA', 'CarlaUE4.exe'))
# 模式 B（连真实双 Nano）：PC 端闭环桥脚本 + 默认边缘盒 IP
CARLA_DEMO_PY = os.path.normpath(
    os.path.join(_HERE, '..', 'deploy', 'pc_demo', 'carla_demo.py'))
DEFAULT_EDGE_HOST = '192.168.3.123'

BANNER = r"""
┌──────────────────────────────────────────────────────────────┐
│        ADAS 双冗余联合仿真演示系统  (CARLA x SOCCode)        │
│                                                              │
│   感知: CARLA 真值 20Hz ──> 主/备 SOC 节点 (100Hz 真实控制栈)│
│   执行: 虚拟 ESP32 (CRC8 校验/主备仲裁/AEB地板/200ms 看门狗) │
│   降级: UDP 心跳 + 接管种子 ──> 主控宕机无感切换             │
└──────────────────────────────────────────────────────────────┘
"""

RUN_HINT = ('双 SoC 控制台（输入字符后回车，安全联锁开启）：'
            '1=切到备机 0=切回主机 p=杀主控 P=重启主控 b=杀备控 B=重启备控 '
            'h=主控卡死 s=状态面板 ?=帮助 q=结束本场景')

SRC_NAMES = {0: 'PRIMARY', 1: 'BACKUP', 9: 'WATCHDOG'}


# ════════════════════════ CARLA 仿真器管理 ════════════════════════

def carla_reachable(host, port, timeout=2.0):
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        s.close()
        return True
    except OSError:
        return False


def start_carla(settings):
    """启动 CarlaUE4.exe 并等待 RPC 端口就绪。已在运行则直接返回。"""
    if carla_reachable(settings.host, settings.port):
        print('CARLA 已在运行 (%s:%d)' % (settings.host, settings.port))
        return True
    if settings.host not in ('127.0.0.1', 'localhost'):
        print('[!] host=%s 是远程地址，请在远程机器上启动 CARLA' % settings.host)
        return False
    if not os.path.isfile(CARLA_EXE):
        print('[!] 未找到 %s' % CARLA_EXE)
        return False
    args = [CARLA_EXE, '-windowed', '-ResX=1280', '-ResY=720',
            '-carla-rpc-port=%d' % settings.port]
    if settings.low_quality:
        args.append('-quality-level=Low')
    print('启动 %s ...' % ' '.join(os.path.basename(a) for a in args))
    subprocess.Popen(args, cwd=os.path.dirname(CARLA_EXE))
    print('等待 CARLA 就绪（首次启动需 1~2 分钟）', end='', flush=True)
    for _ in range(120):
        if carla_reachable(settings.host, settings.port):
            print(' OK')
            time.sleep(2.0)   # 端口通了再等引擎稳一稳
            return True
        print('.', end='', flush=True)
        time.sleep(2.0)
    print('\n[!] 等待超时，请检查 CARLA 窗口是否报错')
    return False


def stop_carla():
    """结束 CARLA 进程（含 Shipping 子进程）。"""
    killed = False
    for image in ('CarlaUE4-Win64-Shipping.exe', 'CarlaUE4.exe'):
        try:
            r = subprocess.run(['taskkill', '/F', '/IM', image],
                               capture_output=True, text=True)
            if r.returncode == 0:
                killed = True
        except Exception:
            pass
    print('CARLA 已关闭' if killed else 'CARLA 未在运行')


# ════════════════════════ SIL 自检 ════════════════════════

# ════════════════════════ Web 监控台 ════════════════════════

def start_web():
    """启动 Ollama AI 质量监控台（Web 实时分析共享 CSV）。"""
    monitor_dir = os.path.normpath(os.path.join(_HERE, '..', 'ollama模型调用'))
    server_py = os.path.join(monitor_dir, 'server.py')
    if not os.path.isfile(server_py):
        print('[!] 未找到 %s' % server_py)
        return False
    print('启动 Web 监控台（共享 CSV: %s）...' % os.path.join(_HERE, 'logs'))
    print('  浏览器打开 http://127.0.0.1:8765')
    print('  在网页中选择 tail 模式 → 自动跟踪最新仿真日志')
    subprocess.Popen([sys.executable, server_py], cwd=monitor_dir)
    return True


def run_sil():
    """无需 CARLA 的全链路回归（test_failover_sil.main，~40s）。"""
    print('\n── SIL 链路自检（无需 CARLA，约 40 秒）──')
    print('  本机模型 ←UDP→ 主/备 SOC worker ←虚拟UART→ 虚拟 ESP32')
    print('  断言：主→备接管 / 备→主回切 / 双杀看门狗 / 接管无感性\n')
    import test_failover_sil
    code = test_failover_sil.main()
    print('\nSIL 自检%s' % ('通过 ✓' if code == 0 else '未通过 ✗'))
    return code == 0


# ════════════════════════ KPI 报告 ════════════════════════

def _f(row, key, default=None):
    v = row.get(key, '')
    if v in ('', None):
        return default
    try:
        return float(v)
    except ValueError:
        return default


def latest_log():
    files = glob.glob(os.path.join(LOG_DIR, '**', 'cosim_*.csv'), recursive=True)
    return max(files, key=os.path.getmtime) if files else None


def report(path=None):
    """解析运行 CSV，打印 KPI 摘要。"""
    path = path or latest_log()
    if not path or not os.path.isfile(path):
        print('暂无运行日志（logs/cosim_*.csv）')
        return
    with open(path, 'r') as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        print('日志为空: %s' % path)
        return

    t_end = _f(rows[-1], 't', 0.0)
    v_list = [_f(r, 'ego_v') for r in rows if _f(r, 'ego_v') is not None]
    gap_list = [_f(r, 'gap') for r in rows if _f(r, 'gap') is not None]
    off_list = [_f(r, 'lane_offset') for r in rows
                if _f(r, 'lane_offset') is not None]

    # 执行源切换与看门狗
    switches = []
    wd_steps = 0
    aeb_floor_steps = 0
    prev_src = None
    for r in rows:
        src = r.get('src')
        if src != prev_src and prev_src is not None:
            switches.append((_f(r, 't', 0.0), prev_src, src))
        prev_src = src
        if r.get('watchdog') == '1':
            wd_steps += 1
        if (_f(r, 'aeb_floor', 0.0) or 0.0) > 0.0:
            aeb_floor_steps += 1

    # 无感性：每次切换 ±1s 窗口内 a_brake 最大单步阶跃
    times = [_f(r, 't', 0.0) for r in rows]
    abrk = [_f(r, 'a_brake', 0.0) or 0.0 for r in rows]
    seamless = []
    for ts, a, b in switches:
        win = [abrk[i] for i, t in enumerate(times) if abs(t - ts) <= 1.0]
        step = max((abs(win[i + 1] - win[i]) for i in range(len(win) - 1)),
                   default=0.0)
        seamless.append((ts, a, b, step))

    print('\n━━━ KPI 报告: %s ━━━' % os.path.basename(path))
    print('  时长          : %.1f s（%d 帧 @20Hz）' % (t_end, len(rows)))
    if v_list:
        print('  车速          : 最高 %.2f / 平均 %.2f m/s'
              % (max(v_list), sum(v_list) / len(v_list)))
    if gap_list:
        print('  最小车距      : %.2f m' % min(gap_list))
    if off_list:
        rms = (sum(o * o for o in off_list) / len(off_list)) ** 0.5
        print('  车道偏移      : RMS %.3f / 最大 %.3f m'
              % (rms, max(abs(o) for o in off_list)))
    print('  ESP32 AEB 地板: %s' % ('触发 %d 帧' % aeb_floor_steps
                                    if aeb_floor_steps else '未触发'))
    print('  看门狗紧急制动: %s' % ('%d 帧 (%.1fs)' % (wd_steps, wd_steps * 0.05)
                                    if wd_steps else '未触发'))
    if switches:
        print('  执行源切换    :')
        names = {'0': 'PRIMARY', '1': 'BACKUP', '9': 'WATCHDOG'}
        for ts, a, b, step in seamless:
            print('    t=%6.2fs  %-8s -> %-8s  切换窗内减速度最大阶跃 %.2f m/s²'
                  % (ts, names.get(a, a), names.get(b, b), step))
    else:
        print('  执行源切换    : 无（全程 PRIMARY）')
    print('  完整数据      : %s' % path)


# ════════════════════════ 场景运行 ════════════════════════

def show_notes(key):
    scn = SCENARIOS[key]
    print('\n── %s ──' % scn['name'])
    for line in scn.get('notes', []):
        print('  · %s' % line)
    print('  · %s\n' % RUN_HINT)


def run(key, settings):
    if not carla_reachable(settings.host, settings.port):
        print('\n[!] 无法连接 CARLA %s:%d' % (settings.host, settings.port))
        ans = _reader().read_line('    现在启动 CARLA？[Y/n] > ')
        if ans is None or ans.strip().lower() in ('n', 'no'):
            return False
        if not start_carla(settings):
            return False
    from run_cosim import KeyReader, run_scenario
    show_notes(key)
    try:
        switches, log = run_scenario(
            key, host=settings.host, port=settings.port, town=settings.town,
            no_rendering=settings.no_rendering,
            spawn_index=settings.spawn_index,
            realtime=True, keys=KeyReader.instance())
    except SystemExit as e:
        print(e)
        return False
    if switches:
        print('\n本场景执行源切换记录:')
        for t, a, b in switches:
            print('  t=%6.2fs  %s -> %s'
                  % (t, SRC_NAMES.get(a, '-'), SRC_NAMES.get(b, b)))
    print('（菜单选 [r] 可查看本次 KPI 报告）')
    return True


# ════════════════════════ 模式 B：CARLA × 真实双 Nano 闭环 ════════════════════════

def run_nano(settings):
    """模式 B：本机 CARLA 当前端，感知/控制经 UDP 桥接到真实双 Nano + 边缘盒。

    与模式 A（本机自跑 cosim）的区别：控制由真实 .125/.124 Nano 解算，
    本机只跑 CARLA + carla_demo.py（PC↔UDP 桥）。需边缘盒(.123) 跑
    udp_to_ros2 + ros2_to_udp，两台 Nano adas-node 在线（详见 deploy/pc_demo/README.md）。
    """
    if not os.path.isfile(CARLA_DEMO_PY):
        print('[!] 未找到 %s（模式 B 需要 deploy/pc_demo/ 一并拷贝）' % CARLA_DEMO_PY)
        return False
    try:
        import carla  # noqa: F401
    except Exception:
        print('[!] 未检测到 carla Python 客户端。先装 CARLA whl：')
        print('    pip install <CARLA>\\PythonAPI\\carla\\dist\\carla-0.9.16-cp312-cp312-win_amd64.whl')
        return False

    if not carla_reachable(settings.host, settings.port):
        print('\n[!] 无法连接 CARLA %s:%d' % (settings.host, settings.port))
        ans = _reader().read_line('    现在启动 CARLA？[Y/n] > ')
        if ans is None or ans.strip().lower() in ('n', 'no'):
            return False
        if not start_carla(settings):
            return False

    cmd = [sys.executable, CARLA_DEMO_PY,
           '--carla-host', settings.host, '--carla-port', str(settings.port),
           '--edge-host', settings.edge_host]
    if settings.town:
        cmd += ['--town', settings.town]
    if settings.spawn_index is not None:
        cmd += ['--spawn-index', str(settings.spawn_index)]

    print('\n── 模式 B：CARLA × 真实双 Nano 闭环 ──')
    print('  边缘盒 : %s（需在跑 udp_to_ros2 + ros2_to_udp）' % settings.edge_host)
    print('  Nano   : 主 .125 / 备 .124 adas-node 在线，自车由真实 Nano 控制驱动')
    print('  提示   : 在 Nano 上跑 adas-reset.sh 或杀主控 → 副控接管，CARLA 里车不抖')
    print('  结束   : Ctrl-C\n')
    try:
        subprocess.call(cmd, cwd=os.path.dirname(CARLA_DEMO_PY))
    except KeyboardInterrupt:
        print('\n（模式 B 已结束）')
    return True


# ════════════════════════ 设置 ════════════════════════

class Settings:
    def __init__(self, args):
        self.host = args.host
        self.port = args.port
        self.town = args.town
        self.spawn_index = args.spawn_index   # None=用场景默认
        self.no_rendering = args.no_rendering
        self.low_quality = args.low_quality
        # 模式 B（连真实双 Nano）用：边缘盒 IP
        self.edge_host = getattr(args, 'edge_host', None) or DEFAULT_EDGE_HOST

    def show(self):
        print('\n当前设置:')
        print('  [1] CARLA 地址      : %s:%d' % (self.host, self.port))
        print('  [2] 地图            : %s' % self.town)
        print('  [3] 出生点覆盖      : %s'
              % ('场景默认' if self.spawn_index is None else self.spawn_index))
        print('  [4] 服务器端渲染    : %s' % ('关（离屏）' if self.no_rendering else '开'))
        print('  [5] 启动画质        : %s' % ('Low' if self.low_quality else 'Epic'))
        print('  [6] 边缘盒IP(模式B) : %s' % self.edge_host)
        print('  [0] 返回')


def settings_menu(settings):
    reader = _reader()
    while True:
        settings.show()
        c = reader.read_line('设置项 > ')
        if c is None or c.strip() in ('0', '', 'q'):
            return
        c = c.strip()
        if c == '1':
            v = reader.read_line('host[:port]（当前 %s:%d）> '
                                 % (settings.host, settings.port))
            if v and v.strip():
                v = v.strip()
                if ':' in v:
                    h, _, p = v.partition(':')
                    settings.host = h or settings.host
                    try:
                        settings.port = int(p)
                    except ValueError:
                        pass
                else:
                    settings.host = v
        elif c == '2':
            v = reader.read_line('地图（如 Town04）> ')
            if v and v.strip():
                settings.town = v.strip()
        elif c == '3':
            v = reader.read_line('出生点序号（空=场景默认）> ')
            if v is not None:
                v = v.strip()
                settings.spawn_index = int(v) if v.isdigit() else None
        elif c == '4':
            settings.no_rendering = not settings.no_rendering
        elif c == '5':
            settings.low_quality = not settings.low_quality
        elif c == '6':
            v = reader.read_line('边缘盒 IP（当前 %s）> ' % settings.edge_host)
            if v and v.strip():
                settings.edge_host = v.strip()
        else:
            print('无效输入: %r' % c)


# ════════════════════════ 菜单 ════════════════════════

def _reader():
    from run_cosim import KeyReader
    return KeyReader.instance()


def print_menu(settings):
    print(BANNER)
    online = carla_reachable(settings.host, settings.port, timeout=0.3)
    print('  CARLA: %s   地图: %s   日志: %s\n'
          % ('● 在线' if online else '○ 离线（选 c 启动）',
             settings.town, '有' if latest_log() else '无'))
    print('  ── 模式 A：本机自跑（CARLA + 本机 SOC 控制栈，单机闭环）──')
    for i, key in enumerate(ORDER, 1):
        scn = SCENARIOS[key]
        dur = ('%.0fs' % scn['duration']) if scn['duration'] > 0 else '手动'
        print('  [%d] %-10s %-18s 时长: %s' % (i, key, scn['name'], dur))
    print('\n  ── 模式 B：连真实双 Nano（CARLA 当前端，控制在 .125/.124）──')
    print('  [n] CARLA × 真实双 Nano 闭环  (边缘盒 %s，可在 [o] 改)' % settings.edge_host)
    print('\n  ── 工具 ──')
    print('  [c] 启动 CARLA 仿真器        [k] 关闭 CARLA')
    print('  [s] SIL 链路自检（无需 CARLA，~40s）')
    print('  [w] 启动 Web 实时监控台')
    print('  [r] 最近一次运行 KPI 报告')
    print('  [o] 运行参数设置')
    print('  [0] 退出')


def menu_loop(settings):
    reader = _reader()
    while True:
        print_menu(settings)
        choice = reader.read_line('\n选择 > ')
        if choice is None:
            return 0
        choice = choice.strip()
        if choice in ('0', 'q', 'quit', 'exit'):
            return 0
        if not choice:
            continue

        key = None
        if choice.isdigit() and 1 <= int(choice) <= len(ORDER):
            key = ORDER[int(choice) - 1]
        elif choice in SCENARIOS:
            key = choice

        if key is not None:
            run(key, settings)
        elif choice == 'n':
            run_nano(settings)
        elif choice == 'c':
            start_carla(settings)
        elif choice == 'k':
            stop_carla()
        elif choice == 's':
            run_sil()
        elif choice == 'w':
            start_web()
        elif choice == 'r':
            report()
        elif choice == 'o':
            settings_menu(settings)
            continue            # 设置后直接重绘菜单
        else:
            print('无效输入: %r' % choice)
            continue
        if reader.read_line('\n回车返回菜单...') is None:
            return 0


# ════════════════════════ 入口 ════════════════════════

def main(argv=None):
    parser = argparse.ArgumentParser(
        description='ADAS 联合仿真演示控制台（一站式，模式 A 本机自跑 / 模式 B 连真实 Nano）',
        epilog='模式A场景: %s；模式B: nano；其他: sil, web, report, start-carla, stop-carla'
               % ', '.join(ORDER))
    parser.add_argument('command', nargs='?', default=None,
                        help='场景名或命令（缺省进入交互菜单）')
    parser.add_argument('--list', action='store_true', help='列出场景')
    parser.add_argument('--host', default=CARLA_HOST)
    parser.add_argument('--port', default=CARLA_PORT, type=int)
    parser.add_argument('--town', default=TOWN)
    parser.add_argument('--spawn-index', default=None, type=int,
                        help='覆盖场景默认出生点')
    parser.add_argument('--no-rendering', action='store_true',
                        help='服务器端不渲染（离屏回归）')
    parser.add_argument('--low-quality', action='store_true',
                        help='以 -quality-level=Low 启动 CARLA')
    parser.add_argument('--edge-host', default=None,
                        help='模式 B 边缘盒 IP（默认 %s）' % DEFAULT_EDGE_HOST)
    args = parser.parse_args(argv)
    settings = Settings(args)

    if args.list:
        for key in ORDER:
            print('%-10s %s' % (key, SCENARIOS[key]['name']))
        return 0

    if args.command:
        cmd = args.command
        if cmd in SCENARIOS:
            return 0 if run(cmd, settings) else 1
        if cmd == 'nano':
            return 0 if run_nano(settings) else 1
        if cmd == 'sil':
            return 0 if run_sil() else 1
        if cmd == 'web':
            return 0 if start_web() else 1
        if cmd == 'report':
            report()
            return 0
        if cmd == 'start-carla':
            return 0 if start_carla(settings) else 1
        if cmd == 'stop-carla':
            stop_carla()
            return 0
        print('未知场景/命令: %s（可选: %s, nano, sil, web, report, start-carla, stop-carla）'
              % (cmd, ', '.join(ORDER)))
        return 1

    return menu_loop(settings)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
