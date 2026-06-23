#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""一键演示脚本 — 为视频录制自动跑完全部核心场景。

自动启动 CARLA（轻量模式），按顺序运行 LKA → ACC → AEB → 超车 → 降级，
每个场景自动截图关键帧，结束后生成 KPI 报告。全程无需人工输入。

用法：
  python demo_video.py                 # 默认跑 5 个场景（~4 分钟）
  python demo_video.py --scenes lka aeb failover  # 指定场景
  python demo_video.py --no-carla      # 不自动启动 CARLA（已手动启动）
  python demo_video.py --screenshot-dir screenshots  # 截图输出目录
"""

import argparse
import csv
import glob
import math
import os
import socket
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import paths  # noqa: E402 — adds HIL/carla_bridge/pc/ to sys.path for shared bridge modules

from bridge_config import (CARLA_HOST, CARLA_PORT, LOG_DIR,
                            TOWN, FIXED_DT)  # noqa: E402
from scenarios import SCENARIOS  # noqa: E402

CARLA_EXE = os.path.normpath(os.path.join(_HERE, '..', 'CALRA', 'CarlaUE4.exe'))

# 默认演示场景序列（时长经过裁剪，适合 3-5 分钟视频）
DEFAULT_SCENES = [
    ('lka',       30),   # 30s 车道保持
    ('acc',       30),   # 30s 自适应巡航
    ('aeb',       30),   # 30s AEB 急刹（含恢复）
    ('overtake',  45),   # 45s 超车（含回道）
    ('failover',  55),   # 55s 主备降级（含双杀看门狗）
]


# ════════════════════════ 工具函数 ════════════════════════

def carla_reachable(host, port, timeout=2.0):
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        s.close()
        return True
    except OSError:
        return False


def start_carla(host, port, low_quality=True):
    """启动 CarlaUE4.exe 并等待就绪。"""
    if carla_reachable(host, port):
        print('[OK] CARLA 已在运行')
        return True
    if not os.path.isfile(CARLA_EXE):
        print('[!] 未找到 %s' % CARLA_EXE)
        return False
    args = [CARLA_EXE, '-windowed', '-ResX=800', '-ResY=600',
            '-carla-rpc-port=%d' % port]
    if low_quality:
        args.append('-quality-level=Low')
    print('[...] 启动 CARLA: %s' % ' '.join(os.path.basename(a) for a in args))
    subprocess.Popen(args, cwd=os.path.dirname(CARLA_EXE))
    for i in range(120):
        if carla_reachable(host, port):
            print('[OK] CARLA 就绪 (%.0fs)' % (i * 2))
            time.sleep(2.0)
            return True
        if i % 10 == 0:
            print('[...] 等待 CARLA 启动 (%ds)' % (i * 2))
        time.sleep(2.0)
    print('[!] CARLA 启动超时')
    return False


def stop_carla():
    for image in ('CarlaUE4-Win64-Shipping.exe', 'CarlaUE4.exe'):
        try:
            subprocess.run(['taskkill', '/F', '/IM', image],
                           capture_output=True, text=True)
        except Exception:
            pass



def generate_kpi_report(log_path):
    """从 CSV 日志生成 KPI 摘要，返回文本。"""
    if not log_path or not os.path.isfile(log_path):
        return '暂无日志'
    with open(log_path, 'r') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return '日志为空'

    def _f(row, key):
        v = row.get(key, '')
        try:
            return float(v) if v not in ('', None) else None
        except ValueError:
            return None

    t_end = _f(rows[-1], 't') or 0.0
    v_list = [v for r in rows if (v := _f(r, 'ego_v')) is not None]
    gap_list = [v for r in rows if (v := _f(r, 'gap')) is not None]
    off_list = [v for r in rows if (v := _f(r, 'lane_offset')) is not None]

    lines = ['═' * 50]
    lines.append('  KPI 报告: %s' % os.path.basename(log_path))
    lines.append('═' * 50)
    lines.append('  时长: %.1fs (%d 帧 @%dHz)' % (t_end, len(rows), round(1.0 / FIXED_DT)))
    if v_list:
        lines.append('  车速: 最高 %.1f / 平均 %.1f km/h' % (max(v_list) * 3.6, sum(v_list) / len(v_list) * 3.6))
    if gap_list:
        lines.append('  最小车距: %.1f m' % min(gap_list))
    if off_list:
        rms = math.sqrt(sum(o * o for o in off_list) / len(off_list))
        lines.append('  车道偏移 RMS: %.3f m / 最大 %.3f m' % (rms, max(abs(o) for o in off_list)))

    # 切换统计
    switches = []
    prev_src = None
    for r in rows:
        src = r.get('src')
        if src != prev_src and prev_src is not None:
            switches.append((_f(r, 't') or 0, prev_src, src))
        prev_src = src
    if switches:
        names = {'0': 'PRIMARY', '1': 'BACKUP', '9': 'WATCHDOG'}
        lines.append('  执行源切换: %d 次' % len(switches))
        for t, a, b in switches:
            lines.append('    t=%6.1fs  %s -> %s' % (t, names.get(a, a), names.get(b, b)))
    else:
        lines.append('  执行源切换: 无')

    lines.append('═' * 50)
    return '\n'.join(lines)


# ════════════════════════ 场景运行 ════════════════════════

def run_scene(scene_key, duration, host, port, town, screenshot_dir, idx, total=5):
    """运行单个场景，返回日志路径。"""
    scn = SCENARIOS[scene_key]
    print('\n' + '─' * 50)
    print('[%d/%d] %s (%ds)' % (idx + 1, total, scn['name'], duration))
    print('─' * 50)

    # 临时覆盖场景时长（demo 用裁剪版时长）
    orig_duration = scn.get('duration')
    scn['duration'] = float(duration)

    try:
        from run_cosim import run_scenario
        switches, log = run_scenario(
            scene_key, host=host, port=port, town=town,
            no_rendering=False, spawn_index=scn.get('spawn_index'),
            realtime=True, keys=None)  # 无键盘输入
    except KeyboardInterrupt:
        print('\n[!] 用户中断')
        return None
    except Exception as e:
        print('[!] 场景 %s 异常: %s' % (scene_key, e))
        return None
    finally:
        # 恢复原始时长
        if orig_duration is not None:
            scn['duration'] = orig_duration

    if log:
        print('[OK] 日志: %s' % log)
    return log


# ════════════════════════ 主流程 ════════════════════════

def main():
    parser = argparse.ArgumentParser(description='ADAS 一键演示脚本（视频录制用）')
    parser.add_argument('--scenes', nargs='+', default=None,
                        help='要运行的场景列表（默认: lka acc aeb overtake failover）')
    parser.add_argument('--no-carla', action='store_true',
                        help='不自动启动 CARLA')
    parser.add_argument('--host', default=CARLA_HOST)
    parser.add_argument('--port', default=CARLA_PORT, type=int)
    parser.add_argument('--town', default=TOWN)
    parser.add_argument('--screenshot-dir', default='screenshots',
                        help='截图输出目录')
    parser.add_argument('--low-quality', action='store_true', default=True,
                        help='CARLA 低画质模式')
    parser.add_argument('--keep-carla', action='store_true',
                        help='演示结束后不关闭 CARLA')
    args = parser.parse_args()

    # 构建场景序列
    if args.scenes:
        scenes = []
        for s in args.scenes:
            if ':' in s:
                key, dur = s.split(':', 1)
                scenes.append((key, int(dur)))
            elif s in SCENARIOS:
                scenes.append((s, SCENARIOS[s]['duration']))
            else:
                print('[!] 未知场景: %s' % s)
                return 1
    else:
        scenes = DEFAULT_SCENES

    # 创建截图目录
    ss_dir = os.path.join(_HERE, args.screenshot_dir)
    os.makedirs(ss_dir, exist_ok=True)

    print('=' * 50)
    print('  ADAS 双冗余联合仿真 — 一键演示')
    print('  场景: %s' % ' → '.join(k for k, _ in scenes))
    print('  总时长: ~%ds' % sum(d for _, d in scenes))
    print('  截图目录: %s' % ss_dir)
    print('=' * 50)

    # 1) 启动 CARLA
    if not args.no_carla:
        if not start_carla(args.host, args.port, args.low_quality):
            return 1
    else:
        if not carla_reachable(args.host, args.port):
            print('[!] CARLA 不可达 %s:%d' % (args.host, args.port))
            return 1
        print('[OK] CARLA 已连接')

    # 2) 依次运行场景
    logs = []
    t_start = time.time()
    for idx, (key, dur) in enumerate(scenes):
        if key not in SCENARIOS:
            print('[!] 跳过未知场景: %s' % key)
            continue
        log = run_scene(key, dur, args.host, args.port, args.town, ss_dir,
                        idx, total=len(scenes))
        if log:
            logs.append(log)

    t_total = time.time() - t_start

    # 3) 生成汇总报告
    print('\n' + '=' * 50)
    print('  演示完成！总耗时: %.0fs' % t_total)
    print('=' * 50)

    if logs:
        print('\n各场景 KPI:')
        for log in logs:
            print(generate_kpi_report(log))
            print()

    # 4) 保存汇总到文件
    summary_path = os.path.join(LOG_DIR, 'demo_summary.txt')
    with open(summary_path, 'w', encoding='utf-8') as f:
        f.write('ADAS 一键演示汇总\n')
        f.write('运行时间: %s\n' % time.strftime('%Y-%m-%d %H:%M:%S'))
        f.write('场景: %s\n' % ', '.join(k for k, _ in scenes))
        f.write('总耗时: %.0fs\n\n' % t_total)
        for log in logs:
            f.write(generate_kpi_report(log))
            f.write('\n\n')
    print('[OK] 汇总报告: %s' % summary_path)

    # 5) 清理
    if not args.keep_carla:
        print('\n[...] 关闭 CARLA')
        stop_carla()

    return 0


if __name__ == '__main__':
    sys.exit(main())
