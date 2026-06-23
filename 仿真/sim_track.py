#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯 Python 赛道仿真：不依赖 CARLA，自建赛道 + 车辆动力学 + ADAS pipeline。

用法：
    python sim_track.py                    # 跑全部赛道，输出 CSV + 赛道图
    python sim_track.py --track s_curve    # 跑指定赛道
    python sim_track.py --no-plot          # 不生成图，只输出 CSV

输出：
    仿真/logs/2026-MM-DD/sim_<赛道>_HHMMSS.csv
    仿真/logs/2026-MM-DD/sim_<赛道>_HHMMSS.png
"""

import argparse
import csv
import math
import os
import sys
from datetime import datetime

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, 'soc_code'))

import config as soc_cfg
from control.context import (
    ControlManagers, ControlMemory, VehicleSignals,
)
from pipeline import run_pure_pipeline

# ── 各算法管理器 ──
from lateral import LaneWidthEstimator
from control.lead_tracking import LeadTracker
from control.aeb_alert import AebAlertManager
from control.curve_hold import CurveHoldManager
from longitudinal import LongitudinalController, LonSmoothing


# ══════════════════════════════════════════════════════════════
#  赛道定义
# ══════════════════════════════════════════════════════════════

def _make_track_straight():
    """200m 直道。"""
    pts = []
    for i in range(201):
        pts.append((float(i), 0.0))
    return pts, '直道 200m'


def _make_track_curve():
    """200m 弯道（半径 80m 的圆弧，约 140°）。"""
    R = 80.0
    pts = []
    n = 200
    for i in range(n + 1):
        theta = math.pi * i / n  # 0 → π (180°)
        x = R * math.sin(theta)
        y = R * (1.0 - math.cos(theta))
        pts.append((x, y))
    return pts, '弯道 R=80m'


def _make_track_s_curve():
    """S 弯：两段反向圆弧 + 过渡直线。"""
    R = 60.0
    pts = []
    # 第一段：左弯（0→90°）
    n1 = 60
    for i in range(n1 + 1):
        theta = math.pi * 0.5 * i / n1
        x = R * math.sin(theta)
        y = R * (1.0 - math.cos(theta))
        pts.append((x, y))
    x0, y0 = pts[-1]
    # 过渡直线
    for i in range(1, 31):
        pts.append((x0 + i * 1.0, y0))
    x1, y1 = pts[-1]
    # 第二段：右弯（90°→0）
    n2 = 60
    for i in range(1, n2 + 1):
        theta = math.pi * 0.5 * (1.0 - i / n2)
        dx = R * math.sin(theta)
        dy = R * (1.0 - math.cos(theta))
        pts.append((x1 + dx - R * math.sin(math.pi * 0.5) + R,
                     y1 + dy))
    return pts, 'S弯 R=60m'


def _make_track_highway():
    """高速场景：直道+缓弯+直道，400m，带前车。"""
    pts = []
    # 直道 100m
    for i in range(101):
        pts.append((float(i), 0.0))
    # 缓弯 R=200m，30°
    R = 200.0
    n = 80
    x0, y0 = pts[-1]
    for i in range(1, n + 1):
        theta = math.pi / 6.0 * i / n  # 30°
        x = x0 + R * math.sin(theta)
        y = y0 + R * (1.0 - math.cos(theta))
        pts.append((x, y))
    # 直道延伸
    x1, y1 = pts[-1]
    fwd_x = math.cos(math.pi / 6.0)
    fwd_y = math.sin(math.pi / 6.0)
    for i in range(1, 201):
        pts.append((x1 + i * fwd_x, y1 + i * fwd_y))
    return pts, '高速场景 400m'


TRACKS = {
    'straight': _make_track_straight,
    'curve': _make_track_curve,
    's_curve': _make_track_s_curve,
    'highway': _make_track_highway,
}


# ══════════════════════════════════════════════════════════════
#  路径插值 + 曲率
# ══════════════════════════════════════════════════════════════

class RoadPath:
    """离散路径点的弧长参数化插值。"""

    def __init__(self, points):
        self.pts = list(points)
        n = len(self.pts)
        # 累积弧长
        self.s = [0.0]
        for i in range(1, n):
            dx = self.pts[i][0] - self.pts[i - 1][0]
            dy = self.pts[i][1] - self.pts[i - 1][1]
            self.s.append(self.s[-1] + math.hypot(dx, dy))
        self.total_s = self.s[-1]
        # 航向
        self.yaw = [0.0] * n
        for i in range(n):
            j = min(i + 1, n - 1)
            k = max(i - 1, 0)
            if j == k:
                self.yaw[i] = 0.0
            else:
                dx = self.pts[j][0] - self.pts[k][0]
                dy = self.pts[j][1] - self.pts[k][1]
                self.yaw[i] = math.atan2(dy, dx)

    def _find_seg(self, s_query):
        """二分查找所在段。"""
        lo, hi = 0, len(self.s) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if self.s[mid] <= s_query:
                lo = mid
            else:
                hi = mid
        return lo

    def interpolate(self, s_query):
        """返回 (x, y, yaw, curvature)。"""
        s_query = max(0.0, min(s_query, self.total_s))
        seg = self._find_seg(s_query)
        seg_len = self.s[seg + 1] - self.s[seg]
        if seg_len < 1e-6:
            t = 0.0
        else:
            t = (s_query - self.s[seg]) / seg_len
        x = self.pts[seg][0] + t * (self.pts[seg + 1][0] - self.pts[seg][0])
        y = self.pts[seg][1] + t * (self.pts[seg + 1][1] - self.pts[seg][1])
        # 航向插值（处理 ±π 跳变）
        y0, y1 = self.yaw[seg], self.yaw[min(seg + 1, len(self.yaw) - 1)]
        dy = y1 - y0
        while dy > math.pi:
            dy -= 2 * math.pi
        while dy < -math.pi:
            dy += 2 * math.pi
        yaw = y0 + t * dy
        # 曲率（三点法）
        k = max(seg - 1, 0)
        j = min(seg + 2, len(self.pts) - 1)
        if j > k + 1:
            ax, ay = self.pts[k]
            bx, by = self.pts[(k + j) // 2]
            cx, cy = self.pts[j]
            d = 2.0 * ((bx - ax) * (cy - ay) - (by - ay) * (cx - ax))
            if abs(d) > 1e-6:
                ux = ((bx * bx - ax * ax + by * by - ay * ay) * (cy - ay)
                      - (cy * cy - ay * ay + cx * cx - ax * ax) * (by - ay)) / d
                uy = ((cy * cy - ay * ay + cx * cx - ax * ax) * (bx - ax)
                      - (bx * bx - ax * ax + by * by - ay * ay) * (cx - ax)) / d
                r = math.hypot(ux - ax, uy - ay)
                curv = 1.0 / max(r, 1e-6)
                # 符号：叉积判断左弯(+)/右弯(-)
                cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
                curv *= (1.0 if cross > 0 else -1.0)
            else:
                curv = 0.0
        else:
            curv = 0.0
        return x, y, yaw, curv


# ══════════════════════════════════════════════════════════════
#  车辆动力学模型
# ══════════════════════════════════════════════════════════════

class VehicleModel:
    """自行车模型 + 简单纵向动力学。"""

    L = 2.8  # 轴距 (m)

    def __init__(self, x, y, yaw, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def step(self, delta, a_lon, dt):
        """delta=前轮转角(rad), a_lon=纵向加速度(m/s², 正=加速), dt。"""
        # 纵向
        self.v = max(0.0, self.v + a_lon * dt)
        # 横向（自行车模型）
        if abs(delta) > 1e-6:
            R = self.L / math.tan(delta)
            beta = self.v * dt / R
            self.x += R * (math.sin(self.yaw + beta) - math.sin(self.yaw))
            self.y += R * (math.cos(self.yaw) - math.cos(self.yaw + beta))
            self.yaw += beta
        else:
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
        # 规范化 yaw
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi


# ══════════════════════════════════════════════════════════════
#  主仿真循环
# ══════════════════════════════════════════════════════════════

def run_track(track_name, output_dir, gen_plot=True):
    """跑一个赛道场景，返回 CSV 路径和 PNG 路径。"""
    # ── 赛道 ──
    pts, desc = TRACKS[track_name]()
    road = RoadPath(pts)
    print('赛道: %s  总长 %.0fm' % (desc, road.total_s))

    # ── 仿真参数 ──
    dt = 0.01          # 100Hz（与 SOC 控制环一致）
    v_target = 15.0    # 目标巡航速度 m/s (54 km/h)
    lead_gap0 = 50.0   # 前车初始距离
    lead_v = 12.0      # 前车速度 m/s
    lead_brake_t = 8.0 # 前车刹车时刻
    lead_brake_a = -5.0

    # ── 自车初始状态 ──
    sx0, sy0, syaw0, _ = road.interpolate(0.0)
    ego = VehicleModel(sx0, sy0, syaw0, v=v_target * 0.5)

    # ── 前车初始状态 ──
    lead_s = lead_gap0
    lead_v_cur = lead_v
    lead_x, lead_y, lead_yaw, _ = road.interpolate(lead_s)

    # ── ADAS 管理器初始化 ──
    memory = ControlMemory(dt=dt)
    memory.filtered_v_tgt = v_target
    managers = ControlManagers(
        lane_est=LaneWidthEstimator(loop_hz=100),
        lead_tracker=LeadTracker(),
        aeb_alert=AebAlertManager(),
        curve_hold=CurveHoldManager(),
        lon_ctrl=LongitudinalController(dt=dt),
        lon_smooth=LonSmoothing(dt=dt),
    )

    # ── 日志 ──
    now = datetime.now()
    day_dir = os.path.join(output_dir, now.strftime('%Y-%m-%d'))
    os.makedirs(day_dir, exist_ok=True)
    ts = now.strftime('%H%M%S')
    csv_path = os.path.join(day_dir, 'sim_%s_%s.csv' % (track_name, ts))
    png_path = os.path.join(day_dir, 'sim_%s_%s.png' % (track_name, ts))

    log_rows = []
    ego_traj = []   # (x, y) 用于绘图
    lead_traj = []

    # ── 仿真主循环 ──
    t = 0.0
    max_t = road.total_s / v_target * 1.5 + 10.0  # 跑到路径末尾 + 余量
    ego_s = 0.0   # 自车在路径上的弧长投影

    print('仿真中... (dt=%.3fs, v_target=%.1fm/s, max_t=%.1fs)' % (dt, v_target, max_t))

    while t < max_t and ego_s < road.total_s - 1.0:
        # ── 感知：从路径提取 road_psi, lane_offset ──
        rx, ry, road_yaw, road_curv = road.interpolate(ego_s)

        # 自车相对路径的横向偏移
        dx = ego.x - rx
        dy = ego.y - ry
        c, s = math.cos(road_yaw), math.sin(road_yaw)
        lane_offset = -s * dx + c * dy

        # 航向差
        ego_yaw_err = ego.yaw - road_yaw
        while ego_yaw_err > math.pi:
            ego_yaw_err -= 2 * math.pi
        while ego_yaw_err < -math.pi:
            ego_yaw_err += 2 * math.pi

        road_psi = road_yaw  # 控制器用的 road_psi

        # ── 前车状态 ──
        # 前车沿路径行驶
        if t >= lead_brake_t:
            lead_v_cur = max(0.0, lead_v_cur + lead_brake_a * dt)
        lead_s += lead_v_cur * dt
        lx, ly, lyaw, _ = road.interpolate(lead_s)
        lead_x, lead_y, lead_yaw = lx, ly, lyaw

        # 前车距离（沿路径弧长近似）
        gap = lead_s - ego_s
        lead_present = 0.5 < gap < 120.0

        # ── 构建 VehicleSignals ──
        signals = VehicleSignals()
        signals.ego_x = ego.x
        signals.ego_y = ego.y
        signals.ego_yaw = ego.yaw
        signals.ego_v = ego.v
        signals.ego_received = True
        signals.ego_psi_received = True
        signals.road_psi = road_psi
        signals.road_received = True
        signals.road_last_rx = t
        signals.ego_last_rx = t
        signals.lane_offset = lane_offset
        signals.lane_offset_received = True
        signals.lane_offset_last_rx = t

        if lead_present:
            signals.lead_x = lead_x
            signals.lead_y = lead_y
            signals.lead_yaw = lead_yaw
            signals.lead_v = lead_v_cur
            signals.lead_cls = 1
            signals.lead_received = True
            signals.lead_last_rx_time = t
            signals.lead_v_last_rx_time = t

        # ── 运行 ADAS pipeline ──
        snap = signals.snapshot()
        result = run_pure_pipeline(t, snap, memory, managers)

        # ── 执行器输出 ──
        delta = result.lateral_ctx.delta        # 方向盘转角 (rad)
        a_lon_cmd = result.lon_cmd              # 纵向指令 (正=减速)

        # pipeline 输出：正=减速 → 转为加速度（正=加速）
        a_lon = -a_lon_cmd

        # ── 更新车辆动力学 ──
        ego.step(delta, a_lon, dt)

        # 更新自车弧长投影
        proj = (ego.x - rx) * c + (ego.y - ry) * s
        ego_s += proj + ego.v * dt * 0.1  # 投影 + 微调
        # 用实际位置重新投影到路径上（简化：在最近点附近搜索）
        best_ds = 0.0
        best_dist = 1e9
        for ds_off in range(-5, 6):
            ss = ego_s + ds_off * 2.0
            if ss < 0 or ss > road.total_s:
                continue
            px, py, _, _ = road.interpolate(ss)
            d = math.hypot(ego.x - px, ego.y - py)
            if d < best_dist:
                best_dist = d
                best_ds = ss
        ego_s = best_ds

        # ── 记录 ──
        ego_traj.append((ego.x, ego.y))
        lead_traj.append((lead_x, lead_y))

        log_rows.append({
            't': '%.3f' % t,
            'ego_v': '%.3f' % ego.v,
            'gap': '%.3f' % gap if lead_present else '',
            'lane_offset': '%.4f' % lane_offset,
            'src': 'PRIMARY',
            'watchdog': '0',
            'delta_cmd': '%.4f' % delta,
            'a_brake': '%.3f' % a_lon_cmd,
            'steer': '%.4f' % delta,
            'throttle': '%.3f' % max(0.0, a_lon / 3.0),
            'brake': '%.3f' % max(0.0, -a_lon / 8.0),
            'pri_alive': '1',
            'bak_alive': '1',
            'aeb_floor': '0',
        })

        t += dt

    # ── 写 CSV ──
    with open(csv_path, 'w', newline='') as fh:
        cols = ['t', 'ego_v', 'gap', 'lane_offset', 'src', 'watchdog',
                'delta_cmd', 'a_brake', 'steer', 'throttle', 'brake',
                'pri_alive', 'bak_alive', 'aeb_floor']
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(log_rows)
    print('CSV: %s (%d 帧)' % (csv_path, len(log_rows)))

    # ── 绘图 ──
    if gen_plot:
        _plot(track_name, desc, road, ego_traj, lead_traj, log_rows, png_path)
        print('PNG: %s' % png_path)

    return csv_path, png_path


# ══════════════════════════════════════════════════════════════
#  绘图
# ══════════════════════════════════════════════════════════════

def _plot(track_name, desc, road, ego_traj, lead_traj, rows, png_path):
    """生成赛道 + 轨迹 + 数据面板的组合图。"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
        plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
    except ImportError:
        print('[!] 需要 matplotlib: pip install matplotlib')
        return

    fig = plt.figure(figsize=(16, 10))
    gs = gridspec.GridSpec(2, 2, hspace=0.35, wspace=0.3)

    # ── 左上：赛道 + 轨迹 ──
    ax1 = fig.add_subplot(gs[0, 0])
    road_x = [p[0] for p in road.pts]
    road_y = [p[1] for p in road.pts]
    ax1.plot(road_x, road_y, 'k-', linewidth=8, alpha=0.15, label='车道')
    ax1.plot(road_x, road_y, 'k--', linewidth=1, alpha=0.4)

    ex = [p[0] for p in ego_traj]
    ey = [p[1] for p in ego_traj]
    lx = [p[0] for p in lead_traj]
    ly = [p[1] for p in lead_traj]
    ax1.plot(ex, ey, 'b-', linewidth=1.5, label='自车', zorder=3)
    ax1.plot(lx, ly, 'r-', linewidth=1.5, label='前车', zorder=3)
    ax1.plot(ex[0], ey[0], 'g^', markersize=10, label='起点', zorder=4)
    ax1.plot(ex[-1], ey[-1], 'bs', markersize=8, label='终点', zorder=4)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('赛道轨迹 — %s' % desc)
    ax1.legend(fontsize=8, loc='best')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    # ── 右上：速度 ──
    ts = [float(r['t']) for r in rows]
    vs = [float(r['ego_v']) for r in rows]
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(ts, vs, 'b-', linewidth=1.2, label='自车速度')
    # 前车速度（有值时）
    lvs = []
    for r in rows:
        try:
            lvs.append(float(r['gap']))
        except (ValueError, TypeError):
            lvs.append(None)
    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('速度 (m/s)')
    ax2.set_title('车速')
    ax2.grid(True, alpha=0.3)

    # ── 左下：车道偏移 ──
    offsets = [float(r['lane_offset']) for r in rows]
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(ts, offsets, 'g-', linewidth=1.0)
    ax3.axhline(y=0.5, color='orange', linestyle='--', linewidth=0.8, alpha=0.7, label='预警 ±0.5m')
    ax3.axhline(y=-0.5, color='orange', linestyle='--', linewidth=0.8, alpha=0.7)
    ax3.axhline(y=0.0, color='gray', linestyle=':', linewidth=0.5)
    ax3.set_xlabel('时间 (s)')
    ax3.set_ylabel('偏移 (m)')
    ax3.set_title('车道横向偏移')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # ── 右下：车距 + 控制输出 ──
    ax4 = fig.add_subplot(gs[1, 1])
    gaps = []
    for r in rows:
        try:
            gaps.append(float(r['gap']))
        except (ValueError, TypeError):
            gaps.append(None)
    ax4.plot(ts, gaps, 'r-', linewidth=1.0, label='车距 (m)')
    ax4.set_xlabel('时间 (s)')
    ax4.set_ylabel('车距 (m)', color='r')
    ax4.tick_params(axis='y', labelcolor='r')

    ax4b = ax4.twinx()
    abrakes = [float(r['a_brake']) for r in rows]
    ax4b.plot(ts, abrakes, 'm-', linewidth=0.8, alpha=0.7, label='纵向指令')
    ax4b.set_ylabel('纵向指令 (m/s²)', color='m')
    ax4b.tick_params(axis='y', labelcolor='m')
    ax4.set_title('车距 & 纵向控制')
    ax4.grid(True, alpha=0.3)

    fig.suptitle('ADAS 赛道仿真 — %s' % track_name, fontsize=14, fontweight='bold')
    fig.savefig(png_path, dpi=120, bbox_inches='tight')
    plt.close(fig)


# ══════════════════════════════════════════════════════════════
#  入口
# ══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='ADAS 纯 Python 赛道仿真')
    parser.add_argument('--track', default=None,
                        choices=list(TRACKS.keys()),
                        help='指定赛道（默认跑全部）')
    parser.add_argument('--no-plot', action='store_true',
                        help='不生成图，只输出 CSV')
    parser.add_argument('--output-dir', default=os.path.join(_HERE, 'logs'),
                        help='输出目录（默认 仿真/logs/）')
    args = parser.parse_args()

    tracks = [args.track] if args.track else list(TRACKS.keys())
    results = []
    for name in tracks:
        print('\n━━━ 赛道: %s ━━━' % name)
        csv_p, png_p = run_track(name, args.output_dir, gen_plot=not args.no_plot)
        results.append((name, csv_p, png_p))

    print('\n━━━ 全部完成 ━━━')
    for name, c, p in results:
        print('  %-12s CSV: %s' % (name, c))
        if p:
            print('  %-12s PNG: %s' % ('', p))


if __name__ == '__main__':
    main()
