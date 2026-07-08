#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""全速率遥测落盘。

每个控制周期把关键内部量写成 CSV，供离线定量分析（plot_telemetry.py /
replay.py / 评测框架复用）。

设计与 serial_link.Esp32Serial 一致：所有磁盘 IO 都在后台线程内完成，
控制线程只做非阻塞 put_nowait，绝不让 csv 写盘/flush 拖慢 100Hz 控制环。

开关：环境变量 TELEMETRY=0 完全关闭（record() 变成空操作，零开销）。
路径：/tmp/adas_<role>_telemetry_<启动时间>.csv

生命周期（防实机文件堆积，2026-07-03）：
  - 懒建文件：CSV 在**第一行数据**到达时才创建——空闲重启不再留下 645 字节
    的纯表头文件（实机曾积 76 个）。
  - 保留清理：写盘线程启动时按 mtime 只保留本 role 最近 TELEMETRY_KEEP
    （默认 20）个历史 CSV，更旧的删除。TELEMETRY_KEEP=0 关闭清理。
"""

import csv
import glob
import logging
import os
import queue
import threading
import time

# 列顺序固定，header 与每行严格对应。新增字段往后追加，不要插中间，
# 否则历史 CSV 与新脚本列错位。
FIELDS = (
    't_wall',            # 墙钟时间 (epoch s)，用于和 Simulink/外部对齐
    't_mono',            # 单调时钟 (s)
    'cycle',             # 控制周期计数
    'ego_x', 'ego_y', 'ego_yaw', 'ego_v',
    'lead_x', 'lead_y', 'lead_v',
    'road_psi', 'filtered_road_psi',
    'raw_cte', 'filtered_cte',
    'raw_curv', 'filtered_curv', 'curv_guard', 'in_curve',
    'delta', 'delta_cte', 'delta_ff', 'boundary_delta', 'psi_i_term',
    'upd_psi',
    'lon_raw_cmd', 'lon_cmd', 'acc_i_term',
    'aeb_active', 'in_curve_hold',
    'dist', 'ttc', 'lead_v_proj', 'min_safe_dist', 'closing_speed',
    'acc_has_lead', 'lead_detected',
    'cur_lane_width', 'lane_safe_margin', 'lane_warn_margin', 'lane_hard_margin',
    'boundary_brake', 'boundary_warn',
    'psi_tx', 'delta_tx', 'speed_tx', 'lon_tx',
    'esp_psi', 'esp_delta', 'esp_brake',
    # class-aware AEB / 接管 cls 冗余（追加在末尾——切勿插中间，会和历史 CSV 错列）
    'lead_cls',          # 主前车 class (0/1/2/3)
    'lead_cls_stale',    # /car{N}_class 话题陈旧（不降级控制，仅观察）
    'takeover_seed_cls', # 接管期使用的种子 cls；非接管期保持 0
    # 命令门控结构化诊断（CommandGate）——追加在末尾，切勿插中间
    'gate_reason',       # normal/aeb_override/backup_takeover/nan_block/sensor_stale/lockstep_fault/ctrl_error
    'gate_severity',     # 0 正常 / 1 告警 / 2 严重
    # AEB 结构化诊断 overlay（对标 Autoware AEB node；只读投影）
    'aeb_level',         # safe / warning / emergency
    'aeb_drac',          # 避撞所需减速度 (m/s²)
    # 统一安全状态聚合（对标 openpilot selfdriveState / onroadEvents）
    'safety_level',      # 0 nominal / 1 degraded / 2 emergency
    'safety_state',      # nominal / degraded / emergency
    # ★5 planning→control 接口（只读投影：横向期望偏移 + 纵向期望速度）
    'target_offset',     # 期望横向偏移 (m)，含超车借道
    'target_speed',      # 期望速度 (m/s)
    # AEB 预测路径碰撞检查（control/aeb_path_check.py）——追加在末尾，切勿插中间
    'aeb_path_risk',     # 本拍路径/RSS 命中（未消抖，0/1）
    'aeb_path_tcol',     # 最早预测碰撞时间 (s)
    'aeb_rss_dist',      # 触发目标 RSS 最小安全距离 (m)
)

_QUEUE_MAXSIZE = 4096          # 100Hz 下约 40s 缓冲
_FLUSH_INTERVAL_S = 1.0        # 周期性 flush，崩溃时最多丢 1s 数据


def telemetry_enabled() -> bool:
    """TELEMETRY=0 关闭，其余（含未设置）默认开启。"""
    return os.environ.get('TELEMETRY', '1') != '0'


class Telemetry:
    """后台 CSV 写盘器。

    线程模型：
      - 控制线程：record(row) 非阻塞 put_nowait，队列满则丢最旧帧并计数。
      - 写盘线程：阻塞取队列 → csv.writerow → 周期 flush。
    """

    def __init__(self, role: str):
        self._enabled = telemetry_enabled()
        self._dropped = 0
        self._q = queue.Queue(maxsize=_QUEUE_MAXSIZE)
        self._running = True
        self._fh = None
        self._writer = None
        self._path = None
        self._role = role
        if not self._enabled:
            logging.info('[TELEMETRY] disabled (TELEMETRY=0)')
            return
        ts = time.strftime('%Y%m%d_%H%M%S')
        # 默认 /tmp（Jetson Linux）；TELEMETRY_DIR 可覆盖（Windows 开发/测试用）
        self._out_dir = os.environ.get('TELEMETRY_DIR', '/tmp')
        self._path = os.path.join(
            self._out_dir, 'adas_%s_telemetry_%s.csv' % (role, ts))
        # 懒建文件：此处不 open；第一行数据到达时由写盘线程创建（见 _ensure_open）。
        self._thread = threading.Thread(
            target=self._writer_loop, name='telemetry', daemon=True,
        )
        self._thread.start()
        logging.info('[TELEMETRY] will log to %s (lazy-create on first row)',
                     self._path)

    def _ensure_open(self):
        """首行数据到达时创建 CSV 并写表头（仅写盘线程调用）。

        返回 True 表示文件可写；打开失败关闭遥测（与原 __init__ 失败语义一致）。
        """
        if self._writer is not None:
            return True
        if not self._enabled:
            return False
        try:
            # newline='' 是 csv 模块标准要求，避免空行
            self._fh = open(self._path, 'w', newline='')
            self._writer = csv.writer(self._fh)
            self._writer.writerow(FIELDS)
            self._fh.flush()
            logging.info('[TELEMETRY] logging to %s', self._path)
            return True
        except Exception as e:
            logging.error('[TELEMETRY] cannot open %s: %s (telemetry off)',
                          self._path, e)
            self._enabled = False
            return False

    def _cleanup_old_files(self):
        """按 mtime 只保留本 role 最近 TELEMETRY_KEEP 个历史 CSV（写盘线程启动时）。

        当前会话文件尚未创建（懒建），不在删除范围。失败只记 debug，不影响遥测。
        """
        try:
            keep = int(os.environ.get('TELEMETRY_KEEP', '20'))
        except ValueError:
            keep = 20
        if keep <= 0:
            return
        try:
            pattern = os.path.join(
                self._out_dir, 'adas_%s_telemetry_*.csv' % self._role)
            files = sorted(glob.glob(pattern), key=os.path.getmtime)
            stale = files[:-keep] if len(files) > keep else []
            for f in stale:
                try:
                    os.remove(f)
                except OSError:
                    pass
            if stale:
                logging.info('[TELEMETRY] cleaned %d old csv (keep=%d)',
                             len(stale), keep)
        except Exception as e:
            logging.debug('[TELEMETRY] cleanup error: %s', e)

    @property
    def path(self):
        return self._path

    def record(self, row: dict):
        """控制线程调用：非阻塞投递一行。

        row 是 {字段名: 值}；缺失字段留空。绝不抛异常、绝不阻塞控制环。
        """
        if not self._enabled:
            return
        try:
            self._q.put_nowait(row)
        except queue.Full:
            # 丢最旧保最新，与 serial_link 发送队列同策略
            try:
                self._q.get_nowait()
                self._dropped += 1
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(row)
            except queue.Full:
                self._dropped += 1

    def _writer_loop(self):
        # 保留清理放写盘线程：不阻塞控制线程构造 Telemetry（目录扫描走磁盘 IO）
        self._cleanup_old_files()
        last_flush = time.monotonic()
        while self._running:
            try:
                row = self._q.get(timeout=0.2)
            except queue.Empty:
                row = None
            if row is not None:
                if row is _STOP:
                    break
                if not self._ensure_open():
                    continue
                try:
                    self._writer.writerow(
                        [_fmt(row.get(f, '')) for f in FIELDS]
                    )
                except Exception as e:
                    logging.debug('[TELEMETRY] writerow error: %s', e)
            now = time.monotonic()
            if self._fh is not None and (now - last_flush) >= _FLUSH_INTERVAL_S:
                try:
                    self._fh.flush()
                except Exception:
                    pass
                last_flush = now
                if self._dropped:
                    logging.warning('[TELEMETRY] dropped %d rows (disk slow?)',
                                    self._dropped)
                    self._dropped = 0

    def close(self):
        """停止写盘线程并落盘剩余数据。"""
        if not self._enabled:
            return
        self._running = False
        try:
            self._q.put_nowait(_STOP)
        except queue.Full:
            pass
        try:
            self._thread.join(timeout=1.0)
        except Exception:
            pass
        # 排空残留队列，尽量不丢数据（懒建下若有残留行而文件未建，先补建）
        try:
            while True:
                row = self._q.get_nowait()
                if row is _STOP:
                    continue
                if not self._ensure_open():
                    break
                self._writer.writerow([_fmt(row.get(f, '')) for f in FIELDS])
        except queue.Empty:
            pass
        except Exception:
            pass
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass


_STOP = object()


def _fmt(v):
    """数值统一成紧凑字符串；bool→0/1；非有限值原样写 nan/inf（numpy 可读）。"""
    if isinstance(v, bool):
        return 1 if v else 0
    if isinstance(v, float):
        return repr(v)
    return v
