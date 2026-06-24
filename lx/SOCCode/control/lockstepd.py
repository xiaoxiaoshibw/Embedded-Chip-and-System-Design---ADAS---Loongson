#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单板软件双核锁步守护进程。

独立进程，绑定到指定 CPU 核（默认 core 2），通过 TCP localhost 接收
主核控制管线前状态快照，重算并逐拍比较。独立 GIL → 真实并行。

协议：长度前缀 pickle（先 4 字节大端 uint32，后 pickle 字节流）。
端口：LOCKSTEPD_PORT 环境变量，默认 19998。

启动（由 ADAS.py main() 自动执行）：
  taskset -c 2 python3 control/lockstepd.py

降级：lockstepd 不可用时 ADAS 照常运行，锁步安全 best-effort。
"""

from __future__ import absolute_import, division, print_function

import copy
import logging
import os
import pickle
import signal
import socket
import struct
import sys
import time

try:
    import ctypes
    _LIBC = ctypes.CDLL(None)
    _PR_SET_PDEATHSIG = 1
    _LIBC.prctl(_PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0)
except Exception:
    pass

_SELF_DIR = os.path.dirname(os.path.abspath(__file__))
_SOCCode_DIR = os.path.dirname(_SELF_DIR)
if _SOCCode_DIR not in sys.path:
    sys.path.insert(0, _SOCCode_DIR)

# 预先导入所有控制模块，让 pickle 在反序列化时能找到这些类
from pipeline import run_pure_pipeline
from control.context import ControlManagers, ControlMemory, VehicleSignals
from control.lead_tracking import LeadTracker
from control.aeb_alert import AebAlertManager
from control.curve_hold import CurveHoldManager
from control.overtake import OvertakeManager
from control.lead_estimator import LeadCaKalman
from lateral import LaneWidthEstimator
from longitudinal import LonSmoothing, LongitudinalController

PORT = int(os.environ.get('LOCKSTEPD_PORT', '19998'))
_BACKLOG = 1


# 半帧兜底：进入半帧后连续 recv 超时达到此次数（每次 1s）才判坏连接放弃；
# 帧间空闲（buf 为空）不计入，可无限等待，避免控制环静默待机时被误关。
_HALF_FRAME_MAX_TIMEOUTS = 5


def _recv_exact(conn, n):
    """从 socket 接收精确 n 字节。返回 bytes 或 None（连接断开）。

    帧间空闲（buf 为空）容忍 conn.settimeout(1.0) 触发的 socket.timeout，
    持续等待下一帧——控制环静默待机（无感知）时长时间不发请求，旧实现因
    1s recv 超时直接 close，使 lockstepd_client 被迫反复重连（CLOSE-WAIT）。
    进入半帧后则用 _HALF_FRAME_MAX_TIMEOUTS 兜底，防对端发半包挂死永久阻塞。
    """
    buf = b''
    half_frame_timeouts = 0
    while len(buf) < n:
        try:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
            half_frame_timeouts = 0  # 收到数据，重置半帧计数
        except socket.timeout:
            if not buf:
                continue  # 帧间空闲，继续等待下一帧
            half_frame_timeouts += 1
            if half_frame_timeouts >= _HALF_FRAME_MAX_TIMEOUTS:
                return None
    return buf


def _unpickle_skip_lock(data):
    """反序列化主核发来的状态：自动处理 VehicleSignals 中的 threading.Lock。"""
    d = pickle.loads(data)
    # 如果 signals 有 _lock 属性，说明主核忘了剥离，这里补剥
    signals = d.get('signals')
    if signals is not None:
        # VehicleSignals 的 _lock field 存了 Lock 对象，去掉它
        if hasattr(signals, '_lock'):
            del signals._lock
    return d


class LockstepEngine(object):
    """影子核比较器。维护状态独立于主核。"""

    def __init__(self):
        self.delta_eps = 1e-9
        self.lon_eps = 1e-9
        self.debounce_n = 2
        self.fault = False
        self.fault_reason = ''
        self.compared = 0
        self.mismatch_total = 0
        self._mismatch_run = 0
        self._inject = False
        self.inject_delta = 0.05

    def configure(self, delta_eps, lon_eps, debounce_n, inject, inject_delta):
        self.delta_eps = float(delta_eps)
        self.lon_eps = float(lon_eps)
        self.debounce_n = max(1, int(debounce_n))
        self._inject = bool(inject)
        self.inject_delta = float(inject_delta)

    def set_inject(self, on):
        self._inject = bool(on)

    def clear_fault(self):
        self.fault = False
        self.fault_reason = ''
        self._mismatch_run = 0

    def submit(self, now, signals, memory, managers, takeover_rate, ml_result,
               main_delta, main_lon, main_aeb):
        """执行影子计算并比较。"""
        try:
            # VehicleSignals 的 _lock 在主核侧已被剥离
            shadow = run_pure_pipeline(
                now, signals, memory, managers, takeover_rate,
                ml_result=ml_result,
            )
        except Exception as exc:
            logging.warning('[LOCKSTEPD] 影子计算异常（忽略本拍）：%r', exc)
            self._mismatch_run = 0
            return

        s_delta = shadow.lateral_ctx.delta
        s_lon = shadow.lon_cmd
        s_aeb = bool(shadow.lon_ctx.aeb_active)

        if self._inject:
            s_delta = s_delta + self.inject_delta

        self.compared += 1
        d_delta = abs(s_delta - main_delta)
        d_lon = abs(s_lon - main_lon)
        mism = (d_delta > self.delta_eps or d_lon > self.lon_eps
                or s_aeb != main_aeb)
        if mism:
            self.mismatch_total += 1
            self._mismatch_run += 1
            if self._mismatch_run >= self.debounce_n and not self.fault:
                self.fault = True
                self.fault_reason = (
                    'Δdelta=%.5f Δlon=%.5f AEB(主=%d/影=%d) 连续=%d拍'
                    % (d_delta, d_lon, int(main_aeb), int(s_aeb),
                       self._mismatch_run))
                logging.critical('[LOCKSTEPD] 比较器失配 -> 报故障: %s',
                                 self.fault_reason)
        else:
            self._mismatch_run = 0


def main():
    logging.basicConfig(
        format='[LOCKSTEPD] %(levelname)s %(message)s',
        level=logging.INFO,
    )
    engine = LockstepEngine()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', PORT))
    server.listen(_BACKLOG)
    logging.info('ready on port %d', PORT)

    while True:
        conn, addr = server.accept()
        conn.settimeout(1.0)  # recv 超时 1s，防止半帧永久阻塞
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        logging.debug('connection from %s:%d', addr[0], addr[1])
        try:
            while True:
                raw_len = _recv_exact(conn, 4)
                if raw_len is None:
                    break
                msglen = struct.unpack('!I', raw_len)[0]
                data = _recv_exact(conn, msglen)
                if data is None:
                    break

                msg = pickle.loads(data)
                msg_type = msg.get('type', '')

                if msg_type == 'init':
                    engine.configure(
                        msg.get('delta_eps', 1e-9),
                        msg.get('lon_eps', 1e-9),
                        msg.get('debounce_n', 2),
                        msg.get('inject', False),
                        msg.get('inject_delta', 0.05),
                    )
                    resp = pickle.dumps({'type': 'init_ack'}, protocol=2)
                    conn.sendall(struct.pack('!I', len(resp)) + resp)

                elif msg_type == 'submit':
                    now = msg.get('now', time.monotonic())
                    signals = msg.get('signals')
                    memory = msg.get('memory')
                    managers = msg.get('managers')
                    takeover_rate = msg.get('takeover_rate')
                    ml_result = msg.get('ml_result')
                    main_delta = msg.get('main_delta', 0.0)
                    main_lon = msg.get('main_lon', 0.0)
                    main_aeb = msg.get('main_aeb', False)

                    engine.submit(now, signals, memory, managers,
                                  takeover_rate, ml_result,
                                  main_delta, main_lon, main_aeb)

                    resp = pickle.dumps({
                        'type': 'submit_ack',
                        'fault': engine.fault,
                        'fault_reason': engine.fault_reason,
                        'compared': engine.compared,
                        'mismatch_total': engine.mismatch_total,
                    }, protocol=2)
                    conn.sendall(struct.pack('!I', len(resp)) + resp)

                elif msg_type == 'status':
                    resp = pickle.dumps({
                        'type': 'status',
                        'fault': engine.fault,
                        'fault_reason': engine.fault_reason,
                        'compared': engine.compared,
                        'mismatch_total': engine.mismatch_total,
                    }, protocol=2)
                    conn.sendall(struct.pack('!I', len(resp)) + resp)

                elif msg_type == 'set_inject':
                    engine.set_inject(msg.get('value', False))
                    resp = pickle.dumps({'type': 'ok'}, protocol=2)
                    conn.sendall(struct.pack('!I', len(resp)) + resp)

                elif msg_type == 'clear_fault':
                    engine.clear_fault()
                    resp = pickle.dumps({'type': 'ok'}, protocol=2)
                    conn.sendall(struct.pack('!I', len(resp)) + resp)

                else:
                    resp = pickle.dumps({
                        'type': 'error',
                        'reason': 'unknown type: ' + msg_type,
                    }, protocol=2)
                    conn.sendall(struct.pack('!I', len(resp)) + resp)

        except (ConnectionResetError, BrokenPipeError, EOFError, OSError):
            pass
        except Exception as exc:
            logging.error('handler error: %r', exc)
        finally:
            try:
                conn.close()
            except OSError:
                pass


if __name__ == '__main__':
    main()
