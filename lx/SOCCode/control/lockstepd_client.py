#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单板软件双核锁步守护进程客户端（ADAS.py 侧）。

通过 TCP 与 lockstepd 独立进程通信，接收锁步比较结果。
独立进程 = 独立 GIL + 崩溃隔离 + 核隔离（core 2）。

使用方式（ADAS.py __init__ 中）：
    if LOCKSTEP_ENABLED:
        try:
            from control.lockstepd_client import LockstepdClient, snapshot_managers
            self.lockstep = LockstepdClient(
                host=LOCKSTEPD_HOST, port=LOCKSTEPD_PORT,
                delta_eps=LOCKSTEP_DELTA_EPS, ...)
        except Exception:
            self.lockstep = None

snapshot_managers() 替代原 lockstep.snapshot_managers()。
"""

from __future__ import absolute_import, division, print_function

import copy
import dataclasses
import logging
import os
import pickle
import socket
import struct
import time

from control.context import ControlManagers

logger = logging.getLogger(__name__)


def snapshot_managers(mgr):
    """深拷贝一份 ControlManagers 供影子核独立演算，但 **排除 ml_bridge**
    （不可深拷贝；影子核改用主核传入的 ml_result）。"""
    fields = {}
    for f in dataclasses.fields(mgr):
        if f.name == 'ml_bridge':
            fields[f.name] = None
        else:
            fields[f.name] = copy.deepcopy(getattr(mgr, f.name))
    return ControlManagers(**fields)


def _recv_exact(conn, n):
    buf = b''
    while len(buf) < n:
        try:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        except socket.timeout:
            continue
        except OSError:
            return None
    return buf


class LockstepdClient(object):
    """锁步核 TCP 客户端。与 lockstepd 守护进程通信。

    接口与旧版 lockstep.LockstepChecker 兼容：
      - .submit(now, signals, memory, managers, takeover_rate, ml_result,
                 main_delta, main_lon, main_aeb)
      - .fault — bool，影子核是否报故障
      - .fault_reason — str，故障原因
      - .enabled — bool
      - .compared — 已比较拍数
      - .mismatch_total — 总失配拍数
    """

    def __init__(self, host='127.0.0.1', port=19998,
                 delta_eps=1e-9, lon_eps=1e-9, debounce_n=2,
                 inject=False, inject_delta=0.05, connect_timeout=5.0):
        self.enabled = False
        self.fault = False
        self.fault_reason = ''
        self.compared = 0
        self.mismatch_total = 0

        deadline = time.monotonic() + connect_timeout
        last_err = None
        sock = None
        while time.monotonic() < deadline:
            try:
                sock = socket.create_connection((host, port), timeout=1.0)
                break
            except (ConnectionRefusedError, OSError) as e:
                last_err = e
                time.sleep(0.1)

        if sock is None:
            raise ConnectionError(
                'cannot connect to lockstepd at %s:%d (timeout=%.1fs): %s'
                % (host, port, connect_timeout, last_err))

        self._sock = sock
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(0.01)  # 10ms 超时用于非阻塞轮询
        self._host = host
        self._port = port
        # 健康监控：最后一次收到响应的时间。超过 HEALTHY_TIMEOUT_S 无响应
        # 视为 daemon 挂起，自动降级。
        self._last_response_t = time.monotonic()
        self._HEALTHY_TIMEOUT_S = 0.5

        # 发送 init 配置
        self._send_pickle({
            'type': 'init',
            'delta_eps': float(delta_eps),
            'lon_eps': float(lon_eps),
            'debounce_n': int(debounce_n),
            'inject': bool(inject),
            'inject_delta': float(inject_delta),
        })
        self._recv_pickle()  # 等待 init_ack
        self._last_response_t = time.monotonic()
        self.enabled = True
        logger.info(
            '[LOCKSTEP] 锁步守护客户端已连接 %s:%d '
            'eps=%.0e/%.0e debounce=%d inject=%s',
            host, port, delta_eps, lon_eps, debounce_n, inject)

    def __del__(self):
        try:
            self._sock.close()
        except Exception:
            pass

    # ── 内部：打包通信 ──

    def _send_pickle(self, obj):
        data = pickle.dumps(obj, protocol=2)
        self._sock.sendall(struct.pack('!I', len(data)) + data)

    def _recv_pickle(self):
        """阻塞接收一个响应。"""
        raw = _recv_exact(self._sock, 4)
        if raw is None:
            raise ConnectionError('lockstepd connection closed')
        msglen = struct.unpack('!I', raw)[0]
        body = _recv_exact(self._sock, msglen)
        if body is None:
            raise ConnectionError('lockstepd connection closed')
        return pickle.loads(body)

    def _poll_response(self):
        """非阻塞检查 lockstepd 响应（上一拍的 submit_ack）。

        健康监控：超过 HEALTHY_TIMEOUT_S 无响应则自动降级（enabled=False）。
        """
        now = time.monotonic()
        try:
            self._sock.settimeout(0.0)  # 非阻塞
            raw = self._sock.recv(4)
            if not raw:
                return
            self._sock.settimeout(1.0)
            msglen = struct.unpack('!I', raw)[0]
            body = _recv_exact(self._sock, msglen)
            if body is None:
                return
            resp = pickle.loads(body)
            if resp.get('type') == 'submit_ack':
                self._last_response_t = now
                if resp.get('fault'):
                    self.fault = True
                    self.fault_reason = resp.get('fault_reason', '')
                self.compared = resp.get('compared', self.compared)
                self.mismatch_total = resp.get('mismatch_total', self.mismatch_total)
        except (socket.error, BlockingIOError, OSError):
            pass
        finally:
            try:
                self._sock.settimeout(0.01)
            except OSError:
                pass
            # 健康检查：lockstepd 无响应超过阈值 → 降级
            if (self.enabled and now - self._last_response_t > self._HEALTHY_TIMEOUT_S):
                self.enabled = False
                logger.warning(
                    '[LOCKSTEP] 无响应 %.1fs（>%.1fs），锁步降级为 disabled',
                    now - self._last_response_t, self._HEALTHY_TIMEOUT_S)

    # ── 公共接口 ──

    def submit(self, now, signals, memory, managers, takeover_rate, ml_result,
               main_delta, main_lon, main_aeb):
        """投递本拍状态给 lockstepd。非阻塞：先 poll 上次结果，再发新请求。

        与旧版 LockstepChecker.submit 接口兼容。
        """
        if not self.enabled:
            return

        # 先检查上一拍的响应
        self._poll_response()

        # 剥离 VehicleSignals 中不可 pickle 的锁（实际由 __getstate__ 处理，
        # 但做二次保险）
        if hasattr(signals, '_lock'):
            signals = copy.copy(signals)
            signals._lock = None  # type: ignore

        # 发送新请求
        try:
            self._send_pickle({
                'type': 'submit',
                'now': now,
                'signals': signals,
                'memory': memory,
                'managers': managers,
                'takeover_rate': takeover_rate,
                'ml_result': ml_result,
                'main_delta': float(main_delta),
                'main_lon': float(main_lon),
                'main_aeb': bool(main_aeb),
            })
        except (ConnectionError, OSError) as e:
            self.enabled = False
            logger.warning('[LOCKSTEP] submit failed, disabled: %s', e)

    def clear_fault(self):
        self.fault = False
        self.fault_reason = ''

    def set_inject(self, on):
        try:
            self._send_pickle({'type': 'set_inject', 'value': bool(on)})
        except OSError:
            pass
