#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ML 推理桥接模块。

将 SOC 控制环的 VehicleSignals 适配为 ML 推理 API 所需的特征格式，
并处理 100Hz → 10Hz 降采样。ML 推理结果作为规则控制的辅助信号，
不替代任何安全关键路径。

推理引擎从线程内执行（旧版）改为连接 ml_inferd 独立守护进程（新版）：
- 独立 GIL：推理不阻塞 100Hz 控制环
- 崩溃隔离：ONNX Runtime segfault 不拖垮 ADAS
- 核隔离：ml_inferd 绑 core 3，避免缓存污染 core 0 控制路径

降级：ml_inferd 不可用时自动降级为 no-op，控制行为不变。
"""

from __future__ import absolute_import, division, print_function

import json
import logging
import os
import socket
import struct
import threading
import time

from common import is_finite
from config import LOOP_HZ, ML_INFERD_HOST, ML_INFERD_PORT

# 降采样：ML 模型以 10Hz 训练，控制环以 100Hz 运行
_ML_SAMPLE_INTERVAL = max(1, LOOP_HZ // 10)

# 连接保活：reader 线程帧间空闲超过此值即发 ping，避免 ml_inferd 端 1s recv
# 超时关连接（与 ml_inferd 帧间空闲容忍互为冗余兜底），并据 pong 监测健康。
_ML_KEEPALIVE_S = 0.5
# 断线后 reader 线程后台重连退避（秒）。重连只发生在 reader 线程，不阻塞控制环。
_ML_RECONNECT_S = 1.0
# reader 帧间空闲哨兵：让 reader 循环有机会发保活后再继续等待。
_ML_IDLE = object()

# 推理错误日志限频
_last_infer_err_t = [0.0]


def _log_infer_error(e):
    t = time.time()
    if t - _last_infer_err_t[0] > 5.0:
        logging.warning('[ML] inference error: %s', e)
        _last_infer_err_t[0] = t


class MlPrediction(object):
    """ML 推理结果（单周期）。"""
    __slots__ = ('acc_pred', 'aeb_class', 'aeb_probs', 'should_brake', 'brake_intensity')

    def __init__(self):
        self.acc_pred = 0.0          # ACC 预测加速度 (m/s²)
        self.aeb_class = 0           # AEB 分类: 0=safe, 1=warning, 2=emergency
        self.aeb_probs = None        # AEB 概率分布 (3,)
        self.should_brake = False    # ML 建议制动
        self.brake_intensity = 0.0   # 制动强度 0~1


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


class MlBridge(object):
    """ML 推理桥接器，通过 TCP 连接 ml_inferd 守护进程。

    用法（控制环中按降采样频率调用）：
        ml_result = ml_bridge.update(now, signals, lead_ctx)
        if ml_result and ml_result.aeb_class >= 1:
            # ML 认为有碰撞风险，可作为辅助信号
            ...

    守护进程不可用时静默降级为 no-op。
    """

    def __init__(self):
        self._cycle_count = 0
        self._last_result = MlPrediction()
        self._enabled = False
        self._mode = 'disabled'
        self._sock = None
        self._lock = threading.Lock()        # 保护 _last_result
        self._send_lock = threading.Lock()   # 串行化所有 sendall（控制环特征 + reader 保活）
        self._last_send_t = 0.0
        self._reader_stop = threading.Event()
        self._reader = None

        # 首连 ml_inferd；无论成败都启动 reader 线程，由其负责后台重连
        # （解决 ADAS 早于 ml_inferd bind、运行中瞬断/空闲被关 等恢复场景）。
        try:
            self._sock = self._open_socket()
            self._enabled = True
            self._last_send_t = time.time()
            self._mode = 'remote'
            logging.info(
                '[ML] bridge enabled, remote=%s:%d interval=%d cycles',
                ML_INFERD_HOST, ML_INFERD_PORT, _ML_SAMPLE_INTERVAL)
        except Exception as e:
            self._sock = None
            self._enabled = False
            self._mode = 'disabled'
            logging.info('[ML] ml_inferd 暂不可用，reader 后台重连中（%s）', e)
        self._reader = threading.Thread(
            target=self._reader_loop, name='ml-bridge-reader', daemon=True)
        self._reader.start()

    @staticmethod
    def _open_socket():
        s = socket.create_connection(
            (ML_INFERD_HOST, ML_INFERD_PORT), timeout=2.0)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(1.0)  # reader recv 超时；帧间空闲由哨兵转保活
        return s

    # ── 属性 ──

    @property
    def enabled(self):
        return self._enabled

    @property
    def backend(self):
        return 'remote' if self._enabled else None

    # ── reader 线程：读推理结果 + 帧间保活 + 断线自动重连 ──

    def _reader_loop(self):
        """后台读线程：所有阻塞操作（recv/重连/保活 send）都在此线程，
        绝不进入 100Hz 控制环。断线后自动退避重连，不再永久降级。"""
        while not self._reader_stop.is_set():
            sock = self._sock
            if sock is None or not self._enabled:
                if self._reader_stop.wait(_ML_RECONNECT_S):
                    break
                if self._reconnect():
                    logging.info('[ML] ml_inferd 已重连，ML 恢复')
                continue
            try:
                msg = self._read_message(sock)
                if msg is None:
                    self._on_drop('connection lost')
                    continue
                if msg is _ML_IDLE:
                    self._maybe_keepalive()
                    continue
                if msg.get('pong'):
                    continue  # 保活响应，不作为预测结果
                pred = MlPrediction()
                pred.acc_pred = float(msg.get('acc_pred', 0.0))
                pred.aeb_class = int(msg.get('aeb_class', 0))
                probs = msg.get('aeb_probs')
                if probs is not None and len(probs) >= 3:
                    pred.aeb_probs = [float(p) for p in probs[:3]]
                    p1, p2 = float(probs[1]), float(probs[2])
                    pred.should_brake = (p1 + p2) > 0.5
                    pred.brake_intensity = p1 * 0.5 + p2 * 1.0
                with self._lock:
                    self._last_result = pred
            except (ConnectionError, OSError):
                self._on_drop('connection lost')
            except Exception as e:
                _log_infer_error(e)

        # 线程退出（仅 _reader_stop 置位时）：清理 socket
        with self._send_lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
            self._enabled = False
            self._mode = 'stopped'

    def _read_message(self, sock):
        """读一条长度前缀消息。返回 dict / None（断开）/ _ML_IDLE（帧间空闲）。"""
        raw_len = self._recv_or_idle(sock, 4)
        if raw_len is None:
            return None
        if raw_len is _ML_IDLE:
            return _ML_IDLE
        msglen = struct.unpack('!I', raw_len)[0]
        if msglen > 65536:  # 合理性检查
            return None
        data = _recv_exact(sock, msglen)
        if data is None:
            return None
        return json.loads(data.decode('utf-8'))

    @staticmethod
    def _recv_or_idle(sock, n):
        """接收 n 字节；帧间（尚无字节）空闲超时返回 _ML_IDLE 以便发保活。"""
        buf = b''
        while len(buf) < n:
            try:
                chunk = sock.recv(n - len(buf))
                if not chunk:
                    return None
                buf += chunk
            except socket.timeout:
                if not buf:
                    return _ML_IDLE
                continue  # 半帧中途：继续等（4 字节前缀随包原子发出，半前缀极罕见）
            except OSError:
                return None
        return buf

    def _maybe_keepalive(self):
        """帧间空闲超过 _ML_KEEPALIVE_S 即发 ping，保活兼健康探测。"""
        if time.time() - self._last_send_t < _ML_KEEPALIVE_S:
            return
        try:
            self._send_json({'ping': True})
        except OSError:
            pass  # 失败将由下一轮 recv 检出断开并触发重连

    def _on_drop(self, why):
        if self._enabled:
            logging.warning('[ML] ml_inferd %s，后台重连中…', why)
        self._enabled = False
        self._mode = 'disconnected'

    def _reconnect(self):
        """reader 线程内重连。成功返回 True。阻塞仅发生在 reader 线程。"""
        try:
            new_sock = self._open_socket()
        except Exception:
            return False
        with self._send_lock:
            old = self._sock
            self._sock = new_sock
            self._last_send_t = time.time()
            self._enabled = True
            self._mode = 'remote'
        if old is not None:
            try:
                old.close()
            except OSError:
                pass
        return True

    # ── 公共接口 ──

    def _send_json(self, msg_dict):
        """向 ml_inferd 发送 JSON 请求。写超时 10ms，超时即标记不可用。

        被控制环（特征/reset）与 reader 线程（保活 ping）共同调用，故用
        _send_lock 串行化，避免两个 sendall 交错破坏长度前缀分帧。锁内只做
        ≤10ms 的发送，绝不持锁阻塞重连——控制环最多等一次 10ms 发送。
        """
        data = json.dumps(msg_dict).encode('utf-8')
        with self._send_lock:
            sock = self._sock
            if sock is None:
                raise OSError('ml_inferd socket not connected')
            old_to = sock.gettimeout()
            sock.settimeout(0.01)  # 10ms 写超时
            try:
                sock.sendall(struct.pack('!I', len(data)) + data)
                self._last_send_t = time.time()
            except (socket.timeout, OSError) as e:
                self._enabled = False
                self._mode = 'send_failed'
                logging.warning('[ML] send to ml_inferd failed: %s', e)
                raise
            finally:
                try:
                    sock.settimeout(old_to)
                except OSError:
                    pass

    def reset(self):
        """重置 ML 缓冲区（场景切换时调用）。"""
        self._cycle_count = 0
        with self._lock:
            self._last_result = MlPrediction()
        if self._sock is not None:
            try:
                self._send_json({'reset': True})
            except OSError:
                pass

    def update(self, now, signals, lead_ctx):
        # type: (float, object, object) -> MlPrediction | None
        """每控制周期调用一次，按降采样频率执行推理。

        返回 MlPrediction（始终返回最新可用结果，非采样/预热周期返回上一次结果）。
        ML 不可用时返回 None。
        """
        if not self._enabled:
            return None
        if self._sock is None:
            return None

        self._cycle_count += 1
        if self._cycle_count % _ML_SAMPLE_INTERVAL != 0:
            with self._lock:
                return self._last_result

        # 构建特征（轻量 numpy，留在控制环线程）
        acc_features = self._build_acc_features(signals, lead_ctx)
        aeb_features = self._build_aeb_features(signals, lead_ctx)

        if acc_features is None and aeb_features is None:
            with self._lock:
                return self._last_result

        # 发送到 ml_inferd（非阻塞，~0.05ms TCP 发送）
        msg = {}
        if acc_features is not None:
            msg['acc_features'] = list(acc_features)
        if aeb_features is not None:
            msg['aeb_features'] = list(aeb_features)
        try:
            self._send_json(msg)
        except OSError:
            self._enabled = False
            return None

        # 返回 reader 线程已更新的最新结果
        with self._lock:
            return self._last_result

    # ── 特征构建（与原 ml_bridge 完全一致）──

    @staticmethod
    def _build_acc_features(signals, lead_ctx):
        """从 VehicleSignals 构建 ACC 特征 (7维)。"""
        if not signals.lead_received or not signals.ego_received:
            return None
        v_ego = signals.ego_v
        if not is_finite(v_ego) or v_ego < 0:
            return None
        gap = float(lead_ctx.x_rel) if is_finite(lead_ctx.x_rel) else 0.0
        if gap <= 0:
            return None
        v_lead = float(lead_ctx.predicted_lead_v_proj) if is_finite(lead_ctx.predicted_lead_v_proj) else 0.0
        rel_speed = v_lead - v_ego
        acc_ego = 0.0
        acc_lead = float(getattr(lead_ctx, 'lead_accel', 0.0) or 0.0)
        if not is_finite(acc_lead):
            acc_lead = 0.0
        thw = gap / max(v_ego, 0.1)
        return (gap, v_ego, v_lead, rel_speed, acc_ego, acc_lead, thw)

    @staticmethod
    def _build_aeb_features(signals, lead_ctx):
        """从 VehicleSignals 构建 AEB 特征 (10维)。"""
        if not signals.lead_received or not signals.ego_received:
            return None
        v_ego = signals.ego_v
        if not is_finite(v_ego) or v_ego < 0:
            return None
        gap = float(lead_ctx.x_rel) if is_finite(lead_ctx.x_rel) else 0.0
        if gap <= 0:
            return None
        v_lead = float(lead_ctx.predicted_lead_v_proj) if is_finite(lead_ctx.predicted_lead_v_proj) else 0.0
        rel_speed = v_lead - v_ego
        closing_speed = max(v_ego - v_lead, 0.0)
        if closing_speed > 0.1:
            ttc = gap / closing_speed
            ttc = min(ttc, 100.0)
            inverse_ttc = 1.0 / ttc
            drac = (closing_speed ** 2) / (2.0 * gap)
            drac = min(drac, 20.0)
        else:
            ttc = 100.0
            inverse_ttc = 0.0
            drac = 0.0
        thw = gap / max(v_ego, 0.1)
        acc_lead = float(getattr(lead_ctx, 'lead_accel', 0.0) or 0.0)
        if not is_finite(acc_lead):
            acc_lead = 0.0
        return (gap, v_ego, v_lead, rel_speed, ttc, inverse_ttc, drac, thw, closing_speed, acc_lead)
