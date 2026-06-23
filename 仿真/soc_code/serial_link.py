#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ESP32 串口链路封装。

负责与 ESP32 的串口通信：
  - 发送：控制循环把 payload put 进发送队列，后台 tx 线程从队列取出写入串口。
  - 接收：后台 rx 线程持续读取串口字节，解析 P/D/B 标签后更新共享回读值。
  - 断线重连：写入/读取失败时标记断开；reopen 也在后台线程执行，
    不会阻塞控制线程。

设计原则：所有 pyserial 的阻塞调用（Serial(), write(), read()）都封装在
后台线程内，主控制循环只做非阻塞的 put_nowait / 原子读取，从而保证
10ms 控制周期不会被串口 IO 拖慢。
"""

import logging
import math
import os
import queue
import threading
import time
from typing import Optional

import serial

from config import BAUDRATE, MAX_DELTA, SERIAL_ESP32
from common import is_finite, parse_tagged_lines

# 回传新鲜度超时：超过此时间未收到任何有效 P/D/B 帧，视为回传值陈旧。
# 用于上层判断 ESP32 是否仍在正常通信（仅监控，不影响控制）。
_READBACK_STALE_TIMEOUT_S = 1.0


# 发送队列容量：控制帧只关心最新值，4 已经远超 ESP32 处理节奏。
_TX_QUEUE_MAXSIZE = 4

# 后台线程循环间隔：串口空闲时的休眠节拍 (s)，10ms 与控制周期同量级。
_BG_LOOP_IDLE_SLEEP_S = 0.005

# reopen 指数退避：首次失败仅退避 _REOPEN_BACKOFF_FIRST_S，每次失败翻倍，
# 上限 _REOPEN_BACKOFF_MAX_S。避免开机阶段第一次 open race 时被罚 1 整秒，
# 错过 ESP32 ready 窗口；同时仍能在长时间断开时进入低频探测。
_REOPEN_BACKOFF_FIRST_S = 0.2
_REOPEN_BACKOFF_MAX_S = 1.0
# 保留旧常量名以兼容外部引用（实际未被使用，但避免符号检索时遗漏）。
_REOPEN_BACKOFF_S = _REOPEN_BACKOFF_MAX_S


class Esp32Serial:
    """ESP32 串口通信管理类。

    线程模型：
      - 主线程：调用 send() / drain_rx() / esp_psi 等属性访问。
      - tx 线程：从 _tx_q 取 payload 写入串口。
      - rx 线程：持续 read 串口，解析后更新 _esp_* 回读值。
      - 两个后台线程共享 _open_lock，确保任意时刻只有一个线程在尝试
        打开串口或读写 self.ser。
    """

    def __init__(self):
        # 串口对象与状态由后台线程操作；主线程不直接读写 self.ser。
        self.ser: Optional[serial.Serial] = None
        self._open_lock = threading.Lock()
        self._next_reopen_t = 0.0
        # 连续 open 失败计数（成功打开后清零）。用于指数退避计算下次尝试时间。
        self._reopen_fail_count: int = 0

        # 接收缓冲区只在 rx 线程内访问，无需加锁。
        self._rx_buf = bytearray()

        # 回读值用 _state_lock 保护，主线程通过 property 读取。
        self._state_lock = threading.Lock()
        self._esp_psi = 0.0
        self._esp_delta = 0.0
        self._esp_brake = 0.0
        # 持续解析失败计数（用于触发链路异常断开）。
        self._consec_bad_frames = 0
        self._last_rx_t = 0.0

        # 发送队列：丢旧保新策略，详见 send()。
        self._tx_q: "queue.Queue[bytes]" = queue.Queue(maxsize=_TX_QUEUE_MAXSIZE)
        self._tx_dropped = 0  # 累计丢弃帧计数，外部可读

        # 线程控制
        self._running = True
        self._tx_thread = threading.Thread(
            target=self._tx_loop, name='esp32-tx', daemon=True,
        )
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name='esp32-rx', daemon=True,
        )
        self._tx_thread.start()
        self._rx_thread.start()

    # ------------------------------------------------------------------
    # 属性：主线程读取的回读值
    # ------------------------------------------------------------------
    @property
    def esp_psi(self) -> float:
        with self._state_lock:
            return self._esp_psi

    @property
    def esp_delta(self) -> float:
        with self._state_lock:
            return self._esp_delta

    @property
    def esp_brake(self) -> float:
        with self._state_lock:
            return self._esp_brake

    @property
    def tx_dropped(self) -> int:
        """累计丢弃的发送帧数量（用于上层监控背压）。"""
        return self._tx_dropped

    @property
    def readback_stale(self) -> bool:
        """回传值是否陈旧（超过 _READBACK_STALE_TIMEOUT_S 未收到有效帧）。

        用于上层判断 ESP32 回传链路是否正常。如果 ESP32 停止发送 P:
        但继续发送 D:/B:，此属性仍为 False（只要有任一有效标签即可）。
        只有完全无有效帧超时才返回 True。
        """
        with self._state_lock:
            if self._last_rx_t <= 0.0:
                return True
            return (time.monotonic() - self._last_rx_t) > _READBACK_STALE_TIMEOUT_S

    # ------------------------------------------------------------------
    # 设备打开 / 关闭 / 标记断开
    # ------------------------------------------------------------------
    def _device_present(self) -> bool:
        """非阻塞地预检设备节点是否存在，避免 serial.Serial() 走完整枚举。"""
        try:
            return os.path.exists(SERIAL_ESP32)
        except Exception:
            return False

    def _open(self) -> Optional[serial.Serial]:
        """尝试打开 ESP32 串口设备（仅由后台线程调用）。

        write_timeout 缩短到 5ms，与一个控制周期同量级，避免单次写阻塞
        吃掉多个周期。
        """
        if not self._device_present():
            return None
        try:
            s = serial.Serial(
                SERIAL_ESP32,
                BAUDRATE,
                timeout=0,           # 非阻塞读
                write_timeout=0.005,  # 5ms 写超时
            )
            logging.info('[ESP32] %s opened', SERIAL_ESP32)
            return s
        except Exception as e:
            logging.warning('[ESP32] open failed: %s', e)
            return None

    def _mark_disconnected(self, reason: str, exc: Optional[Exception] = None):
        """标记串口断开，关闭句柄并记录日志（仅由后台线程调用）。"""
        with self._open_lock:
            if self.ser is None:
                return
            if exc is None:
                logging.warning('[ESP32] disconnected: %s', reason)
            else:
                logging.warning('[ESP32] disconnected: %s (%s)', reason, exc)
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            # 断开瞬间允许立即触发一次重连尝试（退避在 _try_reopen 内部累积）。
            self._next_reopen_t = time.monotonic()
            self._reopen_fail_count = 0
            self._consec_bad_frames = 0
            self._rx_buf.clear()

    def _try_reopen(self):
        """在断开后按退避间隔尝试重新打开串口（仅由后台线程调用）。

        _open() 必须放到锁外执行（pyserial 内部可能阻塞数百 ms），
        但跨锁释放窗口期间 close() 可能将 _running 置 False。这种情况下
        刚 open 出来的句柄必须立即关闭，避免线程退出后 fd 泄漏。
        """
        with self._open_lock:
            if self.ser is not None:
                return
            if not self._running:
                return
            now = time.monotonic()
            if now < self._next_reopen_t:
                return
            # 暂时把下次尝试设为远未来，防止本次 open 阻塞期间 tx/rx 线程同时进入
            self._next_reopen_t = now + _REOPEN_BACKOFF_MAX_S
        # _open 可能阻塞，放到锁外执行
        new_ser = self._open()
        if new_ser is None:
            # 失败：按指数退避计算下次尝试时间。
            # 0.2 → 0.4 → 0.8 → 1.0(上限)。首次失败仅 200ms，避免开机 race。
            with self._open_lock:
                self._reopen_fail_count = min(self._reopen_fail_count + 1, 16)
                backoff = min(
                    _REOPEN_BACKOFF_MAX_S,
                    _REOPEN_BACKOFF_FIRST_S * (2 ** (self._reopen_fail_count - 1)),
                )
                self._next_reopen_t = time.monotonic() + backoff
            return
        with self._open_lock:
            # 跨锁释放期间可能已被 close() 标记停止；此时不能再发布句柄
            if not self._running or self.ser is not None:
                try:
                    new_ser.close()
                except Exception:
                    pass
                return
            self.ser = new_ser
            self._reopen_fail_count = 0

    # ------------------------------------------------------------------
    # 主线程入口：send / drain_rx
    # ------------------------------------------------------------------
    def send(self, payload_bytes: bytes):
        """非阻塞投递控制帧到发送队列。

        丢旧保新策略：队列满时先弹出最旧的一帧再放入新帧，
        保证 ESP32 总是收到尽可能新的控制指令。
        """
        try:
            self._tx_q.put_nowait(payload_bytes)
        except queue.Full:
            try:
                self._tx_q.get_nowait()
                self._tx_dropped += 1
            except queue.Empty:
                pass
            try:
                self._tx_q.put_nowait(payload_bytes)
            except queue.Full:
                self._tx_dropped += 1

    def drain_rx(self):
        """兼容旧接口：rx 已由后台线程完成，此处为空操作。

        保留方法签名是为了不修改 ADAS 主循环。
        """
        return

    # ------------------------------------------------------------------
    # 后台线程：tx
    # ------------------------------------------------------------------
    def _tx_loop(self):
        """发送线程：阻塞等待队列中的 payload 并写入串口。"""
        while self._running:
            try:
                # 短超时阻塞，便于检查 _running 与触发 reopen。
                payload = self._tx_q.get(timeout=0.05)
            except queue.Empty:
                if self.ser is None:
                    self._try_reopen()
                continue

            # 空 payload 是 close() 发送的唤醒信号，不写入串口
            if not payload:
                continue

            if self.ser is None:
                self._try_reopen()
                if self.ser is None:
                    # 串口仍未打开，丢弃这一帧（队列里只保留最新值）。
                    continue
            try:
                self.ser.write(payload)
            except serial.SerialTimeoutException:
                try:
                    self.ser.reset_output_buffer()
                except Exception:
                    pass
            except Exception as e:
                self._mark_disconnected('send error', e)

    # ------------------------------------------------------------------
    # 后台线程：rx
    # ------------------------------------------------------------------
    def _rx_loop(self):
        """接收线程：持续读取串口，解析 P/D/B 标签更新回读值。"""
        while self._running:
            if self.ser is None:
                self._try_reopen()
                if self.ser is None:
                    time.sleep(_REOPEN_BACKOFF_MAX_S * 0.25)
                    continue

            try:
                n = self.ser.in_waiting
                if n:
                    chunk = self.ser.read(min(n, 512))
                    if chunk:
                        self._rx_buf.extend(chunk)
                        if len(self._rx_buf) > 4096:
                            del self._rx_buf[:-2048]
                else:
                    time.sleep(_BG_LOOP_IDLE_SLEEP_S)
                    continue
            except Exception as e:
                self._mark_disconnected('read error', e)
                continue

            self._parse_and_update()

    def _parse_and_update(self):
        """解析当前 rx 缓冲并更新回读值；持续解析失败时主动断开。

        对回读值做物理范围校验：PSI ∈ [-π, π], DELTA ∈ [-1, 1], BRAKE ∈ [-1, 10]。
        超出范围的值被钳位到边界，避免 ESP32 固件异常或线路干扰导致非物理值传播。
        """
        before = len(self._rx_buf)
        parsed = parse_tagged_lines(self._rx_buf)
        consumed = before - len(self._rx_buf)

        good_tags = sum(
            1 for k in ('P', 'D', 'B')
            if k in parsed and is_finite(parsed[k])
        )

        if good_tags > 0:
            with self._state_lock:
                if 'P' in parsed and is_finite(parsed['P']):
                    v = parsed['P']
                    # PSI 应在 [-π, π] 范围内
                    if v < -math.pi:
                        v = -math.pi
                    elif v > math.pi:
                        v = math.pi
                    self._esp_psi = v
                if 'D' in parsed and is_finite(parsed['D']):
                    v = parsed['D']
                    # DELTA 回读钳到与上行限幅一致的 ±MAX_DELTA，
                    # 任何超出物理上限的回读都视为干扰，按上限截断
                    if v < -MAX_DELTA:
                        v = -MAX_DELTA
                    elif v > MAX_DELTA:
                        v = MAX_DELTA
                    self._esp_delta = v
                if 'B' in parsed and is_finite(parsed['B']):
                    v = parsed['B']
                    # BRAKE 应在 [-1, 10] 范围内（负值=驱动, 正值=制动）
                    if v < -1.0:
                        v = -1.0
                    elif v > 10.0:
                        v = 10.0
                    self._esp_brake = v
                self._last_rx_t = time.monotonic()
                self._consec_bad_frames = 0
        elif consumed > 0:
            # 消耗了字节但一个有效标签都没拿到，累计错误。
            self._consec_bad_frames += 1
            if self._consec_bad_frames >= 50:
                self._mark_disconnected('persistent parse error')

    # ------------------------------------------------------------------
    # 关闭
    # ------------------------------------------------------------------
    def close(self):
        """停止后台线程并关闭串口。"""
        self._running = False
        # 唤醒可能阻塞在 get() 的 tx 线程
        try:
            self._tx_q.put_nowait(b'')
        except queue.Full:
            pass
        for t in (self._tx_thread, self._rx_thread):
            try:
                t.join(timeout=0.5)
            except Exception:
                pass
        with self._open_lock:
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
