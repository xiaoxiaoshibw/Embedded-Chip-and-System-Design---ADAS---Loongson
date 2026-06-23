#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""虚拟 ESP32 执行器（MCUcode/ADAS_Test/main/main.c 的 Python 移植）。

在桥进程内复现 ESP32 的全部安全语义，使"SOC → UART → MCU 仲裁 → 执行器"
链路在联合仿真中字节级打通：
  - parse_jetson_line：解析 SOC 发来的真实 build_esp32_payload() 帧，
    含 CRC-8/MAXIM 校验（多项式 0x31），CRC 不匹配整帧丢弃；
  - arbitrate：主新鲜→主；主超时(150ms)备新鲜→切备（SWITCH 日志）；
    主恢复自动切回；
  - update_aeb：硬件兜底 AEB 地板（dist<=hard_floor 全力制动）；
  - 通信看门狗：两路同时静默 200ms → 紧急制动帧（SRC:9）。

两条"虚拟 UART"用同一个 UDP 端口承载，报文前缀 "P "/"B " 区分物理串口。
"""

import math
import socket
import threading
import time

import paths  # noqa: E402 — adds HIL/carla_bridge/pc/ to sys.path

from bridge_config import (
    ESP32_UART_PORT,
    JETSON_LON_CMD_MAX_BRAKE_DECEL,
    JETSON_LON_CMD_MAX_DRIVE_ACCEL,
    JETSON_TIMEOUT_MS,
    MCU_AEB_MAX_BRAKE_DECEL,
    MCU_AEB_MIN_CLOSING_SPEED,
    SAFE_DIST_SOFT_BUFFER,
    WATCHDOG_TIMEOUT_MS,
)


def _crc8_dallas(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _clampf(v, lo, hi):
    return max(lo, min(hi, v))


class JetsonState:
    """对应 main.c 的 JetsonState 结构体。"""

    __slots__ = ('ttc', 'dist', 'psi', 'delta', 'speed', 'lon_cmd', 'offset',
                 'lead_speed', 'safe_dist', 'warn_margin', 'hard_margin',
                 'curv', 'valid', 'last_rx', 'rx_count', 'crc_fail_count')

    def __init__(self):
        self.ttc = 999.0
        self.dist = 999.0
        self.psi = 0.0
        self.delta = 0.0
        self.speed = 0.0
        self.lon_cmd = 0.0
        self.offset = 0.0
        self.lead_speed = 0.0
        self.safe_dist = 0.0
        self.warn_margin = 0.0
        self.hard_margin = 0.0
        self.curv = 0.0
        self.valid = False
        self.last_rx = -1e9
        self.rx_count = 0
        self.crc_fail_count = 0


_TAG_FIELDS = {
    'TTC': 'ttc', 'DIST': 'dist', 'PSI': 'psi', 'DELTA': 'delta',
    'SPEED': 'speed', 'ACC': 'lon_cmd', 'OFFSET': 'offset',
    'LEADV': 'lead_speed', 'DSAFE': 'safe_dist', 'WMRN': 'warn_margin',
    'WHRD': 'hard_margin', 'CURV': 'curv',
}


def parse_jetson_line(line: str, st: JetsonState, now: float) -> bool:
    """解析一帧 SOC 控制帧（移植 main.c parse_jetson_line，CRC 校验一致）。"""
    idx = line.find(' CRC:')
    if idx >= 0:
        body = line[:idx + 1]                  # 含 "CRC:" 前的空格
        crc_str = line[idx + 5:].strip()
        try:
            expected = int(crc_str, 16)
        except ValueError:
            expected = None
        if expected is not None:
            calc = _crc8_dallas(body.encode('ascii', errors='ignore'))
            if calc != expected:
                st.crc_fail_count += 1
                return False
    got = 0
    for token in line.split():
        tag, _, val = token.partition(':')
        attr = _TAG_FIELDS.get(tag)
        if attr is None:
            continue
        try:
            v = float(val)
        except ValueError:
            continue
        if math.isnan(v) or math.isinf(v):
            continue
        setattr(st, attr, v)
        got += 1
    if got < 4:
        return False
    # 与 main.c 同样的范围裁剪
    st.lon_cmd = _clampf(st.lon_cmd,
                         -JETSON_LON_CMD_MAX_DRIVE_ACCEL,
                         JETSON_LON_CMD_MAX_BRAKE_DECEL)
    st.valid = True
    st.last_rx = now
    st.rx_count += 1
    return True


def update_aeb(s: JetsonState) -> float:
    """ESP32 硬件兜底 AEB（逐行移植 main.c update_aeb）。"""
    if math.isnan(s.dist) or s.dist <= 0.0:
        return 0.0
    curv = abs(s.curv)
    ego_speed = max(s.speed, 0.0)
    lead_speed = max(s.lead_speed, 0.0)
    closing_speed = max(0.0, ego_speed - lead_speed)

    curv_comp = _clampf(1.0 + 3.0 * curv, 1.0, 1.3)
    safe_dist = (s.safe_dist * curv_comp if s.safe_dist > 0.0
                 else 5.0 * curv_comp)
    safe_soft = safe_dist + SAFE_DIST_SOFT_BUFFER

    aeb_full_brake_floor = 2.5
    hard_floor = max(aeb_full_brake_floor, safe_dist * 0.40)
    hard_floor = min(hard_floor, max(aeb_full_brake_floor, safe_dist - 1.0))

    if s.dist <= hard_floor:
        return MCU_AEB_MAX_BRAKE_DECEL
    if closing_speed < MCU_AEB_MIN_CLOSING_SPEED:
        return 0.0
    if s.dist < safe_soft:
        speed_ratio = _clampf(
            (closing_speed - MCU_AEB_MIN_CLOSING_SPEED) / 2.4, 0.0, 1.0)
        if s.dist < safe_dist:
            r = _clampf((safe_dist - s.dist) / (safe_dist - hard_floor + 1e-6),
                        0.0, 1.0)
            return MCU_AEB_MAX_BRAKE_DECEL * speed_ratio * r * r
        r = _clampf((safe_soft - s.dist) / (safe_soft - safe_dist + 1e-6),
                    0.0, 1.0)
        return JETSON_LON_CMD_MAX_BRAKE_DECEL * speed_ratio * r * r
    return 0.0


class VirtualEsp32:
    """虚拟 ESP32：UDP 收帧线程 + 周期 control_step（由桥主循环调用）。

    step() 返回 dict：psi/delta/a_brake/src/watchdog/aeb_floor 等，
    src ∈ {0=primary, 1=secondary, 9=watchdog}（与真实固件 SRC 字段一致）。
    """

    def __init__(self, port: int = ESP32_UART_PORT):
        self._lock = threading.Lock()
        self.pri = JetsonState()
        self.sec = JetsonState()
        self.use_secondary = False
        self.watchdog_active = False
        self.events = []            # (t, 文本) 事件日志，HUD/复盘用
        self._running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('127.0.0.1', port))
        self._sock.settimeout(0.05)
        threading.Thread(target=self._rx_loop, daemon=True).start()

    # ── 虚拟 UART 接收（对应 uart_rx_task）──
    def _rx_loop(self):
        buf = {'P': '', 'B': ''}
        while self._running:
            try:
                data, _ = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                if not self._running:
                    return
                continue
            try:
                text = data.decode('ascii', errors='ignore')
            except Exception:
                continue
            if len(text) < 2 or text[0] not in 'PB' or text[1] != ' ':
                continue
            role, payload = text[0], text[2:]
            buf[role] += payload
            while '\n' in buf[role]:
                line, buf[role] = buf[role].split('\n', 1)
                if not line.strip():
                    continue
                st = self.pri if role == 'P' else self.sec
                now = time.monotonic()
                with self._lock:
                    parse_jetson_line(line, st, now)

    # ── 主备仲裁（对应 arbitrate()）──
    def _fresh(self, st: JetsonState, now: float) -> bool:
        return st.valid and (now - st.last_rx) * 1000.0 <= JETSON_TIMEOUT_MS

    def _arbitrate(self, now: float) -> JetsonState:
        pri_fresh = self._fresh(self.pri, now)
        sec_fresh = self._fresh(self.sec, now)
        if pri_fresh:
            if self.use_secondary:
                age_ms = (now - self.pri.last_rx) * 1000.0
                self.events.append((now, 'SWITCH:primary_recovered_%dms' % age_ms))
                self.use_secondary = False
            return self.pri
        if sec_fresh:
            if not self.use_secondary:
                age_ms = (now - self.pri.last_rx) * 1000.0
                self.events.append((now, 'SWITCH:pri_timeout_%dms' % age_ms))
                self.use_secondary = True
            return self.sec
        return self.sec if self.use_secondary else self.pri

    # ── control_step + 看门狗（对应 control_task / comm_watchdog_task）──
    def step(self):
        now = time.monotonic()
        with self._lock:
            local = self._arbitrate(now)
            pri_age = (now - self.pri.last_rx) * 1000.0
            sec_age = (now - self.sec.last_rx) * 1000.0
            pri_alive = self.pri.valid and pri_age <= WATCHDOG_TIMEOUT_MS
            sec_alive = self.sec.valid and sec_age <= WATCHDOG_TIMEOUT_MS

            # 通信看门狗：两路都静默 → 紧急制动（最高优先级，绕过控制）
            if not pri_alive and not sec_alive:
                if not self.watchdog_active:
                    self.events.append((now, 'WATCHDOG:TIMEOUT pri=%dms sec=%dms'
                                        % (pri_age, sec_age)))
                self.watchdog_active = True
                return {
                    'psi': 0.0, 'delta': 0.0,
                    'a_brake': MCU_AEB_MAX_BRAKE_DECEL,
                    'src': 9, 'watchdog': True, 'aeb_floor': 0.0,
                    'state': local,
                }
            if self.watchdog_active:
                self.events.append((now, 'WATCHDOG:RECOVERED'))
                self.watchdog_active = False

            if not self._fresh(local, now):
                # 选中源已过期但另一路还没到看门狗阈值：与固件一致输出全制动
                return {
                    'psi': 0.0, 'delta': 0.0,
                    'a_brake': MCU_AEB_MAX_BRAKE_DECEL,
                    'src': 1 if self.use_secondary else 0,
                    'watchdog': False, 'aeb_floor': 0.0, 'state': local,
                }

            # update_lka / update_acc / update_aeb 合成
            psi = _clampf(local.psi if math.isfinite(local.psi) else 0.0,
                          -9.9999, 9.9999)
            delta = _clampf(local.delta if math.isfinite(local.delta) else 0.0,
                            -9.99, 9.99)
            acc_out = _clampf(local.lon_cmd,
                              -JETSON_LON_CMD_MAX_DRIVE_ACCEL,
                              JETSON_LON_CMD_MAX_BRAKE_DECEL)
            aeb_out = update_aeb(local)
            br = acc_out
            if aeb_out > 0.0:
                br = max(aeb_out, max(acc_out, 0.0))
            br = _clampf(br, -JETSON_LON_CMD_MAX_DRIVE_ACCEL,
                         MCU_AEB_MAX_BRAKE_DECEL)
            return {
                'psi': psi, 'delta': delta, 'a_brake': br,
                'src': 1 if self.use_secondary else 0,
                'watchdog': False, 'aeb_floor': aeb_out, 'state': local,
            }

    def drain_events(self):
        with self._lock:
            ev, self.events = self.events, []
        return ev

    def close(self):
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass
