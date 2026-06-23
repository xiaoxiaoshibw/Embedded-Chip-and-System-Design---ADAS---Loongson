#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""通用工具函数。

提供数值裁剪、角度归一化、四元数转航向、日志初始化以及串口报文解析等
被多个模块共用的基础工具。
"""

import atexit
import logging
import math
import queue
import time as _time_module  # 模块级缓存，避免 RateLimitedCritical.log() 每次调用都触发字典查找
from logging.handlers import QueueHandler, QueueListener, RotatingFileHandler
from typing import Dict, Optional

from config import LOG_LEVEL
import runtime

# 全局 QueueListener 引用：避免被 GC 收走，保证日志线程持续运行。
_log_listener: Optional[QueueListener] = None


def clamp(v, lo, hi):
    """将数值 v 限制在 [lo, hi] 区间内。"""
    return max(lo, min(hi, v))


def apply_deadband(v, band):
    """硬死区：|v| <= band 时输出 0，否则输出原始值。"""
    if abs(v) <= band:
        return 0.0
    return v


def soft_deadband(v, band):
    """软死区：在 |v| <= band 时按 25% 衰减，在 band~2*band 之间线性过渡到原始值。

    相比 apply_deadband，软死区避免了在死区边缘的跳变。
    """
    if band <= 1e-6:
        return v
    av = abs(v)
    if av <= band:
        return v * 0.25
    if av >= band * 2.0:
        return v
    # 在 band~2*band 区间线性插值衰减系数
    scale = (av - band) / band
    return math.copysign(av * (0.25 + 0.75 * scale), v)


def wrap_angle(a):
    """将任意角度 a 归一化到 [-π, π]。"""
    return math.atan2(math.sin(a), math.cos(a))


def is_finite(v):
    """判断 v 既不是 inf 也不是 nan。"""
    return (not math.isinf(v)) and (not math.isnan(v))


def quaternion_to_yaw(qx, qy, qz, qw):
    """将四元数转换为偏航角 (yaw)，绕 Z 轴旋转。"""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def setup_logging():
    """初始化日志系统：异步落盘（QueueHandler 投递 → QueueListener 后台线程消费）。

    控制线程内的 logging 调用只做无锁 put，不会被文件 fsync 或终端刷新阻塞，
    彻底消除日志 IO 对 10ms 控制周期的影响。
    """
    global _log_listener

    logger = logging.getLogger()
    logger.setLevel(LOG_LEVEL)
    logger.handlers = []

    # 真正的落盘 handler 由后台线程持有，不挂到 root logger 上
    fmt = logging.Formatter('%(asctime)s | %(levelname)s | %(message)s')
    stream_h = logging.StreamHandler()
    stream_h.setFormatter(fmt)
    file_h = RotatingFileHandler(
        runtime.LOG_FILE, maxBytes=2 * 1024 * 1024, backupCount=3,
    )
    file_h.setFormatter(fmt)

    # 队列容量给到 4096：在 100Hz 下足够缓冲 40 秒的偶发突发，
    # 满了之后 QueueHandler 默认会阻塞写者；这里用无界队列保实时性，
    # 由 backupCount 限制磁盘占用。
    log_q: queue.Queue = queue.Queue(-1)
    root_handler = QueueHandler(log_q)
    logger.addHandler(root_handler)

    # 复位旧 listener（重复调用 setup_logging 时）并关闭旧文件句柄，
    # 否则 RotatingFileHandler 会一直占着 fd，72 小时运行中累积成 fd 泄漏。
    if _log_listener is not None:
        try:
            _log_listener.stop()
        except Exception:
            pass
        for h in getattr(_log_listener, 'handlers', ()):
            try:
                h.close()
            except Exception:
                pass

    _log_listener = QueueListener(
        log_q, stream_h, file_h, respect_handler_level=False,
    )
    _log_listener.start()
    atexit.register(_log_listener.stop)


_TAGGED_LINE_MAX_LEN = 256


class RateLimitedCritical:
    """CRITICAL 事件首次正常输出，窗口内重复事件降为 WARNING。

    用于 takeover / emergency_stop 这类事件：第一次发生时希望触发告警短信，
    但 flapping/连环异常场景下不希望被刷屏。
    """

    def __init__(self, window_s: float = 1.0):
        self.window_s = window_s
        self._last_t: Dict[str, float] = {}

    def log(self, tag: str, fmt: str, *args):
        """按 tag 维度限频：首次 CRITICAL，window_s 内的重复降为 WARNING。"""
        now = _time_module.monotonic()
        last = self._last_t.get(tag, 0.0)
        level = logging.CRITICAL if (now - last) > self.window_s else logging.WARNING
        self._last_t[tag] = now
        logging.log(level, fmt, *args)


def parse_tagged_lines(buf):
    """从串口接收缓冲区中逐行提取 "TAG:value" 格式的键值对。

    每行以 '\\n' 分隔，解析成功的浮点值存入 result 字典。
    已解析的行会从 buf 中移除，未完成行留在缓冲区尾部。

    保护：
      - 单行长度超过 _TAGGED_LINE_MAX_LEN 视为乱码，直接丢弃，避免
        恶意/异常长行触发后续粘包错位。
      - 未见换行但缓冲区尾段已超长时（无换行单帧），主动截断前段。
    """
    result = {}
    while True:
        try:
            nl = buf.index(b'\n')
        except ValueError:
            # 没找到换行：如果当前残段已经超过单行上限，丢弃前面的字节，
            # 只保留最后 _TAGGED_LINE_MAX_LEN 字节继续等下一帧的换行。
            if len(buf) > _TAGGED_LINE_MAX_LEN:
                del buf[:-_TAGGED_LINE_MAX_LEN]
            break
        line = buf[:nl]
        del buf[:nl + 1]
        if len(line) > _TAGGED_LINE_MAX_LEN:
            # 超长行：跳过，不影响后续帧解析
            continue
        try:
            s = line.decode('ascii', errors='ignore').strip()
        except Exception:
            continue
        if ':' not in s:
            continue
        tag, _, val_str = s.partition(':')
        try:
            result[tag.strip()] = float(val_str.strip())
        except Exception:
            pass
    return result