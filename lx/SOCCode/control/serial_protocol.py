#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Jetson → ESP32 串口帧编码。

将控制周期产出的 12 维浮点参数编码为 ascii 字符串帧，
格式为 "TAG:value" 键值对，末尾附加 CRC8 校验字段，换行结束。

帧格式：
  TTC:xx DIST:xx PSI:xx DELTA:xx SPEED:xx ACC:xx OFFSET:xx
  LEADV:xx DSAFE:xx WMRN:xx WHRD:xx CURV:xx CRC:xx\\n

CRC8 算法：Dallas/Maxim 多项式 0x31，对 "CRC:" 之前的所有字节（含末尾空格）
逐字节计算，结果以两位十六进制大写输出。ESP32 侧用相同算法验证，
不匹配则丢弃该帧，防止 UART 线路噪声导致的单字节翻转污染控制量。
"""

from dataclasses import dataclass


def _build_crc8_table():
    """构造 CRC-8/MAXIM（Dallas 多项式 0x31）的 256 项查表。"""
    table = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return tuple(table)


# 模块加载时算一次；每帧 CRC 只做逐字节查表，省去 100Hz 下的逐位循环。
_CRC8_TABLE = _build_crc8_table()


def _crc8_dallas(data: bytes) -> int:
    """CRC-8/MAXIM（Dallas 多项式 0x31，初值 0x00，无反转）。

    逐字节查表计算，返回 0~255 的校验值；结果与逐位实现完全一致。
    """
    crc = 0
    for byte in data:
        crc = _CRC8_TABLE[crc ^ byte]
    return crc


@dataclass(frozen=True)
class Esp32ControlFrame:
    """Jetson 发往 ESP32 的控制帧数据。"""
    ttc: float                    # 碰撞时间 (s)
    dist: float                   # 前车距离 (m)
    psi: float                    # 航向角 (rad)
    delta: float                  # 方向盘转角 (rad)
    speed: float                   # 自车速度 (m/s)
    lon: float                     # 纵向加速度指令 (m/s²)
    offset: float                  # 车道偏移 (m)
    lead_v_proj: float             # 前车投影速度 (m/s)
    min_safe_dist: float           # 最小安全距离 (m)
    lane_warn_margin: float        # 车道预警余量 (m)
    lane_hard_margin: float        # 车道硬边界余量 (m)
    filtered_curv: float           # 滤波曲率 (1/m)


def build_esp32_payload(frame: Esp32ControlFrame) -> bytes:
    """将控制帧编码为带 CRC8 校验的 ascii 字节流，通过串口发送给 ESP32。

    格式: "TTC:xx ... CURV:xx CRC:XX\\n"
    CRC 覆盖 "CRC:" 之前的所有字节（含末尾空格），ESP32 侧验证后丢弃不匹配帧。
    """
    body = (
        'TTC:{:.2f} DIST:{:.2f} PSI:{:.4f} DELTA:{:.4f} '
        'SPEED:{:.2f} ACC:{:+.2f} OFFSET:{:.3f} LEADV:{:.2f} DSAFE:{:.2f} '
        'WMRN:{:.2f} WHRD:{:.2f} CURV:{:.4f} '
        .format(
            frame.ttc,
            frame.dist,
            frame.psi,
            frame.delta,
            frame.speed,
            frame.lon,
            frame.offset,
            frame.lead_v_proj,
            frame.min_safe_dist,
            frame.lane_warn_margin,
            frame.lane_hard_margin,
            frame.filtered_curv,
        )
    )
    body_bytes = body.encode('ascii')
    crc = _crc8_dallas(body_bytes)
    return body_bytes + f'CRC:{crc:02X}\n'.encode('ascii')