#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""统一安全状态聚合器 —— 对标 openpilot `selfdriveState`/`onroadEvents`、Autoware diagnostic aggregator。

本项目的安全信号原本分散：CommandGate 的 `gate_reason`/`severity`、AEB overlay 的 `aeb_level`、
主备 `failover_available`、ESP32 回传 `readback_stale`、双核锁步失配。真实 ADAS 会把这些聚成
**一个权威的系统安全态**：「现在系统安不安全、为什么」。本模块做这件事——把上述信号聚成
`SafetyStatus{level, state, reasons}`，供监控/报告一眼看清整机安全裕度。

**关键约束：纯只读聚合，不读写任何控制状态、不改任何下发数值（控制零改）。**
level 取各来源严重度的最大值（gate_severity 已编码 gate_reason 的严重度）。

Python 3.6 兼容（JetPack 4.x）。中文注释，遵循 SOCCode 约定。
"""

from dataclasses import dataclass
from typing import Tuple

SAFETY_NOMINAL = "nominal"      # level 0：一切正常
SAFETY_DEGRADED = "degraded"    # level 1：降级运行（告警，仍可控）
SAFETY_EMERGENCY = "emergency"  # level 2：紧急（安全干预生效）


@dataclass(frozen=True)
class SafetyStatus:
    """整机安全态快照。"""
    level: int            # 0 nominal / 1 degraded / 2 emergency（各来源严重度取最大）
    state: str            # nominal / degraded / emergency
    reasons: Tuple        # 当前所有非正常来由（已排序去重）


def aggregate_safety_status(gate_reason="normal", gate_severity=0, aeb_level="safe",
                            failover_available=True, esp32_stale=False,
                            lockstep_fault=False):
    # type: (...) -> SafetyStatus
    """把分散的安全信号聚成统一安全态。只读，不改任何控制。"""
    level = int(gate_severity)
    reasons = []

    if gate_reason and gate_reason != "normal":
        reasons.append("gate_" + str(gate_reason))

    if aeb_level == "emergency":
        level = max(level, 2)
        reasons.append("aeb_emergency")
    elif aeb_level == "warning":
        level = max(level, 1)
        reasons.append("aeb_warning")

    if not failover_available:
        level = max(level, 1)
        reasons.append("no_failover")

    if esp32_stale:
        level = max(level, 1)
        reasons.append("esp32_link_stale")

    if lockstep_fault:
        level = max(level, 2)
        reasons.append("lockstep_fault")

    if level >= 2:
        state = SAFETY_EMERGENCY
    elif level >= 1:
        state = SAFETY_DEGRADED
    else:
        state = SAFETY_NOMINAL

    # 去重 + 排序，保证同一组信号产出稳定的 reasons（便于对比/告警去抖）
    return SafetyStatus(level=level, state=state,
                        reasons=tuple(sorted(set(reasons))))
