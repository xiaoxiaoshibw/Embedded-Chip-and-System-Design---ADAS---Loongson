#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""启动期安全配置校验 —— 对标 Apollo/Autoware 的配置 schema 校验。

在节点启动时对（含 `params.yaml` 覆盖后的）**有效配置**做物理合理性 + 一致性检查，
把不合理的安全关键参数以 WARNING/CRITICAL 记入日志，给报告提供“启动即自检安全配置”
的证据链，并能抓住坏的运行时覆盖。

**关键约束：只读校验、只记日志。绝不修改任何参数、绝不改变控制行为**——坏配置仍按
原值运行（本层是审计/告警，不是熔断）。这样接入零控制风险、与原行为字节级一致。

Python 3.6 兼容（JetPack 4.x）。中文注释，遵循 SOCCode 约定。
"""

import logging

LEVEL_WARN = "WARN"
LEVEL_CRITICAL = "CRITICAL"


def _check_range(issues, name, value, lo, hi, level=LEVEL_CRITICAL):
    """把 value 不在 [lo, hi] 的情况追加到 issues。非数值记 CRITICAL。"""
    try:
        v = float(value)
    except (TypeError, ValueError):
        issues.append((LEVEL_CRITICAL, name, "非数值: %r" % (value,)))
        return
    if v != v:  # NaN
        issues.append((LEVEL_CRITICAL, name, "NaN"))
        return
    if not (lo <= v <= hi):
        issues.append((level, name, "%.4g 越界 [%.4g, %.4g]" % (v, lo, hi)))


def validate_config():
    """返回 [(level, name, message)]；空列表表示所有安全关键参数合理。"""
    import config as c
    issues = []

    # ── 执行器物理范围 ──
    _check_range(issues, "MAX_DELTA", c.MAX_DELTA, 0.01, 1.5708)            # (0, π/2] rad
    _check_range(issues, "LON_CMD_MAX_BRAKE_DECEL", c.LON_CMD_MAX_BRAKE_DECEL, 0.5, 12.0)
    _check_range(issues, "LON_CMD_MAX_DRIVE_ACCEL", c.LON_CMD_MAX_DRIVE_ACCEL, 0.1, 8.0)
    _check_range(issues, "LANE_DEFAULT_WIDTH", c.LANE_DEFAULT_WIDTH, 2.0, 6.0)

    # ── 实时性 ──
    _check_range(issues, "LOOP_HZ", c.LOOP_HZ, 10.0, 1000.0)
    _check_range(issues, "CTRL_CONSECUTIVE_ERROR_LIMIT", c.CTRL_CONSECUTIVE_ERROR_LIMIT, 1, 100)

    # ── AEB / TTC 阈值 ──
    _check_range(issues, "TTC_BRAKE_FULL", c.TTC_BRAKE_FULL, 0.5, 30.0)
    _check_range(issues, "TTC_BRAKE_START", c.TTC_BRAKE_START, 1.0, 60.0)
    _check_range(issues, "AEB_EMERGENCY_DIST", c.AEB_EMERGENCY_DIST, 0.5, 30.0)
    _check_range(issues, "AEB_MAX_ENGAGE_DIST", c.AEB_MAX_ENGAGE_DIST, 5.0, 100.0)

    # ── 主备 / 感知超时 ──
    _check_range(issues, "HEARTBEAT_TIMEOUT_S", c.HEARTBEAT_TIMEOUT_S, 0.02, 2.0)
    _check_range(issues, "SENSOR_TIMEOUT_BRAKE_CMD", c.SENSOR_TIMEOUT_BRAKE_CMD, 0.0, 12.0)

    # ── 一致性（排序/符号）约束 ──
    try:
        if float(c.TTC_BRAKE_FULL) >= float(c.TTC_BRAKE_START):
            issues.append((LEVEL_CRITICAL, "TTC_ORDER",
                           "TTC_BRAKE_FULL(%.2f) 应 < TTC_BRAKE_START(%.2f)"
                           % (c.TTC_BRAKE_FULL, c.TTC_BRAKE_START)))
        if float(c.AEB_EMERGENCY_DIST) >= float(c.AEB_MAX_ENGAGE_DIST):
            issues.append((LEVEL_CRITICAL, "AEB_DIST_ORDER",
                           "AEB_EMERGENCY_DIST(%.1f) 应 < AEB_MAX_ENGAGE_DIST(%.1f)"
                           % (c.AEB_EMERGENCY_DIST, c.AEB_MAX_ENGAGE_DIST)))
    except (TypeError, ValueError):
        pass  # 上面 _check_range 已分别报非数值

    return issues


def log_config_issues():
    """启动时调用：校验配置并按级别记日志。返回 issues 数量（0=全部合理）。"""
    try:
        issues = validate_config()
    except Exception as exc:
        logging.warning("[CONFIG] 配置校验异常（忽略，不影响控制）：%r", exc)
        return 0
    if not issues:
        logging.info("[CONFIG] 安全配置自检通过（所有安全关键参数在合理范围内）")
        return 0
    for level, name, msg in issues:
        if level == LEVEL_CRITICAL:
            logging.critical("[CONFIG] 安全配置异常 %s：%s（仍按原值运行，请核查）", name, msg)
        else:
            logging.warning("[CONFIG] 安全配置告警 %s：%s", name, msg)
    return len(issues)
