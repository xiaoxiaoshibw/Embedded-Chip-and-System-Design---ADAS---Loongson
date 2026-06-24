#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""控制线程级 CPU 亲和性隔离（实时控制核）。

把 100Hz 控制主循环（最终调用 rclpy.spin 的主线程）独占钉到控制核，把进程内其余
所有线程（DDS C++ 线程、ML 异步推理、串口 / 遥测 / 日志 / 心跳守护线程）赶到剩余允许
核，使控制核成为一条"干净"的实时核——压低 100Hz 循环的尾延迟抖动（>budget 超限）。

安全约定（best-effort，绝不影响控制功能）：
- 仅在 Linux 且进程允许集含控制核、核数 ≥ 2 时启用，否则整体 no-op
  （开发机 / SIL / 单核环境自动跳过；offline replay / run_scenario 不调用本模块）。
- 全程 try/except，任何失败只记一次日志，控制节点照常运行。
- 用 /proc/self/task 枚举线程（含非 Python 的 DDS 线程），不依赖 threading。
- Linux 上主线程 TID == 进程 PID，据此区分"控制线程"与其余线程，无需 ctypes 取 tid
  （3.6 也安全）。
- 低频守护线程周期重扫，覆盖启动后才出现的线程（ML 推理、CARLA 接入时的 DDS 线程）。
"""

import logging
import os
import threading
import time

logger = logging.getLogger(__name__)

_TASK_DIR = "/proc/self/task"


def _sweep(control_core, aux_cores, control_tid):
    """主线程 → {control_core}，其余线程 → aux_cores。返回 (moved, failed)。"""
    moved = 0
    failed = 0
    try:
        tids = os.listdir(_TASK_DIR)
    except OSError:
        return (0, 0)
    for name in tids:
        try:
            tid = int(name)
        except ValueError:
            continue
        try:
            if tid == control_tid:
                os.sched_setaffinity(tid, {control_core})
            else:
                os.sched_setaffinity(tid, aux_cores)
                moved += 1
        except OSError:
            # 线程可能在枚举与设置之间退出（ESRCH），忽略。
            failed += 1
    return (moved, failed)


def isolate_control_core(control_core=0, resweep_s=3.0):
    """把控制主循环独占到 control_core，其余线程赶到剩余允许核。

    必须在控制主线程（最终调用 rclpy.spin 的线程）上调用。返回 True 表示已启用。
    """
    control_tid = os.getpid()  # Linux 主线程 TID == 进程 PID
    try:
        allowed = set(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        logger.info("[RT] sched_getaffinity 不可用，跳过线程级钉核")
        return False
    if control_core not in allowed or len(allowed) < 2:
        logger.info("[RT] 允许集 %s 不含控制核 %d 或核数 < 2，跳过线程级钉核",
                    sorted(allowed), control_core)
        return False
    aux_cores = allowed - {control_core}
    moved, failed = _sweep(control_core, aux_cores, control_tid)
    logger.info("[RT] 线程级钉核启用：控制主循环(tid=%d) → core%d 独占；"
                "其余线程 → core%s（moved=%d failed=%d）",
                control_tid, control_core, sorted(aux_cores), moved, failed)

    def _keeper():
        # 周期重扫，把启动后新生的线程（ML / DDS discovery）持续赶到 aux_cores。
        while True:
            time.sleep(resweep_s)
            try:
                _sweep(control_core, aux_cores, control_tid)
            except Exception:
                pass

    t = threading.Thread(target=_keeper, name="rt-affinity-keeper", daemon=True)
    t.start()
    return True
