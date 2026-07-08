#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""虚拟 ESP32：`lx/MCUcode/ADAS_Test/main/main.c` 主备仲裁 + 双路通信看门狗的
最小 Python 移植（仅这两段逻辑，非完整固件移植）。

背景：`仿真/`（原本承载完整虚拟 ESP32 + CARLA 联合仿真的目录）当前不在本次
checkout 里。这里只按需移植 `WATCHDOG_TIMEOUT_MS` 压缩下限这一件事所需的最小
逻辑——`arbitrate()`（主备切换）与 `comm_watchdog_task()`（双路静默检测），
供 `experiment_watchdog_limit.py` 做 SIL 级别的快速重复试验。**不是**完整
虚拟 ESP32，不含串口帧解析、AEB 硬件地板、CRC 校验等其余部分。

移植对照（逐行对应 main.c，行号为原文件行号）：
  - `state_is_fresh()`      ← main.c:370-374
  - `arbitrate()`           ← main.c:379-404
  - `comm_watchdog_task()` 的判定段（不含 FreeRTOS 任务调度本身）
                            ← main.c:611-646

时间基准用显式传入的模拟毫秒时间戳（不读墙钟），保证实验可用远快于真实时间
的速度重复扫描成百上千次试验。

Python 3.6 兼容；注释中文。
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class ChannelState:
    """对应 main.c 的 JetsonState（这里只保留仲裁/看门狗需要的字段）。"""
    valid: bool = False
    last_rx_ms: float = float('-inf')


@dataclass
class WatchdogEvent:
    """对应 main.c 里 printf 的一条日志（仅保留判定必需的字段）。"""
    t_ms: float
    kind: str        # 'switch_recovered' / 'switch_timeout' / 'wdt_timeout' / 'wdt_recovered'


class VirtualEsp32Watchdog:
    """`arbitrate()` + `comm_watchdog_task()` 判定逻辑的移植体。

    调用约定（对应真实固件的两个独立任务）：
      - `on_frame(channel, t_ms)`：串口收到一帧有效数据时调用
        （对应 `uart_rx_task` 更新 `JetsonState.last_rx_us`）。
      - `arbitrate(t_ms)`：仲裁任务每拍调用一次，返回当前应使用的来源。
        （main.c 里 `control_task` 每 10ms 调一次）。
      - `watchdog_check(t_ms)`：看门狗任务按 `watchdog_check_ms` 周期调用，
        返回是否处于紧急制动态（`estop_active`）。
    """

    def __init__(self, jetson_timeout_ms, watchdog_timeout_ms,
                watchdog_check_ms):
        self.jetson_timeout_ms = float(jetson_timeout_ms)
        self.watchdog_timeout_ms = float(watchdog_timeout_ms)
        self.watchdog_check_ms = float(watchdog_check_ms)
        self.pri = ChannelState()
        self.sec = ChannelState()
        self.use_secondary = False
        self.estop_active = False
        self.events = []   # type: List[WatchdogEvent]

    def on_frame(self, channel, t_ms):
        """channel: 'pri' 或 'sec'。对应 main.c uart_rx_task 收到合法帧时
        更新 JetsonState.last_rx_us + valid=true。"""
        st = self.pri if channel == 'pri' else self.sec
        st.valid = True
        st.last_rx_ms = t_ms

    @staticmethod
    def _is_fresh(st, now_ms, timeout_ms):
        """对应 main.c state_is_fresh()：valid && age <= timeout。"""
        if not st.valid:
            return False
        return (now_ms - st.last_rx_ms) <= timeout_ms

    def arbitrate(self, now_ms):
        """对应 main.c arbitrate()。返回 'pri' / 'sec'。

        主控新鲜 → 用主控（若之前在用备控，记一次恢复切回）；
        主控不新鲜、备控新鲜 → 用备控（若之前在用主控，记一次超时切换）；
        两路都不新鲜 → 保持上次来源不变（不产生新事件——与固件行为一致，
        这种情况留给 watchdog_check 处理）。
        """
        pri_fresh = self._is_fresh(self.pri, now_ms, self.jetson_timeout_ms)
        sec_fresh = self._is_fresh(self.sec, now_ms, self.jetson_timeout_ms)

        if pri_fresh:
            if self.use_secondary:
                self.events.append(WatchdogEvent(now_ms, 'switch_recovered'))
                self.use_secondary = False
            return 'pri'

        if sec_fresh:
            if not self.use_secondary:
                self.events.append(WatchdogEvent(now_ms, 'switch_timeout'))
                self.use_secondary = True
            return 'sec'

        return 'sec' if self.use_secondary else 'pri'

    def watchdog_check(self, now_ms):
        """对应 main.c comm_watchdog_task() 每 WATCHDOG_CHECK_MS 的判定段。

        两路都不新鲜（按 watchdog_timeout_ms 判据，不是 jetson_timeout_ms）
        才进入/维持紧急制动态；任一路新鲜即视为正常，若之前在紧急态则记一次
        恢复。返回当前 estop_active。
        """
        pri_alive = self._is_fresh(self.pri, now_ms, self.watchdog_timeout_ms)
        sec_alive = self._is_fresh(self.sec, now_ms, self.watchdog_timeout_ms)

        if pri_alive or sec_alive:
            if self.estop_active:
                self.events.append(WatchdogEvent(now_ms, 'wdt_recovered'))
                self.estop_active = False
        else:
            if not self.estop_active:
                self.estop_active = True
                self.events.append(WatchdogEvent(now_ms, 'wdt_timeout'))

        return self.estop_active

    def first_event(self, kind):
        # type: (str) -> Optional[WatchdogEvent]
        for e in self.events:
            if e.kind == kind:
                return e
        return None
