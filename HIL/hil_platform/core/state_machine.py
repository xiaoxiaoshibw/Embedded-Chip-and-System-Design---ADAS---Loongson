# -*- coding: utf-8 -*-
"""仿真状态机。

状态：
    IDLE     已连接 CARLA（或 mock），但未加载场景
    READY    已加载场景和参数，等待开始
    RUNNING  正在仿真
    PAUSED   暂停仿真，保留当前状态
    STOPPED  本次实验停止，数据已保存
    ERROR    异常状态

只允许下列受控跃迁，非法跃迁抛 InvalidTransition，避免 Web/CLI 乱序调用把
底层 CARLA 状态带乱。
"""

from __future__ import annotations

import threading
from enum import Enum
from typing import Dict, Set


class SimState(str, Enum):
    IDLE = "IDLE"
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    STOPPED = "STOPPED"
    ERROR = "ERROR"


class InvalidTransition(Exception):
    """非法状态跃迁。"""


# 允许的跃迁表（ERROR 可从任意状态进入，单独处理）
_ALLOWED: Dict[SimState, Set[SimState]] = {
    SimState.IDLE: {SimState.READY},
    SimState.READY: {SimState.RUNNING, SimState.READY, SimState.IDLE},
    SimState.RUNNING: {SimState.PAUSED, SimState.STOPPED},
    SimState.PAUSED: {SimState.RUNNING, SimState.STOPPED},
    SimState.STOPPED: {SimState.READY, SimState.IDLE},
    SimState.ERROR: {SimState.IDLE, SimState.READY},
}


class StateMachine:
    """线程安全的状态机。"""

    def __init__(self, initial: SimState = SimState.IDLE):
        self._state = initial
        self._lock = threading.RLock()

    @property
    def state(self) -> SimState:
        with self._lock:
            return self._state

    def can(self, target: SimState) -> bool:
        with self._lock:
            if target == SimState.ERROR:
                return True
            return target in _ALLOWED.get(self._state, set())

    def transition(self, target: SimState) -> SimState:
        """执行跃迁；非法则抛异常。返回新状态。"""
        with self._lock:
            if target == SimState.ERROR:
                self._state = SimState.ERROR
                return self._state
            if target not in _ALLOWED.get(self._state, set()):
                raise InvalidTransition(
                    "非法状态跃迁：%s -> %s" % (self._state.value, target.value)
                )
            self._state = target
            return self._state

    def force(self, target: SimState) -> SimState:
        """强制设置状态（仅 reset/内部恢复使用）。"""
        with self._lock:
            self._state = target
            return self._state
