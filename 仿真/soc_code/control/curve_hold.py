#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""弯道保持状态机。

当前车在弯道中丢失时，进入弯道保持模式：用 PI 控制器
维持丢失时刻的速度，直到弯道结束或前车重新获取。
"""

import logging

from config import (
    CURVE_HOLD_ACTIVATE_LOSS_S,
    CURVE_HOLD_CURV_THRESH,
    CURVE_HOLD_EXIT_CURV,
    CURVE_HOLD_REACQ_STABLE_S,
    CURVE_HOLD_TIMEOUT_S,
)
from control.state import CurveHoldState


class CurveHoldManager:
    """管理弯道保持状态的进入与退出。"""

    def __init__(self):
        self.state = CurveHoldState()

    def update(self, now: float, has_lead: bool, raw_has_lead: bool,
               filtered_curv: float, ego_v: float) -> bool:
        """每周期更新弯道保持状态机。

        参数:
            now: 单调时钟
            has_lead: 经过确认的前车标志
            raw_has_lead: 原始前车标志
            filtered_curv: 滤波曲率
            ego_v: 自车速度

        返回:
            是否处于弯道保持状态
        """
        state = self.state
        in_curve = abs(filtered_curv) > CURVE_HOLD_CURV_THRESH

        # 记录前车丢失时刻
        if raw_has_lead:
            state.loss_since = -1e9
        elif in_curve and state.prev_raw_has_lead:
            state.loss_since = now

        if not state.active:
            # 激活条件：弯道内 + 前车丢失超过阈值时间
            if (
                in_curve
                and state.loss_since > 0
                and (now - state.loss_since) >= CURVE_HOLD_ACTIVATE_LOSS_S
            ):
                state.active = True
                state.v_target = max(ego_v, 0.0)
                state.start_t = now
                # 激活时清零 v_i：防止上次退出时残留的积分项在新一轮保持中
                # 造成速度控制初期偏高（曲率在阈值附近抖动时尤为明显）。
                state.v_i = 0.0
                state.reacq_since = -1e9
                logging.warning(
                    '[CURVE_HOLD] activated: curv=%.4f v_hold=%.2f m/s',
                    filtered_curv, state.v_target
                )
        else:
            elapsed = now - state.start_t
            # 退出条件 1：曲率降低到直道水平
            if abs(filtered_curv) < CURVE_HOLD_EXIT_CURV:
                logging.info('[CURVE_HOLD] exit: straight road (curv=%.4f)', filtered_curv)
                state.active = False
                state.v_i = 0.0
                state.reacq_since = -1e9
            # 退出条件 2：保持超时
            elif elapsed > CURVE_HOLD_TIMEOUT_S:
                logging.warning('[CURVE_HOLD] exit: timeout %.1fs', elapsed)
                state.active = False
                state.v_i = 0.0
                state.reacq_since = -1e9
            # 退出条件 3：前车重新获取并稳定
            elif has_lead:
                if state.reacq_since < 0:
                    state.reacq_since = now
                elif (now - state.reacq_since) >= CURVE_HOLD_REACQ_STABLE_S:
                    logging.info('[CURVE_HOLD] exit: lead reacquired')
                    state.active = False
                    state.v_i = 0.0
                    state.reacq_since = -1e9
            else:
                state.reacq_since = -1e9

        state.prev_has_lead = has_lead
        state.prev_raw_has_lead = raw_has_lead
        return state.active