#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""AEB 告警状态机。

当前车突然丢失时进入告警状态，保持当前速度一段时间，
防止由于感知短暂丢失导致的突然加速。告警超时后恢复巡航。
"""

import logging

from config import AEB_ALERT_HOLD_TIME_S, AEB_ALERT_TIMEOUT_S, AEB_STOP_HOLD_S
from control.state import AebAlertState, LeadContext


class AebAlertManager:
    """管理前车丢失后的告警状态：进入、保持、退出。"""

    def __init__(self):
        self.state = AebAlertState()

    def update(self, now: float, ego_v: float, lead_ctx: LeadContext):
        """每周期调用一次，更新告警状态机。

        状态转移:
          - 有前车 → 重置告警，重新就绪
          - 无前车 + 已就绪 + 超时 → 触发告警
          - 告警超时 → 退出告警，冷却后可再次就绪
        """
        state = self.state

        # 有有效前车 → 重置告警，标记就绪
        if lead_ctx.lead_valid_for_alert:
            state.has_lead = True
            state.armed = True
            state.last_lead_time = now
            state.hold_speed = max(ego_v, 0.0)
            state.stop_hold_until = 0.0
            if state.active:
                logging.info('[AEB_ALERT] exit: lead reacquired')
                state.active = False
            return

        # 无前车时处理告警逻辑
        if not state.active and now >= state.cooldown_until:
            # 速度极低或前车速度无效 → 不触发告警
            if ego_v < 0.5 or lead_ctx.lead_speed_invalid_for_alert:
                state.has_lead = False
                state.armed = False
                state.last_lead_time = now
                state.hold_speed = 0.0
                state.stop_hold_until = 0.0
            # 已就绪 + 超时 → 触发告警
            elif state.armed and (now - state.last_lead_time) > AEB_ALERT_TIMEOUT_S:
                state.active = True
                state.start_t = now
                state.armed = False
                state.hold_speed = max(ego_v, 0.0)
                if ego_v < 0.3:
                    # 近乎停车时额外保持
                    state.stop_hold_until = now + AEB_STOP_HOLD_S
                logging.warning('[AEB_ALERT] activated: lost lead, hold current speed')
            return

        # 告警保持超时 → 退出告警
        if state.active and (now - state.start_t) > AEB_ALERT_HOLD_TIME_S:
            logging.info('[AEB_ALERT] exit: timeout, resume normal')
            state.active = False
            state.has_lead = False
            state.hold_speed = 0.0
            state.stop_hold_until = 0.0
            state.cooldown_until = now + 2.0