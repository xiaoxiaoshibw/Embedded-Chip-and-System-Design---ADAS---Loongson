#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""超车状态机：检测前方静止前车 → 等待确认 → 切换至左车道完成超车 → 返回右车道。

工作前提（与赛道几何耦合）：
  - 道路为双车道，自车默认行驶在右车道（路径 path = 右车道中心线）。
  - heng_error/lane_offset 的符号约定为 "左正右负"（与 chart_94 一致），
    因此左车道中心相对路径的偏移量为 +OVT_LANE_OFFSET_M。

状态机：
  IDLE     正常 ACC，target_lane_offset=0
  WAIT     自车与前车均静止且距离接近，启动确认计时；前车恢复或自车开始运动则取消
  ACTIVE   切换到左车道，期间抑制前车（让 ACC 退化为巡航以加速通过停止前车）
  PASSING  已基本进入左车道，ACC 自动释放前车（lead_lat_gate 之外），不再抑制
  RETURN   超过前车足够距离后，目标偏移归零，平滑回到右车道
  IDLE 复位完成

只对 ControlMemory.target_lane_offset 与 ControlMemory.suppress_lead_for_overtake
做副作用，主管线其它阶段对存量字段保持向后兼容。
"""

import logging
import math
from dataclasses import dataclass

from common import is_finite
from config import (
    LANE_DEFAULT_WIDTH,
    OVT_CONFIRM_TIME_S,
    OVT_EGO_STILL_V,
    OVT_LANE_OFFSET_M,
    OVT_LANE_SHIFT_RATE_M_S,
    OVT_LEAD_LONG_STILL_S,
    OVT_LEAD_PASSED_FWD_M,
    OVT_LEAD_STILL_V,
    OVT_RESUME_LEAD_V,
    OVT_RETURN_DONE_M,
    OVT_SHIFT_DONE_M,
    OVT_TRIGGER_DIST_M,
    OVT_TRIGGER_MIN_DIST_M,
)


# 状态字符串常量，集中放在一处避免散落字面量
S_IDLE = 'idle'
S_WAIT = 'wait'
S_ACTIVE = 'active'
S_PASSING = 'passing'
S_RETURN = 'return'


@dataclass
class OvertakeState:
    """超车状态机跨周期状态。"""
    state: str = S_IDLE
    state_enter_t: float = -1e9          # 当前状态进入的时间
    ego_x_at_active: float = 0.0         # 进入 ACTIVE 时的自车 X
    ego_y_at_active: float = 0.0         # 进入 ACTIVE 时的自车 Y
    lead_x_at_active: float = 0.0        # 进入 ACTIVE 时的前车 X（停止位置）
    lead_y_at_active: float = 0.0        # 进入 ACTIVE 时的前车 Y
    lead_yaw_at_active: float = 0.0      # 进入 ACTIVE 时前车朝向（用于纵向投影）
    # 前/自车持续静止开始时刻：跌破阈值时打点，恢复时清零。
    # 用来区分"前车长期阻塞"（应触发超车）和"AEB 短停"（不应触发）。
    lead_still_since: float = -1e9
    ego_still_since: float = -1e9


class OvertakeManager:
    """超车决策与目标偏移生成。

    依赖：
      - 上一拍 LeadTracker.state（用于读取已确认的前车纵向距离 last_lead_x_rel）。
      - 当前 signals（自车速度/位置、前车速度/位置）。
    输出：
      - target_lane_offset (m)：写入 memory.target_lane_offset。
      - suppress_lead_for_overtake (bool)：写入 memory.suppress_lead_for_overtake，
        在 ACTIVE 状态切换初期短暂抑制前车，使 ACC 退化为巡航。
    """

    def __init__(self):
        self.state = OvertakeState()

    def _enter(self, new_state: str, now: float, signals=None):
        """切换到新状态并记录关键变量。"""
        old = self.state.state
        self.state.state = new_state
        self.state.state_enter_t = now
        if new_state == S_ACTIVE and signals is not None:
            self.state.ego_x_at_active = signals.ego_x
            self.state.ego_y_at_active = signals.ego_y
            self.state.lead_x_at_active = signals.lead_x
            self.state.lead_y_at_active = signals.lead_y
            self.state.lead_yaw_at_active = signals.lead_yaw
        logging.info('[OVERTAKE] %s -> %s (t=%.2f)', old, new_state, now)

    def _passed_lead(self, signals) -> bool:
        """通过把 (ego - lead_at_active) 投影到 lead 方向上判断是否已超过前车。

        在弯道上简单用 X 比较会出错（坐标会绕回），故用 lead 朝向上的纵向投影。
        前车停止后 lead_x/y/yaw 不再变化，此投影是唯一可靠的"超过"判据。
        """
        s = self.state
        dx = signals.ego_x - s.lead_x_at_active
        dy = signals.ego_y - s.lead_y_at_active
        if not (is_finite(dx) and is_finite(dy)):
            return False
        fwd = dx * math.cos(s.lead_yaw_at_active) + dy * math.sin(s.lead_yaw_at_active)
        return fwd >= OVT_LEAD_PASSED_FWD_M

    def update(self, now, signals, memory, lead_tracker_state) -> None:
        """每周期调用一次：根据感知与上一拍 LeadTracker 状态推进状态机。

        参数:
            now: 单调时钟
            signals: VehicleSignals
            memory: ControlMemory（输出 target_lane_offset / suppress_lead_for_overtake）
            lead_tracker_state: LeadTrackerState（用于读取上一拍确认的前车距离）
        """
        s = self.state
        ego_v = abs(signals.ego_v) if is_finite(signals.ego_v) else 0.0
        lead_v = abs(signals.lead_v) if (signals.lead_received and is_finite(signals.lead_v)) else 0.0

        # 前车"持续静止"打点：用于区分"前车长期阻塞"与"前车刚被刹停"。
        # 阈值收紧到 OVT_LEAD_STILL_V，且只在前车确实被感知到（lead_received）时计时。
        if signals.lead_received and lead_v < OVT_LEAD_STILL_V:
            if s.lead_still_since < 0:
                s.lead_still_since = now
        else:
            s.lead_still_since = -1e9
        lead_long_still = (
            s.lead_still_since > 0
            and (now - s.lead_still_since) >= OVT_LEAD_LONG_STILL_S
        )

        # 自车"持续静止"打点：AEB/ACC 把车短暂刹停的场景里 ego 只会停几秒，
        # 要求 ego 也持续静止 OVT_LEAD_LONG_STILL_S 秒以上才考虑超车，
        # 把"短暂跟车停"和"前方长期阻塞"区分开。
        if ego_v < OVT_EGO_STILL_V:
            if s.ego_still_since < 0:
                s.ego_still_since = now
        else:
            s.ego_still_since = -1e9
        ego_long_still = (
            s.ego_still_since > 0
            and (now - s.ego_still_since) >= OVT_LEAD_LONG_STILL_S
        )

        # 上一拍确认的前车纵向距离；首拍 last_lead_x_rel=0 不会触发（受 OVT_TRIGGER_MIN_DIST_M 保护）
        prev_lead_dist = lead_tracker_state.last_lead_x_rel
        had_lead_recently = (
            (now - lead_tracker_state.last_confirmed_lead_t) <= 0.5
            and OVT_TRIGGER_MIN_DIST_M <= prev_lead_dist <= OVT_TRIGGER_DIST_M
        )

        # 当前已经在哪条"车道"附近：用横向控制器低通后的 filtered_cte（慢变量），
        # 而非未滤波的 signals.lane_offset，避免噪声尖峰误触发状态转移。
        cur_off = memory.filtered_cte

        # ── 状态转移 ──
        if s.state == S_IDLE:
            # 触发等待：前车在前方近距离 + 前车长期静止 + 自车长期停稳。
            # 仅"瞬时静止"会把 AEB 后短停误判成超车场景；要求双方都"长期静止"。
            if had_lead_recently and ego_long_still and lead_long_still:
                self._enter(S_WAIT, now)

        elif s.state == S_WAIT:
            # 取消条件：前车恢复行驶 / 自车开始运动 / 前车消失
            if lead_v > OVT_RESUME_LEAD_V or ego_v > 1.5 or not had_lead_recently:
                self._enter(S_IDLE, now)
            elif (now - s.state_enter_t) >= OVT_CONFIRM_TIME_S:
                # 确认前车真的不动 → 启动超车
                self._enter(S_ACTIVE, now, signals)

        elif s.state == S_ACTIVE:
            # 自车横向偏移已经达到左车道附近 → 进入 PASSING（停止抑制前车）
            if cur_off >= OVT_SHIFT_DONE_M:
                self._enter(S_PASSING, now)
            # 也容忍超时后强制进入下一状态，避免卡死。从静止起步到完成横向迁移
            # 至少要 ego 加速到 ~3 m/s 后行驶 ~5 m 才能产生足够横向位移，
            # 给 25s 的余量适应低速场景。
            elif (now - s.state_enter_t) >= 25.0:
                logging.warning('[OVERTAKE] active phase timeout, advancing')
                self._enter(S_PASSING, now)

        elif s.state == S_PASSING:
            # 在左车道行驶，监测是否已经超过前车（用 lead 朝向上的纵向投影判断）
            if self._passed_lead(signals):
                self._enter(S_RETURN, now)

        elif s.state == S_RETURN:
            # 横向偏移基本归零 → 回到 IDLE
            if abs(cur_off) <= OVT_RETURN_DONE_M:
                self._enter(S_IDLE, now)

        # ── 输出生成 ──
        if s.state in (S_ACTIVE, S_PASSING):
            target = OVT_LANE_OFFSET_M
        else:  # IDLE / WAIT / RETURN 都希望回到/保持右车道
            target = 0.0

        # ACTIVE 与 PASSING 全程都抑制前车：
        #   - ACTIVE：车还在右车道，前车在车前 lead_lat_gate 内；不抑制 ACC 会把车锁死，
        #             既不前进也无法侧移，连带把横向控制器也卡住。
        #   - PASSING：理论上车已经切到左车道，前车应被 lead_lat_gate 自然剔除；
        #             但若 ACTIVE 因横向未到位而超时进入 PASSING（车实际仍在右车道），
        #             停止抑制就会让 ACC 立刻把刚启动的车再次刹停。统一抑制到
        #             "已经超过前车纵向"为止，避免这种竞态。
        suppress = s.state in (S_ACTIVE, S_PASSING)

        # 目标车道偏移按 OVT_LANE_SHIFT_RATE_M_S 爬升到 target，不做瞬时阶跃：
        # 横向控制器以 (filtered_cte - target_lane_offset) 为追踪误差，阶跃会让
        # CTE 误差突变、只能靠转向限速器被动平滑；斜坡参考则物理可跟踪。
        prev_target = memory.target_lane_offset
        max_shift = OVT_LANE_SHIFT_RATE_M_S * memory.dt
        if target > prev_target:
            memory.target_lane_offset = min(target, prev_target + max_shift)
        elif target < prev_target:
            memory.target_lane_offset = max(target, prev_target - max_shift)
        memory.suppress_lead_for_overtake = suppress
