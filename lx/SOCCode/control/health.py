#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""控制环路门控：汇总各路感知就绪状态与主备存活状态，
决定本周期是否进入控制解算。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ControlHealth:
    """控制环路健康状态汇总。"""
    peer_active: bool          # 主备心跳是否正常（主机永远为 True）
    ego_ready: bool            # 自车位姿是否就绪（已接收 且 未卡帧）
    road_ready: bool           # 道路航向是否就绪（已接收 且 未卡帧）
    lead_ready: bool           # 前车数据是否已接收
    lane_offset_ready: bool    # 车道偏移是否已接收
    ego_stale: bool = False    # 自车位姿是否卡帧（已接收过但停更）
    road_stale: bool = False   # 道路航向是否卡帧（已接收过但停更）
    lead_cls_stale: bool = False  # /car{N}_class 话题陈旧（不影响 control_active，仅供告警/遥测）

    @property
    def control_active(self) -> bool:
        """控制激活条件：主备存活 + 自车就绪 + 道路就绪。"""
        return self.peer_active and self.ego_ready and self.road_ready


def evaluate_control_health(peer_active: bool,
                            now: float,
                            ego_received: bool,
                            ego_last_rx: float,
                            road_received: bool,
                            road_last_rx: float,
                            lead_ready: bool,
                            lane_offset_ready: bool,
                            stale_timeout_s: float,
                            lead_cls_last_rx: float = -1e9,
                            lead_cls_stale_timeout_s: float = 0.0) -> ControlHealth:
    """构建 ControlHealth。

    ego/road 的 received 标志一旦置位永不复位，单靠它无法发现话题停更。
    这里额外用 now - last_rx > stale_timeout_s 判定卡帧：卡帧时 ego_ready /
    road_ready 视为 False，使控制环走已有的感知中断降级路径（轻制动）。

    lead_cls 不参与降级：只标记 lead_cls_stale，由调用方限频告警 / 写遥测；
    保留最后一次 cls 比强制降到 UNKNOWN 更接近实际威胁（行人 class 话题
    临时丢一帧不应让 AEB 退化到车辆阈值）。
    lead_cls_stale_timeout_s<=0 时跳过本检查（多目标未启用）。
    """
    ego_stale = ego_received and (now - ego_last_rx) > stale_timeout_s
    road_stale = road_received and (now - road_last_rx) > stale_timeout_s
    lead_cls_stale = (
        lead_cls_stale_timeout_s > 0.0
        and lead_ready
        and (now - lead_cls_last_rx) > lead_cls_stale_timeout_s
    )
    return ControlHealth(
        peer_active=peer_active,
        ego_ready=ego_received and not ego_stale,
        road_ready=road_received and not road_stale,
        lead_ready=lead_ready,
        lane_offset_ready=lane_offset_ready,
        ego_stale=ego_stale,
        road_stale=road_stale,
        lead_cls_stale=lead_cls_stale,
    )