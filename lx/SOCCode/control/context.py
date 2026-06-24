#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""控制子包数据模型。

包含控制环路中各个阶段共享的状态与上下文数据类，
以及各算法管理器的回调入口包装函数。
"""

import copy
import threading
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from config import ACC_KA, ACC_KD, ACC_KI, ACC_KV, K_PSI_D, K_PSI_I, K_PSI_P

if TYPE_CHECKING:
    from control.aeb_alert import AebAlertManager
    from control.curve_hold import CurveHoldManager
    from control.lead_tracking import LeadTracker
    from control.overtake import OvertakeManager
    from lateral import LaneWidthEstimator
    from longitudinal import LonSmoothing, LongitudinalController


@dataclass
class VehicleSignals:
    """来自 ROS 话题的车辆感知信号，每个回调更新对应字段。"""
    ego_x: float = 0.0                     # 自车 X 坐标
    ego_y: float = 0.0                     # 自车 Y 坐标
    ego_yaw: float = 0.0                   # 自车航向角 (rad)
    ego_v: float = 0.0                     # 自车速度 (m/s)
    lead_x: float = 0.0                    # 前车 X 坐标
    lead_y: float = 0.0                    # 前车 Y 坐标
    lead_yaw: float = 0.0                  # 前车航向角 (rad)
    lead_v: float = 0.0                    # 前车速度 (m/s)
    # 主前车的 actor class（来自 MultiTargetTracker.SelectionResult.cls 或 car2_class 话题）。
    # 0=UNKNOWN（缺省），1=VEHICLE，2=OBSTACLE，3=PEDESTRIAN；AEB 按 class 取差异化 TTC 阈值。
    # 多目标未启用 / class 话题未发布时维持 0，AEB 走 UNKNOWN 系数 1.0 = 与原行为一致。
    lead_cls: int = 0
    road_psi: float = 0.0                  # 道路航向角 (rad)
    lane_offset: float = 0.0               # 车道横向偏移 (m)
    ego_received: bool = False              # 是否已收到自车位姿
    ego_psi_received: bool = False          # 是否已收到自车航向话题
    lead_received: bool = False             # 是否已收到前车位姿
    road_received: bool = False             # 是否已收到道路航向
    lane_offset_received: bool = False      # 是否已收到车道偏移
    lead_last_rx_time: float = 0.0          # 前车位姿最近接收时刻
    lead_v_last_rx_time: float = 0.0        # 前车速度最近接收时刻
    lane_offset_last_rx: float = 0.0        # 车道偏移最近接收时刻
    ego_last_rx: float = 0.0                # 自车位姿最近接收时刻（卡帧检测）
    road_last_rx: float = 0.0               # 道路航向最近接收时刻（卡帧检测）
    # /car{N}_class 话题最近接收时刻：超过 LEAD_CLASS_STALE_TIMEOUT_S 视为 class
    # 信息陈旧（仅遥测/限频告警，不主动降级 cls 值——见 health.py 注释）。
    lead_cls_last_rx_time: float = -1e9

    # 跨线程同步锁：保护 snapshot() 拷贝与 ROS 回调写入的一致性。
    # ROS 回调线程写入 self.signals，控制循环线程读取。
    # Python GIL 保证单个 float/bool 赋值原子性，但不保证多字段
    # 一致性（如 ego_x=N 帧、ego_y=N-1 帧）。锁将竞争窗口从整个
    # 10ms 周期缩小到微秒级的 copy 操作，保证快照内所有字段来自
    # 同一帧或相邻帧。
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False, compare=False)

    def __enter__(self):
        """上下文管理器入口：获取信号锁，保护多字段原子写入。"""
        self._lock.acquire()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口：释放信号锁。"""
        self._lock.release()
        return False

    def __getstate__(self):
        """pickle 支持：threading.Lock 在 Python 3.6 不可 pickle，临时剥离。

        锁步守护进程（lockstepd）需要序列化 VehicleSignals 到 TCP 另一侧。
        守护进程为单线程，不竞争锁，反序列化后重建新锁即可。
        """
        state = self.__dict__.copy()
        state.pop('_lock', None)
        return state

    def __setstate__(self, state):
        """pickle 反序列化后重建锁。"""
        self.__dict__.update(state)
        self._lock = threading.Lock()

    def snapshot(self):
        """返回当前状态的浅拷贝快照，供控制循环单周期使用。

        在锁保护下执行 copy.copy()，确保拷贝期间 ROS 回调不会
        修改任何字段，快照内所有字段来自同一帧或相邻帧。
        """
        with self._lock:
            return copy.copy(self)


@dataclass
class ControlGains:
    """运行时可调控制增益。"""
    lat_kp: float = K_PSI_P
    lat_ki: float = K_PSI_I
    lat_kd: float = K_PSI_D
    acc_kd: float = ACC_KD
    acc_ki: float = ACC_KI
    acc_kv: float = ACC_KV
    acc_ka: float = ACC_KA


@dataclass
class ControlMemory:
    """控制环需要跨周期保持的内部状态。

    每个 100Hz 周期读写这些字段，实现积分器、滤波器等有状态算法。
    """
    dt: float                               # 控制周期 (s)
    cycle_count: int = 0                    # 周期计数器
    psi_i_term: float = 0.0                 # 航向 PID 的 I 项累积
    psi_prev_err: float = 0.0               # 航向 PID 的上一拍误差
    filtered_road_psi: float = 0.0           # 低通滤波后的道路航向
    prev_road_psi: float = 0.0              # 上一拍道路航向（用于计算转向率）
    last_delta: float = 0.0                 # 上一拍方向盘转角（用于变化率限制）
    filtered_cte: float = 0.0               # 低通滤波后的横向偏移
    cte_prev: float = 0.0                   # 上一拍 CTE（用于微分）
    filtered_curv: float = 0.0              # 低通滤波后的曲率
    filtered_v_tgt: float = 0.0             # 低通滤波后的目标速度
    filtered_lead_v_proj: float = 0.0       # 低通滤波后的前车投影速度
    last_lead_v_proj: float = 0.0           # 上一拍前车投影速度
    last_lead_v_time: float = 0.0           # 上一拍前车速度时间戳
    filtered_lead_accel: float = 0.0         # 低通滤波后的前车加速度
    last_curve_t: float = -1e9              # 上次检测到弯道的时间
    last_acc_has_lead: bool = False          # 上周期 ACC 是否有前车
    last_lead_reacq_t: float = -1e9         # 上次重新获取前车的时间
    filtered_lon: float = 0.0               # 低通滤波后的纵向指令
    acc_acquire_ff_clamp_logged: bool = False  # 前车重获制动夹紧日志标记
    aeb_full_confirm_count: int = 0          # AEB 全制动确认计数
    lane_safe_margin: float = 0.0            # 车道安全余量 (m)
    lane_warn_margin: float = 0.0           # 车道预警余量 (m)
    lane_hard_margin: float = 0.0            # 车道硬边界余量 (m)
    gains: ControlGains = field(default_factory=ControlGains)  # 运行时控制增益
    # 横向控制帧门控：仿真端以 20Hz 发布感知，本循环以 100Hz 运行，
    # 仅在道路航向有新帧时推进横向有状态计算，其余拍沿用上一帧结果。
    lat_last_road_rx: float = -1.0           # 上次已处理的道路航向帧接收时刻
    lat_last_update_t: float = -1.0          # 上次横向更新时刻（用于真实 dt）
    lat_cached_ctx: object = None            # 上一帧 LateralContext（帧间沿用）
    # 边界修正与 road_psi 帧门控解耦：边界修正只依赖 lane_offset，
    # 在 road_psi 无新帧、但 lane_offset 有新帧的拍上单独刷新边界。
    lat_base_ctx: object = None              # 上一道路帧的"未叠加边界"LateralContext
    lat_base_delta: float = 0.0              # 上一道路帧"叠加边界前"的方向盘转角 (rad)
    lat_frame_dt: float = 0.0                # 上一道路帧的真实 dt（边界限幅/upd_psi 复用）
    lat_last_lane_rx: float = -1.0           # 上次已处理的车道偏移帧接收时刻
    # ── 超车（双车道）目标偏移 ──
    # 超车状态机写入的目标横向偏移：0 表示沿路径（右车道）行驶，
    # +OVT_LANE_OFFSET_M 表示切到左车道。横向控制器用 (cte - target_lane_offset)
    # 作为追踪误差，边界判定也使用偏移后的相对值。
    target_lane_offset: float = 0.0
    # 超车 ACTIVE 阶段抑制前车：把 ACC 退化为巡航以便加速绕过停止前车。
    suppress_lead_for_overtake: bool = False


@dataclass(frozen=True)
class LateralContext:
    """横向控制单周期计算结果，传递给纵向模块和主循环。"""
    dyn_prev: float = 0.0                   # 自适应预览时间 (s)
    rrate: float = 0.0                       # 道路航向变化率 (rad/s)
    prev_psi: float = 0.0                   # 预览航向角 (rad)
    raw_curv: float = 0.0                    # 原始曲率 (1/m)
    curv_guard: float = 0.0                  # 曲率保护值（= max(|滤波|, |原始|)）
    in_curve: bool = False                   # 是否处于弯道
    delta: float = 0.0                       # 最终方向盘转角 (rad)
    delta_ff: float = 0.0                    # 曲率前馈方向盘转角分量 (rad)
    delta_cte: float = 0.0                   # CTE 修正方向盘转角分量 (rad)
    boundary_delta: float = 0.0             # 边界修正转角 (rad)
    boundary_brake: float = 0.0             # 边界制动力 (m/s²)
    boundary_warn: bool = False              # 是否触发边界预警
    raw_cte: float = 0.0                     # 原始横向偏移 (m)
    cur_off: float = 0.0                     # 当前车道偏移 (m)
    upd_psi: float = 0.0                     # 预测下一拍航向角 (rad)


@dataclass(frozen=True)
class ControlManagers:
    """各算法管理器的集合，传入控制策略计算函数。"""
    lane_est: "LaneWidthEstimator"
    lead_tracker: "LeadTracker"
    aeb_alert: "AebAlertManager"
    curve_hold: "CurveHoldManager"
    lon_ctrl: "LongitudinalController"
    lon_smooth: "LonSmoothing"
    overtake: "OvertakeManager" = None
    ml_bridge: object = None                 # MlBridge 实例（可选，ML_ENABLED 时创建）
    lateral_model: object = None             # 可选模型化横向控制器，None=PID 路径
    comfort_layer: object = None             # 可选舒适层，None=legacy 平滑路径
    # 前车状态估计器（可选，LEAD_ESTIMATOR='kalman' 时创建）。None=走 legacy
    # 有限差分加速度路径，与改造前逐字节一致。见 control/lead_estimator.py。
    lead_estimator: object = None
