#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Jetson Nano ADAS ROS2 节点。
主机启动：
python3 adas.py --role primary
备机启动：
python3 adas.py --role backup

包含 LKA、ACC、AEB、ESP32 串口通信，以及主备 Nano 的 UDP 心跳切换。
主循环频率 100 Hz。
"""

import math
import logging
import os
import pickle
import signal
import socket
import struct
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64, Int32, String

from config import *
from config import build_arg_parser
from common import (
    RateLimitedCritical,
    clamp,
    is_finite,
    quaternion_to_yaw,
    setup_logging,
    wrap_angle,
)
from heartbeat import (
    HB_STATE_ACTIVE_CONTROL,
    HB_STATE_NO_INPUT_IDLE,
    PeerHeartbeat,
)
from lateral import (
    LaneWidthEstimator,
    LateralSmoothing,
    lane_margins_from_width,
)
from longitudinal import (
    aeb_curv_suppress,
    LonSmoothing,
    LongitudinalController,
)
import runtime
from control.aeb_alert import AebAlertManager
from control.context import ControlManagers, ControlMemory, VehicleSignals
from control.ml_bridge import MlBridge
from control.curve_hold import CurveHoldManager
from control.health import evaluate_control_health
from control.comfort import make_comfort_layer
from control.lead_estimator import make_lead_estimator
from control.lead_tracking import LeadTracker
from control.model_lateral import make_lateral_model_controller
from control.mpc_longitudinal import make_lon_controller
from control.overtake import OvertakeManager
from control.perception import CAR2_ID, PerceptionLayer
from control.serial_protocol import Esp32ControlFrame, build_esp32_payload


from pipeline import run_pure_pipeline
import copy as _copy
from serial_link import Esp32Serial
from telemetry import Telemetry


# ==========================================================
# ROS2 Main Node
# ==========================================================
class AdasNode(Node):

    def __init__(self):
        super().__init__(f'adas_{runtime.NANO_ROLE}')
        setup_logging()
        logging.info('=== ADAS node started role=%s IS_PRIMARY=%s dt=%.3fs ===',
                     runtime.NANO_ROLE, runtime.IS_PRIMARY, 1.0 / LOOP_HZ)

        # 对外发布 Jetson 侧最终控制量，以及 ESP32 回读状态，便于联调与监控。
        self.pub_psi = self.create_publisher(Float64, TOPIC_JETSON_PSI, 1)
        self.pub_delta = self.create_publisher(Float64, TOPIC_JETSON_DELTA, 1)
        self.pub_brake = self.create_publisher(Float64, TOPIC_JETSON_BRAKE, 1)
        self.pub_offset = self.create_publisher(Float64, TOPIC_JETSON_LANE_OFFSET, 1)
        self.pub_esp_psi = self.create_publisher(Float64, TOPIC_ESP32_PSI, 10)
        self.pub_esp_delta = self.create_publisher(Float64, TOPIC_ESP32_DELTA, 10)
        self.pub_esp_brake = self.create_publisher(Float64, TOPIC_ESP32_BRAKE, 10)
        self.pub_role = self.create_publisher(String, TOPIC_JETSON_ACTIVE_ROLE, 10)
        self.pub_lane_w = self.create_publisher(Float64, TOPIC_JETSON_LANE_WIDTH_EST, 10)
        # 主前车 class 监控话题：俯瞰图可叠加显示，便于人工核对 Simulink 的 class 流转。
        self.pub_lead_cls = self.create_publisher(Int32, TOPIC_JETSON_LEAD_CLS, 10)
        # 主备冗余可用性指示：主机持有备机 watchdog，超时则发布 False。
        # 备机端永远发布 True（备机就是 plan B）。
        self.pub_failover = self.create_publisher(
            Bool, TOPIC_JETSON_FAILOVER_AVAILABLE, 10,
        )

        # 订阅自车、前车、道路和车道线相关观测，统一写入 self.signals。
        # 高频感知话题改用 sensor_data QoS（BEST_EFFORT + KEEP_LAST 1），
        # DDS 不再为这些消息做可靠重传，避免在 DDS 拥塞时累积排队导致控制环阻塞。
        sensor_qos = qos_profile_sensor_data
        self.create_subscription(Pose, TOPIC_CAR1_XY, self._ego_pose_callback, sensor_qos)
        self.create_subscription(Float64, TOPIC_CAR1_PSI, self._ego_psi_callback, sensor_qos)
        self.create_subscription(Pose, TOPIC_CAR2_XY, self._lead_pose_callback, sensor_qos)
        self.create_subscription(Float64, TOPIC_CAR1_V, self._ego_v_callback, sensor_qos)
        self.create_subscription(Float64, TOPIC_CAR2_V, self._lead_v_callback, sensor_qos)
        self.create_subscription(Float64, TOPIC_ROAD_PSI, self._road_psi_callback, sensor_qos)
        self.create_subscription(Float64, TOPIC_HENG_ERROR, self._lane_offset_callback, sensor_qos)
        # 运行时增益热更新（低频指令，用默认可靠 QoS）。
        self.create_subscription(String, TOPIC_SET_PARAM, self._set_param_callback, 10)

        # 外设/算法管理器初始化：串口链路、主备心跳、车道宽估计、目标车跟踪等。
        self.esp32    = Esp32Serial()
        self.peer_hb  = PeerHeartbeat()
        self.telemetry = Telemetry(runtime.NANO_ROLE)
        self.lane_est = LaneWidthEstimator(LOOP_HZ)
        self.lead_tracker = LeadTracker()
        self.aeb_alert = AebAlertManager()
        self.curve_hold = CurveHoldManager()
        self.overtake = OvertakeManager()

        self.lon_ctrl = make_lon_controller(dt=1.0 / float(LOOP_HZ))
        self.lon_smooth = LonSmoothing(dt=1.0 / float(LOOP_HZ))
        self.lead_estimator = make_lead_estimator()
        self.lateral_model = make_lateral_model_controller()
        self.comfort_layer = make_comfort_layer(dt=1.0 / float(LOOP_HZ))
        # 横向平滑层：与 LonSmoothing 对称的最外层防跳变兜底。
        # lateral_controller 内部仍有 MAX_DELTA_RATE 等硬限位；此处只做
        # 平滑 + takeover guard 期收紧。
        self.lat_smooth = LateralSmoothing(dt=1.0 / float(LOOP_HZ))
        self.signals = VehicleSignals()
        self.memory = ControlMemory(dt=1.0 / float(LOOP_HZ))
        (self.memory.lane_safe_margin,
         self.memory.lane_warn_margin,
         self.memory.lane_hard_margin) = lane_margins_from_width(LANE_DEFAULT_WIDTH)
        # ML 推理桥接（可选，受 config.ML_ENABLED 控制）
        self.ml_bridge = MlBridge() if ML_ENABLED else None
        self.managers = ControlManagers(
            lane_est=self.lane_est,
            lead_tracker=self.lead_tracker,
            aeb_alert=self.aeb_alert,
            curve_hold=self.curve_hold,
            lon_ctrl=self.lon_ctrl,
            lon_smooth=self.lon_smooth,
            overtake=self.overtake,
            ml_bridge=self.ml_bridge,
            lateral_model=self.lateral_model,
            comfort_layer=self.comfort_layer,
            lead_estimator=self.lead_estimator,
        )

        # ── 单板软件双核锁步（可选，默认关）：通过 lockstepd 独立进程
        # 在 core 2 上重算并比对控制输出。best-effort。
        self.lockstep = None
        self._last_lockstep_log_t = 0.0
        if LOCKSTEP_ENABLED:
            try:
                from control.lockstepd_client import LockstepdClient
                self.lockstep = LockstepdClient(
                    host=LOCKSTEPD_HOST, port=LOCKSTEPD_PORT,
                    delta_eps=LOCKSTEP_DELTA_EPS,
                    lon_eps=LOCKSTEP_LON_EPS,
                    debounce_n=LOCKSTEP_DEBOUNCE_N,
                    inject=LOCKSTEP_INJECT,
                    inject_delta=LOCKSTEP_INJECT_DELTA,
                )
                logging.info('[LOCKSTEP] 双核锁步已启用：lockstepd=%s:%d inject=%s',
                             LOCKSTEPD_HOST, LOCKSTEPD_PORT, LOCKSTEP_INJECT)
            except Exception as exc:
                self.lockstep = None
                logging.warning('[LOCKSTEP] 启用失败（忽略，控制不受影响）：%r', exc)

        # 感知层（PerceptionLayer）始终启用：把 car2 + 可选的 car3..carN 当成
        # 一组毫米波雷达目标持续监听，每周期产出 PerceptionFrame 共享给下游
        # （Overtake、未来避障 / AEB 监控）。
        # - MULTI_TARGET_COUNT==1：只跟 car2；本层不写回 signals.lead_*（仍由
        #   car2 callback 直接写），与改造前字节级一致。仅为下游/遥测提供视图。
        # - MULTI_TARGET_COUNT>1：订阅 car3..carN（行人也走这条路径，仅
        #   /car{N}_class=3），本层在 _select_primary_lead 中承担主前车选举与
        #   signals.lead_* 写回（替换原 MultiTargetTracker.select）。
        self.perception = PerceptionLayer()
        # 行人/障碍单目标场景同样支持：Simulink 端发 /car2_class 即生效，
        # 未发布时 lead_cls 维持 0=UNKNOWN，AEB 查表回退乘子 1.0，行为不变。
        self.create_subscription(
            Int32, TOPIC_CAR2_CLASS,
            self._car2_class_cb, qos_profile_sensor_data)
        if MULTI_TARGET_COUNT > 1:
            for k in range(3, MULTI_TARGET_COUNT + 2):
                self.create_subscription(
                    Pose, MULTI_TARGET_TOPIC_XY_FMT.format(k),
                    self._make_mt_pose_cb(k), qos_profile_sensor_data)
                self.create_subscription(
                    Float64, MULTI_TARGET_TOPIC_V_FMT.format(k),
                    self._make_mt_v_cb(k), qos_profile_sensor_data)
                self.create_subscription(
                    Int32, MULTI_TARGET_TOPIC_CLASS_FMT.format(k),
                    self._make_mt_class_cb(k), qos_profile_sensor_data)
            logging.info('[PERCEPTION] multi-target enabled: tracking car2..car%d',
                         MULTI_TARGET_COUNT + 1)

        # 主备接管保护窗口截止时刻；监控期内对 lon 施加更严的变化率限制，
        # 避免主机最后一帧与备机第一帧之间的非物理跳变。delta 的接管期收紧
        # 通过 self.lat_smooth.update(max_rate_override=TAKEOVER_DELTA_RATE) 完成，
        # 不再需要单独的 _takeover_prev_delta 字段。
        self._takeover_guard_until: float = 0.0
        # 保护窗口期间用于做 lon 限幅参考的上一帧输出（接管瞬间初始化为种子）。
        self._takeover_prev_lon: float = 0.0
        # 接管时主机最后一帧是否处于 AEB；为 True 则保护期内用更宽松的衰减
        # 速率（TAKEOVER_LON_RATE_AEB_RELEASE），避免备机继承 200ms 全制动。
        self._takeover_aeb_seed: bool = False
        # 接管时主前车 class（来自心跳 CLS 字段，可选）；行人/障碍走 vulnerable 速率。
        self._takeover_seed_cls: int = 0
        # 上一次完成接管初始化的时刻，用于 flapping 冷却判定
        self._last_takeover_init_t: float = 0.0
        # 感知层选举出的主车 ID：用于检测切主事件并复位 lead 相关滤波。
        self._last_lead_tid = None
        # 最新一帧 PerceptionFrame，供 pipeline / 遥测 / 未来避障读取。
        self._last_frame = None

        # ROS 话题发布失败日志限频：避免 DDS 异常时狂刷日志
        self._publish_err_last_log_t: float = 0.0
        self._publish_err_count: int = 0

        # 连续异常计数器：超过阈值触发紧急停车
        self._consecutive_errors: int = 0
        # 感知数据中断计时：记录最后一次成功进入控制解算的时刻。
        # None = 自启动以来从未激活过（上电待机），用于区分"冷启动无数据=静默待机"
        # 与"行驶中丢数据=主动轻制动"。一旦激活即写入墙钟时刻。
        self._last_control_active_t = None  # type: Optional[float]
        # 紧急停车 / 感知中断制动 限频时间戳
        self._last_emergency_stop_t: float = 0.0
        self._last_sensor_brake_t: float = 0.0
        # 静默待机：保持帧限频时间戳 / 是否处于待机 / 待机提示日志限频
        self._last_standby_tx_t: float = 0.0
        self._in_standby: bool = False
        self._standby_log_t: float = 0.0
        # 控制输出 NaN 防护限频：持续 NaN 时 100Hz 会刷屏，按窗口聚合上报
        self._last_nan_tx_log_t: float = 0.0
        self._nan_tx_count: int = 0
        # 周期耗时监控：当前窗口内的最大耗时与超预算次数，限频上报
        self._loop_slow_count: int = 0
        self._loop_max_elapsed_s: float = 0.0
        self._last_slow_log_t: float = 0.0
        # CRITICAL 限频器：takeover / emergency_stop 类事件首次 CRITICAL，
        # 1s 内重复事件降为 WARNING，避免 flapping 时告警刷屏
        self._critical_log = RateLimitedCritical(window_s=1.0)

        # 以固定 dt 进入控制循环，当前设计目标为 100 Hz。
        self.timer = self.create_timer(self.memory.dt, self.control_loop)

    # ----------------------------------------------------------
    def _publish_role_status(self, active: bool):
        # 根据当前主备状态发布角色字符串，供外部系统判断谁在真正输出控制。
        # active 由控制循环统一计算并传入，避免单周期内多次调用 is_active()
        # 引起的重复锁竞争与边沿检测语义混乱。
        role_text = (
            'primary'
            if runtime.IS_PRIMARY
            else ('secondary_active' if active else 'secondary_standby')
        )
        m = String()
        m.data = role_text
        self.pub_role.publish(m)
        # 同处发布主备冗余可用性：主机由 backup watchdog 决定，
        # 备机始终发布 True。
        try:
            fm = Bool()
            fm.data = bool(self.peer_hb.is_failover_available())
            self.pub_failover.publish(fm)
        except Exception:
            pass

    def _publish_float64(self, pub, value: float):
        """发布单个 Float64 话题消息。"""
        m = Float64()
        m.data = float(value)
        pub.publish(m)

    def _publish_outputs(self, psi_tx: float, delta_tx: float, lon_tx: float,
                         cur_off: float, cur_lane_width: float):
        # ROS 话题发布与控制闭环无关，因此这里吞掉异常，避免影响主循环实时性。
        try:
            for pub, val in (
                (self.pub_psi, psi_tx),
                (self.pub_delta, delta_tx),
                (self.pub_brake, lon_tx),
            ):
                self._publish_float64(pub, val)
            self._publish_float64(self.pub_offset, cur_off)
            for pub, val in (
                (self.pub_esp_psi, self.esp32.esp_psi),
                (self.pub_esp_delta, self.esp32.esp_delta),
                (self.pub_esp_brake, self.esp32.esp_brake),
            ):
                self._publish_float64(pub, val)
            self._publish_float64(self.pub_lane_w, cur_lane_width)
            # 主前车 class 监控；Int32 数据字段名为 data。
            cls_msg = Int32()
            cls_msg.data = int(self.signals.lead_cls)
            self.pub_lead_cls.publish(cls_msg)
        except Exception as e:
            # DDS 拓扑断开会让 publish 持续抛错，限频到每秒一次并带计数
            self._publish_err_count += 1
            now = time.monotonic()
            if now - self._publish_err_last_log_t >= 1.0:
                logging.warning(
                    'publish error (count=%d in last window): %s',
                    self._publish_err_count, e,
                )
                self._publish_err_last_log_t = now
                self._publish_err_count = 0

    def _build_esp32_payload(self, ttc_tx: float, dist_tx: float,
                             psi_tx: float, delta_tx: float, speed_tx: float,
                             lon_tx: float, cur_off: float,
                             lead_v_proj: float, min_safe_dist: float) -> bytes:
        # 统一在这里组帧，确保 ROS 输出与串口发送使用同一份控制结果。
        return build_esp32_payload(
            Esp32ControlFrame(
                ttc=ttc_tx,
                dist=dist_tx,
                psi=psi_tx,
                delta=delta_tx,
                speed=speed_tx,
                lon=lon_tx,
                offset=cur_off,
                lead_v_proj=lead_v_proj,
                min_safe_dist=min_safe_dist,
                lane_warn_margin=self.memory.lane_warn_margin,
                lane_hard_margin=self.memory.lane_hard_margin,
                filtered_curv=self.memory.filtered_curv,
            )
        )

    def _handle_takeover_edge(self, now: float):
        """检测主备接管边沿，从主机最后一帧种子初始化控制状态。

        在 False→True 翻转的那一拍：
          1. 从 peer_hb 取出主机最后广播的 PSI/DELTA/ACC；
          2. 校验种子有限性 + 范围；非法字段回落到当前内部状态；
          3. 把种子写入 lon_smooth 与 memory.last_delta，避免从零起算；
          4. 启动一个短窗口（TAKEOVER_GUARD_DURATION_S），窗口内对 lon/delta
             单独施加更严的变化率限幅。

        flapping 冷却：距离上次接管初始化不到 TAKEOVER_COOLDOWN_S 时，
        只延长保护窗口，不再重置 lon_smooth，避免内部状态被反复覆盖。
        """
        seed = self.peer_hb.consume_takeover_seed()
        if seed is None:
            return
        psi_seed, delta_seed, lon_seed, aeb_seed, cls_seed = seed

        # NaN/Inf 防护：clamp(NaN, ...) 在 Python 中返回 NaN，必须显式过滤
        if not is_finite(delta_seed):
            delta_seed = self.memory.last_delta
        if not is_finite(lon_seed):
            lon_seed = self._takeover_prev_lon
        if not is_finite(psi_seed):
            psi_seed = 0.0

        # 截断到执行器接受范围，防止主机心跳被异常值污染
        delta_seed = clamp(delta_seed, -MAX_DELTA, MAX_DELTA)
        lon_seed = clamp(lon_seed, -LON_CMD_MAX_DRIVE_ACCEL, LON_CMD_MAX_BRAKE_DECEL)

        # flapping 冷却：上次接管不到 TAKEOVER_COOLDOWN_S，仅延长保护窗，
        # 不再覆盖控制器内部状态
        in_cooldown = (now - self._last_takeover_init_t) < TAKEOVER_COOLDOWN_S
        self._takeover_guard_until = now + TAKEOVER_GUARD_DURATION_S
        # AEB 种子标记跟随最新一次接管（无论是否在 cooldown），让保护窗速率
        # 反映"当前期望从全制动衰减"还是"常规守护"。
        self._takeover_aeb_seed = bool(aeb_seed)
        # cls 种子用于非 AEB 接管时选 vulnerable 衰减速率（行人/障碍）。
        # 心跳缺 CLS 字段时 cls_seed=0=UNKNOWN，与 _is_vulnerable_cls 不匹配，
        # 行为退化到 TAKEOVER_LON_RATE，与改造前一致。
        self._takeover_seed_cls = int(cls_seed)
        if in_cooldown:
            # flapping：不重置控制器内部状态，但把保护窗限幅参考刷新为
            # lon_smooth 的限幅起点（_prev，不是 _filtered），与下一拍 update()
            # 内部限幅起点同源，避免基准与起点错位引入预期外阶跃。
            self._takeover_prev_lon = self.lon_smooth.prev
            logging.warning(
                '[TAKEOVER] flapping detected (%.2fs since last init), extending guard only',
                now - self._last_takeover_init_t,
            )
            return

        self.lon_smooth.reset(lon_seed)
        if self.comfort_layer is not None:
            self.comfort_layer.reset(lon_seed)
        self.lat_smooth.reset(delta_seed)
        self.memory.last_delta = delta_seed
        # M-05: takeover 时重置横向 PID 积分项，避免主机残留的 psi_i_term
        # 导致备机首帧转向角偏移。主机和备机的道路航向可能有微小差异，
        # 残留积分会在新航向下产生错误的转向力矩。
        self.memory.psi_i_term = 0.0
        self.memory.psi_prev_err = 0.0
        self._takeover_prev_lon = lon_seed
        self._last_takeover_init_t = now
        self._critical_log.log(
            'takeover',
            '[TAKEOVER] seed psi=%.4f delta=%.4f lon=%+.2f aeb=%d cls=%d guard=%.0fms',
            psi_seed, delta_seed, lon_seed, int(self._takeover_aeb_seed),
            self._takeover_seed_cls, TAKEOVER_GUARD_DURATION_S * 1000.0,
        )

    def _takeover_lon_rate(self) -> float:
        """接管保护期内 lon 的变化率上限。三分支语义：
          1. AEB 种子 → AEB_RELEASE（最宽松，让备机快速从全制动衰减）
          2. 行人/障碍 cls 且非 AEB → VULNERABLE（更严，禁止快速放车）
          3. 其他 → 常规 TAKEOVER_LON_RATE
        必须在 _control_loop_impl 和 _apply_takeover_guard 两处复用同一规则，
        否则两层限幅会用不同基准导致末端阶跃。
        """
        if self._takeover_aeb_seed:
            return TAKEOVER_LON_RATE_AEB_RELEASE
        if self._takeover_seed_cls in (ACTOR_CLASS_PEDESTRIAN, ACTOR_CLASS_OBSTACLE):
            return TAKEOVER_LON_RATE_VULNERABLE
        return TAKEOVER_LON_RATE

    def _apply_takeover_guard(self, now: float, lon_tx: float, delta_tx: float):
        """接管保护窗口内对 lon_tx 施加额外的变化率限幅（保险网）。

        lon_tx 的限幅已经通过 lon_smooth.update(max_rate_override=...) 在
        平滑器内部完成，此处不再二次覆盖 lon_smooth 状态，仅作为最终输出
        保险（防止任何遗漏路径绕过 lon_smooth）。

        delta_tx 的接管期收紧已由 self.lat_smooth.update(max_rate_override=...)
        在 _control_loop_impl 中完成，本函数不再触碰 delta_tx。

        AEB 种子接管时使用更宽松的 TAKEOVER_LON_RATE_AEB_RELEASE，避免备机
        被锁在主机临死前的全制动 200ms 不动。
        """
        if now >= self._takeover_guard_until:
            return lon_tx, delta_tx
        dt = self.memory.dt
        lon_rate = self._takeover_lon_rate()
        max_lon_step = lon_rate * dt
        lon_tx = clamp(
            lon_tx,
            self._takeover_prev_lon - max_lon_step,
            self._takeover_prev_lon + max_lon_step,
        )
        self._takeover_prev_lon = lon_tx
        return lon_tx, delta_tx

    def _sync_peer_output(self, psi_tx: float, delta_tx: float, lon_tx: float,
                          aeb_active: bool = False,
                          state: str = HB_STATE_ACTIVE_CONTROL):
        # 主机广播实际控制输出；备机仅上报存活，避免双机同时接管。
        # aeb_active 反映本帧 lon_tx 是否由 AEB 路径产生（lon_ctx.aeb_active），
        # 备机据此在接管瞬间决定保护期速率，避免继承 200ms 全制动。
        if runtime.IS_PRIMARY:
            self.peer_hb.send_hb(psi_tx, delta_tx, lon_tx,
                                 aeb_active=aeb_active,
                                 lead_cls=int(self.signals.lead_cls),
                                 state=state)
        else:
            self.peer_hb.send_backup_alive(True)

    def _sync_idle_primary_heartbeat(self):
        """主机无感知输入时仍发送状态心跳，避免备机误判主机失联。

        这类心跳只表达"主机进程健康但无输入待机"，不武装备机接管门控。
        控制量使用安全保持值，便于备机保留有限种子但不会把 idle 当成可接管
        的 active 控制状态。
        """
        if not runtime.IS_PRIMARY:
            return
        self.peer_hb.send_hb(
            0.0, 0.0, STANDBY_HOLD_BRAKE_CMD,
            aeb_active=False,
            lead_cls=0,
            state=HB_STATE_NO_INPUT_IDLE,
        )

    def _log_cycle_summary(self, lateral_ctx, lead_ctx, lon_ctx,
                           in_curve_hold: bool, ttc_tx: float,
                           cur_lane_width: float, lon_cmd: float,
                           lon_raw_cmd: float):
        # 周期诊断日志：所有标量直接从本周期已构建的 context 对象读取，
        # 避免长位置参数链路导致的静默错位。
        if self.memory.cycle_count % LOG_EVERY_N_CYCLES != 0:
            return
        lead_state = self.lead_tracker.state
        # 超车诊断字段（新增）：tgt_off / 状态名 / suppress 标志，
        # 直接从 memory + manager 读，避免新增传参链路。
        ovt_state = (self.overtake.state.state if self.overtake is not None
                     else 'none')
        ovt_target = float(getattr(self.memory, 'target_lane_offset', 0.0) or 0.0)
        ovt_suppress = bool(getattr(self.memory, 'suppress_lead_for_overtake', False))
        logging.info(
            '[%s] ovt=%s tgt_off=%.2f sup=%s '
            'lead=%s lead_det=%s raw_has_lead=%s acc_valid=%s acc_reject=%s '
            'lead_in_lane_for_acc=%s lead_confirm_count=%d lead_v_proj_pred=%.2f '
            'last_acc_has_lead=%s lane_out_cnt=%d dist_open_cnt=%d '
            'lane_out_release=%s dist_opening_release=%s '
            'lead_acquire_grace_active=%s a_ff_before=%.3f a_ff_after=%.3f '
            'aeb=%s alert=%s ch=%s in_curve=%s cls=%d '
            'dist=%.2f ttc=%.2f v=%.2f lon=%.2f(raw=%.2f) I=%.3f a_ff=%.3f '
            'delta=%.2fdeg(cte%+.2fdeg ff%+.2fdeg bnd%+.2fdeg) '
            'cte=%.3f(raw=%.3f) curv=%.4f '
            'psi=%.4f pt=%.2f rrate=%.4f I=%.2fdeg bwarn=%s '
            'lat_max=%.2f lcf=%d cs=%.2f dsafe=%.2f lv=%.2f '
            'lw=%.2f(lk=%s) wmrn=%.2f whrd=%.2f bl=%d br=%d',
            runtime.NANO_ROLE,
            ovt_state, ovt_target, ovt_suppress,
            lead_ctx.acc_has_lead, lead_ctx.lead_detected, lead_ctx.raw_has_lead,
            lead_ctx.acc_lead_valid, lead_ctx.acc_reject_reason or 'none',
            lead_ctx.lead_in_lane_for_acc, lead_state.lead_confirm_count,
            lead_ctx.predicted_lead_v_proj,
            self.memory.last_acc_has_lead,
            lead_state.acc_lane_out_release_count,
            lead_state.acc_dist_opening_release_count,
            lead_ctx.lane_out_release,
            lead_ctx.dist_opening_release,
            lon_ctx.lead_acquire_grace_active,
            lon_ctx.acc_ff_before,
            lon_ctx.acc_ff_after,
            lon_ctx.aeb_active, self.aeb_alert.state.active,
            in_curve_hold, lateral_ctx.in_curve, self.signals.lead_cls,
            lon_ctx.dist, ttc_tx, self.signals.ego_v, lon_cmd,
            lon_raw_cmd,
            self.lon_ctrl.i_term,
            self.memory.filtered_lead_accel,
            math.degrees(lateral_ctx.delta),
            math.degrees(lateral_ctx.delta_cte),
            math.degrees(lateral_ctx.delta_ff),
            math.degrees(lateral_ctx.boundary_delta),
            self.memory.filtered_cte, lateral_ctx.raw_cte,
            self.memory.filtered_curv,
            lateral_ctx.upd_psi, lateral_ctx.dyn_prev,
            lateral_ctx.rrate, math.degrees(self.memory.psi_i_term),
            lateral_ctx.boundary_warn,
            lead_ctx.lead_lat_max, lead_state.lead_confirm_count,
            aeb_curv_suppress(self.memory.filtered_curv),
            lon_ctx.min_safe_dist, lon_ctx.lead_v_proj,
            cur_lane_width, self.lane_est.is_locked,
            self.memory.lane_warn_margin, self.memory.lane_hard_margin,
            *self.lane_est.sample_counts,
        )

    # ----------------------------------------------------------
    def control_loop(self):
        """主控制循环，100Hz 调用，附带周期耗时监控。"""
        t0 = time.perf_counter()
        try:
            self._control_loop_impl()
            # 成功执行一次后重置连续异常计数
            self._consecutive_errors = 0
        except Exception as e:
            self._consecutive_errors += 1
            logging.error('[CONTROL LOOP] Exception (#%d): %s',
                          self._consecutive_errors, e, exc_info=True)
            # 连续异常超过阈值 → 紧急停车
            if self._consecutive_errors >= CTRL_CONSECUTIVE_ERROR_LIMIT:
                self._send_emergency_stop()
            # 确保周期计数器继续递增，避免状态异常
            self.memory.cycle_count += 1
        finally:
            elapsed = time.perf_counter() - t0
            self._record_loop_timing(elapsed)

    def _record_loop_timing(self, elapsed_s: float):
        """统计单周期耗时；超预算则按窗口聚合上报，避免日志风暴。"""
        if elapsed_s > self._loop_max_elapsed_s:
            self._loop_max_elapsed_s = elapsed_s
        if elapsed_s > CONTROL_LOOP_BUDGET_S:
            self._loop_slow_count += 1
        now = time.monotonic()
        if (now - self._last_slow_log_t) >= CONTROL_LOOP_SLOW_LOG_INTERVAL_S:
            if self._loop_slow_count > 0:
                logging.warning(
                    '[LOOP TIMING] slow=%d/window max=%.2fms budget=%.2fms tx_dropped=%d',
                    self._loop_slow_count,
                    self._loop_max_elapsed_s * 1000.0,
                    CONTROL_LOOP_BUDGET_S * 1000.0,
                    self.esp32.tx_dropped,
                )
            self._loop_slow_count = 0
            self._loop_max_elapsed_s = 0.0
            self._last_slow_log_t = now

    @staticmethod
    def _build_safe_fallback_payload(lon_cmd: float) -> bytes:
        """构造一组完全脱离 self.memory 的安全帧。

        紧急停车/感知中断时不读取任何可能被 NaN 污染的状态字段，所有数值
        都是硬编码的有限数，保证 ESP32 解析不会因 'nan'/'inf' 字符串失败。
        """
        return build_esp32_payload(
            Esp32ControlFrame(
                ttc=999.99,
                dist=999.99,
                psi=0.0,
                delta=0.0,
                speed=0.0,
                lon=float(lon_cmd),
                offset=0.0,
                lead_v_proj=0.0,
                min_safe_dist=0.0,
                lane_warn_margin=LANE_DEFAULT_WIDTH * 0.5 * 0.6,
                lane_hard_margin=LANE_DEFAULT_WIDTH * 0.5 * 0.4,
                filtered_curv=0.0,
            )
        )

    def _send_emergency_stop(self):
        """向 ESP32 发送紧急停车帧：方向盘归零 + 最大制动。

        限频到 100ms 一次，避免连续异常时刷屏 ESP32 与日志。
        """
        now = time.monotonic()
        if (now - self._last_emergency_stop_t) < EMERGENCY_STOP_MIN_INTERVAL_S:
            return
        self._last_emergency_stop_t = now
        try:
            payload = self._build_safe_fallback_payload(LON_CMD_MAX_BRAKE_DECEL)
            self.esp32.send(payload)
            self._critical_log.log(
                'emergency_stop',
                '[EMERGENCY STOP] %d consecutive errors, sending max brake',
                self._consecutive_errors,
            )
        except Exception as e2:
            logging.critical('[EMERGENCY STOP] failed to send: %s', e2)

    def _send_sensor_timeout_brake(self):
        """感知数据中断时发送轻制动帧，防止车辆在无新控制下继续运动。

        同样限频到 100ms 一次。
        """
        now = time.monotonic()
        if (now - self._last_sensor_brake_t) < EMERGENCY_STOP_MIN_INTERVAL_S:
            return
        self._last_sensor_brake_t = now
        try:
            payload = self._build_safe_fallback_payload(SENSOR_TIMEOUT_BRAKE_CMD)
            self.esp32.send(payload)
        except Exception:
            pass

    def _handle_no_data(self, now, health):
        """感知数据不就绪时的处理。

        区分两种"无数据"：
        - 自启动以来从未激活（上电待机）→ 直接静默待机，不主动刹车；
        - 行驶中突然丢数据 → 先主动轻制动把车刹停（安全），持续无数据超过
          STANDBY_ENTER_S 后沉降为静默待机保持。
        IDLE_STANDBY_ENABLED=False 时退回旧行为（一直发感知中断轻制动）。
        """
        if not IDLE_STANDBY_ENABLED:
            ref = self._last_control_active_t if self._last_control_active_t is not None else now
            if (now - ref) > SENSOR_TIMEOUT_BRAKE_S:
                self._send_sensor_timeout_brake()
            if self.memory.cycle_count % LOG_EVERY_N_CYCLES == 0:
                logging.info('[%s] waiting for data ego=%s(stale=%s) road=%s(stale=%s)',
                             runtime.NANO_ROLE, health.ego_ready, health.ego_stale,
                             health.road_ready, health.road_stale)
            return

        if self._last_control_active_t is None:
            # 上电以来从未激活 → 静默待机（不主动刹车）
            self._idle_standby(now)
            return

        idle_for = now - self._last_control_active_t
        if idle_for <= STANDBY_ENTER_S:
            # 刚丢数据（车可能仍在运动）→ 主动轻制动把车刹停
            if idle_for > SENSOR_TIMEOUT_BRAKE_S:
                self._send_sensor_timeout_brake()
            if self.memory.cycle_count % LOG_EVERY_N_CYCLES == 0:
                logging.warning('[%s] 感知中断 %.1fs，主动轻制动刹停中…',
                                runtime.NANO_ROLE, idle_for)
        else:
            # 已刹停 + 持续无数据 → 沉降为静默待机保持
            self._idle_standby(now)

    def _idle_standby(self, now):
        """静默待机：低频发"方向0 + 轻保持刹车"保持帧，日志大幅限频。

        保持帧让 ESP32 维持 SRC:0（安静、车被稳住）；帧间隔 <
        ESP32 通信看门狗(200ms)，故不会触发硬件全力制动。感知恢复即退出。
        """
        if (now - self._last_standby_tx_t) >= STANDBY_KEEPALIVE_INTERVAL_S:
            self._last_standby_tx_t = now
            try:
                payload = self._build_safe_fallback_payload(STANDBY_HOLD_BRAKE_CMD)
                self.esp32.send(payload)
            except Exception:
                pass
        if not self._in_standby:
            self._in_standby = True
            self._standby_log_t = now
            logging.info('[%s] 无感知数据，进入静默待机（等待数据…）', runtime.NANO_ROLE)
        elif (now - self._standby_log_t) >= STANDBY_LOG_INTERVAL_S:
            self._standby_log_t = now
            logging.info('[%s] 静默待机中（仍无感知数据）', runtime.NANO_ROLE)

    def _hot_standby_tx(self, now: float):
        """备机热待机发帧：standby 期解算并向 ESP32 持续发一份新鲜备帧。

        MCU 仲裁主优先、主新鲜时忽略备帧；主控一旦超时备帧已就绪 → 瞬间干净
        切换，消除“主超时但备机还没开始发帧”竞态窗口里的全力制动冲击。

        与 active 控制路径（_control_loop_impl 主体）的区别：
          · 不消费接管种子、不进接管保护窗（still standby，未接管）；
          · 不广播控制心跳（备机只 send_backup_alive，主控才广播种子）；
          · 不更新 _last_control_active_t、不记主控遥测；
          · NaN 帧直接不发（保持 MCU 上一帧；正式接管由 active 路径 NaN 守护处理）。
        控制内核随感知热运行，lat_smooth 持续更新保持温启动；接管沿仍由
        _handle_takeover_edge 消费主控种子对齐 lon/lat 平滑，与原语义一致。
        """
        signals_snap = self.signals.snapshot()
        _res = run_pure_pipeline(
            now, signals_snap, self.memory, self.managers, None,
        )
        lateral_ctx = _res.lateral_ctx
        lon_ctx = _res.lon_ctx
        lon_cmd = _res.lon_cmd

        bad_fields = self._find_nonfinite_tx_fields(
            lateral_ctx, lon_ctx, lon_cmd, _res.cur_lane_width)
        if bad_fields:
            return

        ttc_tx = lon_ctx.ttc if is_finite(lon_ctx.ttc) else 999.99
        dist_tx = clamp(lon_ctx.dist, 0.0, 999.99)
        psi_tx = clamp(lateral_ctx.upd_psi, -9.9999, 9.9999)
        delta_tx = clamp(lateral_ctx.delta, -9.9999, 9.9999)
        speed_tx = clamp(self.signals.ego_v, -99.99, 99.99)
        lon_tx = clamp(lon_cmd, -LON_CMD_MAX_DRIVE_ACCEL, LON_CMD_MAX_BRAKE_DECEL)
        # 横向平滑常规速率（standby 无接管保护窗）；写回 last_delta 保持连续。
        delta_tx = self.lat_smooth.update(delta_tx)
        self.memory.last_delta = delta_tx

        payload = self._build_esp32_payload(
            ttc_tx, dist_tx, psi_tx, delta_tx, speed_tx,
            lon_tx, lateral_ctx.cur_off, lon_ctx.lead_v_proj,
            lon_ctx.min_safe_dist)
        self.esp32.send(payload)

    def _find_nonfinite_tx_fields(self, lateral_ctx, lon_ctx,
                                  lon_cmd: float, cur_lane_width: float):
        """返回所有将进入 ESP32 帧的原始标量中非有限（NaN/Inf）字段名列表。

        必须在 clamp 之前对原始值检查：clamp(v,lo,hi)=max(lo,min(hi,v)) 在
        Python 中对 NaN 返回上界 hi，会把 NaN 静默放大成最大转角/最大制动，
        事后检查 *_tx 永远是有限值，无法发现。所以这里查的是 clamp 前的源值。

        ttc 不在此列：inf 是"无前车"的正常哨兵，已由 send 段
        `ttc_tx = ... if is_finite else 999.99` 单独处理（NaN 也会落到 999.99）。
        """
        candidates = (
            ('upd_psi', lateral_ctx.upd_psi),
            ('delta', lateral_ctx.delta),
            ('cur_off', lateral_ctx.cur_off),
            ('ego_v', self.signals.ego_v),
            ('lon_cmd', lon_cmd),
            ('dist', lon_ctx.dist),
            ('lead_v_proj', lon_ctx.lead_v_proj),
            ('min_safe_dist', lon_ctx.min_safe_dist),
            ('cur_lane_width', cur_lane_width),
            ('lane_warn_margin', self.memory.lane_warn_margin),
            ('lane_hard_margin', self.memory.lane_hard_margin),
            ('filtered_curv', self.memory.filtered_curv),
        )
        return [name for name, v in candidates if not is_finite(v)]

    def _handle_nan_tx(self, bad_fields):
        """检测到控制输出含 NaN/Inf：发安全帧（转角0 + 轻制动）替代被污染的帧。

        - ESP32：发 _build_safe_fallback_payload（全部硬编码有限值）。
        - ROS：发布安全值而非 NaN，便于监控看出"已降级"而不是把 NaN 扩散出去。
        - 主备：向备机广播安全种子（0/0/制动），避免 NaN 污染接管种子。
        - 日志：logging.error 指出是哪些字段 NaN，按窗口限频避免 100Hz 刷屏。
        """
        self._nan_tx_count += 1
        now = time.monotonic()
        if (now - self._last_nan_tx_log_t) >= 1.0:
            logging.error(
                '[NAN GUARD] non-finite tx field(s)=%s (count=%d in last window), '
                'sending safe fallback frame',
                ','.join(bad_fields), self._nan_tx_count,
            )
            self._last_nan_tx_log_t = now
            self._nan_tx_count = 0
        try:
            payload = self._build_safe_fallback_payload(SENSOR_TIMEOUT_BRAKE_CMD)
            self.esp32.send(payload)
        except Exception as e:
            logging.critical('[NAN GUARD] failed to send safe frame: %s', e)
        # ROS 输出与主备种子都用安全值，杜绝 NaN 向外扩散
        self._publish_outputs(0.0, 0.0, SENSOR_TIMEOUT_BRAKE_CMD, 0.0,
                              LANE_DEFAULT_WIDTH)
        self._sync_peer_output(0.0, 0.0, SENSOR_TIMEOUT_BRAKE_CMD)

    def _lockstep_snapshot(self, signals_snap, takeover_rate):
        """锁步前状态拷贝（best-effort）。返回 (signals, memory, managers,
        takeover_rate) 供影子核重算；未启用或拷贝失败返回 None（控制不受影响）。

        - signals 浅拷贝即可：纯管线只读 signals（内部 suppress 自带 copy.copy），
          且 VehicleSignals 含 threading.Lock 不可深拷贝。
        - memory / managers 须深拷贝：管线就地改写它们，影子需独立副本；managers
          排除 ml_bridge（不可深拷贝），影子改用主核 ml_result。
        """
        ls = self.lockstep
        if ls is None or not ls.enabled:
            return None
        try:
            from control.lockstepd_client import snapshot_managers
            return (
                _copy.copy(signals_snap),
                _copy.deepcopy(self.memory),
                snapshot_managers(self.managers),
                takeover_rate,
            )
        except Exception as exc:
            if (time.monotonic() - self._last_lockstep_log_t) >= 5.0:
                logging.warning('[LOCKSTEP] 前状态拷贝失败（跳过本拍）：%r', exc)
                self._last_lockstep_log_t = time.monotonic()
            return None

    def _handle_lockstep_fault(self, now):
        """锁步失配 → 安全态：受控制动 + 安全 ROS 输出 + 安全种子，限频记 critical。"""
        if (now - self._last_lockstep_log_t) >= 1.0:
            logging.critical(
                '[LOCKSTEP] 双核比较失配，控制器进入安全态（受控制动 %.1f m/s²）：%s',
                LOCKSTEP_SAFE_BRAKE_CMD, self.lockstep.fault_reason,
            )
            self._last_lockstep_log_t = now
        try:
            payload = self._build_safe_fallback_payload(LOCKSTEP_SAFE_BRAKE_CMD)
            self.esp32.send(payload)
        except Exception as exc:
            logging.critical('[LOCKSTEP] 安全帧发送失败：%s', exc)
        self._publish_outputs(0.0, 0.0, LOCKSTEP_SAFE_BRAKE_CMD, 0.0,
                              LANE_DEFAULT_WIDTH)
        self._sync_peer_output(0.0, 0.0, LOCKSTEP_SAFE_BRAKE_CMD)

    def _record_telemetry(self, now, lateral_ctx, lon_ctx, lead_ctx,
                          in_curve_hold, lon_cmd, lon_raw_cmd,
                          psi_tx, delta_tx, speed_tx, lon_tx, cur_lane_width):
        """组织本周期遥测行并非阻塞投递（写盘在后台线程，零阻塞控制环）。"""
        self.telemetry.record({
            't_wall': time.time(),
            't_mono': now,
            'cycle': self.memory.cycle_count,
            'ego_x': self.signals.ego_x,
            'ego_y': self.signals.ego_y,
            'ego_yaw': self.signals.ego_yaw,
            'ego_v': self.signals.ego_v,
            'lead_x': self.signals.lead_x,
            'lead_y': self.signals.lead_y,
            'lead_v': self.signals.lead_v,
            'road_psi': self.signals.road_psi,
            'filtered_road_psi': self.memory.filtered_road_psi,
            'raw_cte': lateral_ctx.raw_cte,
            'filtered_cte': self.memory.filtered_cte,
            'raw_curv': lateral_ctx.raw_curv,
            'filtered_curv': self.memory.filtered_curv,
            'curv_guard': lateral_ctx.curv_guard,
            'in_curve': lateral_ctx.in_curve,
            'delta': lateral_ctx.delta,
            'delta_cte': lateral_ctx.delta_cte,
            'delta_ff': lateral_ctx.delta_ff,
            'boundary_delta': lateral_ctx.boundary_delta,
            'psi_i_term': self.memory.psi_i_term,
            'upd_psi': lateral_ctx.upd_psi,
            'lon_raw_cmd': lon_raw_cmd,
            'lon_cmd': lon_cmd,
            'acc_i_term': self.lon_ctrl.i_term,
            'aeb_active': lon_ctx.aeb_active,
            'lead_cls': self.signals.lead_cls,
            'in_curve_hold': in_curve_hold,
            'dist': lon_ctx.dist,
            'ttc': lon_ctx.ttc,
            'lead_v_proj': lon_ctx.lead_v_proj,
            'min_safe_dist': lon_ctx.min_safe_dist,
            'closing_speed': lon_ctx.closing_speed,
            'acc_has_lead': lead_ctx.acc_has_lead,
            'lead_detected': lead_ctx.lead_detected,
            'cur_lane_width': cur_lane_width,
            'lane_safe_margin': self.memory.lane_safe_margin,
            'lane_warn_margin': self.memory.lane_warn_margin,
            'lane_hard_margin': self.memory.lane_hard_margin,
            'boundary_brake': lateral_ctx.boundary_brake,
            'boundary_warn': lateral_ctx.boundary_warn,
            'psi_tx': psi_tx,
            'delta_tx': delta_tx,
            'speed_tx': speed_tx,
            'lon_tx': lon_tx,
            'esp_psi': self.esp32.esp_psi,
            'esp_delta': self.esp32.esp_delta,
            'esp_brake': self.esp32.esp_brake,
            # class-aware AEB / 接管 cls 冗余诊断字段
            'lead_cls_stale': bool(getattr(self, '_last_lead_cls_stale', False)),
            'takeover_seed_cls': (self._takeover_seed_cls
                                  if now < self._takeover_guard_until
                                  else 0),
        })

    def _select_primary_lead(self, now):
        """构建 PerceptionFrame，并按需用感知层选举结果写回 signals.lead_*。

        - MULTI_TARGET_COUNT==1：感知层仅作下游/遥测视图，不写 signals
          （callback 已直接写过，与改造前字节级一致）。
        - MULTI_TARGET_COUNT>1：感知层承担主前车选举与回写（替换原
          MultiTargetTracker.select() 路径）；选不出主前车时同步 car2 的真实
          class，防止上一拍 cls 残留导致 AEB 用错乘子。

        车道宽用上一拍的 lane_est 估计，filtered_curv 用 memory 当前值；
        二者都是慢变量，用于横向窗口判定足够。
        """
        frame = self.perception.build_frame(
            self.signals.ego_x, self.signals.ego_y, self.signals.ego_yaw,
            self.managers.lane_est.width, self.memory.filtered_curv, now,
        )
        # 暴露给后续阶段（pipeline / 遥测 / 未来避障）使用。
        self._last_frame = frame

        # 单目标模式：感知层只读不写。
        if MULTI_TARGET_COUNT <= 1:
            return

        if frame.primary_tid is None:
            # 选不出主前车：把 car2 真实 class 同步回 signals，避免上一拍 cls 残留。
            with self.signals:
                self.signals.lead_cls = self.perception.get_cls(CAR2_ID)
            return

        primary = frame.tracks[frame.primary_tid]
        # 切主检测：新选中目标 ID 与上拍不同时，重置 LeadTracker 内部计数及
        # ControlMemory 中的前车相关滤波，避免新旧目标的速度/距离被低通串成
        # 虚假趋势（约定见 [[lead-swap-reset]]）。
        if self._last_lead_tid is not None and primary.tid != self._last_lead_tid:
            self.lead_tracker.reset_on_lead_swap()
            self.memory.filtered_lead_v_proj = 0.0
            self.memory.filtered_lead_accel = 0.0
            self.memory.last_lead_v_proj = 0.0
            self.memory.last_lead_v_time = 0.0
            self.memory.filtered_v_tgt = 0.0
            self.managers.lon_ctrl.reset()
            self._critical_log.log(
                'lead_swap',
                '[PERCEPTION] lead swap car%s -> car%s, lead state reset',
                self._last_lead_tid, primary.tid,
            )
        self._last_lead_tid = primary.tid
        # 写回 signals 使用感知层仓库中的原始世界坐标（与 callback 写入语义一致），
        # 不是 TrackRel 里的滤波相对量——LeadTracker 自己会再做一次同 alpha 低通。
        store = self.perception._targets[primary.tid]
        with self.signals:
            self.signals.lead_x = store.x
            self.signals.lead_y = store.y
            self.signals.lead_yaw = store.yaw
            self.signals.lead_v = store.v
            self.signals.lead_cls = primary.cls
            self.signals.lead_received = True
            self.signals.lead_last_rx_time = now
            self.signals.lead_v_last_rx_time = now
        if self.memory.cycle_count % LOG_EVERY_N_CYCLES == 0:
            logging.info(
                '[PERCEPTION] primary=car%s cls=%d via_cutin=%s x_rel=%.1f '
                'y_rel=%.2f n_fresh=%d',
                primary.tid, primary.cls, primary.cutin,
                primary.x_rel, primary.y_rel, frame.n_fresh,
            )

    def _control_loop_impl(self):
        """控制循环实际实现。

        数据流顺序：
        1. 刷新串口接收和主备状态。
        2. 检查关键输入是否就绪，不满足则直接返回。
        3. 依次计算车道/LKA、目标车上下文、弯道保持、AEB、纵向控制。
        4. 发布 ROS 输出，发送 ESP32 控制帧，并同步主备心跳。
        """
        self.esp32.drain_rx()
        now = time.monotonic()

        # 单周期内只查询一次 is_active()，确保边沿检测语义与日志输出一致
        peer_active = self.peer_hb.is_active()
        self._publish_role_status(peer_active)

        health = evaluate_control_health(
            peer_active=peer_active,
            now=now,
            ego_received=self.signals.ego_received,
            ego_last_rx=self.signals.ego_last_rx,
            road_received=self.signals.road_received,
            road_last_rx=self.signals.road_last_rx,
            lead_ready=self.signals.lead_received,
            lane_offset_ready=self.signals.lane_offset_received,
            stale_timeout_s=SENSOR_STALE_TIMEOUT_S,
            lead_cls_last_rx=self.signals.lead_cls_last_rx_time,
            lead_cls_stale_timeout_s=LEAD_CLASS_STALE_TIMEOUT_S,
        )

        # 当前节点无资格输出控制时，只维持备机存活状态。
        if not health.peer_active:
            self.peer_hb.send_backup_alive(False)
            # 备机热待机：standby 期也持续发一份新鲜备帧给 MCU。MCU 仲裁主优先、
            # 主新鲜时忽略备帧；主控一旦超时备帧已就绪 → 瞬间干净切换，消除接管
            # 竞态全力制动冲击。需感知就绪；NaN/异常不发（保持上一帧，安全兜底）。
            if (BACKUP_HOT_STANDBY and not runtime.IS_PRIMARY
                    and health.ego_ready and health.road_ready):
                try:
                    self._hot_standby_tx(now)
                except Exception:
                    pass
            self.memory.cycle_count += 1
            return

        # 自车或道路基础状态不完整时，不进入控制解算。
        if not (health.ego_ready and health.road_ready):
            self.peer_hb.send_backup_alive(False)
            self._sync_idle_primary_heartbeat()
            self._handle_no_data(now, health)
            self.memory.cycle_count += 1
            return

        # 感知就绪：若刚从静默待机恢复，打一条恢复日志
        if self._in_standby:
            self._in_standby = False
            logging.info('[%s] 感知数据恢复，退出待机，开始控制', runtime.NANO_ROLE)
        # 更新最后活跃时刻
        self._last_control_active_t = now

        # 锁步比较器已报故障（影子核与主核输出失配）→ 进入安全态（受控制动），
        # 不再下发常规控制帧。这是软件双核锁步要演示的"算错即安全停"的行为。
        if self.lockstep is not None and self.lockstep.fault:
            self._handle_lockstep_fault(now)
            self.memory.cycle_count += 1
            return

        # class 话题陈旧告警：不降级控制，仅限频提醒并写遥测。
        if health.lead_cls_stale and self.memory.cycle_count % LOG_EVERY_N_CYCLES == 0:
            logging.warning(
                '[CLASS] /car{N}_class stale (>%.1fs) — AEB sticks to last cls=%d',
                LEAD_CLASS_STALE_TIMEOUT_S, self.signals.lead_cls,
            )
        self._last_lead_cls_stale = health.lead_cls_stale

        # 检测主备接管边沿：在第一次以 active 身份进入控制解算前完成种子初始化
        self._handle_takeover_edge(now)

        # 感知层：每周期构建一份 PerceptionFrame（含所有 fresh track 的相对位置）。
        # MULTI_TARGET_COUNT>1 时还承担主前车选举与 signals.lead_* 写回；==1 时
        # 仅作下游/遥测视图，下游单目标管线沿用 callback 直写的 signals.lead_*。
        self._select_primary_lead(now)

        # 线程安全：创建 signals 快照供本周期控制计算使用。
        # ROS 回调线程持续写入 self.signals（ego_x/y/yaw 等），控制线程读取。
        # snapshot() 将竞争窗口从整个 10ms 周期缩小到微秒级 copy 操作，
        # 保证本周期看到的 pose 三元组 (x,y,yaw) 来自同一帧或相邻帧。
        # pipeline 内部的 overtake suppress 会临时修改快照，下周期重新 snapshot。
        signals_snap = self.signals.snapshot()

        # 纯控制计算统一走 pipeline.run_pure_pipeline，在线/离线复用同一内核。
        # 与抽取前逐行等价：相同调用顺序、相同就地副作用。
        # takeover_rate 仅读 now 与 _takeover_guard_until，无副作用，提前计算无影响。
        # 接管期速率三分支由 _takeover_lon_rate() 统一计算：
        # AEB 种子→AEB_RELEASE，行人/障碍→VULNERABLE，其他→TAKEOVER_LON_RATE。
        if now < self._takeover_guard_until:
            takeover_rate = self._takeover_lon_rate()
        else:
            takeover_rate = None
        # 锁步：在主核改写 memory/managers **之前**深拷贝一份本拍前状态，供影子核
        # 重算（同输入 + 同前状态 + 同 ml_result → 确定性一致，零误报）。best-effort。
        ls_pre = self._lockstep_snapshot(signals_snap, takeover_rate)
        _res = run_pure_pipeline(
            now, signals_snap, self.memory, self.managers, takeover_rate,
        )
        cur_lane_width = _res.cur_lane_width
        lateral_ctx = _res.lateral_ctx
        lead_ctx = _res.lead_ctx
        lon_ctx = _res.lon_ctx
        in_curve_hold = _res.in_curve_hold
        lon_cmd = _res.lon_cmd
        lon_raw_cmd = _res.lon_raw_cmd

        # 锁步：把主核本拍输出 (delta / lon_cmd / AEB) 投给影子核做逐拍比较。
        if ls_pre is not None:
            self.lockstep.submit(
                now, ls_pre[0], ls_pre[1], ls_pre[2], ls_pre[3], _res.ml_result,
                lateral_ctx.delta, lon_cmd, lon_ctx.aeb_active,
            )

        # NaN 防护：必须在 clamp 之前检查原始值（clamp 会把 NaN 放大成上界，
        # 例如 delta=NaN → 最大转角，且 CRC 仍合法不会被 ESP32 丢帧）。
        # 任一字段非有限就整帧改走安全回退，绝不把被污染的帧发出去。
        bad_fields = self._find_nonfinite_tx_fields(
            lateral_ctx, lon_ctx, lon_cmd, cur_lane_width)
        if bad_fields:
            self._handle_nan_tx(bad_fields)
            self.memory.cycle_count += 1
            return

        # 串口协议字段范围有限，发送前统一裁剪到约定区间。
        ttc_tx = lon_ctx.ttc if is_finite(lon_ctx.ttc) else 999.99
        dist_tx = clamp(lon_ctx.dist, 0.0, 999.99)
        psi_tx = clamp(lateral_ctx.upd_psi, -9.9999, 9.9999)
        delta_tx = clamp(lateral_ctx.delta, -9.9999, 9.9999)
        speed_tx = clamp(self.signals.ego_v, -99.99, 99.99)
        lon_tx = clamp(lon_cmd, -LON_CMD_MAX_DRIVE_ACCEL, LON_CMD_MAX_BRAKE_DECEL)

        # 横向最外层平滑：常规走 LAT_RATE_NORMAL，接管保护窗内收紧到
        # TAKEOVER_DELTA_RATE。lon 的接管期收紧由 _apply_takeover_guard 单独处理
        # （lon_smooth 已经在 pipeline 内部接收 takeover_rate）。
        takeover_lat_rate = (
            TAKEOVER_DELTA_RATE
            if now < self._takeover_guard_until
            else None
        )
        delta_tx = self.lat_smooth.update(delta_tx,
                                          max_rate_override=takeover_lat_rate)
        # 写回 memory.last_delta，让边界等内部状态在保护窗结束后能继续基于
        # 真实输出做参考（与旧版 _apply_takeover_guard 中的 last_delta 写回等价）。
        self.memory.last_delta = delta_tx

        # 主备接管保护窗口内对 lon 单独施加更严的变化率限制（保险网）
        lon_tx, delta_tx = self._apply_takeover_guard(now, lon_tx, delta_tx)

        self._publish_outputs(psi_tx, delta_tx, lon_tx, lateral_ctx.cur_off, cur_lane_width)

        payload = self._build_esp32_payload(
            ttc_tx, dist_tx, psi_tx, delta_tx, speed_tx,
            lon_tx, lateral_ctx.cur_off, lon_ctx.lead_v_proj, lon_ctx.min_safe_dist
        )
        self.esp32.send(payload)

        self._sync_peer_output(psi_tx, delta_tx, lon_tx,
                               aeb_active=bool(lon_ctx.aeb_active),
                               state=HB_STATE_ACTIVE_CONTROL)

        self._log_cycle_summary(
            lateral_ctx, lead_ctx, lon_ctx,
            in_curve_hold, ttc_tx, cur_lane_width, lon_cmd, lon_raw_cmd,
        )

        self._record_telemetry(now, lateral_ctx, lon_ctx, lead_ctx,
                               in_curve_hold, lon_cmd, lon_raw_cmd,
                               psi_tx, delta_tx, speed_tx, lon_tx,
                               cur_lane_width)

        self.memory.last_acc_has_lead = lead_ctx.acc_has_lead
        self.memory.cycle_count += 1

    # ----------------------------------------------------------
    @staticmethod
    def _safe_float(value, lo: float, hi: float):
        """对来自 ROS 话题的标量做有限性 + 范围校验。

        非有限值返回 None，由调用方决定丢弃；有限值钳到 [lo, hi]。
        """
        try:
            v = float(value)
        except (TypeError, ValueError):
            return None
        if not is_finite(v):
            return None
        if v < lo:
            return lo
        if v > hi:
            return hi
        return v

    def _ego_pose_callback(self, msg):
        # 自车位姿既可直接用于控制，也可在缺少独立航向话题时回退使用四元数解算航向。
        x = self._safe_float(msg.position.x, -1e6, 1e6)
        y = self._safe_float(msg.position.y, -1e6, 1e6)
        if x is None or y is None:
            return
        qx, qy, qz, qw = (msg.orientation.x, msg.orientation.y,
                           msg.orientation.z, msg.orientation.w)
        ego_yaw = None
        if (
            not self.signals.ego_psi_received
            and is_finite(qx*qx + qy*qy + qz*qz + qw*qw)
            and (qx*qx+qy*qy+qz*qz+qw*qw) > 1e-8
        ):
            ego_yaw = wrap_angle(quaternion_to_yaw(qx, qy, qz, qw))
        now_mono = time.monotonic()
        with self.signals:
            self.signals.ego_x, self.signals.ego_y = x, y
            if ego_yaw is not None:
                self.signals.ego_yaw = ego_yaw
            self.signals.ego_received = True
            self.signals.ego_last_rx = now_mono

    def _lead_pose_callback(self, msg):
        # 前车位姿回调同时更新时间戳，供前车跟踪模块判断目标是否仍然有效。
        x = self._safe_float(msg.position.x, -1e6, 1e6)
        y = self._safe_float(msg.position.y, -1e6, 1e6)
        if x is None or y is None:
            return
        qx, qy, qz, qw = (msg.orientation.x, msg.orientation.y,
                          msg.orientation.z, msg.orientation.w)
        lead_yaw = 0.0
        if is_finite(qx*qx + qy*qy + qz*qz + qw*qw) and (qx*qx+qy*qy+qz*qz+qw*qw) > 1e-8:
            lead_yaw = wrap_angle(quaternion_to_yaw(qx, qy, qz, qw))
        now_mono = time.monotonic()
        with self.signals:
            self.signals.lead_x, self.signals.lead_y = x, y
            self.signals.lead_yaw = lead_yaw
            self.signals.lead_received = True
            self.signals.lead_last_rx_time = now_mono
        # car2 同步喂入感知层（作为 id=2 的航迹；始终启用，无 None 分支）
        self.perception.ingest_pose(CAR2_ID, x, y, lead_yaw, now_mono)

    def _make_mt_pose_cb(self, tid):
        """生成 car{tid} 位姿订阅回调（闭包绑定 tid）。"""
        def _cb(msg):
            x = self._safe_float(msg.position.x, -1e6, 1e6)
            y = self._safe_float(msg.position.y, -1e6, 1e6)
            if x is None or y is None:
                return
            qx, qy, qz, qw = (msg.orientation.x, msg.orientation.y,
                              msg.orientation.z, msg.orientation.w)
            yaw = 0.0
            nrm = qx*qx + qy*qy + qz*qz + qw*qw
            if is_finite(nrm) and nrm > 1e-8:
                yaw = wrap_angle(quaternion_to_yaw(qx, qy, qz, qw))
            self.perception.ingest_pose(tid, x, y, yaw, time.monotonic())
        return _cb

    def _make_mt_v_cb(self, tid):
        """生成 car{tid} 速度订阅回调（闭包绑定 tid）。"""
        def _cb(msg):
            v = self._safe_float(msg.data, -200.0, 200.0)
            if v is not None:
                self.perception.ingest_v(tid, v)
        return _cb

    def _make_mt_class_cb(self, tid):
        """生成 car{tid} 分类订阅回调（闭包绑定 tid）。Int32 透传，越界丢弃在 perception 内做。
        刷新 signals.lead_cls_last_rx_time 以便 health.py 判定 class 话题新鲜度。
        """
        def _cb(msg):
            self.perception.ingest_cls(tid, msg.data)
            with self.signals:
                self.signals.lead_cls_last_rx_time = time.monotonic()
        return _cb

    def _car2_class_cb(self, msg):
        """/car2_class 回调：写 signals.lead_cls 并同步喂入感知层。

        - 单目标模式：signals.lead_cls 由本回调直接维护（与改造前一致），
          感知层只作为下游/遥测视图。
        - 多目标模式：本回调仍写 signals.lead_cls，但下一拍 _select_primary_lead
          会根据感知层选举结果覆盖（若选中 car2 则等同），保证 cls 与 lead pose
          一致。未发布时 lead_cls 维持 0=UNKNOWN，AEB 查表回退乘子 1.0。
        """
        try:
            c = int(msg.data)
        except (TypeError, ValueError):
            return
        if c < 0 or c > 255:
            return
        now_mono = time.monotonic()
        with self.signals:
            self.signals.lead_cls = c
            self.signals.lead_cls_last_rx_time = now_mono
        self.perception.ingest_cls(CAR2_ID, c)

    def _ego_v_callback(self, msg):
        """自车速度回调（带有限性与范围校验）。"""
        v = self._safe_float(msg.data, -200.0, 200.0)
        if v is not None:
            with self.signals:
                self.signals.ego_v = v

    def _ego_psi_callback(self, msg):
        """自车航向回调（覆盖四元数解算值，归一化到 [-π, π]）。"""
        v = self._safe_float(msg.data, -1e4, 1e4)
        if v is None:
            return
        with self.signals:
            self.signals.ego_yaw = wrap_angle(v)
            self.signals.ego_psi_received = True

    def _lead_v_callback(self, msg):
        """前车速度回调，同时更新速度接收时间戳。"""
        v = self._safe_float(msg.data, -200.0, 200.0)
        if v is None:
            return
        now_mono = time.monotonic()
        with self.signals:
            self.signals.lead_v = v
            self.signals.lead_v_last_rx_time = now_mono
        self.perception.ingest_v(CAR2_ID, v)

    def _road_psi_callback(self, msg):
        """道路航向回调（归一化到 [-π, π]）。"""
        v = self._safe_float(msg.data, -1e4, 1e4)
        if v is None:
            return
        with self.signals:
            self.signals.road_psi = wrap_angle(v)
            self.signals.road_received = True
            self.signals.road_last_rx = time.monotonic()

    def _lane_offset_callback(self, msg):
        # 车道横向偏移是 LKA 和车道宽估计的共同输入，需要保留最近一次接收时刻。
        v = self._safe_float(msg.data, -10.0, 10.0)
        if v is None:
            return
        with self.signals:
            self.signals.lane_offset = v
            self.signals.lane_offset_received = True
            self.signals.lane_offset_last_rx = time.monotonic()

    # ----------------------------------------------------------
    # 运行时增益热更新白名单：param 名 -> (gains 属性, lon_ctrl.set_gains 关键字)
    # 只允许这几个增益类参数在线改（已有 memory.gains / set_gains 通路）；
    # 其它参数一律拒绝，避免运行中改安全限幅类常量引发不可控行为。
    _HOT_PARAM_MAP = {
        'K_PSI_P': ('lat_kp', None),
        'K_PSI_I': ('lat_ki', None),
        'K_PSI_D': ('lat_kd', None),
        'ACC_KD': ('acc_kd', 'acc_kd'),
        'ACC_KI': ('acc_ki', 'acc_ki'),
        'ACC_KV': ('acc_kv', 'acc_kv'),
        'ACC_KA': ('acc_ka', 'acc_ka'),
    }

    def _set_param_callback(self, msg):
        """处理 /adas/set_param 的 "NAME=VALUE"：仅白名单增益可热更新。

        校验：格式合法 + 在白名单内 + 值有限且非负（与
        LongitudinalController.set_gains 的 max(0.0,..) 语义一致）。
        非法/越权一律拒绝并 WARNING，不改任何状态。
        """
        try:
            raw = str(msg.data).strip()
        except Exception:
            return
        if '=' not in raw:
            logging.warning('[SET_PARAM] bad format %r (need NAME=VALUE)', raw)
            return
        name, _, val_s = raw.partition('=')
        name = name.strip()
        val_s = val_s.strip()
        mapping = self._HOT_PARAM_MAP.get(name)
        if mapping is None:
            logging.warning('[SET_PARAM] reject non-whitelisted param %r '
                            '(allowed: %s)', name,
                            ','.join(sorted(self._HOT_PARAM_MAP)))
            return
        try:
            value = float(val_s)
        except (TypeError, ValueError):
            logging.warning('[SET_PARAM] %s: non-numeric value %r', name, val_s)
            return
        if not is_finite(value) or value < 0.0:
            logging.warning('[SET_PARAM] %s: value out of range %r '
                            '(must be finite >= 0)', name, value)
            return
        gain_attr, lon_kw = mapping
        setattr(self.memory.gains, gain_attr, value)
        if lon_kw is not None:
            self.lon_ctrl.set_gains(**{lon_kw: value})
        logging.info(
            '[SET_PARAM] %s -> %.5f applied (gains.%s%s)',
            name, value, gain_attr,
            '' if lon_kw is None else ' + lon_ctrl',
        )

    # ----------------------------------------------------------
    def destroy_node(self):
        """ROS 节点销毁时关闭串口和心跳连接。"""
        self.esp32.close()
        self.peer_hb.close()
        self.telemetry.close()
        super().destroy_node()


# ==========================================================
# Entry Point
# ==========================================================

# 使用列表存储后台守护进程引用，供 main() 退出时清理
_daemon_procs = []  # type: list


def _cleanup_orphans():
    """启动前扫杀残留的守护进程孤儿（上次非正常退出的遗留）。"""
    for pat in ['lockstepd.py', 'ml_inferd.py']:
        try:
            subprocess.call(['pkill', '-f', pat],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass
    time.sleep(0.3)


def _spawn_daemon(name, script_rel, core, port_env, port_default, timeout=5.0):
    """启动一个守护子进程并钉到指定核。

    改进（对比旧版）：
      - stderr → PIPE 并由守护线程引流到 logging，崩溃不再静默
      - 进程启动后立即检查存活，已退出则不等 port 超时
      - 连接成功/进程退出后返回 proc 或 None
    """
    port = int(os.environ.get(port_env, port_default))
    script_path = os.path.join(os.path.dirname(__file__), script_rel)
    if not os.path.isfile(script_path):
        logging.warning('[DAEMON] script not found: %s', script_path)
        return None

    try:
        proc = subprocess.Popen(
            ['taskset', '-c', str(core), sys.executable, script_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
    except Exception as exc:
        logging.warning('[DAEMON] failed to spawn %s: %r', name, exc)
        return None

    # 守护线程引流 stderr → logging（崩溃不再静默）
    def _drain_stderr():
        try:
            for line in iter(proc.stderr.readline, b''):
                if line:
                    logging.error('[%s] %s', name,
                                  line.decode('utf-8', errors='replace').rstrip())
        except Exception:
            pass
        finally:
            try:
                proc.stderr.close()
            except Exception:
                pass

    t = threading.Thread(target=_drain_stderr,
                         name='drain-%s' % name, daemon=True)
    t.start()

    # 进程已退出 → 不等 port
    if proc.poll() is not None:
        logging.warning('[DAEMON] %s exited immediately (rc=%d)', name, proc.returncode)
        return None

    # 等待端口就绪
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            s = socket.create_connection(('127.0.0.1', port), timeout=0.5)
            s.close()
            logging.info('[DAEMON] %s ready (pid=%d core=%d port=%d)',
                         name, proc.pid, core, port)
            return proc
        except (ConnectionRefusedError, OSError):
            if proc.poll() is not None:
                logging.warning('[DAEMON] %s exited during startup (rc=%d)',
                                name, proc.returncode)
                return None
            time.sleep(0.1)

    logging.warning('[DAEMON] %s startup timeout (%.1fs), continuing', name, timeout)
    return proc


def main(argv=None):
    # 命令行角色参数优先写入环境变量，再统一交给 runtime 配置主备身份。
    parser = build_arg_parser()
    cli_args, ros_args = parser.parse_known_args(argv)
    if cli_args.role:
        os.environ['NANO_ROLE'] = cli_args.role

    runtime.configure_runtime(cli_args.role)

    # ── 启动后台守护进程（锁步 + ML 推理）──
    # 先扫杀孤儿：覆盖 kill -9 / 异常崩溃后残留的旧 daemon
    _cleanup_orphans()
    # SIGCHLD 忽略：防止子进程退出时积累僵尸（daemon crash 场景）
    try:
        signal.signal(signal.SIGCHLD, signal.SIG_IGN)
    except Exception:
        pass

    global _daemon_procs
    _daemon_procs = []

    if LOCKSTEP_ENABLED:
        p = _spawn_daemon('lockstepd', 'control/lockstepd.py', LOCKSTEPD_CORE,
                          'LOCKSTEPD_PORT', '19998')
        if p is not None:
            _daemon_procs.append(('lockstepd', p))

    if ML_ENABLED:
        p = _spawn_daemon('ml_inferd', 'control/ml_inferd.py', ML_INFERD_CORE,
                          'ML_INFERD_PORT', '19999')
        if p is not None:
            _daemon_procs.append(('ml_inferd', p))

    rclpy.init(args=ros_args)
    node = None
    try:
        node = AdasNode()
        # 线程级钉核：把控制主循环独占到控制核，其余线程赶到辅助核。
        if RT_THREAD_PIN:
            try:
                import rt_affinity
                rt_affinity.isolate_control_core(RT_CONTROL_CORE, RT_PIN_RESWEEP_S)
            except Exception as exc:
                logging.getLogger(__name__).warning("[RT] 线程级钉核启用失败（忽略）：%r", exc)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        logging.critical('[MAIN] 未捕获异常: %r', exc, exc_info=True)
    finally:
        # 清理 ROS 节点（可能未初始化或已销毁）
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        # 清理守护进程
        for name, proc in _daemon_procs:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass
            logging.info('[DAEMON] %s (pid=%d) 已停止', name, proc.pid)


if __name__ == '__main__':
    main()
