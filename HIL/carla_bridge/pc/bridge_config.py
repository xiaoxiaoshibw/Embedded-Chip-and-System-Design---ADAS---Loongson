#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CARLA 联合仿真桥配置。

集中存放桥接侧基础设施参数（端口、坐标符号约定、执行器映射）。
- SOC 控制算法参数在 lx/SOCCode/config.py，桥不重复定义；
- 演示场景（前车脚本/故障时间线）在 scenarios.py。
"""

import os

# ── 路径 ──
_HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(_HERE, 'logs')

# ── CARLA 连接 ──
CARLA_HOST = '127.0.0.1'
CARLA_PORT = 2000
CARLA_TIMEOUT_S = 60.0          # load_world 可能很慢
TOWN = 'Town04'                 # 有高速环路，适合 LKA/ACC 演示
FIXED_DT = 0.05                 # 同步模式步长 = 20Hz，与"仿真端以 20Hz 发布感知"一致

# ── 车辆 ──
EGO_BLUEPRINT = 'vehicle.tesla.model3'
LEAD_BLUEPRINT = 'vehicle.audi.tt'

# ── UDP 端口（全部 localhost）──
SENSOR_PORT_PRIMARY = 9101      # 桥 → 主控 感知帧
SENSOR_PORT_BACKUP = 9102       # 桥 → 备控 感知帧
ESP32_UART_PORT = 9110          # 双 worker → 桥（虚拟 UART，帧前缀 "P "/"B "）
STATUS_PORT = 9120              # worker → 桥 状态 JSON
FAULT_PORT_PRIMARY = 9301       # 桥 → 主控 故障注入指令
FAULT_PORT_BACKUP = 9302
HB_PORT_PRIMARY = 9201          # 主控心跳监听端口（备→主）
HB_PORT_BACKUP = 9202           # 备控心跳监听端口（主→备）

# ── 坐标/符号约定 ──
# 控制器的运动学（yaw += v/L·tan(delta)、lat_e += v·sin(yaw-psi_road)）在
# CARLA 左手系（y 右、yaw 顺时针为正）下自洽：CARLA yaw(rad) 直接作 psi，
# lane_offset 取"右正"，steer 与 delta 同号，全程不做坐标系翻转。
# 推论：超车状态机的 +OVT_LANE_OFFSET_M 在 CARLA 中表现为向【右】借道，
# 超车场景的出生点需保证右侧有车道。
STEER_SIGN = 1.0
LANE_OFFSET_SIGN = 1.0
# Reject spawn transforms that are not on/near a drivable CARLA lane.
SPAWN_WAYPOINT_MAX_DIST_M = 4.0
# steer 归一化：'physical' = delta/前轮最大转角（物理正确）；
# 'max_delta' = delta/MAX_DELTA（与 lx/SOCCode/carla_bridge.py 相同，增益约 2.8 倍）
STEER_MODE = 'physical'

# ── 执行器映射（lon_cmd 正=减速 m/s²）──
THROTTLE_ACCEL_GAIN = 3.0       # 油门=1.0 时近似加速度 (m/s²)
BRAKE_DECEL_GAIN = 8.0          # 刹车=1.0 时近似减速度 (m/s²)
DRAG_COMP = 0.04                # 速度阻力补偿系数：throttle_ff = DRAG_COMP * v
ACCEL_DEADBAND = 0.05           # |a_des| 低于此值不踩油门/刹车
START_THROTTLE_MIN = 0.22        # CARLA Model3 静止起步最小油门
START_THROTTLE_SPEED_MPS = 0.5   # 低于该速度才启用起步最小油门

# ── 虚拟 ESP32（与 lx/MCUcode main.c 一致的常量）──
# JETSON_TIMEOUT_MS = 主控失活仲裁阈值，主→备接管时延≈此值。58ms 为接管时延极限扫描
# 实验确定的“干净接管 + 健康期零误切”可靠下限（仿真/experiment_takeover_limit.py：
# HB=0.035s/轮询5ms 时备机就绪~50ms，58ms 留~8ms 余量；接管≈47ms，原 150）。改此处须
# 同步改 lx/MCUcode/ADAS_Test/main/main.c 同名宏，保持字节级一致。
JETSON_TIMEOUT_MS = 58
WATCHDOG_TIMEOUT_MS = 200
MCU_AEB_MAX_BRAKE_DECEL = 9.99
MCU_AEB_MIN_CLOSING_SPEED = 0.6
SAFE_DIST_SOFT_BUFFER = 3.0
JETSON_LON_CMD_MAX_BRAKE_DECEL = 6.0
JETSON_LON_CMD_MAX_DRIVE_ACCEL = 6.0

# ── ROS2 桥接模式 ──
ROS2_NODE_NAME = 'carla_sim_bridge'
ROS2_WS_PORT = 8766              # WebSocket 广播端口（Web 前端连接此端口）

# ── Web 监控台 ──
WEB_HTTP_HOST = '127.0.0.1'
WEB_HTTP_PORT = 8765
