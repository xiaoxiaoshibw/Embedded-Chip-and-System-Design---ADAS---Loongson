#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ADAS 节点配置与运行时参数。

所有可调参数集中在此文件中管理，按功能模块分组：
  - 角色与心跳
  - ROS 话题名
  - ESP32 串口
  - 车道宽度估计
  - 横向控制（LKA）
  - 边界约束
  - 纵向控制（ACC / 巡航 / AEB）
  - 前车跟踪与确认
  - 弯道保持
  - 纵向平滑
  - AEB 紧急制动告警
"""

import argparse
import logging
import math
import os
import sys
from dataclasses import dataclass
from typing import Optional


# ── 角色常量 ──
ROLE_PRIMARY = 'primary'
ROLE_BACKUP = 'backup'


@dataclass(frozen=True)
class RuntimeConfig:
    """运行时配置数据类，由命令行参数或环境变量生成。"""
    nano_role: str
    is_primary: bool
    primary_ip: str
    secondary_ip: str
    hb_port: int
    hb_grace_s: float
    log_file: str


def build_arg_parser() -> argparse.ArgumentParser:
    """构建命令行参数解析器，--role 指定主/备角色。"""
    parser = argparse.ArgumentParser(description='Jetson Nano ADAS 节点')
    parser.add_argument(
        '--role',
        type=str,
        choices=[ROLE_PRIMARY, ROLE_BACKUP],
        help='节点角色: primary 或 backup',
    )
    return parser


def resolve_runtime_config(role: Optional[str] = None) -> RuntimeConfig:
    """根据角色和环境变量解析运行时配置。

    优先级：命令行 role > 环境变量 NANO_ROLE > 默认 primary。
    """
    nano_role = (role or os.environ.get('NANO_ROLE', ROLE_PRIMARY)).lower()
    return RuntimeConfig(
        nano_role=nano_role,
        is_primary=(nano_role == ROLE_PRIMARY),
        primary_ip=os.environ.get('PRIMARY_IP', '192.168.31.131'),
        secondary_ip=os.environ.get('SECONDARY_IP', '192.168.31.161'),
        hb_port=int(os.environ.get('HB_PORT', '9877')),
        hb_grace_s=float(os.environ.get('HB_GRACE', '3.0')),
        log_file=f'/tmp/adas_{nano_role}.log',
    )


# ── ROS 话题名 ──
TOPIC_JETSON_PSI = '/jetson/psi'           # Jetson 计算的航向角
TOPIC_JETSON_DELTA = '/jetson/delta'       # Jetson 计算的方向盘转角
TOPIC_JETSON_BRAKE = '/jetson/brake'       # Jetson 计算的纵向加速度指令
TOPIC_JETSON_LANE_OFFSET = '/jetson/lane_offset'  # 车道横向偏移输出
TOPIC_ESP32_PSI = '/esp32/psi'             # ESP32 回读航向角
TOPIC_ESP32_DELTA = '/esp32/delta'         # ESP32 回读方向盘转角
TOPIC_ESP32_BRAKE = '/esp32/brake'         # ESP32 回读制动
TOPIC_JETSON_ACTIVE_ROLE = '/jetson/active_role'  # 当前激活角色
TOPIC_JETSON_LANE_WIDTH_EST = '/jetson/lane_width_est'  # 估计车道宽
TOPIC_JETSON_LEAD_CLS = '/jetson/lead_cls' # 主前车 actor class（Int32），便于俯瞰图叠加监控
# 主机持有备机心跳 watchdog，超时则发布 False，告知上层"现在没有 plan B"。
# 备机端始终发布 True（备机本身就是 plan B）。
TOPIC_JETSON_FAILOVER_AVAILABLE = '/jetson/failover_available'  # 主备冗余是否可用
TOPIC_CAR1_XY = '/car1_xy'                 # 自车位姿
TOPIC_CAR1_PSI = '/car1_psi'               # 自车航向
TOPIC_CAR2_XY = '/car2xy'                  # 前车位姿
TOPIC_CAR1_V = '/car1_v'                   # 自车速度
TOPIC_CAR2_V = '/car2_v'                   # 前车速度
TOPIC_ROAD_PSI = '/road_psi'               # 道路航向
TOPIC_HENG_ERROR = '/heng_error'            # 车道横向偏移
TOPIC_SET_PARAM = '/adas/set_param'         # 运行时增益热更新 (String "NAME=VALUE")

# ── 多目标跟踪 ──
# MULTI_TARGET_COUNT=1：完全等价于原单前车行为（不订阅额外话题、tracker 不介入）。
# >1 时额外订阅 car3..carN（编号沿用 car2 之后），做简单航迹管理 + cut-in 预判。
# car2 仍是基准前车（话题名 /car2xy /car2_v 保持历史不变）。
MULTI_TARGET_COUNT = 1                      # 跟踪车辆总数（含 car2），1=关闭多目标
MULTI_TARGET_TOPIC_XY_FMT = '/car{}_xy'    # 额外车位姿话题格式（car3 起）
MULTI_TARGET_TOPIC_V_FMT = '/car{}_v'      # 额外车速度话题格式（car3 起）
# actor 分类（Int32 话题，可选）。未发布时 _Target.cls 保持 ACTOR_CLASS_UNKNOWN，
# 下游按"车辆"语义处理，行为与未启用 class 时一致。
TOPIC_CAR2_CLASS = '/car2_class'           # 基准前车分类（与 car2xy/_v 配套）
MULTI_TARGET_TOPIC_CLASS_FMT = '/car{}_class'  # 额外 actor 分类话题格式（car3 起）
ACTOR_CLASS_UNKNOWN = 0
ACTOR_CLASS_VEHICLE = 1
ACTOR_CLASS_OBSTACLE = 2
ACTOR_CLASS_PEDESTRIAN = 3

# （AEB 按 class 差异化的乘子表见下方 AEB_MAX_ENGAGE_DIST 之后；
#  在此处只声明 ACTOR_CLASS_* 常量，乘子表需要 AEB_MAX_ENGAGE_DIST 已定义。）
MULTI_TARGET_FWD_MAX = 60.0                # 纳入跟踪的最大前向距离 (m)
MULTI_TARGET_FWD_MIN = 0.5                 # 最小前向距离 (m，过滤并排/后方)
# cut-in 预判：相邻车道目标横向逼近本车道时提前纳入
CUTIN_HORIZON_S = 1.2                      # 横向位置预测时域 (s)
CUTIN_CORRIDOR_RATIO = 1.7                 # 相邻走廊宽 = 车道横向窗口 × 此比例
CUTIN_MIN_LAT_RATE = 0.08                  # 视为切入的最小横向逼近速率 (m/s)
CUTIN_LAT_RATE_ALPHA = 0.15                # 横向逼近速率低通系数（抑制差分噪声）

# ── 纵向控制器选择 ──
# 'pid'（默认）：原 LongitudinalController，行为与改造前逐字节一致。
# 'mpc'：有限时域 LQ + 约束投影控制器（构造时算一次增益，在线 O(1)），
#        带 PID 自动回退。Jetson 算力安全（微秒级，远低于 8ms 预算）。
LON_CONTROLLER = 'pid'

# ── Lateral controller selection ───────────────────────────
# 'pid' (default): existing heading-PID + curvature FF + CTE path.
# 'stanley': model-based path-tracking drop-in with automatic fallback to PID.
LAT_CONTROLLER = 'pid'
STANLEY_K_CTE = 0.85
STANLEY_SOFTENING_V = 2.0
STANLEY_HEADING_GAIN = 1.0
STANLEY_CTE_MAX = 1.2

# ── Comfort layer selection ────────────────────────────────
# 'legacy' (default): existing LonSmoothing path only.
# 'jerk': jerk-bounded target shaper before LonSmoothing for ACC/cruise comfort.
COMFORT_LAYER = 'legacy'
COMFORT_JERK_ACCEL = 0.8
COMFORT_JERK_DECEL = 1.4
COMFORT_JERK_RELEASE = 1.0

# ── ML 推理开关 ──
# 全部可由环境变量逐机覆盖（部署时写在 adas.env，无需改代码）。
#
# ADAS_ML_ENABLED=1 开 ML：加载 ml/ml/ 中训练好的模型作为纵向控制辅助信号。
# 默认 0（关）：不加载 ML 模型，控制行为与改造前完全一致。
# 需要推理后端（onnxruntime 或 PyTorch）+ numpy；缺失时自动降级为关。
ML_ENABLED = os.environ.get('ADAS_ML_ENABLED', '0') == '1'

# 推理后端 ADAS_ML_BACKEND：
# 'onnx'（推荐）：onnxruntime 跑 checkpoints/*.onnx。更轻（~50MB）、更快（实测约 2x）、
#                 尾部延迟更紧、内存小，aarch64/龙芯都装得动 → 边缘部署首选。
# 'torch'：PyTorch 跑 *.pt（开发机回归/对拍用；依赖重，~1GB）。
# 'auto'：优先 onnx，不可用再退 torch，都不行则禁用 ML。
ML_BACKEND = os.environ.get('ADAS_ML_BACKEND', 'onnx')

# 推理线程数 ADAS_ML_THREADS。Nano 上务必设 1：实测多线程会因线程池调度产生 >10ms
# 尾部尖峰，单线程尾部全程干净（详见 ml/ml/bench_inference.py 基准）。0=用后端默认。
ML_NUM_THREADS = int(os.environ.get('ADAS_ML_THREADS', '1'))

# ADAS_ML_ASYNC=1（默认，推荐）：推理在守护线程异步执行，控制环只做非阻塞入队 +
#   原子读最新结果，彻底把推理耗时与 100Hz/10ms 预算解耦 —— 即便单次推理偶发变慢也不吃 tick。
# 0：推理同步内联在控制环里（离线 replay/run_scenario 需要逐帧确定性时可用；
#   但 Nano 实时部署不要用 0）。
ML_ASYNC = os.environ.get('ADAS_ML_ASYNC', '1') == '1'

MPC_TS = 0.10                              # MPC 内部模型步长 (s)，用于增益设计
MPC_Q_E = 1.0                              # 车距误差权重
MPC_Q_V = 2.5                              # 相对速度权重（偏大→更早收敛接近速度）
MPC_R = 12.0                               # 控制量权重（偏大→更柔和）
MPC_RICCATI_ITERS = 200                    # 构造时 Riccati 迭代上限（收敛即停）
MPC_LEAD_FF_GAIN = 1.0                     # 前车加速度前馈增益

# ── 前车状态估计器选择 ──
# 'legacy'（默认）：前车投影速度走 LeadTracker 一阶低通、加速度走有限差分+低通，
#                   与改造前逐字节一致（回滚路径）。
# 'kalman'：恒加速度（CA）卡尔曼滤波器同时估计投影速度与加速度，
#           带 Mahalanobis 新息门控做单帧野值剔除（替代手写 glitch 检测），
#           给 ACC 前馈与 AEB 触发时机更平滑、更低滞后的状态估计。
#           异常自动永久降级为"直通测量+零加速度"，绝不丢控制。
LEAD_ESTIMATOR = 'legacy'
# CA 卡尔曼过程噪声（jerk 功率谱密度，(m/s^3)^2/Hz 量级）：越大越信任测量、跟得快但更抖。
LEAD_KF_JERK_PSD = 6.0
# 投影速度量测方差 ((m/s)^2)：毫米波/真值速度噪声约 0.3~0.5 m/s → 方差 ~0.1~0.25。
LEAD_KF_MEAS_VAR = 0.20
# 初始协方差先验（重获/切换后用大先验快速收敛到测量，避免低通慢爬升滞后）。
LEAD_KF_INIT_V_VAR = 1.0                   # 初始速度方差 ((m/s)^2)
LEAD_KF_INIT_A_VAR = 4.0                   # 初始加速度方差 ((m/s^2)^2)
# 新息门控：|新息| 超过 GATE_SIGMA·σ 视为单帧野值，跳过校正仅保留预测。
LEAD_KF_GATE_SIGMA = 4.0
# 连续野值超过此数则接受为"真实突变"重新锁定，防止真实急减速被一直拒掉。
LEAD_KF_MAX_CONSEC_OUTLIERS = 5


# ── ESP32 串口配置 ──
# Jetson Nano 40 针 GPIO 硬件 UART 默认是 /dev/ttyTHS1；
# 若改用 USB 转串口则为 /dev/ttyUSB0。通过环境变量 SERIAL_ESP32 覆盖，
# 不必改代码即可适配不同接线/板子。
SERIAL_ESP32 = os.environ.get('SERIAL_ESP32', '/dev/ttyTHS1')  # 串口设备路径
BAUDRATE = int(os.environ.get('SERIAL_BAUDRATE', '115200'))     # 串口波特率

# ── 日志与主循环配置 ──
LOG_LEVEL = logging.INFO                   # 日志级别
LOG_EVERY_N_CYCLES = 100                   # 每隔 N 个控制周期输出一次摘要


def _resolve_loop_hz() -> int:
    """主控循环频率。Jetson Nano 算力有限，可用环境变量 LOOP_HZ 降频
    （如 50Hz）。dt = 1/LOOP_HZ 会在 ADAS.py 中自动传导到所有控制器，
    无需改其它代码。限制在 [10, 200] 防止非法值导致 dt 越界
    （CTRL_DT_MAX=0.05s 对应 20Hz，低于 20Hz 时由各控制器自行钳位）。"""
    try:
        hz = int(float(os.environ.get('LOOP_HZ', '100')))
    except (TypeError, ValueError):
        hz = 100
    return max(10, min(200, hz))


LOOP_HZ = _resolve_loop_hz()               # 主控循环频率 (Hz)，可经环境变量覆盖

# ── 实时控制核隔离（线程级 CPU 亲和性，见 rt_affinity.py）──
# 把 100Hz 控制主循环独占到 RT_CONTROL_CORE，进程内其余线程（DDS / ML / 串口 / 遥测 /
# 日志 / 心跳）赶到剩余允许核——控制核保持"干净"，压低循环尾延迟抖动。仅在进程允许集含
# 控制核且核数 ≥ 2 时生效（Nano 上经 systemd CPUAffinity / taskset 钉到 0,1 后 = 控制
# 核 0 + 辅助核 1）；开发机 / SIL / offline 工具不调用，天然 no-op。环境变量可覆盖。
RT_THREAD_PIN = os.environ.get('RT_THREAD_PIN', '1') not in ('0', 'false', 'False')
RT_CONTROL_CORE = int(os.environ.get('RT_CONTROL_CORE', '0'))
RT_PIN_RESWEEP_S = 3.0                      # 守护线程重扫间隔 (s)，覆盖后启动的线程

# ── 单板软件双核锁步（lockstep，见 lockstep.py）──
# 主核(core0)与影子/Checker 核(LOCKSTEP_CHECKER_CORE)对同一拍输入各算一遍控制管线，
# 逐拍比较 delta/lon/AEB，连续 LOCKSTEP_DEBOUNCE_N 拍失配即报故障进安全态。默认关闭
# （开启会在控制核外多算一遍 + 每拍深拷贝前状态，仅用于演示/验证）。环境变量可覆盖。
LOCKSTEP_ENABLED = os.environ.get('LOCKSTEP_ENABLED', '0') in ('1', 'true', 'True')
LOCKSTEP_CHECKER_CORE = int(os.environ.get('LOCKSTEP_CHECKER_CORE', '2'))
LOCKSTEP_DELTA_EPS = 1e-9                   # delta 失配阈值 (rad)；同 CPU 同码应 bit 一致
LOCKSTEP_LON_EPS = 1e-9                     # lon_cmd 失配阈值 (m/s²)
LOCKSTEP_DEBOUNCE_N = 2                     # 连续失配多少拍才报故障（滤抖）
LOCKSTEP_INJECT = os.environ.get('LOCKSTEP_INJECT', '0') in ('1', 'true', 'True')
LOCKSTEP_INJECT_DELTA = 0.05               # 注入故障时影子 delta 偏移 (rad)，演示用
LOCKSTEP_SAFE_BRAKE_CMD = 2.5              # 失配安全态受控制动量 (m/s²，正=制动)

# ── 心跳参数 ──
# 接管时序预算：阈值 + 接收轮询(≤5ms, socket timeout) + 备机首拍解算发帧(≤10ms)
# ≈ 备机就绪 ~50ms，必须 < ESP32 主控失活仲裁阈值 JETSON_TIMEOUT_MS（main.c / bridge_config
# 现为 58ms），否则 ESP32 在备机接上之前先进"选中源过期/看门狗"全力制动（SRC 0→9→1），
# 接管不再无感。0.035s = 连续丢 3.5 帧 100Hz 心跳才判失效——再小会被主控帧正常抖动误判
# 失活（备机空转；本机软件帧抖动 ~30ms，真机专用 UART 更小）。该组合由接管时延极限扫描实验
# 确定（仿真/experiment_takeover_limit.py：HB=0.035s / JETSON=58ms / 轮询 5ms 是 3/3
# "干净接管 + 健康期 ESP32 零误切" 的可靠下限，实测接管≈47ms，较旧 0.08s/150ms 的 156ms
# 快约 3.3×）。主机帧新鲜时 ESP32 仍优先主机，恢复后备机经 HB_STANDBY_HANDOFF_S 回 standby。
HEARTBEAT_TIMEOUT_S = 0.035                # 心跳超时判定阈值 (s)
HB_SEND_INTERVAL_S = 0.01                  # 心跳发送间隔 (s)，100Hz；须显著小于 HEARTBEAT_TIMEOUT_S
# 备机检测到主机恢复后再保持 active 一段时间，给主机一个完成控制初始化的窗口，
# 避免"备机让出 → 主机还没就绪"的真空。
HB_STANDBY_HANDOFF_S = 0.3
# 主机侧 watchdog：超过此窗口未收到备机存活心跳，则视为 failover 不可用。
# 比 HEARTBEAT_TIMEOUT_S 宽松多档，避免备机偶发抖动误报"无冗余"。
HB_BACKUP_TIMEOUT_S = 2.0

# ── 车道宽度估计：左右偏移样本取 95 分位，再低通滤波 ──
LANE_DEFAULT_WIDTH = 3.8                   # 默认单车道宽度 (m)
LANE_WIDTH_MIN = 3.5                       # 车道宽下限 (m)
LANE_WIDTH_MAX = 14.0                      # 车道宽上限 (m)
LANE_EST_MIN_SAMPLES = 60                   # 最少样本数才触发重算
LANE_EST_TIMEOUT_S = 2.0                   # 偏移数据超时则锁定车道宽
LANE_WIDTH_FILTER_ALPHA = 0.008            # 低通滤波系数 (越小越平滑)
LANE_WIDTH_MAX_RATE = 0.15                 # 车道宽每周期最大变化量 (m)

# ── 估计窗口 ──
LANE_EST_WINDOW_STRAIGHT = 20.0            # 直道窗口长度 (s·Hz)
LANE_EST_WINDOW_CURVE = 6.0                # 弯道窗口长度 (s·Hz)
LANE_EST_CURV_THRESH = 0.008               # 曲率阈值：超过则用弯道窗口

# ── 高速弯道下的横向偏移补偿 ──
K_LAT_COMP = 0.18                           # 横向补偿增益
LANE_EST_PERCENTILE = 95                    # 取偏移的分位数

# ── LKA 参数：航向 PID + 曲率前馈 + CTE 修正 ──
K_PSI_P = 0.9                              # 航向比例增益
K_PSI_I = 0.06                             # 航向积分增益
K_PSI_D = 0.03                             # 航向微分增益
MAX_PSI_ERR = math.radians(60)             # 航向误差限幅 (rad)
MAX_PSI_I = math.radians(8)                # 航向积分限幅 (rad)
MAX_PSI_D = math.radians(180)              # 航向微分限幅 (rad)

# ── 预览控制参数 ──
K_PREVIEW_GAIN = 0.55                      # 预览增益
MAX_DELTA = math.radians(25)               # 最大方向盘转角 (rad)
MAX_DELTA_RATE = math.radians(50)          # 方向盘转角变化率限幅 (rad/s)
K_DELTA = 1.4                              # 航向误差到转角的转换增益
STEER_SIGN = 1.0                           # 转向方向符号
WHEEL_BASE = 3.0                           # 轴距 (m)

# ── 弯道预览衰减 ──
CURVE_PREVIEW_ATTEN_MAX = 0.45             # 最大预览衰减比例
CURVE_PREVIEW_ATTEN_SCALE = 0.020          # 衰减曲率缩放

# ── 预览时间配置 ──
PREVIEW_TIME_MIN = 0.8                     # 最小预览时间 (s)
PREVIEW_TIME_MAX = 2.0                     # 最大预览时间 (s)
PREVIEW_SPEED_REF = 16.7                   # 预览时间参考速度 (m/s)

# ── 曲率前馈参数 ──
K_FF_CURV = 0.4                            # 曲率前馈增益
MAX_FF_DELTA = math.radians(20)            # 最大前馈转角 (rad)
CURVE_FF_ATTEN_MAX = 0.35                 # 弯道前馈最大衰减
CURVE_FF_ATTEN_SCALE = 0.050              # 弯道前馈衰减缩放

# ── CTE 横向偏移修正 ──
K_CTE = 0.06                               # CTE 比例增益
K_CTE_D = 0.02                             # CTE 微分增益
MAX_CTE_CORR = math.radians(12)            # CTE 修正最大转角 (rad)
CTE_EFFECTIVE_LIMIT = 1.2                  # CTE 有效限幅 (m)
MAX_CTE_DOT = 5.0                          # CTE 微分项限幅 (m/s)
CTE_FILTER_ALPHA = 0.1111                  # CTE 低通滤波系数
CURVE_CTE_BOOST_MAX = 0.60                 # 弯道 CTE 增益提升上限
CURVE_CTE_BOOST_SCALE = 0.020             # 弯道 CTE 提升缩放

# ── 道路航向滤波器 ──
ROAD_PSI_FILTER_ALPHA = 0.25               # 道路航向低通系数

# ── 弯道检测参数 ──
CORNERING_RRATE_THRESH = 0.08              # 转向率阈值 (rad/s)
I_DECAY_IN_CORNER = 0.97                   # 弯道中 I 项衰减系数

# ── 控制周期/积分守卫 ──
CTRL_DT_MIN = 0.002                        # 控制周期下界 (s) — 防止除零/微分爆裂
CTRL_DT_MAX = 0.050                        # 控制周期上界 (s)
PSI_I_LOW_SPEED_GATE = 0.5                 # 航向 I 项启用最小速度 (m/s)
PSI_I_LOW_SPEED_DECAY = 0.95               # 低速下 I 项每周期衰减系数
BOUNDARY_DELTA_RATE_MULT = 2.0             # 边界修正可超出 rate limit 的倍数

# ── 边界约束参数：软约束预警，硬约束强制修正 ──
VEHICLE_HALF_WIDTH = 0.90                  # 车辆半宽 (m)
MIN_LANE_SAFE_MARGIN = 0.5                 # 最小车道安全余量 (m)
LANE_WARN_RATIO = 0.55                     # 预警边界占安全余量比例
LANE_HARD_RATIO = 0.92                     # 硬边界占安全余量比例
K_LATERAL_SOFT = 0.45                      # 软边界修正增益
K_LATERAL_HARD = 1.10                      # 硬边界修正增益
BOUNDARY_BRAKE_EXTRA = 1.4                 # 边界制动附加增益

# ── ACC 参数：距离/相对速度控制，叠加前车加速度前馈 ──
ACC_D0 = 2.5                               # 最小跟车间距 (m)
ACC_TIME_GAP = 2.0                         # 时距系数 (s)
ACC_KD = 0.4                               # 距离误差增益
ACC_KI = 0.02                              # 积分增益
ACC_KV = 0.8                               # 速度差增益
ACC_KA = 1.0                                # 前车加速度前馈增益
ACC_FF_MAX = 0.6                            # 前馈限幅 (m/s²)

# ── 滤波器系数 ──
# ACC v_rel 非对称低通：
#   - 朝"更接近"方向（v_rel 减小，自车逼近前车）→ 用 CLOSING，响应快，
#     让 ACC 提前 1~2 拍跟上前车减速，减少把工况推给 AEB 的概率；
#   - 朝"更远离"方向（v_rel 增大）→ 用 OPENING，保持原慢响应，抑制驱动侧抖动。
ACC_VDIFF_ALPHA_CLOSING = 0.40             # 接近方向低通系数（快响应）
ACC_VDIFF_ALPHA_OPENING = 0.10             # 远离方向低通系数（慢响应，等价原 0.1）

# ── I 项限幅与保护 ──
ACC_I_MAX = 1.5                            # 积分项最大值 (m)
ACC_I_PAUSE_VDIFF = 1.5                    # 速度差低于此暂停积分 (m/s)
ACC_GAP_ERR_DRIVE_CAP = 8.0               # 加速方向距离误差上限 (m)
ACC_GAP_ERR_BRAKE_CAP = 6.0               # 制动方向距离误差上限 (m)
ACC_DRIVE_MAX_BASE = 0.6                   # 加速上限基准 (m/s²)
ACC_DRIVE_MAX_GAIN_V = 0.30                # 加速上限速度增益
ACC_DRIVE_MAX_GAIN_D = 0.05                # 加速上限距离增益
ACC_DRIVE_MAX_LIMIT = 1.6                  # 加速上限最大值 (m/s²)
ACC_BRAKE_MAX_BASE = 1.2                  # 制动上限基准 (m/s²)
ACC_BRAKE_MAX_GAIN_V = 0.35                # 制动上限速度增益
ACC_BRAKE_MAX_GAIN_D = 0.08                # 制动上限距离增益
ACC_BRAKE_MAX_LIMIT = 2.5                  # 制动上限最大值 (m/s²)
ACC_STEADY_GAP_BAND = 0.8                  # 稳态距离带宽 (m)
ACC_STEADY_VREL_BAND = 0.30                # 稳态速度差带宽 (m/s)
ACC_I_DECAY_SAT = 0.98                     # 饱和时积分衰减
ACC_I_DECAY_STEADY = 0.92                  # 稳态时积分衰减
LON_CMD_MAX_BRAKE_DECEL = 6.0              # 最大制动减速度指令 (m/s²)
LON_CMD_MAX_DRIVE_ACCEL = 6.0              # 最大驱动加速度指令 (m/s²)
LEAD_TIMEOUT_S = 0.5                       # 前车数据超时 (s)

# ── 巡航控制参数 ──
CRUISE_KP = 0.25                           # 巡航比例增益
DRIVER_SET_SPEED = 8.0                     # 驾驶员设定速度 (m/s)
SYSTEM_MAX_CRUISE = 10.0                   # 系统最高巡航速度 (m/s)
ROAD_LIMIT_SPEED = 8.0                     # 道路限速 (m/s)

# ── AEB TTC 参数 ──
TTC_BRAKE_START = 15.0                     # TTC 开始制动阈值 (s)
TTC_BRAKE_FULL = 5.0                       # TTC 全制动阈值 (s)
AEB_SAFE_DIST_BUFFER = 8.0                # 安全距离缓冲 (m)

# ── 安全距离计算参数 ──
SAFE_REACTION_TIME = 0.35                   # 反应时间 (s)
SAFE_EGO_MAX_DECEL = 6.0                   # 自车最大减速度 (m/s²)
SAFE_LEAD_MAX_DECEL = 8.0                  # 前车最大减速度 (m/s²)
SAFE_DIST_STANDSTILL = 6.0                 # 静止安全距离 (m)
SAFE_DIST_MAX = 120.0                      # 最大安全距离 (m)
SAFE_DIST_LOW_SPEED_REF = 8.0              # 低速参考安全距离 (m)

# ── 弯道速度控制参数 ──
CORNERING_MAX_LAT_ACCEL = 2.2              # 弯道最大侧向加速度 (m/s²)
CORNERING_SPEED_MIN = 3                    # 弯道最低速度 (m/s)

# ── 滤波器系数 ──
CURV_FILTER_ALPHA = 0.12                   # 曲率低通系数
VTGT_FILTER_ALPHA = 0.0204                 # 目标速度低通系数
LON_FILTER_ALPHA = 0.1429                  # 纵向指令低通系数

# ── 前车检测参数 ──
LEAD_LAT_STRAIGHT_RATIO = 0.33            # 直道横向比例门限
LEAD_LAT_CURVE_RATIO = 0.18                # 弯道横向比例门限
LEAD_LAT_MAX_STRAIGHT_MIN = 1.8            # 直道最小横向范围 (m)
LEAD_LAT_MAX_STRAIGHT_MAX = 3.5            # 直道最大横向范围 (m)
LEAD_LAT_MAX_CURVE_MIN = 1.2              # 弯道最小横向范围 (m)
LEAD_LAT_MAX_CURVE_MAX = 2.4              # 弯道最大横向范围 (m)

# ── 曲率相关阈值 ──
CURV_LEAD_THRESH = 0.01                    # 判定弯道的前车检测曲率阈值
AEB_CURV_SUPPRESS_MAX = 0.50              # AEB 弯道抑制上限
AEB_CURV_SCALE = 0.03                     # AEB 弯道抑制缩放

# ── 前车确认机制 ──
LEAD_CONFIRM_CYCLES = 5                   # 确认前车所需连续周期数

# ── TTC AEB 限制参数 ──
TTC_AEB_MAX_DIST = 20.0                   # TTC AEB 最大距离 (m)
TTC_AEB_MAX_LAT_RATIO = 0.60              # TTC AEB 最大横向比
AEB_MAX_ENGAGE_DIST = 25.0                # AEB 最大触发距离 (m)

# ── AEB 按 class 差异化 ──
# 启用条件：MULTI_TARGET_COUNT>1 且 Simulink 端发布了 /car{N}_class。
# 未启用时主前车 lead_cls 维持 ACTOR_CLASS_UNKNOWN，乘子=1.0，与原行为逐字节一致。
#   - TTC_MULT >1 → 更早进入服务/全制动（行人 > 障碍 > 车）
#   - ENGAGE_DIST 放宽 → 易碎目标更远就开始介入
#   - BYPASS_MIN_LEAD_V → 静止障碍/行人 lead_v≈0，直接绕过 ACC_MIN_VALID_LEAD_V/
#     ACC_CLOSE_SLOW_LEAD_DIST 网关，否则 AEB 永远进不去
AEB_CLASS_TTC_MULT = {
    ACTOR_CLASS_UNKNOWN:    1.0,
    ACTOR_CLASS_VEHICLE:    1.0,
    ACTOR_CLASS_OBSTACLE:   1.4,
    ACTOR_CLASS_PEDESTRIAN: 1.6,
}
AEB_CLASS_ENGAGE_DIST = {
    ACTOR_CLASS_UNKNOWN:    AEB_MAX_ENGAGE_DIST,
    ACTOR_CLASS_VEHICLE:    AEB_MAX_ENGAGE_DIST,
    ACTOR_CLASS_OBSTACLE:   32.0,
    ACTOR_CLASS_PEDESTRIAN: 40.0,
}
AEB_CLASS_BYPASS_MIN_LEAD_V = {
    ACTOR_CLASS_UNKNOWN:    False,
    ACTOR_CLASS_VEHICLE:    False,
    ACTOR_CLASS_OBSTACLE:   True,
    ACTOR_CLASS_PEDESTRIAN: True,
}
# AEB 横向门按 class 放宽：行人/障碍的 y_rel 可能在常规车道窗口边缘
# (lead_lat_max * TTC_AEB_MAX_LAT_RATIO) 之外，照搬车辆门会让 aeb_allowed=False
# 直接拒绝 AEB，行人横穿场景永远进不去全制动。这里乘到 lead_lat_gate 上，
# 只放宽 AEB 用横向门，不动 ACC 的 in_lane 判定（在 lead_tracking.py 里）。
AEB_CLASS_LAT_GATE_MULT = {
    ACTOR_CLASS_UNKNOWN:    1.0,
    ACTOR_CLASS_VEHICLE:    1.0,
    ACTOR_CLASS_OBSTACLE:   1.15,
    ACTOR_CLASS_PEDESTRIAN: 1.35,
}
# AEB_CLASS_FULL_CONFIRM_CYCLES 见下方 AEB_FULL_CONFIRM_CYCLES 之后定义
# （此处不能前置引用 AEB_FULL_CONFIRM_CYCLES，它在 502 行）。

# ── 前车跟踪最大距离 ──
LEAD_MAX_TRACK_DIST = 60.0                 # 前车跟踪最大纵向距离 (m)

# ── 前车加速度估计器参数（两阶段低通差分） ──
LEAD_ACCEL_TAU_FAST = 0.05                # 快速滤波时间常数 (s)
LEAD_ACCEL_TAU_SLOW = 0.20                # 慢速滤波时间常数 (s)
LEAD_ACCEL_TAU_DIFF = 0.15                # 差分滤波时间常数 (s)
LEAD_ACCEL_MAX = 4.0                      # 加速度估计限幅 (m/s²)

# ── 纵向平滑参数 ──
LON_RATE_ACCEL_CRUISE = 1.20               # 巡航加速变化率限幅 (m/s³)
LON_RATE_DECEL_CRUISE = 2.50               # 巡航减速变化率限幅 (m/s³)
LON_RATE_ACCEL_ACC = 1.80                  # 跟车加速变化率限幅 (m/s³)（原 1.00 过慢）
LON_RATE_DECEL_ACC = 3.00                  # 跟车减速变化率限幅 (m/s³)
LON_RATE_BRAKE_RELEASE = 4.0              # 制动释放变化率 (m/s³)
LON_RATE_AEB = 60.0                        # AEB 变化率 (m/s³)
LON_RATE_BOUNDARY = 8.0                    # 边界制动变化率 (m/s³)
LON_OUTPUT_ALPHA = 0.25                    # 输出低通系数

# ── Safety supervisor ──
SAFETY_SUPERVISOR_ENABLED = True
SAFETY_REACTION_TIME = 1.0                 # Dynamic envelope reaction time (s)
SAFETY_DIST_BUFFER = 1.2                   # Extra stand-off over computed safe distance (m)
SAFETY_PREBRAKE_MARGIN = 4.0               # Start pre-braking this far before envelope (m)
SAFETY_PREBRAKE_MAX = 4.5                  # Max supervisor pre-brake before hard envelope (m/s²)
SAFETY_FULL_BRAKE_MARGIN = 0.5             # Full brake inside safe distance minus this margin (m)
SAFETY_CUTIN_TTC = 3.0                     # Pre-brake cut-in targets with short TTC (s)
SAFETY_CUTIN_BRAKE = 3.0                   # Cut-in pre-brake command (m/s²)
SAFETY_CURVE_LAT_ACCEL = 1.2               # Conservative curve speed envelope (m/s²)
SAFETY_CURVE_SPEED_KP = 1.5                # Brake gain above curve speed cap
SAFETY_CTE_WARN = 0.8                      # Lateral error that starts longitudinal support (m)
SAFETY_CTE_HARD = 1.4                      # Lateral error that requests strong decel (m)

# ── 横向平滑参数（LateralSmoothing） ──
# 与 LonSmoothing 对称：常规走速率限幅 + 一阶低通，takeover guard 时按外部 override 收紧。
# LAT_RATE_NORMAL 比 lateral 控制器内部的 MAX_DELTA_RATE 略宽松，作为最外层防跳变兜底。
LAT_RATE_NORMAL = math.radians(80)          # 横向命令常规速率限幅 (rad/s)
LAT_OUTPUT_ALPHA = 0.50                     # 横向输出低通系数

# ── 主备接管期跳变限幅 ──
# 备机刚接管后的一段时间内，对 lon/delta 单独施加更严的变化率限制，
# 避免主机最后一帧与备机第一帧之间出现非物理跳变。
TAKEOVER_GUARD_DURATION_S = 0.2             # 接管保护窗口时长 (s)
TAKEOVER_LON_RATE = 6.0                     # 保护期纵向变化率 (m/s³)
# 当种子的 AEB flag = 1（主机临死前是全制动）时，备机不应原样继承 200ms 全制动。
# 这条更宽松的速率让备机能在保护窗内向自己评估出的较小制动值快速衰减，
# 通常在 100~200ms 内把 lon_tx 从 10.0 拉到 ACC/巡航工况下的数值。
TAKEOVER_LON_RATE_AEB_RELEASE = 12.0        # AEB 种子接管时的衰减速率 (m/s³)
# 主机非 AEB 但前车是行人/障碍：备机接管时也不应快速放车，给自身评估留时间。
# 比 TAKEOVER_LON_RATE 更严（数值更小=更慢变化），避免行人横穿瞬间备机突然加速。
TAKEOVER_LON_RATE_VULNERABLE = 4.0          # 行人/障碍接管时的衰减速率 (m/s³)
TAKEOVER_DELTA_RATE = math.radians(25)      # 保护期方向盘变化率 (rad/s)
# 主备 flapping 抑制：上次接管边沿后此窗口内再次触发，只延长保护期，
# 不重新 reset lon_smooth，避免内部状态被反复覆盖。
TAKEOVER_COOLDOWN_S = 1.0

# ── 安全降级参数 ──
# 控制循环连续异常超过阈值时触发紧急停车，向 ESP32 发送最大制动指令。
CTRL_CONSECUTIVE_ERROR_LIMIT = 5            # 连续异常周期数阈值
# 感知数据中断超过此时间后发送轻制动指令（防止车辆在无控制下继续运动）。
SENSOR_TIMEOUT_BRAKE_S = 0.5                # 感知中断制动触发时间 (s)
SENSOR_TIMEOUT_BRAKE_CMD = 2.0              # 感知中断时的制动力 (m/s²)
# 行人/障碍 class 话题卡帧判定：lead xy/v 正常但 /car{N}_class 停更超过此时间，
# 视为 class 信息陈旧，仅限频 WARNING + 遥测标记，不主动降级 cls 值
# （保留最后一次值更接近实际威胁，强制降到 UNKNOWN 会误关 class-aware AEB）。
LEAD_CLASS_STALE_TIMEOUT_S = 2.0
# 自车/道路话题卡帧判定：received 标志一旦置位永不复位，必须用最近接收
# 时刻判定话题是否停更（Simulink 暂停 / 节点挂死时旧位姿会被当成有效）。
# 默认 0.3s：100Hz 仿真下约 30 帧无更新即视为卡帧。
SENSOR_STALE_TIMEOUT_S = 0.3                # 自车/道路话题卡帧超时 (s)
# 紧急停车 / 感知中断制动 帧发送的最小间隔，避免连续异常时刷屏。
EMERGENCY_STOP_MIN_INTERVAL_S = 0.1         # 100ms：10 个 ESP32 周期

# ── 无数据静默待机（IDLE STANDBY）──
# True（默认）：区分"冷启动/长时间无感知"与"行驶中突然丢数据"——
#   · 从未激活过（上电待机）：直接进入静默待机，不主动刹车、日志大幅限频；
#   · 行驶中丢数据：先按 SENSOR_TIMEOUT_BRAKE 主动轻制动把车刹停（安全），
#     持续无数据超过 STANDBY_ENTER_S 后沉降为静默待机保持。
#   待机期低频发"方向0 + 轻保持刹车"保持帧，ESP32 维持 SRC:0 安静、车被稳住，
#   且帧间隔 < ESP32 通信看门狗(200ms) 不会触发硬件全力制动。
#   感知数据一恢复立即退出待机、恢复控制。
# False：旧行为——自启动起只要无数据就持续发感知中断轻制动。
IDLE_STANDBY_ENABLED = True
# 待机保持帧发送间隔。必须 < ESP32 通信看门狗超时(0.2s)，否则会触发全力制动。
STANDBY_KEEPALIVE_INTERVAL_S = 0.1          # 100ms
# 待机保持帧的制动力（轻保持，仅防溜车；非主动减速）。
STANDBY_HOLD_BRAKE_CMD = 1.0                # m/s²
# 行驶中丢数据后，主动轻制动持续此时长（把车刹停）再沉降为静默待机。
STANDBY_ENTER_S = 3.0                       # s
# 待机期 "仍无数据" 提示日志的最小间隔（进入待机时另打一条）。
STANDBY_LOG_INTERVAL_S = 10.0              # s

# 备机【热待机】：备机在 standby（未接管）时也解算并持续向 ESP32 发控制帧。
# True（默认）：MCU 仲裁在主控帧新鲜时只用主、忽略备帧（主优先）；主控一旦超时，
#   备帧已在 MCU 侧就绪 → 瞬间干净切换，消除“主超时但备机还没开始发帧”竞态窗口里
#   偶发的 ~10 m/s² 全力制动一脚。备机活着时备帧不被采用，故严格更安全。
#   SIL 110 次扫描验证：可用段接管窗 >2 m/s² 冲击 8/100 → 0/100，接管时延不变。
# False：旧行为——备机 standby 不发帧，接管后才开始发（存在竞态冲击尾巴）。
# 注意：开启后备机控制内核全程随感知热运行（lead/curve/overtake 状态保持温启动，
#   接管更连续）；接管沿仍照常消费主控种子对齐 lon/lat 平滑。
BACKUP_HOT_STANDBY = True

# control_loop 单周期耗时超过此阈值视为超时，触发 warning（限频后输出）。
# 默认目标 10ms 周期，留 80% 余量 → 8ms 报警。
CONTROL_LOOP_BUDGET_S = 0.008
# 周期超时日志限频，避免一旦慢下来就刷屏
CONTROL_LOOP_SLOW_LOG_INTERVAL_S = 1.0

# ── 弯道丢失前车匀速保持参数 ──
CURVE_HOLD_CURV_THRESH = 0.006             # 弯道保持触发曲率阈值
CURVE_HOLD_SPEED_KP = 0.35                 # 弯道保持速度比例增益
CURVE_HOLD_SPEED_KI = 0.02                  # 弯道保持速度积分增益
CURVE_HOLD_I_MAX = 2.0                     # 弯道保持速度积分项上限 (m/s·s)
CURVE_HOLD_EXIT_CURV = 0.006               # 弯道退出曲率阈值
CURVE_HOLD_TIMEOUT_S = 8.0                 # 弯道保持超时 (s)
CURVE_HOLD_ACTIVATE_LOSS_S = 0.25          # 前车丢失后激活保持的延迟 (s)
CURVE_HOLD_REACQ_STABLE_S = 0.20           # 前车重新获取的稳定确认时间 (s)
LEAD_KEEPALIVE_S = 0.35                     # 前车丢失后的保活时间 (s)

# ── AEB 紧急制动参数 ──
AEB_EMERGENCY_DIST = 5.0                    # 紧急制动距离 (m)
AEB_ALERT_TIMEOUT_S = 3.0                   # AEB 告警确认超时 (s)
AEB_ALERT_HOLD_TIME_S = 5.0                 # AEB 告警保持时间 (s)
AEB_ALERT_ARM_MIN_LEAD_V = 1.0             # AEB 告警最低前车速度 (m/s)
AEB_ALERT_INVALID_LEAD_V = 0.5              # 前车速度低于此视为无效
AEB_ALERT_ARM_CONFIRM_CYCLES = 3            # AEB 告警确认周期数

# ── 超车（双车道）参数 ──
# 触发条件：前车在前方 OVT_TRIGGER_MIN_DIST_M..OVT_TRIGGER_DIST_M 范围内，
# 且自车与前车均接近静止超过 OVT_CONFIRM_TIME_S。
# 几何前提：双车道，路径=右车道中心，左车道中心相对路径 +OVT_LANE_OFFSET_M
# （heng_error 约定左正右负，与 chart_94 一致）。
OVT_TRIGGER_DIST_M = 30.0                   # 触发超车的最大前车距离 (m)
OVT_TRIGGER_MIN_DIST_M = 3.0                # 触发超车的最小前车距离 (m)，避免几乎贴靠时强行变道
OVT_LEAD_STILL_V = 0.5                      # 前车视为静止的速度阈值 (m/s)
OVT_EGO_STILL_V = 0.5                       # 自车视为静止的速度阈值 (m/s)
OVT_CONFIRM_TIME_S = 2.0                    # 静止确认时间 (s)
OVT_LEAD_LONG_STILL_S = 8.0                 # 前车持续静止多久才视为"长期阻塞"（区分 AEB 短停）
OVT_RESUME_LEAD_V = 1.0                     # 前车恢复行驶视为取消超车的速度阈值 (m/s)
OVT_LANE_OFFSET_M = 3.5                     # 左车道中心相对路径的横向偏移 (m)
OVT_SHIFT_DONE_M = OVT_LANE_OFFSET_M * 0.8  # 视为已切入左车道的偏移阈值 (m)
OVT_LEAD_PASSED_FWD_M = 12.0                # 自车纵向超过前车多少米后开始返回 (m)
OVT_RETURN_DONE_M = 0.4                     # 视为已返回右车道的偏移阈值 (m)
# 目标车道偏移的爬升速率：超车状态机不再瞬时阶跃 target_lane_offset，
# 而是按此速率向目标平移，给横向控制器一个物理可跟踪的换道参考。
OVT_LANE_SHIFT_RATE_M_S = 0.7               # 目标车道偏移爬升速率 (m/s)
# 超车抑制期间的纯巡航参数：避开标准巡航分支里"前车刚丢失/前车记忆未过期"
# 等保护，直接给一个温和的目标速度和加速度上限，保证从静止能稳定起步绕过。
OVT_CRUISE_TARGET_V = 5.0                    # 超车巡航目标速度 (m/s)
OVT_CRUISE_DRIVE_ACCEL = 1.2                 # 超车巡航最大驱动加速度 (m/s²)

# ── 弯道禁止加速阈值 ──
CURV_NO_ACCEL_THRESH = 0.008               # 曲率超过此值禁止加速

# ── 前车检测参数（续） ──
LEAD_MEMORY_S = 1.5                         # 前车位置记忆时间 (s)
LEAD_LOSS_COAST_S = 1.0                     # 前车丢失后巡航过渡时间 (s)
LEAD_CURVE_LOSS_HOLD_S = 1.0                # 弯道前车丢失保持时间 (s)
LEAD_CURVE_HOLD_LAT_RATIO = 0.34            # 弯道保持横向比例
LEAD_CURVE_HOLD_LAT_MAX = 3.8               # 弯道保持最大横向 (m)
LEAD_REL_FILTER_ALPHA = 0.12                # 前车相对位置低通系数
LEAD_V_PROJ_FILTER_ALPHA = 0.18             # 前车投影速度低通系数（原 0.04 过慢导致跟车加速延迟）
LEAD_ACCEL_FILTER_ALPHA = 0.20              # 前车加速度低通系数（原 0.10 过慢）
LEAD_REACQ_PROTECT_S = 1.2                  # 前车重新获取保护时间 (s)
LEAD_REACQ_MIN_PROJ_RATIO = 0.65            # 重新获取最低投影速度比
LEAD_REACQ_MIN_LAST_RATIO = 0.80            # 重新获取最低上轮速度比
LEAD_REACQ_MAX_DECEL = 0.18                 # 重新获取最大减速度假设 (m/s²)
LEAD_DROP_GLITCH_DIST = 20.0                # 前车速度跳变检测距离 (m)
LEAD_DROP_GLITCH_RATIO = 0.55               # 速度跳变比阈值
LEAD_DROP_GLITCH_BLEND = 0.92                # 速度跳变融合系数

# ── 舒适加减速参数 ──
ACC_COMFORT_ACCEL = 0.30                    # 舒适加速度 (m/s²)
ACC_COMFORT_DECEL = 0.45                    # 舒适减速度 (m/s²)
CURVE_EXIT_GRACE_S = 1.2                    # 弯道退出保护时间 (s)

# ── ACC 可用性/舒适性保护参数 ──
ACC_REACQ_DRIVE_MAX = 0.80                  # 重新获取最大驱动 (m/s²)
ACC_REACQ_BRAKE_MAX = 1.50                  # 重新获取最大制动 (m/s²)
ACC_REACQ_FF_MAX = 0.60                     # 重新获取最大前馈 (m/s²)
ACC_NORMAL_BRAKE_MAX = 3.00                 # 常规制动力上限 (m/s²)
AEB_STOP_HOLD_S = 1.2                       # AEB 停车保持时间 (s)
ACC_MATCH_TAU_S = 1.2                        # 速度匹配时间常数 (s)
ACC_MATCH_BRAKE_MARGIN = 0.6                 # 速度匹配制动余量 (m/s²)
ACC_NO_BRAKE_DIST_MARGIN = 3.0              # 距离余量内禁止制动 (m)
AEB_FULL_CONFIRM_CYCLES = 5                 # AEB 全制动确认周期数（车辆/UNKNOWN 默认）
# AEB 全制动确认按 class 收紧：行人/障碍应当更快确认。
# 在 longitudinal_policy.py:300-329 用 .get(cls, AEB_FULL_CONFIRM_CYCLES) 查表，
# 缺 key（如未来扩 class）自动回退到默认值。退出仍 -2 不分 class。
AEB_CLASS_FULL_CONFIRM_CYCLES = {
    ACTOR_CLASS_UNKNOWN:    AEB_FULL_CONFIRM_CYCLES,
    ACTOR_CLASS_VEHICLE:    AEB_FULL_CONFIRM_CYCLES,
    ACTOR_CLASS_OBSTACLE:   3,
    ACTOR_CLASS_PEDESTRIAN: 2,
}
ACC_MIN_VALID_LEAD_V = 2.0                  # ACC 最低有效前车速度 (m/s)
ACC_CLOSE_SLOW_LEAD_DIST = 25.0             # 近距离低速前车判定距离 (m)
ACC_LEAD_KEEPALIVE_S = 0.12                 # ACC 前车保活时间 (s)
ACC_MAX_VALID_TTC_S = 20.0                  # ACC 最大有效 TTC (s)
ACC_LEAD_OPENING_MIN_DIST = 30.0             # 前车远离最小距离 (m)
ACC_LEAD_OPENING_DELTA_M = 0.02              # 前车远离最小距离增量 (m)
ACC_LEAD_COLLAPSE_RATIO = 0.70              # 前车速度骤降比
ACC_LEAD_COLLAPSE_EGO_GAP = 1.0              # 前车速度骤降自车间隔 (m/s)
ACC_LEAD_RELEASE_LANE_OUT_CYCLES = 4          # 前车偏离车道释放周期数
ACC_LEAD_RELEASE_OPENING_CYCLES = 4           # 前车远离释放周期数
ACC_LEAD_ACQUIRE_GRACE_S = 0.8               # 前车获取保护时间 (s)
ACC_LEAD_ACQUIRE_MAX_BRAKE = -0.7             # 前车获取最大制动限制 (m/s²)


# ==========================================================
# 启动期参数覆盖：可选 params.yaml（与本文件同目录）
# ==========================================================
# 设计要点：本工程大量模块用 `from config import *` 在 import 时捕获常量值，
# 之后再改 config.X 不会传导。因此覆盖必须在 config.py 模块加载末尾就地完成，
# 这样任何 `from config import *` 看到的都是覆盖后的值。
#
# 这里只覆盖 config 标量常量，与 runtime 角色配置无关（runtime 在更晚才
# configure），不存在顺序耦合。PyYAML 可选：装了就用，没装回退极简扁平解析，
# 保证 Jetson 控制节点不强依赖 PyYAML。
APPLIED_PARAM_OVERRIDES = {}

# ── 安全关键参数白名单 ──
# 这些参数涉及执行器物理限位、控制周期下界、AEB 触发阈值等安全关键值，
# 不可通过 params.yaml 覆盖。防止误配置导致转向角超限、制动力不足或除零。
_SAFETY_CRITICAL_PARAMS = frozenset({
    'MAX_DELTA',                    # 最大方向盘转角 (执行器物理限位)
    'MAX_DELTA_RATE',               # 方向盘转角变化率限幅
    'LON_CMD_MAX_BRAKE_DECEL',      # 最大制动减速度指令
    'LON_CMD_MAX_DRIVE_ACCEL',      # 最大驱动加速度指令
    'CTRL_DT_MIN',                  # 控制周期下界 (防除零)
    'CTRL_DT_MAX',                  # 控制周期上界
    'AEB_EMERGENCY_DIST',           # 紧急制动距离
    'TTC_BRAKE_FULL',               # TTC 全制动阈值
    'SAFE_EGO_MAX_DECEL',           # 自车最大减速度
    'SAFE_LEAD_MAX_DECEL',          # 前车最大减速度
    'WHEEL_BASE',                   # 轴距 (影响转向几何)
    'STEER_SIGN',                   # 转向方向符号
})


def _coerce_like(old, raw):
    """把 raw 强制成与 old 同类型，类型不兼容则抛异常（拒绝该项）。"""
    if isinstance(old, bool):
        if isinstance(raw, bool):
            return raw
        s = str(raw).strip().lower()
        if s in ('1', 'true', 'yes', 'on'):
            return True
        if s in ('0', 'false', 'no', 'off'):
            return False
        raise ValueError('not a bool: %r' % (raw,))
    if isinstance(old, int) and not isinstance(old, bool):
        return int(float(raw))
    if isinstance(old, float):
        return float(raw)
    if isinstance(old, str):
        return str(raw)
    raise ValueError('unsupported target type %s' % type(old).__name__)


def _parse_params_text(text):
    """极简扁平解析：每行 `KEY: value` 或 `KEY = value`，# 注释，仅标量。

    仅在没有 PyYAML 时兜底用；params.yaml 只放扁平标量即可。
    """
    out = {}
    for line in text.splitlines():
        line = line.split('#', 1)[0].strip()
        if not line:
            continue
        if ':' in line:
            k, _, v = line.partition(':')
        elif '=' in line:
            k, _, v = line.partition('=')
        else:
            continue
        k = k.strip()
        v = v.strip().strip('"\'')
        if k:
            out[k] = v
    return out


def _load_param_overrides():
    """读取同目录 params.yaml（若存在），就地覆盖匹配的标量常量。

    安全约束：
      - 只接受全大写且已存在的 config 常量名（防止注入新全局/改函数）。
      - 只覆盖 int/float/str/bool 标量，类型按原常量强制转换，失败则跳过。
      - 文件不存在 / 解析失败 / 单项非法都不致命，最多跳过该项。
    """
    path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'params.yaml')
    if not os.path.exists(path):
        return
    try:
        with open(path, 'r') as f:
            text = f.read()
    except Exception as e:
        print('[CONFIG] cannot read %s: %s' % (path, e), file=sys.stderr)
        return

    data = None
    try:
        import yaml
        loaded = yaml.safe_load(text)
        if isinstance(loaded, dict):
            data = loaded
    except Exception:
        data = None
    if data is None:
        data = _parse_params_text(text)

    g = globals()
    for k, raw in data.items():
        if not (isinstance(k, str) and k.isupper() and k in g):
            print('[CONFIG] ignore unknown/forbidden param %r' % (k,),
                  file=sys.stderr)
            continue
        if k in _SAFETY_CRITICAL_PARAMS:
            print('[CONFIG] SAFETY CRITICAL: %s cannot be overridden via params.yaml'
                  % k, file=sys.stderr)
            continue
        old = g[k]
        if not isinstance(old, (int, float, str)):
            print('[CONFIG] skip non-scalar param %r' % (k,), file=sys.stderr)
            continue
        try:
            new = _coerce_like(old, raw)
        except Exception as e:
            print('[CONFIG] bad value for %s: %s' % (k, e), file=sys.stderr)
            continue
        g[k] = new
        APPLIED_PARAM_OVERRIDES[k] = (old, new)

    if APPLIED_PARAM_OVERRIDES:
        print('[CONFIG] params.yaml overrides: %s' % ', '.join(
            '%s %r->%r' % (k, o, n)
            for k, (o, n) in APPLIED_PARAM_OVERRIDES.items()),
            file=sys.stderr)


_load_param_overrides()
