#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Ubuntu 原生 ROS2 CARLA 桥节点（HIL 闭环 · 取代 Windows TCP 桥 + Nano TCP 网关）。

运行位置：装有 CARLA 的 Ubuntu 24.04 + ROS2 Jazzy（Python 3.12），与 `import rclpy` 同环境。

它把原来「Windows 跑 CARLA → TCP 发感知帧 → Nano 上 hil_ros_gateway.py 翻译成 ROS2 话题」
这两层合并成**一个 rclpy 节点**：CARLA 真值直接 publish 成 ROS2 话题，Nano 上的两台
ADAS.py（Foxy）跨网 DDS 直接订阅；ADAS 回发的 /jetson/* 或 /esp32/* 执行量本节点订阅后
驱动 CARLA 自车。Nano 端从此**不再需要 gateway**，只跑 ADAS.py（domain 43）。

闭环数据流：
    CarlaUE4（同步 20Hz）
      ↕ Python API
    carla_ros_node.py（本节点）
      ├─ publish /car1_xy /car1_psi /car1_v /car2xy /car2_v /car2_class
      │           /road_psi /heng_error          （感知话题，qos_sensor_data）
      └─ subscribe /jetson/{psi,delta,brake} 或 /esp32/{...}
                   /jetson/active_role /jetson/failover_available
                                  ↕ ROS2 DOMAIN 43 · DDS（跨网到双 Nano）
    Primary/Backup Nano：ADAS.py --role primary/backup（真实 LKA/ACC/AEB + 主备接管）

线程模型：主线程跑 CARLA 步进环（world.tick 在同步模式下阻塞 ~50ms），ROS2 executor 在
后台线程 spin 处理订阅回调（更新最新执行量），与原 Windows 桥「主循环 + 异步接收线程」一致。
publish 由步进环线程调用（rclpy 发布线程安全），订阅回调由 spin 线程更新带锁的最新执行量。
"""

import argparse
import csv
import glob
import math
import os
import sys
import threading
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
# 仓库根 = HIL/carla_bridge/pc 上溯三级（CALRA 在仓库根）
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
# 让 bridge_config / carla_link / scenarios 可从任意 CWD 导入
if HERE not in sys.path:
    sys.path.insert(0, HERE)


def _import_carla():
    """优先用环境里已装的 carla；缺失时回退到 CALRA 下的 Linux egg/wheel。"""
    try:
        import carla  # noqa: F401
        return carla
    except ImportError:
        pass
    dist = os.path.join(ROOT, "CALRA", "PythonAPI", "carla", "dist")
    candidates = []
    candidates += sorted(glob.glob(os.path.join(dist, "carla-*linux*.egg")))
    candidates += sorted(glob.glob(os.path.join(dist, "carla-*linux*.whl")))
    candidates += sorted(glob.glob(os.path.join(dist, "carla-*.egg")))
    for path in candidates:
        if path not in sys.path:
            sys.path.append(path)
        try:
            import carla  # noqa: F401
            return carla
        except ImportError:
            continue
    raise SystemExit(
        "无法 import carla。请先在本 Ubuntu 的 Python（与 rclpy 同环境）里装好 CARLA 客户端：\n"
        "  pip install <CALRA>/PythonAPI/carla/dist/carla-0.9.16-cp3xx-linux_x86_64.whl\n"
        "或把对应的 .egg 加入 PYTHONPATH。"
    )


import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64, Int32, String

from bridge_config import CARLA_HOST, CARLA_PORT, FIXED_DT, TOWN
from carla_link import CarlaLink
from scenarios import ORDER, SCENARIOS

# ── 话题名（与 nano/hil_ros_gateway.py 及 lx/SOCCode/config.py 字节级一致）──
TOPIC_CAR1_XY = "/car1_xy"
TOPIC_CAR1_PSI = "/car1_psi"
TOPIC_CAR1_V = "/car1_v"
TOPIC_CAR2_XY = "/car2xy"
TOPIC_CAR2_V = "/car2_v"
TOPIC_CAR2_CLASS = "/car2_class"
TOPIC_ROAD_PSI = "/road_psi"
TOPIC_HENG_ERROR = "/heng_error"

TOPIC_JETSON_PSI = "/jetson/psi"
TOPIC_JETSON_DELTA = "/jetson/delta"
TOPIC_JETSON_BRAKE = "/jetson/brake"
TOPIC_ESP32_PSI = "/esp32/psi"
TOPIC_ESP32_DELTA = "/esp32/delta"
TOPIC_ESP32_BRAKE = "/esp32/brake"
TOPIC_ACTIVE_ROLE = "/jetson/active_role"
TOPIC_FAILOVER_AVAILABLE = "/jetson/failover_available"


def _pose(x, y, yaw=0.0, z=0.0):
    msg = Pose()
    msg.position.x = float(x)
    msg.position.y = float(y)
    msg.position.z = float(z)
    half = 0.5 * float(yaw)
    msg.orientation.z = math.sin(half)
    msg.orientation.w = math.cos(half)
    return msg


class CarlaRosNode(Node):
    """CARLA ↔ ROS2 桥：发感知话题、订阅 Nano 执行量、维护最新执行量供步进环读取。"""

    def __init__(self, args):
        super().__init__("carla_hil_bridge")
        self.args = args
        self._lock = threading.Lock()
        self._latest = {
            "jetson": {"psi": 0.0, "delta": 0.0, "brake": 0.0, "rx_t": 0.0},
            "esp32": {"psi": 0.0, "delta": 0.0, "brake": 0.0, "rx_t": 0.0},
        }
        self._active_role = "unknown"
        self._failover_available = False
        self._stale_timeout_s = float(args.stale_timeout_s)

        # 感知话题：BEST_EFFORT + KEEP_LAST 1（与 ADAS.py sensor_qos 匹配）
        self.pub_car1_xy = self.create_publisher(Pose, TOPIC_CAR1_XY, qos_profile_sensor_data)
        self.pub_car1_psi = self.create_publisher(Float64, TOPIC_CAR1_PSI, qos_profile_sensor_data)
        self.pub_car1_v = self.create_publisher(Float64, TOPIC_CAR1_V, qos_profile_sensor_data)
        self.pub_car2_xy = self.create_publisher(Pose, TOPIC_CAR2_XY, qos_profile_sensor_data)
        self.pub_car2_v = self.create_publisher(Float64, TOPIC_CAR2_V, qos_profile_sensor_data)
        self.pub_car2_class = self.create_publisher(Int32, TOPIC_CAR2_CLASS, qos_profile_sensor_data)
        self.pub_road_psi = self.create_publisher(Float64, TOPIC_ROAD_PSI, qos_profile_sensor_data)
        self.pub_heng_error = self.create_publisher(Float64, TOPIC_HENG_ERROR, qos_profile_sensor_data)

        # 订阅 ADAS.py 回发的执行量与主备状态（默认可靠 QoS，depth 10）
        self.create_subscription(Float64, TOPIC_JETSON_PSI, self._make_cb("jetson", "psi"), 10)
        self.create_subscription(Float64, TOPIC_JETSON_DELTA, self._make_cb("jetson", "delta"), 10)
        self.create_subscription(Float64, TOPIC_JETSON_BRAKE, self._make_cb("jetson", "brake"), 10)
        self.create_subscription(Float64, TOPIC_ESP32_PSI, self._make_cb("esp32", "psi"), 10)
        self.create_subscription(Float64, TOPIC_ESP32_DELTA, self._make_cb("esp32", "delta"), 10)
        self.create_subscription(Float64, TOPIC_ESP32_BRAKE, self._make_cb("esp32", "brake"), 10)
        self.create_subscription(String, TOPIC_ACTIVE_ROLE, self._active_role_cb, 10)
        self.create_subscription(Bool, TOPIC_FAILOVER_AVAILABLE, self._failover_cb, 10)

        self.get_logger().info(
            "CARLA HIL ROS2 桥就绪：scenario=%s source=%s domain=%s"
            % (args.scenario, args.actuation_source, os.environ.get("ROS_DOMAIN_ID", "?"))
        )

    def _make_cb(self, source, field):
        def _cb(msg):
            with self._lock:
                self._latest[source][field] = float(msg.data)
                self._latest[source]["rx_t"] = time.monotonic()
        return _cb

    def _active_role_cb(self, msg):
        with self._lock:
            self._active_role = str(msg.data)

    def _failover_cb(self, msg):
        with self._lock:
            self._failover_available = bool(msg.data)

    # ── 步进环线程调用：把一帧真值感知发到 ROS2 ──
    def publish_sensors(self, frame):
        self.pub_car1_xy.publish(_pose(frame.get("ego_x", 0.0), frame.get("ego_y", 0.0)))
        self.pub_car1_psi.publish(Float64(data=float(frame.get("ego_yaw", 0.0))))
        self.pub_car1_v.publish(Float64(data=float(frame.get("ego_v", 0.0))))
        self.pub_road_psi.publish(Float64(data=float(frame.get("road_psi", 0.0))))
        self.pub_heng_error.publish(Float64(data=float(frame.get("lane_offset", 0.0))))
        if bool(frame.get("lead_present", False)):
            self.pub_car2_xy.publish(_pose(
                frame.get("lead_x", 9999.0),
                frame.get("lead_y", 9999.0),
                frame.get("lead_yaw", 0.0),
            ))
            self.pub_car2_v.publish(Float64(data=float(frame.get("lead_v", 0.0))))
            self.pub_car2_class.publish(Int32(data=int(frame.get("lead_cls", 1))))
        else:
            self.pub_car2_xy.publish(_pose(9999.0, 9999.0))
            self.pub_car2_v.publish(Float64(data=0.0))
            self.pub_car2_class.publish(Int32(data=0))

    # ── 步进环线程调用：取最新执行量（带超时安全兜底）──
    def get_actuation(self):
        with self._lock:
            source = self.args.actuation_source
            act = dict(self._latest[source])
            active_role = self._active_role
            failover = self._failover_available
        age = time.monotonic() - act["rx_t"] if act["rx_t"] else 9999.0
        if act["rx_t"] == 0.0 or age > self._stale_timeout_s:
            # 链路丢失 / 从未收到 → 安全制动兜底（与原 Windows 桥一致）
            return {
                "psi": 0.0,
                "delta": 0.0,
                "a_brake": 6.0,
                "source": "stale",
                "active_role": "unknown",
                "failover_available": False,
                "age_s": age,
            }
        return {
            "psi": act["psi"],
            "delta": act["delta"],
            "a_brake": act["brake"],
            "source": source,
            "active_role": active_role,
            "failover_available": failover,
            "age_s": age,
        }


def carla_loop(node, link, args, scenario):
    """主线程：CARLA 同步 20Hz 步进环——tick → sense → publish → 读执行量 → apply → 记 CSV。"""
    duration = float(args.duration if args.duration is not None else scenario.get("duration", 0.0))
    log_dir = os.path.join(HERE, "logs")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(
        log_dir, "hil_%s_%s.csv" % (args.scenario, datetime.now().strftime("%Y%m%d_%H%M%S")))
    print("CSV log: %s" % log_path, flush=True)

    seq = 0
    last_print = 0.0
    start_wall = time.monotonic()
    with open(log_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "t", "seq", "ego_v", "lead_present", "gap", "lane_offset",
            "source", "active_role", "failover_available",
            "delta", "a_brake", "steer", "throttle", "brake", "actuation_age_s",
        ])
        while rclpy.ok():
            sim_t = link.tick()
            if duration > 0.0 and sim_t >= duration:
                break

            frame, gap = link.sense(sim_t)
            node.publish_sensors(frame)

            link.drive_lead(sim_t)
            act = node.get_actuation()
            steer, throttle, brake = link.apply_ego(act)
            link.update_spectator()

            writer.writerow([
                "%.3f" % sim_t, seq,
                "%.3f" % frame.get("ego_v", 0.0),
                int(bool(frame.get("lead_present", False))),
                "%.3f" % gap if gap != float("inf") else "inf",
                "%.3f" % frame.get("lane_offset", 0.0),
                act["source"], act["active_role"], int(act["failover_available"]),
                "%.6f" % act["delta"], "%.3f" % act["a_brake"],
                "%.4f" % steer, "%.4f" % throttle, "%.4f" % brake,
                "%.3f" % act["age_s"],
            ])
            fh.flush()

            if sim_t - last_print >= 1.0:
                print(
                    "t=%5.1f seq=%d v=%.2f gap=%s src=%s role=%s delta=%.3f brake=%.2f age=%.0fms"
                    % (sim_t, seq, frame.get("ego_v", 0.0),
                       "%.1f" % gap if gap != float("inf") else "inf",
                       act["source"], act["active_role"], act["delta"],
                       act["a_brake"], act["age_s"] * 1000.0),
                    flush=True,
                )
                last_print = sim_t

            seq += 1
            if args.realtime:
                target = start_wall + sim_t
                sleep_s = target - time.monotonic()
                if sleep_s > 0.0:
                    time.sleep(min(sleep_s, 0.05))


def build_arg_parser():
    parser = argparse.ArgumentParser(description="Ubuntu 原生 ROS2 CARLA HIL 桥节点")
    parser.add_argument("--scenario", default="acc", choices=ORDER)
    parser.add_argument("--actuation-source", choices=["jetson", "esp32"], default="jetson",
                        help="读哪一路执行量驱动自车：jetson=Jetson 直出；esp32=ESP32 仲裁后")
    parser.add_argument("--stale-timeout-s", type=float, default=0.5,
                        help="执行量超过此时长未更新 → 安全制动兜底")
    parser.add_argument("--carla-host", default=CARLA_HOST)
    parser.add_argument("--carla-port", type=int, default=CARLA_PORT)
    parser.add_argument("--town", default=TOWN)
    parser.add_argument("--spawn-index", type=int, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--no-rendering", action="store_true")
    parser.add_argument("--no-realtime", dest="realtime", action="store_false")
    parser.set_defaults(realtime=True)
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    carla = _import_carla()
    scenario = SCENARIOS[args.scenario]

    rclpy.init()
    link = CarlaLink(
        carla, args.carla_host, args.carla_port, scenario,
        town=args.town, no_rendering=args.no_rendering, spawn_index=args.spawn_index,
    )
    node = CarlaRosNode(args)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="ros-spin", daemon=True)
    spin_thread.start()

    print("CARLA HIL ROS2 桥启动，actuation source=%s（须与两台 Nano 的 ADAS.py 同 domain 43）"
          % args.actuation_source, flush=True)
    try:
        carla_loop(node, link, args, scenario)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            link.close()
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
