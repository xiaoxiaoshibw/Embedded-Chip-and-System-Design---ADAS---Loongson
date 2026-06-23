#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS2 gateway that connects CARLA UDP frames to the existing Nano ADAS node.

Run on the primary Nano:
    source /opt/ros/foxy/setup.bash
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
    python3 hil_ros_gateway.py --pc-host 192.168.3.8 --actuation-source jetson
"""

import argparse
import json
import socket
import threading
import time

try:
    import rclpy
    from geometry_msgs.msg import Pose
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool, Float64, Int32, String
except ModuleNotFoundError:
    rclpy = None
    Node = object
    Pose = None
    qos_profile_sensor_data = None
    Bool = Float64 = Int32 = String = None

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

def _now_ms():
    return int(time.monotonic() * 1000)


def _pose(x, y, z=0.0):
    msg = Pose()
    msg.position.x = float(x)
    msg.position.y = float(y)
    msg.position.z = float(z)
    msg.orientation.w = 1.0
    return msg


class HilRosGateway(Node):
    def __init__(self, args):
        super().__init__("hil_ros_gateway")
        self.args = args
        self._lock = threading.Lock()
        self._latest = {
            "jetson": {"psi": 0.0, "delta": 0.0, "brake": 0.0, "stamp_ms": 0},
            "esp32": {"psi": 0.0, "delta": 0.0, "brake": 0.0, "stamp_ms": 0},
        }
        self._active_role = "unknown"
        self._failover_available = False
        self._last_sensor_seq = -1
        self._last_sensor_ms = 0

        self.pub_car1_xy = self.create_publisher(Pose, TOPIC_CAR1_XY, qos_profile_sensor_data)
        self.pub_car1_psi = self.create_publisher(Float64, TOPIC_CAR1_PSI, qos_profile_sensor_data)
        self.pub_car1_v = self.create_publisher(Float64, TOPIC_CAR1_V, qos_profile_sensor_data)
        self.pub_car2_xy = self.create_publisher(Pose, TOPIC_CAR2_XY, qos_profile_sensor_data)
        self.pub_car2_v = self.create_publisher(Float64, TOPIC_CAR2_V, qos_profile_sensor_data)
        self.pub_car2_class = self.create_publisher(Int32, TOPIC_CAR2_CLASS, qos_profile_sensor_data)
        self.pub_road_psi = self.create_publisher(Float64, TOPIC_ROAD_PSI, qos_profile_sensor_data)
        self.pub_heng_error = self.create_publisher(Float64, TOPIC_HENG_ERROR, qos_profile_sensor_data)

        self.create_subscription(Float64, TOPIC_JETSON_PSI, self._jetson_psi_cb, 10)
        self.create_subscription(Float64, TOPIC_JETSON_DELTA, self._jetson_delta_cb, 10)
        self.create_subscription(Float64, TOPIC_JETSON_BRAKE, self._jetson_brake_cb, 10)
        self.create_subscription(Float64, TOPIC_ESP32_PSI, self._esp32_psi_cb, 10)
        self.create_subscription(Float64, TOPIC_ESP32_DELTA, self._esp32_delta_cb, 10)
        self.create_subscription(Float64, TOPIC_ESP32_BRAKE, self._esp32_brake_cb, 10)
        self.create_subscription(String, TOPIC_ACTIVE_ROLE, self._active_role_cb, 10)
        self.create_subscription(Bool, TOPIC_FAILOVER_AVAILABLE, self._failover_cb, 10)

        self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx_sock.bind((args.bind_host, args.sensor_port))
        self._rx_sock.settimeout(0.2)

        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pc_addr = (args.pc_host, args.actuation_port)

        self._running = True
        threading.Thread(target=self._sensor_loop, name="hil-sensor-rx", daemon=True).start()
        self.create_timer(1.0 / float(args.status_hz), self._send_actuation)
        self.create_timer(1.0, self._log_status)

        self.get_logger().info(
            "HIL ROS gateway listening UDP %s:%d, PC=%s:%d, source=%s"
            % (
                args.bind_host,
                args.sensor_port,
                args.pc_host,
                args.actuation_port,
                args.actuation_source,
            )
        )

    def _set_latest(self, source, field, value):
        with self._lock:
            self._latest[source][field] = float(value)
            self._latest[source]["stamp_ms"] = _now_ms()

    def _jetson_psi_cb(self, msg):
        self._set_latest("jetson", "psi", msg.data)

    def _jetson_delta_cb(self, msg):
        self._set_latest("jetson", "delta", msg.data)

    def _jetson_brake_cb(self, msg):
        self._set_latest("jetson", "brake", msg.data)

    def _esp32_psi_cb(self, msg):
        self._set_latest("esp32", "psi", msg.data)

    def _esp32_delta_cb(self, msg):
        self._set_latest("esp32", "delta", msg.data)

    def _esp32_brake_cb(self, msg):
        self._set_latest("esp32", "brake", msg.data)

    def _active_role_cb(self, msg):
        with self._lock:
            self._active_role = str(msg.data)

    def _failover_cb(self, msg):
        with self._lock:
            self._failover_available = bool(msg.data)

    def _sensor_loop(self):
        while self._running:
            try:
                data, _ = self._rx_sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                frame = json.loads(data.decode("utf-8"))
                self._publish_sensor_frame(frame)
            except Exception as exc:
                self.get_logger().warning("bad sensor frame: %r" % (exc,))

    def _publish_sensor_frame(self, frame):
        seq = int(frame.get("seq", -1))
        ego = frame.get("ego") or {}
        lead = frame.get("lead") or {}

        self.pub_car1_xy.publish(_pose(ego.get("x", 0.0), ego.get("y", 0.0)))
        self.pub_car1_psi.publish(Float64(data=float(ego.get("yaw", 0.0))))
        self.pub_car1_v.publish(Float64(data=float(ego.get("v", 0.0))))
        self.pub_road_psi.publish(Float64(data=float(frame.get("road_psi", 0.0))))
        self.pub_heng_error.publish(Float64(data=float(frame.get("lane_offset", 0.0))))

        if bool(lead.get("present", False)):
            self.pub_car2_xy.publish(_pose(lead.get("x", 9999.0), lead.get("y", 9999.0)))
            self.pub_car2_v.publish(Float64(data=float(lead.get("v", 0.0))))
            self.pub_car2_class.publish(Int32(data=int(lead.get("cls", 1))))
        else:
            self.pub_car2_xy.publish(_pose(9999.0, 9999.0))
            self.pub_car2_v.publish(Float64(data=0.0))
            self.pub_car2_class.publish(Int32(data=0))

        with self._lock:
            self._last_sensor_seq = seq
            self._last_sensor_ms = _now_ms()

    def _send_actuation(self):
        with self._lock:
            source = self.args.actuation_source
            act = dict(self._latest[source])
            payload = {
                "seq": self._last_sensor_seq,
                "t_gateway": time.time(),
                "source": source,
                "active_role": self._active_role,
                "failover_available": self._failover_available,
                "psi": act["psi"],
                "delta": act["delta"],
                "brake": act["brake"],
                "actuation_stale_ms": max(0, _now_ms() - int(act["stamp_ms"] or 0)),
                "sensor_stale_ms": max(0, _now_ms() - int(self._last_sensor_ms or 0)),
            }
        try:
            self._tx_sock.sendto(json.dumps(payload, separators=(",", ":")).encode("utf-8"), self._pc_addr)
        except OSError as exc:
            self.get_logger().warning("failed to send actuation: %r" % (exc,))

    def _log_status(self):
        with self._lock:
            sensor_age = max(0, _now_ms() - int(self._last_sensor_ms or 0))
            active_role = self._active_role
            failover = self._failover_available
            seq = self._last_sensor_seq
        self.get_logger().info(
            "sensor seq=%d age=%dms source=%s active=%s failover=%s"
            % (seq, sensor_age, self.args.actuation_source, active_role, failover)
        )

    def close(self):
        self._running = False
        try:
            self._rx_sock.close()
            self._tx_sock.close()
        except OSError:
            pass


def build_arg_parser():
    parser = argparse.ArgumentParser(description="CARLA UDP <-> Nano ROS2 HIL gateway")
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--pc-host", default="192.168.3.8")
    parser.add_argument("--sensor-port", type=int, default=42100)
    parser.add_argument("--actuation-port", type=int, default=42101)
    parser.add_argument("--actuation-source", choices=["jetson", "esp32"], default="jetson")
    parser.add_argument("--status-hz", type=float, default=50.0)
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    if rclpy is None:
        raise SystemExit(
            "rclpy is not installed in this Python. Run this script on the Nano "
            "after: source /opt/ros/foxy/setup.bash"
        )
    rclpy.init()
    node = HilRosGateway(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
