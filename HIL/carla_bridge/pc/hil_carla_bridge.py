#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Windows-side CARLA bridge for LAN HIL with two real Jetson Nano boards."""

import argparse
import csv
import json
import os
import socket
import threading
import time
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
# 仓库根 = HIL/carla_bridge/pc 上溯三级（CALRA 在仓库根，不随 HIL/ 整理而移动）
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))

from bridge_config import CARLA_HOST, CARLA_PORT, TOWN
from carla_link import CarlaLink
from scenarios import ORDER, SCENARIOS


def import_carla():
    try:
        import carla
        return carla
    except ImportError:
        wheel = os.path.join(
            ROOT,
            "CALRA",
            "PythonAPI",
            "carla",
            "dist",
            "carla-0.9.16-cp312-cp312-win_amd64.whl",
        )
        raise SystemExit(
            "Cannot import carla. Install the Python 3.12 wheel first:\n"
            "  python -m pip install \"%s\"" % wheel
        )


class ActuationReceiver:
    def __init__(self, gateway_host, tcp_port, stale_timeout_s):
        self._lock = threading.Lock()
        self._latest = None
        self._latest_rx_t = 0.0
        self._running = True
        self._stale_timeout_s = float(stale_timeout_s)
        self._sock = socket.create_connection((gateway_host, tcp_port), timeout=10.0)
        self._sock.settimeout(0.2)
        self._send_lock = threading.Lock()
        threading.Thread(target=self._loop, name="hil-tcp-rx", daemon=True).start()

    def _loop(self):
        buf = ""
        while self._running:
            try:
                data = self._sock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            try:
                buf += data.decode("utf-8")
            except UnicodeDecodeError:
                continue
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line)
                except Exception:
                    continue
                with self._lock:
                    self._latest = msg
                    self._latest_rx_t = time.monotonic()

    def send_sensor(self, payload):
        with self._send_lock:
            self._sock.sendall(payload + b"\n")

    def get(self):
        with self._lock:
            msg = dict(self._latest) if self._latest else None
            age = time.monotonic() - self._latest_rx_t if self._latest_rx_t else 9999.0
        if msg is None or age > self._stale_timeout_s:
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
            "psi": float(msg.get("psi", 0.0)),
            "delta": float(msg.get("delta", 0.0)),
            "a_brake": float(msg.get("brake", 0.0)),
            "source": msg.get("source", "unknown"),
            "active_role": msg.get("active_role", "unknown"),
            "failover_available": bool(msg.get("failover_available", False)),
            "age_s": age,
            "actuation_stale_ms": int(msg.get("actuation_stale_ms", 0)),
            "sensor_stale_ms": int(msg.get("sensor_stale_ms", 0)),
        }

    def close(self):
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass


def _make_sensor_payload(seq, frame):
    return {
        "seq": seq,
        "t": float(frame.get("t", 0.0)),
        "ego": {
            "x": float(frame.get("ego_x", 0.0)),
            "y": float(frame.get("ego_y", 0.0)),
            "yaw": float(frame.get("ego_yaw", 0.0)),
            "v": float(frame.get("ego_v", 0.0)),
        },
        "road_psi": float(frame.get("road_psi", 0.0)),
        "lane_offset": float(frame.get("lane_offset", 0.0)),
        "lead": {
            "present": bool(frame.get("lead_present", False)),
            "x": float(frame.get("lead_x", 9999.0)),
            "y": float(frame.get("lead_y", 9999.0)),
            "yaw": float(frame.get("lead_yaw", 0.0)),
            "v": float(frame.get("lead_v", 0.0)),
            "cls": int(frame.get("lead_cls", 1 if frame.get("lead_present", False) else 0)),
        },
    }


def run(args):
    carla = import_carla()
    scenario = SCENARIOS[args.scenario]
    link = CarlaLink(
        carla,
        args.carla_host,
        args.carla_port,
        scenario,
        town=args.town,
        no_rendering=args.no_rendering,
        spawn_index=args.spawn_index,
    )
    receiver = ActuationReceiver(args.gateway_host, args.tcp_port, args.stale_timeout_s)
    duration = float(args.duration if args.duration is not None else scenario.get("duration", 0.0))

    log_dir = os.path.join(HERE, "logs")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, "hil_%s_%s.csv" % (args.scenario, datetime.now().strftime("%Y%m%d_%H%M%S")))
    print("HIL bridge connected to gateway TCP %s:%d" % (args.gateway_host, args.tcp_port), flush=True)
    print("CSV log: %s" % log_path, flush=True)

    seq = 0
    last_print = 0.0
    start_wall = time.monotonic()
    with open(log_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow([
            "t",
            "seq",
            "ego_v",
            "lead_present",
            "gap",
            "lane_offset",
            "source",
            "active_role",
            "failover_available",
            "delta",
            "a_brake",
            "steer",
            "throttle",
            "brake",
            "actuation_age_s",
            "actuation_stale_ms",
            "sensor_stale_ms",
        ])
        try:
            while True:
                sim_t = link.tick()
                if duration > 0.0 and sim_t >= duration:
                    break

                frame, gap = link.sense(sim_t)
                payload = _make_sensor_payload(seq, frame)
                receiver.send_sensor(json.dumps(payload, separators=(",", ":")).encode("utf-8"))

                link.drive_lead(sim_t)
                act = receiver.get()
                steer, throttle, brake = link.apply_ego(act)
                link.update_spectator()

                writer.writerow([
                    "%.3f" % sim_t,
                    seq,
                    "%.3f" % frame.get("ego_v", 0.0),
                    int(bool(frame.get("lead_present", False))),
                    "%.3f" % gap if gap != float("inf") else "inf",
                    "%.3f" % frame.get("lane_offset", 0.0),
                    act["source"],
                    act["active_role"],
                    int(act["failover_available"]),
                    "%.6f" % act["delta"],
                    "%.3f" % act["a_brake"],
                    "%.4f" % steer,
                    "%.4f" % throttle,
                    "%.4f" % brake,
                    "%.3f" % act["age_s"],
                    act.get("actuation_stale_ms", ""),
                    act.get("sensor_stale_ms", ""),
                ])
                fh.flush()

                if sim_t - last_print >= 1.0:
                    print(
                        "t=%5.1f seq=%d v=%.2f gap=%s src=%s role=%s delta=%.3f brake=%.2f age=%.0fms"
                        % (
                            sim_t,
                            seq,
                            frame.get("ego_v", 0.0),
                            "%.1f" % gap if gap != float("inf") else "inf",
                            act["source"],
                            act["active_role"],
                            act["delta"],
                            act["a_brake"],
                            act["age_s"] * 1000.0,
                        ),
                        flush=True,
                    )
                    last_print = sim_t

                seq += 1
                if args.realtime:
                    target = start_wall + sim_t
                    sleep_s = target - time.monotonic()
                    if sleep_s > 0.0:
                        time.sleep(min(sleep_s, 0.05))
        finally:
            receiver.close()
            link.close()


def build_arg_parser():
    parser = argparse.ArgumentParser(description="LAN HIL CARLA bridge")
    parser.add_argument("--scenario", default="acc", choices=ORDER)
    parser.add_argument("--gateway-host", default="192.168.3.125")
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--sensor-port", type=int, default=42100)
    parser.add_argument("--actuation-port", type=int, default=42101)
    parser.add_argument("--tcp-port", type=int, default=42110)
    parser.add_argument("--actuation-source", choices=["jetson", "esp32"], default="jetson",
                        help="Documentation-only mirror of the gateway source; gateway selects the real source.")
    parser.add_argument("--stale-timeout-s", type=float, default=0.5)
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
    print("Requested actuation source: %s (must match gateway startup)" % args.actuation_source)
    run(args)


if __name__ == "__main__":
    main()
