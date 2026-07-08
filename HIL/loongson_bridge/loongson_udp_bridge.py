#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""龙芯边缘控制节点 —— UDP 桥（无 ROS2）。

在龙芯 2K1000（loongarch64，无 rclpy）上把 `run_pure_pipeline` 真实控制内核
包装成一个**网络控制节点**：经 UDP 收感知帧 → 跑控制内核 → 回控制帧 + 广播心跳，
承担 ROS2 话题在无 ROS2 平台上的等价数据交换。串口下发 ESP32 为可选。

为什么不装 ROS2：Loongnix-Embedded 20 是 Debian10 代 userland（Python3.7/gcc8.3/
cmake3.13），与 ROS2 Humble（需 Python3.10/gcc11/cmake3.22）代际差约 4 年，源码编译
不可行；而本项目控制内核 `run_pure_pipeline` 本就零 ROS 依赖，用 UDP 桥即可让龙芯
作为控制节点接入 ROS2 世界（对端由带 rclpy 的机器做话题↔UDP 翻译）。

协议（JSON 行，UTF-8）：
  感知帧 (对端→板):  {"t":float,"ego_v":..,"road_psi":..,"lane_offset":..,
                      "lead":{"x":..,"y":..,"v":..,"cls":int}|null}
  控制帧 (板→对端):  {"type":"ctrl","seq":n,"delta":..,"lon_cmd":..,"aeb":0/1,
                      "ttc":..|null,"compute_us":..}
  心跳   (板→对端):  {"type":"hb","seq":n,"role":"primary","delta":..,"acc":..,"aeb":0/1}

用法（板上，SOCCode 目录内，需先把本文件放进 lx/SOCCode 或其部署副本）：
  python3 loongson_udp_bridge.py --listen 0.0.0.0 --port 9101 [--serial /dev/ttyUSB0] [--role primary]

实测（2026-07-02，龙芯 2K1000 loongarch64）：739 帧/8s 零丢失、往返 6.6ms、
前车急刹后 AEB 在 TTC=3.52 正确触发（全程还叠加了 OpenBLAS 双核满载）。
"""
import argparse
import json
import math
import socket
import time

from replay import build_stack
from pipeline import run_pure_pipeline


def _apply_perception(signals, frame, now):
    """把 JSON 感知帧写进 VehicleSignals（镜像 run_scenario 的组织方式）。"""
    signals.ego_x = 0.0
    signals.ego_y = 0.0
    signals.ego_yaw = float(frame.get("ego_yaw", 0.0))
    signals.ego_v = float(frame.get("ego_v", 0.0))
    signals.ego_received = True
    signals.ego_psi_received = True
    signals.ego_last_rx = now
    signals.road_psi = float(frame.get("road_psi", 0.0))
    signals.road_received = True
    signals.road_last_rx = now
    signals.lane_offset = float(frame.get("lane_offset", 0.0))
    signals.lane_offset_received = True
    signals.lane_offset_last_rx = now
    lead = frame.get("lead")
    if lead:
        signals.lead_x = float(lead.get("x", 0.0))
        signals.lead_y = float(lead.get("y", 0.0))
        signals.lead_yaw = float(lead.get("yaw", signals.road_psi))
        signals.lead_v = float(lead.get("v", 0.0))
        signals.lead_cls = int(lead.get("cls", 1))
        signals.lead_received = True
        signals.lead_last_rx_time = now
        signals.lead_v_last_rx_time = now


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=9101)
    ap.add_argument("--role", default="primary")
    ap.add_argument("--serial", default="", help="ESP32 串口设备（如 /dev/ttyUSB0），留空则不下发")
    ap.add_argument("--hb-every", type=int, default=10, help="每 N 拍广播一次心跳")
    ap.add_argument("--stats-every", type=int, default=200)
    args = ap.parse_args()

    signals, memory, managers = build_stack()

    ser = None
    build_esp32_payload = None
    Esp32ControlFrame = None
    if args.serial:
        try:
            import serial
            from control.serial_protocol import build_esp32_payload, Esp32ControlFrame
            ser = serial.Serial(args.serial, 115200, timeout=0)
            print("[bridge] ESP32 串口已开: %s" % args.serial)
        except Exception as e:
            print("[bridge] 串口不可用（继续，不下发）: %s" % e)
            ser = None

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.listen, args.port))
    print("[bridge] role=%s 监听 %s:%d，等待感知帧…" % (args.role, args.listen, args.port))

    seq = 0
    n_frames = 0
    comp_sum = 0.0
    comp_max = 0.0
    t_report = time.time()
    while True:
        try:
            data, addr = sock.recvfrom(4096)
        except KeyboardInterrupt:
            print("\n[bridge] 退出")
            break
        try:
            frame = json.loads(data.decode("utf-8"))
        except Exception:
            continue

        now = time.time()
        _apply_perception(signals, frame, now)

        t0 = time.perf_counter()
        res = run_pure_pipeline(now, signals, memory, managers, None)
        comp_us = (time.perf_counter() - t0) * 1e6

        delta = float(res.lateral_ctx.delta)
        lon = float(res.lon_cmd)
        aeb = 1 if bool(res.lon_ctx.aeb_active) else 0
        ttc = res.lon_ctx.ttc
        ttc_out = None if (ttc is None or not math.isfinite(ttc)) else round(ttc, 3)

        seq += 1
        ctrl = {"type": "ctrl", "seq": seq, "delta": round(delta, 5),
                "lon_cmd": round(lon, 4), "aeb": aeb, "ttc": ttc_out,
                "compute_us": round(comp_us, 1)}
        sock.sendto((json.dumps(ctrl) + "\n").encode("utf-8"), addr)

        if args.hb_every and seq % args.hb_every == 0:
            hb = {"type": "hb", "seq": seq, "role": args.role,
                  "delta": round(delta, 5), "acc": round(-lon, 4), "aeb": aeb}
            sock.sendto((json.dumps(hb) + "\n").encode("utf-8"), addr)

        if ser is not None:
            try:
                fr = Esp32ControlFrame(ttc=(ttc if ttc_out is not None else 99.0),
                                       dist=float(getattr(signals, "lead_x", 0.0) or 0.0),
                                       psi=0.0, delta=delta, speed=signals.ego_v,
                                       acc=-lon, offset=signals.lane_offset,
                                       leadv=float(getattr(signals, "lead_v", 0.0) or 0.0),
                                       dsafe=10.0, wmrn=2.0, whrd=3.0, curv=0.0)
                ser.write(build_esp32_payload(fr))
            except Exception:
                pass

        n_frames += 1
        comp_sum += comp_us
        comp_max = max(comp_max, comp_us)
        if args.stats_every and n_frames % args.stats_every == 0:
            dt = time.time() - t_report
            print("[bridge] frames=%d rate=%.0fHz compute mean=%.0fus max=%.0fus last: delta=%.4f lon=%.3f aeb=%d ttc=%s"
                  % (n_frames, args.stats_every / dt if dt > 0 else 0,
                     comp_sum / args.stats_every, comp_max, delta, lon, aeb, ttc_out))
            comp_sum = 0.0
            comp_max = 0.0
            t_report = time.time()

    sock.close()


if __name__ == "__main__":
    main()
