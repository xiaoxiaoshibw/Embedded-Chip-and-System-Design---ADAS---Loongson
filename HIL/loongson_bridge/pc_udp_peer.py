#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""龙芯 UDP 桥的 PC 侧测试对端（无 ROS2 / 无 CARLA）。

向龙芯板发脚本化感知帧（稳态跟车 → 前车急刹），接收板子回传的控制帧，
测往返时延、统计控制响应、计心跳，验证"龙芯作为 UDP 网络控制节点"端到端可用。

用法：python pc_udp_peer.py --board 192.168.137.13 --port 9101 --hz 100 --secs 8
"""
import argparse
import json
import socket
import time


def build_scenario_frame(t):
    """脚本感知：0-3s 稳态跟车(ego16, lead30m@14)，3s 起前车急刹 -3m/s²。"""
    ego_v = 16.0
    lead_v = 14.0
    gap = 30.0
    if t >= 3.0:
        dt = t - 3.0
        lead_v = max(0.0, 14.0 - 3.0 * dt)
        gap = max(2.0, 30.0 - (16.0 - lead_v) * dt * 0.5)
    return {"t": t, "ego_v": ego_v, "ego_yaw": 0.0, "road_psi": 0.01,
            "lane_offset": 0.05,
            "lead": {"x": gap, "y": 0.0, "v": lead_v, "cls": 1}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", required=True, help="龙芯板 IP")
    ap.add_argument("--port", type=int, default=9101)
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--secs", type=float, default=8.0)
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst = (args.board, args.port)
    period = 1.0 / args.hz

    rtts = []
    n_sent = n_ctrl = n_hb = 0
    first_aeb_t = -1.0
    samples = []
    t_start = time.time()
    print("[peer] -> 龙芯 %s:%d  %gHz x %gs" % (args.board, args.port, args.hz, args.secs))
    while time.time() - t_start < args.secs:
        t = time.time() - t_start
        frame = build_scenario_frame(t)
        send_ts = time.perf_counter()
        sock.sendto((json.dumps(frame) + "\n").encode("utf-8"), dst)
        n_sent += 1

        # 1) 阻塞等本拍 ctrl 回执（测往返时延）
        got_ctrl = False
        sock.settimeout(0.5)
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            data = b""
        # 2) 再非阻塞抽干队列（心跳等），不卡超时
        sock.setblocking(False)
        while True:
            try:
                data += b"\n" + sock.recvfrom(4096)[0]
            except (BlockingIOError, OSError):
                break
        sock.setblocking(True)

        for line in data.decode("utf-8").splitlines():
            try:
                msg = json.loads(line)
            except Exception:
                continue
            typ = msg.get("type")
            if typ == "ctrl":
                if not got_ctrl:
                    rtts.append((time.perf_counter() - send_ts) * 1e3)
                    got_ctrl = True
                n_ctrl += 1
                if msg.get("aeb") == 1 and first_aeb_t < 0:
                    first_aeb_t = t
                if n_ctrl % 20 == 0:
                    samples.append((round(t, 2), msg.get("delta"),
                                    msg.get("lon_cmd"), msg.get("aeb"), msg.get("ttc")))
            elif typ == "hb":
                n_hb += 1

        elapsed = time.perf_counter() - send_ts
        if elapsed < period:
            time.sleep(period - elapsed)

    sock.close()
    rtts.sort()
    def pct(p):
        return rtts[min(len(rtts) - 1, int(len(rtts) * p))] if rtts else float("nan")
    print("\n===== 龙芯 UDP 控制节点 端到端结果 =====")
    print("感知帧发送     : %d" % n_sent)
    print("控制帧回执     : %d  (丢失 %d)" % (n_ctrl, n_sent - n_ctrl))
    print("心跳帧接收     : %d" % n_hb)
    if rtts:
        print("往返时延 mean  : %.2f ms" % (sum(rtts) / len(rtts)))
        print("往返时延 p50   : %.2f ms" % pct(0.50))
        print("往返时延 p99   : %.2f ms" % pct(0.99))
        print("往返时延 max   : %.2f ms" % rtts[-1])
    print("首次 AEB 触发   : %s" % ("t=%.2fs（前车急刹后）" % first_aeb_t if first_aeb_t >= 0 else "未触发"))
    print("控制采样 (t,delta,lon_cmd,aeb,ttc):")
    for s in samples:
        print("   ", s)


if __name__ == "__main__":
    main()
