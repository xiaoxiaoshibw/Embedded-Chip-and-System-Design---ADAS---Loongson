#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hilctl —— ADAS HIL 平台命令行客户端。

与 Web 共用同一套 FastAPI 接口（需求第 5 条），自身不直接操作 CARLA。
纯标准库（urllib），无第三方依赖。

示例：
    python -m cli.hilctl load acc_follow --ego-speed 50 --front-distance 30 --weather clear
    python -m cli.hilctl start
    python -m cli.hilctl pause
    python -m cli.hilctl stop
    python -m cli.hilctl reset
    python -m cli.hilctl status
    python -m cli.hilctl update --front-speed 20 --comm-delay 100
    python -m cli.hilctl inject-fault seq_stuck --target nano_a
    python -m cli.hilctl report latest

服务地址：环境变量 HIL_API（默认 http://127.0.0.1:8000）或全局 --api 覆盖。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")  # 防 Windows GBK 崩
except Exception:
    pass

DEFAULT_API = os.environ.get("HIL_API", "http://127.0.0.1:8000")

# CLI flag -> 后端参数名
PARAM_FLAGS = [
    ("--ego-speed", "ego_speed", float),
    ("--front-distance", "front_distance", float),
    ("--front-speed", "front_speed", float),
    ("--cut-in-speed", "cut_in_speed", float),
    ("--cut-in-trigger-distance", "cut_in_trigger_distance", float),
    ("--weather", "weather", str),
    ("--comm-delay", "comm_delay_ms", float),
    ("--sensor-noise", "sensor_noise", float),
    ("--fault-type", "fault_type", str),
    ("--fault-trigger-time", "fault_trigger_time", float),
]


def _req(api: str, method: str, path: str, body=None):
    url = api.rstrip("/") + path
    data = None
    headers = {"Accept": "application/json"}
    if body is not None:
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            raw = resp.read().decode("utf-8")
            ctype = resp.headers.get("Content-Type", "")
            if "application/json" in ctype:
                return json.loads(raw)
            return raw
    except urllib.error.HTTPError as exc:
        msg = exc.read().decode("utf-8", "replace")
        try:
            msg = json.loads(msg).get("detail", msg)
        except Exception:
            pass
        print("[HTTP %d] %s" % (exc.code, msg), file=sys.stderr)
        sys.exit(1)
    except urllib.error.URLError as exc:
        print("无法连接 %s：%s（后端是否已启动？）" % (url, exc.reason), file=sys.stderr)
        sys.exit(2)


def _collect_params(args) -> dict:
    params = {}
    for _flag, name, _typ in PARAM_FLAGS:
        val = getattr(args, name, None)
        if val is not None:
            params[name] = val
    return params


def _add_param_flags(p: argparse.ArgumentParser) -> None:
    for flag, name, typ in PARAM_FLAGS:
        p.add_argument(flag, dest=name, type=typ, default=None)


def _print_status(st: dict) -> None:
    print("─" * 56)
    print(" 状态        : %s" % st.get("state"))
    print(" run_id      : %s" % st.get("run_id"))
    print(" 场景        : %s (%s)" % (st.get("scenario"), st.get("scenario_title")))
    print(" 场景时间    : %s s" % st.get("scenario_time"))
    print(" 生效控制器  : %s" % st.get("active_controller"))
    print(" 是否接管    : %s" % st.get("takeover"))
    print(" 安全制动    : %s" % st.get("safe_brake"))
    print(" 帧数        : %s" % st.get("frame_count"))
    faults = st.get("active_faults") or []
    if faults:
        print(" 激活故障    : %s" % ", ".join(
            "%s@%s" % (f["type"], f["target"]) for f in faults))
    print("─" * 56)


# ── 子命令 ──
def cmd_load(api, args):
    body = {"scenario": args.scenario}
    params = _collect_params(args)
    if params:
        body["params"] = params
    out = _req(api, "POST", "/api/scenario/load", body)
    _print_status(out["status"])


def cmd_start(api, args):
    _print_status(_req(api, "POST", "/api/simulation/start")["status"])


def cmd_pause(api, args):
    _print_status(_req(api, "POST", "/api/simulation/pause")["status"])


def cmd_stop(api, args):
    out = _req(api, "POST", "/api/simulation/stop")
    _print_status(out["status"])
    meta = out.get("meta") or {}
    if meta:
        print("已保存实验记录：%s（时长 %s s）" % (meta.get("run_id"), meta.get("duration")))


def cmd_reset(api, args):
    _print_status(_req(api, "POST", "/api/simulation/reset")["status"])


def cmd_status(api, args):
    _print_status(_req(api, "GET", "/api/status"))


def cmd_update(api, args):
    params = _collect_params(args)
    if not params:
        print("未提供任何参数，无操作。", file=sys.stderr)
        sys.exit(1)
    out = _req(api, "POST", "/api/parameters/update", {"params": params})
    print("已更新参数：")
    for k, v in (out.get("params") or {}).items():
        print("  %-26s = %s" % (k, v))


def cmd_inject_fault(api, args):
    out = _req(api, "POST", "/api/fault/inject",
               {"fault_type": args.fault_type, "target": args.target})
    evt = out.get("event") or {}
    print("已注入故障：%s @ %s（场景时间 %s s）" % (
        evt.get("detail"), evt.get("target"), evt.get("time")))


def cmd_report(api, args):
    target = args.run_id
    if target == "latest":
        runs = _req(api, "GET", "/api/runs").get("runs") or []
        if not runs:
            print("暂无历史记录。", file=sys.stderr)
            sys.exit(1)
        target = runs[0]["run_id"]
    summary = _req(api, "GET", "/api/runs/%s/summary" % target)
    print("════ 实验摘要：%s ════" % target)
    for k in ("result", "collision", "min_ttc", "max_lateral_error",
              "takeover_happened", "takeover_latency_ms",
              "active_controller_final", "safe_brake_triggered"):
        print("  %-26s : %s" % (k, summary.get(k)))
    print("  conclusion                 : %s" % summary.get("conclusion"))
    if args.full:
        print("\n──── report.md ────\n")
        print(_req(api, "GET", "/api/runs/%s/report" % target))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="hilctl", description="ADAS HIL 平台命令行客户端")
    p.add_argument("--api", default=DEFAULT_API, help="后端地址（默认 %s）" % DEFAULT_API)
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("load", help="加载场景")
    sp.add_argument("scenario")
    _add_param_flags(sp)
    sp.set_defaults(func=cmd_load)

    for name, fn, help_ in (
        ("start", cmd_start, "开始仿真"),
        ("pause", cmd_pause, "暂停仿真"),
        ("stop", cmd_stop, "停止并保存"),
        ("reset", cmd_reset, "复位"),
        ("status", cmd_status, "查询状态"),
    ):
        s = sub.add_parser(name, help=help_)
        s.set_defaults(func=fn)

    su = sub.add_parser("update", help="热更新参数")
    _add_param_flags(su)
    su.set_defaults(func=cmd_update)

    si = sub.add_parser("inject-fault", help="注入故障")
    si.add_argument("fault_type",
                    choices=["seq_stuck", "heartbeat_loss", "nan_output",
                             "control_delay", "backup_fail", "dual_fail"])
    si.add_argument("--target", default="nano_a",
                    choices=["nano_a", "nano_b", "both"])
    si.set_defaults(func=cmd_inject_fault)

    sr = sub.add_parser("report", help="查看实验摘要/报告")
    sr.add_argument("run_id", nargs="?", default="latest",
                    help="run_id 或 latest（默认）")
    sr.add_argument("--full", action="store_true", help="附带完整 report.md")
    sr.set_defaults(func=cmd_report)

    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    args.func(args.api, args)


if __name__ == "__main__":
    main()
