#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lightweight backup-Nano edge result collector for HIL runs.

It keeps edge analysis off the primary Nano. The collector samples local ADAS
resource use and recent ADAS log state, then writes JSONL files that the WebUI
backend can pull back into HIL/hil_platform/edge_results.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import time
from pathlib import Path
from typing import Dict, List


def _read_first(path: str) -> str:
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return ""


def _tail(path: str, max_lines: int = 80) -> List[str]:
    try:
        lines = Path(path).read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return []
    return lines[-max_lines:]


def _find_adas() -> Dict[str, object]:
    best = {"pid": None, "cpu": 0.0, "mem": 0.0, "rss_kb": 0, "psr": None}
    stream = os.popen("ps -eo pid,psr,pcpu,pmem,rss,args --sort=-pcpu")
    try:
        for line in stream:
            if "ADAS.py --role" not in line or "grep" in line:
                continue
            parts = line.split(None, 5)
            if len(parts) < 6:
                continue
            best = {
                "pid": int(parts[0]),
                "psr": int(parts[1]),
                "cpu": float(parts[2]),
                "mem": float(parts[3]),
                "rss_kb": int(parts[4]),
                "cmd": parts[5].strip(),
            }
            break
    finally:
        stream.close()
    return best


def _latest_log_state(log_path: str) -> Dict[str, object]:
    lines = _tail(log_path)
    last = lines[-1] if lines else ""
    seq = None
    for line in reversed(lines):
        m = re.search(r"seq[=:\s]+(-?\d+)", line, re.IGNORECASE)
        if m:
            seq = int(m.group(1))
            break
    return {"last_log": last, "seq": seq, "tail_lines": len(lines)}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="/home/jetson/adas/hil/edge_results")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--role", default="backup")
    parser.add_argument("--log", default="/tmp/adas_hil_backup.log")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / ("edge_%s_%s.jsonl" % (args.role, time.strftime("%Y%m%d_%H%M%S")))
    summary_path = out_dir / ("latest_%s.json" % args.role)

    while True:
        temp_raw = _read_first("/sys/devices/virtual/thermal/thermal_zone4/temp")
        temp_c = None
        if temp_raw.isdigit():
            temp_c = int(temp_raw) / 1000.0
        row = {
            "ts": time.time(),
            "host": _read_first("/etc/hostname"),
            "role": args.role,
            "loadavg": _read_first("/proc/loadavg"),
            "temp_c": temp_c,
            "adas": _find_adas(),
            "log": _latest_log_state(args.log),
        }
        line = json.dumps(row, ensure_ascii=False, separators=(",", ":"))
        with out_path.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")
        summary_path.write_text(line + "\n", encoding="utf-8")
        time.sleep(max(0.2, args.interval))


if __name__ == "__main__":
    main()
