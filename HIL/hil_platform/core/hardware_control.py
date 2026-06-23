# -*- coding: utf-8 -*-
"""Restricted hardware-control helpers for the real HIL rig.

The web UI should operate the two Jetson Nano boards through explicit actions,
not by exposing an arbitrary SSH shell. CARLA remains the world/perception
source and actuator sink; these helpers only manage the hardware control path.
"""

from __future__ import annotations

import concurrent.futures
import os
import time
from typing import Any, Dict, Iterable, Tuple

import paramiko


REMOTE_DIR = "/home/jetson/adas/hil"
ROS_DOMAIN_ID = 43


def _targets() -> Dict[str, Dict[str, Any]]:
    return {
        "primary": {
            "host": os.environ.get("GATEWAY_HOST", "192.168.3.125"),
            "user": os.environ.get("NANO_USER", "jetson"),
            "pw": os.environ.get("NANO_PW_PRIMARY", "yahboom"),
            "role": "primary",
        },
        "backup": {
            "host": os.environ.get("BACKUP_HOST", "192.168.3.124"),
            "user": os.environ.get("NANO_USER", "jetson"),
            "pw": os.environ.get("NANO_PW_BACKUP", "jetson"),
            "role": "backup",
        },
    }


def _client(target: str) -> paramiko.SSHClient:
    n = _targets()[target]
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(
        n["host"],
        port=22,
        username=n["user"],
        password=n["pw"],
        timeout=10,
        banner_timeout=15,
        auth_timeout=15,
        look_for_keys=False,
        allow_agent=False,
    )
    return c


def _run(target: str, cmd: str, timeout: int = 60) -> Dict[str, Any]:
    n = _targets()[target]
    started = time.monotonic()
    try:
        c = _client(target)
        try:
            _stdin, stdout, stderr = c.exec_command(cmd, timeout=timeout)
            out = stdout.read().decode("utf-8", "replace")
            err = stderr.read().decode("utf-8", "replace")
            rc = stdout.channel.recv_exit_status()
        finally:
            c.close()
        return {
            "target": target,
            "host": n["host"],
            "ok": rc == 0,
            "rc": rc,
            "stdout": out,
            "stderr": err,
            "elapsed_ms": int((time.monotonic() - started) * 1000),
        }
    except Exception as exc:
        return {
            "target": target,
            "host": n["host"],
            "ok": False,
            "rc": None,
            "stdout": "",
            "stderr": repr(exc),
            "elapsed_ms": int((time.monotonic() - started) * 1000),
        }


def _parallel(targets: Iterable[str], cmd_for: Any, timeout: int = 60) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
        futures = {
            ex.submit(_run, t, cmd_for(t), timeout): t
            for t in targets
        }
        for fut in concurrent.futures.as_completed(futures):
            t = futures[fut]
            out[t] = fut.result()
    return out


def health() -> Dict[str, Any]:
    """Return bounded health details for both Nanos and the primary gateway."""
    def cmd(target: str) -> str:
        log = "/tmp/adas_hil_primary.log" if target == "primary" else "/tmp/adas_hil_backup.log"
        extra = ""
        if target == "primary":
            extra = (
                "echo __GATEWAY__; "
                "ps -o pid,stat,etime,pcpu,pmem,cmd -C python3 | grep 'hil_ros_gateway.py' | grep -v grep || true; "
                "tail -8 /tmp/hil_gateway_esp32.log 2>/dev/null || true; "
                "echo __ROS__; "
                "source /opt/ros/foxy/setup.bash 2>/dev/null; "
                "export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=0; "
                "ros2 node list 2>/dev/null | sort || true; "
            )
        return (
            "echo __HOST__; hostname; "
            "echo __UPTIME__; uptime; "
            "echo __ADAS__; "
            "ps -o pid,stat,etime,pcpu,pmem,cmd -C python3 | grep 'ADAS.py --role' | grep -v grep || true; "
            "echo __LOG__; tail -8 %s 2>/dev/null || true; "
            "%s"
        ) % (log, extra)

    result = _parallel(("primary", "backup"), cmd, timeout=40)
    result["ok"] = all(v.get("ok") for v in result.values())
    return result


def restart_adas(target: str) -> Dict[str, Any]:
    targets = _select_targets(target)

    def cmd(t: str) -> str:
        n = _targets()[t]
        pw = n["pw"]
        return (
            "python3 '%s/stop_gateway.py' || true; "
            "python3 '%s/start_hil_adas.py' --role %s --domain %d --sudo-password '%s'"
        ) % (REMOTE_DIR, REMOTE_DIR, n["role"], ROS_DOMAIN_ID, pw)

    result = _parallel(targets, cmd, timeout=120)
    result["ok"] = all(v.get("ok") for v in result.values())
    return result


def start_gateway(source: str = "esp32") -> Dict[str, Any]:
    if source not in ("esp32", "jetson"):
        raise ValueError("source must be esp32 or jetson")
    cmd = (
        "python3 '%s/stop_gateway.py' || true; "
        "nohup bash -lc 'source /opt/ros/foxy/setup.bash; "
        "export ROS_DOMAIN_ID=%d ROS_LOCALHOST_ONLY=0; "
        "python3 %s/hil_ros_gateway.py --pc-host 192.168.3.8 "
        "--sensor-port 42100 --actuation-port 42101 --tcp-port 42110 "
        "--actuation-source %s' > /tmp/hil_gateway_%s.log 2>&1 < /dev/null & "
        "sleep 1; ps -ef | grep '[h]il_ros_gateway.py' || true; "
        "tail -20 /tmp/hil_gateway_%s.log 2>/dev/null || true"
    ) % (REMOTE_DIR, ROS_DOMAIN_ID, REMOTE_DIR, source, source, source)
    r = _run("primary", cmd, timeout=40)
    r["source"] = source
    return {"ok": bool(r.get("ok")), "primary": r}


def restore_nanos() -> Dict[str, Any]:
    def cmd(t: str) -> str:
        role = _targets()[t]["role"]
        return "pkill -CONT -f 'ADAS.py --role %s' || true; ps -o pid,stat,etime,cmd -C python3 | grep 'ADAS.py --role' | grep -v grep || true" % role

    result = _parallel(("primary", "backup"), cmd, timeout=30)
    result["ok"] = all(v.get("ok") for v in result.values())
    return result


def _select_targets(target: str) -> Tuple[str, ...]:
    if target in ("both", "all"):
        return ("primary", "backup")
    if target in ("primary", "backup"):
        return (target,)
    if target == "nano_a":
        return ("primary",)
    if target == "nano_b":
        return ("backup",)
    raise ValueError("target must be primary, backup, both, nano_a, or nano_b")
