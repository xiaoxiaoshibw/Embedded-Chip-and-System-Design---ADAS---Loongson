#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Restart ADAS.py on a Nano using the existing deploy/nano/adas-run.sh."""

import os
import signal
import subprocess
import time


ADAS_ENTRY = "/home/jetson/adas/lx/SOCCode/ADAS.py"
RUN_SH = "/home/jetson/adas/deploy/nano/adas-run.sh"


def find_adas():
    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as fh:
                raw = fh.read()
        except IOError:
            continue
        cmd = raw.replace(b"\0", b" ").decode("utf-8", "replace")
        if ADAS_ENTRY in cmd:
            pids.append((pid, cmd))
    return pids


def main():
    for pid, cmd in find_adas():
        print("stop ADAS pid=%d %s" % (pid, cmd))
        try:
            os.kill(pid, signal.SIGINT)
        except OSError as exc:
            print("failed to stop pid=%d: %s" % (pid, exc))
    time.sleep(3.0)
    live = find_adas()
    if live:
        for pid, cmd in live:
            print("terminate ADAS pid=%d %s" % (pid, cmd))
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError as exc:
                print("failed to terminate pid=%d: %s" % (pid, exc))
        time.sleep(3.0)
    live = find_adas()
    if live:
        for pid, cmd in live:
            print("kill ADAS pid=%d %s" % (pid, cmd))
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError as exc:
                print("failed to kill pid=%d: %s" % (pid, exc))
        time.sleep(2.0)
    # systemd adas-node.service normally restarts the process. Give it time
    # before falling back to manual launch, otherwise duplicate nodes appear.
    for _ in range(8):
        live = find_adas()
        if live:
            break
        time.sleep(1.0)
    if not live:
        subprocess.Popen(
            ["bash", "-lc", "nohup '%s' > /tmp/adas_manual_restart.log 2>&1 < /dev/null &" % RUN_SH]
        )
        time.sleep(3.0)
    live = find_adas()
    if not live:
        print("ADAS restart requested but no process found yet")
        return
    if len(live) > 1:
        # Keep the oldest live process and remove duplicates from manual/systemd races.
        live = sorted(live, key=lambda x: x[0])
        for pid, cmd in live[1:]:
            print("remove duplicate ADAS pid=%d %s" % (pid, cmd))
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError as exc:
                print("failed duplicate pid=%d: %s" % (pid, exc))
        time.sleep(2.0)
        live = find_adas()
    for pid, cmd in live:
        print("ADAS restarted pid=%d %s" % (pid, cmd))


if __name__ == "__main__":
    main()
