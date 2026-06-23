#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stop running hil_ros_gateway.py processes on a Nano."""

import os
import signal


TARGET = "/home/jetson/adas/hil/hil_ros_gateway.py"


def iter_cmdlines():
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        try:
            with open("/proc/%d/cmdline" % pid, "rb") as fh:
                raw = fh.read()
        except IOError:
            continue
        parts = [p.decode("utf-8", "replace") for p in raw.split(b"\0") if p]
        if parts:
            yield pid, parts


def main():
    self_pid = os.getpid()
    killed = 0
    for pid, parts in iter_cmdlines():
        if pid == self_pid:
            continue
        if TARGET in parts:
            print("kill gateway pid=%d" % pid)
            try:
                os.kill(pid, signal.SIGTERM)
                killed += 1
            except OSError as exc:
                print("failed pid=%d: %s" % (pid, exc))
    print("gateway processes stopped: %d" % killed)


if __name__ == "__main__":
    main()
