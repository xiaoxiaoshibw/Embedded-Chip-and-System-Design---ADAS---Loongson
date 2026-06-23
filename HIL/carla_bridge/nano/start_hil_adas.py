#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Start ADAS.py manually for HIL on an isolated ROS_DOMAIN_ID."""

import argparse
import os
import signal
import subprocess
import time


ADAS_ENTRY = "/home/jetson/adas/lx/SOCCode/ADAS.py"
ADAS_HOME = "/home/jetson/adas"


def run(cmd, input_text=None):
    print("+", cmd)
    p = subprocess.run(cmd, shell=True, text=True, input=input_text,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.stdout:
        print(p.stdout.rstrip())
    return p.returncode


def stop_service(password):
    run("sudo -S systemctl stop adas-node.service || true", password + "\n")


def _is_adas_process(cmd):
    return (
        "ADAS.py" in cmd
        and "--role" in cmd
        and "grep" not in cmd
        and "start_hil_adas.py" not in cmd
    )


def kill_adas():
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/%s/cmdline" % name, "rb") as fh:
                cmd = fh.read().replace(b"\0", b" ").decode("utf-8", "replace")
        except IOError:
            continue
        if ADAS_ENTRY in cmd or _is_adas_process(cmd):
            print("kill ADAS pid=%s %s" % (name, cmd))
            try:
                os.kill(int(name), signal.SIGTERM)
            except OSError as exc:
                print("kill failed pid=%s: %s" % (name, exc))
    time.sleep(2.0)
    # Escalate only for stubborn stale ADAS processes. This is intentionally
    # scoped to ADAS.py --role so it does not touch ROS, gateway, or shell tools.
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        try:
            with open("/proc/%s/cmdline" % name, "rb") as fh:
                cmd = fh.read().replace(b"\0", b" ").decode("utf-8", "replace")
        except IOError:
            continue
        if ADAS_ENTRY in cmd or _is_adas_process(cmd):
            print("kill -9 ADAS pid=%s %s" % (name, cmd))
            try:
                os.kill(int(name), signal.SIGKILL)
            except OSError as exc:
                print("kill -9 failed pid=%s: %s" % (name, exc))
    time.sleep(1.0)


def start_adas(role, domain):
    log = "/tmp/adas_hil_%s.log" % role
    cmd = (
        "bash -lc '"
        "source /etc/adas/adas.env; "
        "source $ROS_SETUP; "
        "export ROS_DOMAIN_ID=%d; "
        "export ROS_LOCALHOST_ONLY=0; "
        "export NANO_ROLE=%s; "
        "export PRIMARY_IP=192.168.3.125; "
        "export SECONDARY_IP=192.168.3.124; "
        "export OPENBLAS_CORETYPE=ARMV8; "
        "cd $ADAS_HOME/lx/SOCCode; "
        "nohup python3 ADAS.py --role %s > %s 2>&1 < /dev/null &'"
    ) % (domain, role, role, log)
    run(cmd)
    time.sleep(3.0)
    run("ps -ef | grep '[A]DAS.py' || true")
    run("tail -20 %s 2>/dev/null || true" % log)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--role", choices=["primary", "backup"], required=True)
    parser.add_argument("--domain", type=int, default=43)
    parser.add_argument("--sudo-password", required=True)
    args = parser.parse_args()
    stop_service(args.sudo_password)
    kill_adas()
    start_adas(args.role, args.domain)


if __name__ == "__main__":
    main()
