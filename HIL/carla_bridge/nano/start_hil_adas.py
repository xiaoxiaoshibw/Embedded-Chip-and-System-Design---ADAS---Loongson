#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Start ADAS.py for HIL under systemd supervision (real-system self-heal).

与真实部署一致：ADAS 不再用裸 ``nohup`` 拉起（被杀即死、无人接管），而是注册为
systemd transient 单元 ``adas-hil-<role>.service``，带 ``Restart=always`` —— 进程
被杀 / 崩溃后约 2s 内自动拉起，正如生产环境的 ``adas-node.service``。

CLI 与旧版完全兼容（``--role`` / ``--domain`` / ``--sudo-password``），故
``hil_platform``（hardware_control / nano_fault）与 launch 脚本无需改动即可获得自愈。
日志仍写 ``/tmp/adas_hil_<role>.log``（hil_platform 据此读状态），用 systemd
``StandardOutput=append:`` 落盘，跨重启追加不丢历史。
"""

import argparse
import os
import shlex
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


def unit_name(role):
    return "adas-hil-%s.service" % role


def stop_service(password):
    # HIL 期间不与生产 adas-node.service 并存（它跑在 domain 42，会和 HIL domain 43 错配）。
    run("sudo -S systemctl stop adas-node.service || true", password + "\n")


def stop_hil_unit(role, password):
    # 停掉同名旧 HIL transient 单元并清 failed 记录，让 systemd-run 能用同名重新注册
    # （--collect 单元停掉即回收）。这样重复调用本脚本是幂等的。
    unit = unit_name(role)
    run("sudo -S systemctl stop %s 2>/dev/null || true" % unit, password + "\n")
    run("sudo -S systemctl reset-failed %s 2>/dev/null || true" % unit, password + "\n")


def _is_adas_process(cmd):
    return (
        "ADAS.py" in cmd
        and "--role" in cmd
        and "grep" not in cmd
        and "start_hil_adas.py" not in cmd
    )


def kill_adas():
    # 兜底清理任何"裸"孤儿 ADAS（老式 nohup 启动残留 / 旧版部署）。受监管单元已在
    # stop_hil_unit 中停掉，故此处不会误杀正常 transient 实例。
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


def start_adas(role, domain, password):
    unit = unit_name(role)
    log = "/tmp/adas_hil_%s.log" % role
    # 默认把 ADAS 钉到 core 0,1（控制主循环 + DDS/ML 两热线程各得一核，减 100Hz 抖动）；
    # gateway 用 core2、edge 用 core3 由 hil_platform 侧分配。可用 ADAS_CPU_LIST 覆盖。
    cpu_list = os.environ.get("ADAS_CPU_LIST", "0,1").strip()
    taskset = ("taskset -c %s " % cpu_list) if cpu_list else ""
    # 单元 ExecStart：source 配置 + ROS2 + HIL 隔离 domain，再 exec ADAS（exec 让
    # python 成为单元 MainPID，被杀即触发 systemd 重启）。$ROS_SETUP / $ADAS_HOME 由
    # /etc/adas/adas.env 提供，保持字面量交给内层 bash 展开。
    #
    # 日志用 shell 内重定向 `>> LOG 2>&1`（保留 /tmp/adas_hil_<role>.log 契约，
    # hil_platform 读它取状态）。不用 systemd StandardOutput=append: —— 那在 User=
    # 且日志文件已存在时会 EACCES（209/STDOUT）；shell 内由 jetson 自己打开自有文件
    # 则无此问题，且跨 systemd 自动重启天然追加不丢历史。
    inner = (
        "source /etc/adas/adas.env; "
        "source \"$ROS_SETUP\"; "
        "export ROS_DOMAIN_ID=%d; "
        "export ROS_LOCALHOST_ONLY=0; "
        "export NANO_ROLE=%s; "
        "export PRIMARY_IP=192.168.3.125; "
        "export SECONDARY_IP=192.168.3.124; "
        "export OPENBLAS_CORETYPE=ARMV8; "
        "cd \"$ADAS_HOME/lx/SOCCode\"; "
        "exec %spython3 ADAS.py --role %s >> %s 2>&1"
    ) % (domain, role, taskset, role, log)

    # 1) 每次显式启动清空日志（自有文件，免 sudo）；systemd 自动重启则在本次会话内追加。
    run(": > %s 2>/dev/null || true" % log)
    # 2) 修 ESP32 串口权限（与生产 adas-node.service 的 ExecStartPre 一致：Tegra udev
    #    开机不可靠，开跑前兜底）。
    run("sudo -S sh -c 'chgrp dialout /dev/ttyTHS1 2>/dev/null; "
        "chmod 660 /dev/ttyTHS1 2>/dev/null; true'", password + "\n")
    # 3) 注册受 systemd 监管的 transient 单元：Restart=always 即真实系统自愈语义。
    props = [
        "--property=Restart=always",
        "--property=RestartSec=2",
        # 永不因频繁重启被 systemd 限流（默认 5 次/10s 会进 failed 不再拉起）。
        "--property=StartLimitIntervalSec=0",
        # 与生产单元一致：优雅停用 SIGINT，让 ADAS 走 KeyboardInterrupt 收尾。
        "--property=KillSignal=SIGINT",
        "--property=User=jetson",
        "--property=Group=jetson",
        "--property=Environment=HOME=/home/jetson",
    ]
    cmd = (
        "sudo -S systemd-run --unit=%s --collect %s /bin/bash -lc %s"
    ) % (unit, " ".join(props), shlex.quote(inner))
    run(cmd, password + "\n")
    time.sleep(3.0)
    run("systemctl --no-pager --full status %s 2>&1 | head -n 14" % unit)
    run("tail -n 12 %s 2>/dev/null || true" % log)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--role", choices=["primary", "backup"], required=True)
    parser.add_argument("--domain", type=int, default=43)
    parser.add_argument("--sudo-password", required=True)
    args = parser.parse_args()
    stop_service(args.sudo_password)
    stop_hil_unit(args.role, args.sudo_password)
    kill_adas()
    start_adas(args.role, args.domain, args.sudo_password)


if __name__ == "__main__":
    main()
