# -*- coding: utf-8 -*-
"""Reusable paramiko helper to run commands / transfer files on the two Jetson Nanos.

Usage:
    python nano_ssh.py A "command"      # run command on backup nano A over LAN
    python nano_ssh.py B "command"      # run command on primary nano B over LAN
    python nano_ssh.py both "command"   # run on both LAN targets
    python nano_ssh.py A_TUNNEL "cmd"   # old SSH jump/NAT endpoint
"""
import sys
import paramiko

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

# Kept for upload.py compatibility; per-target "host" is preferred.
HOST = "10.18.52.130"
NANOS = {
    "A": {"host": "192.168.3.124", "port": 22, "user": "jetson", "pw": "jetson"},
    "B": {"host": "192.168.3.125", "port": 22, "user": "jetson", "pw": "yahboom"},
    "A_TUNNEL": {"host": "10.18.52.130", "port": 52124, "user": "jetson", "pw": "jetson"},
    "B_TUNNEL": {"host": "10.18.52.130", "port": 52125, "user": "jetson", "pw": "yahboom"},
    "C_TUNNEL": {"host": "10.18.52.130", "port": 52123, "user": "jetson", "pw": "jetson"},
}


def connect(key):
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    n = NANOS[key]
    c.connect(n.get("host", HOST), port=n["port"], username=n["user"], password=n["pw"],
              timeout=20, banner_timeout=30, auth_timeout=30,
              look_for_keys=False, allow_agent=False)
    return c


def run(key, cmd, get_pty=False):
    c = connect(key)
    try:
        stdin, stdout, stderr = c.exec_command(cmd, timeout=120, get_pty=get_pty)
        out = stdout.read().decode("utf-8", "replace")
        err = stderr.read().decode("utf-8", "replace")
        rc = stdout.channel.recv_exit_status()
        return rc, out, err
    finally:
        c.close()


def main():
    target = sys.argv[1]
    cmd = sys.argv[2]
    keys = ["A", "B"] if target == "both" else [target]
    for k in keys:
        print("=" * 30, "NANO", k, "=" * 30)
        try:
            rc, out, err = run(k, cmd)
            print("[rc=%d]" % rc)
            if out:
                print(out)
            if err:
                print("--- stderr ---")
                print(err)
        except Exception as e:
            print("ERROR connecting/running on %s: %r" % (k, e))


if __name__ == "__main__":
    main()
