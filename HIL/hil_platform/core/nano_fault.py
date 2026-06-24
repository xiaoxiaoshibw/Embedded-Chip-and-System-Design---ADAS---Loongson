# -*- coding: utf-8 -*-
"""Hardware-level Nano fault injection for the real HIL path.

In nano control mode the WebUI should exercise the real ESP32 watchdog path.
The default injected fault terminates the selected Nano ADAS process, which
removes heartbeat/control output. After the restore window, the controller
starts ADAS again if the Nano is still reachable.
"""

from __future__ import annotations

import threading
import time
import os
from typing import Dict, List

import paramiko


class NanoFaultController:
    def __init__(self, primary_host: str, backup_host: str,
                 primary_pw: str, backup_pw: str, user: str = "jetson",
                 auto_restore_s: float = 8.0):
        self._hosts = {
            "primary": {
                "host": primary_host,
                "user": user,
                "pw": primary_pw,
                "pat": "ADAS.py --role primary",
                "role": "primary",
            },
            "backup": {
                "host": backup_host,
                "user": user,
                "pw": backup_pw,
                "pat": "ADAS.py --role backup",
                "role": "backup",
            },
        }
        self.auto_restore_s = float(auto_restore_s)
        self._timers: List[threading.Timer] = []
        self._busy_until = 0.0
        self._clients: Dict[str, paramiko.SSHClient] = {}
        self._cli_lock = threading.Lock()

    def _client(self, role: str) -> paramiko.SSHClient:
        with self._cli_lock:
            c = self._clients.get(role)
            if c is not None and c.get_transport() is not None and c.get_transport().is_active():
                return c
            n = self._hosts[role]
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(
                n["host"],
                port=22,
                username=n["user"],
                password=n["pw"],
                timeout=8,
                banner_timeout=12,
                auth_timeout=12,
                look_for_keys=False,
                allow_agent=False,
            )
            try:
                c.get_transport().set_keepalive(15)
            except Exception:
                pass
            self._clients[role] = c
            return c

    def warmup(self) -> None:
        for role in self._hosts:
            try:
                self._client(role)
            except Exception:
                pass

    def _exec(self, role: str, cmd: str, timeout: int = 8) -> None:
        try:
            c = self._client(role)
            _in, out, _err = c.exec_command(cmd, timeout=timeout)
            out.channel.recv_exit_status()
        except Exception:
            with self._cli_lock:
                self._clients.pop(role, None)
            raise

    def _signal(self, role: str, sig: str) -> None:
        self._exec(role, "pkill -%s -f '%s' || true" % (sig, self._hosts[role]["pat"]))

    def _restart_adas(self, role: str) -> None:
        adas_role = self._hosts[role]["role"]
        pw = self._hosts[role]["pw"]
        cpu_env = (
            os.environ.get("PRIMARY_ADAS_CPUS", "0,1")
            if role == "primary"
            else os.environ.get("BACKUP_ADAS_CPUS", "0,1")
        )
        cmd = (
            "cd /home/jetson/adas/hil && "
            "source /opt/ros/foxy/setup.bash 2>/dev/null; "
            "export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=0; "
            "export ADAS_CPU_LIST='%s'; "
            "python3 /home/jetson/adas/hil/start_hil_adas.py --role %s --sudo-password '%s'"
        ) % (cpu_env.replace("'", "'\"'\"'"), adas_role, pw.replace("'", "'\"'\"'"))
        self._exec(role, cmd, timeout=20)

    @staticmethod
    def _roles_for(target: str) -> List[str]:
        if target in ("both", "dual"):
            return ["primary", "backup"]
        if target == "nano_b":
            return ["backup"]
        return ["primary"]

    def fault(self, fault_type: str, target: str, sim_t: float) -> Dict:
        now = time.monotonic()
        if now < self._busy_until:
            return {
                "time": round(sim_t, 3),
                "type": "FAULT_BUSY",
                "target": target,
                "detail": "previous hardware fault is restoring; retry in %.0fs" % (
                    self._busy_until - now
                ),
            }
        self._busy_until = now + self.auto_restore_s + 3.0
        roles = self._roles_for(target if fault_type != "dual_fail" else "both")

        def _kill() -> None:
            for role in roles:
                try:
                    self._signal(role, "TERM")
                except Exception:
                    pass

        threading.Thread(target=_kill, name="nano-fault-kill", daemon=True).start()

        if self.auto_restore_s > 0:
            tm = threading.Timer(self.auto_restore_s, self._auto_restore, args=(roles,))
            tm.daemon = True
            tm.start()
            self._timers.append(tm)
        return {
            "time": round(sim_t, 3),
            "type": "FAULT_INJECTED",
            "target": target,
            "detail": "%s(kill %s, %.0fs later restart ADAS)" % (
                fault_type,
                "/".join(roles),
                self.auto_restore_s,
            ),
        }

    def _auto_restore(self, roles: List[str]) -> None:
        for role in roles:
            try:
                self._restart_adas(role)
            except Exception:
                pass

    def restore_all(self) -> None:
        for tm in self._timers:
            try:
                tm.cancel()
            except Exception:
                pass
        self._timers = []
        for role in ("primary", "backup"):
            try:
                self._signal(role, "CONT")
            except Exception:
                pass
            try:
                self._restart_adas(role)
            except Exception:
                pass

    def close(self) -> None:
        self.restore_all()
        with self._cli_lock:
            for c in self._clients.values():
                try:
                    c.close()
                except Exception:
                    pass
            self._clients = {}
