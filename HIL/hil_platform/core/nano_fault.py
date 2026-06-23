# -*- coding: utf-8 -*-
"""真实硬件故障注入：在真实双 Nano 上制造可恢复的故障以演示 ESP32 接管。

实现方式：SSH 给目标 Nano 上的 ADAS.py 进程发 **SIGSTOP**（冻结 → 心跳静默/SEQ
停滞 → 对端 watchdog 判定接管），到点再发 **SIGCONT** 恢复。
- 完全可逆、无需 sudo、不重启进程、不动 SOC 代码；
- 与软件 mock 的故障语义对应：断心跳/seq 卡死 → 冻结主控；双路失败 → 冻结两台。

仅 control_source='nano' 时由 SimulationCore 调用；mock/internal 模式用本地 FaultInjector。
"""

from __future__ import annotations

import threading
import time
from typing import Dict, List, Optional

import paramiko


class NanoFaultController:
    def __init__(self, primary_host: str, backup_host: str,
                 primary_pw: str, backup_pw: str, user: str = "jetson",
                 auto_restore_s: float = 8.0):
        self._hosts = {
            "primary": {"host": primary_host, "user": user, "pw": primary_pw,
                        "pat": "ADAS.py --role primary"},
            "backup": {"host": backup_host, "user": user, "pw": backup_pw,
                       "pat": "ADAS.py --role backup"},
        }
        self.auto_restore_s = float(auto_restore_s)
        self._timers: List[threading.Timer] = []
        self._busy_until = 0.0   # 故障进行中（含自动恢复窗口），期间拒绝重复注入
        # 复用持久 SSH 连接：避免每次故障都重连(~1s)，让冻结近乎瞬时（接近真实 ~ms 级接管）
        self._clients: Dict[str, paramiko.SSHClient] = {}
        self._cli_lock = threading.Lock()

    # ── SSH（持久连接，断了自动重连）──
    def _client(self, role: str) -> paramiko.SSHClient:
        with self._cli_lock:
            c = self._clients.get(role)
            if c is not None and c.get_transport() is not None and c.get_transport().is_active():
                return c
            n = self._hosts[role]
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(n["host"], port=22, username=n["user"], password=n["pw"],
                      timeout=8, banner_timeout=12, auth_timeout=12,
                      look_for_keys=False, allow_agent=False)
            try:
                c.get_transport().set_keepalive(15)
            except Exception:
                pass
            self._clients[role] = c
            return c

    def warmup(self) -> None:
        """预连两台 Nano，使第一次故障注入也无连接延迟。"""
        for role in self._hosts:
            try:
                self._client(role)
            except Exception:
                pass

    def _signal(self, role: str, sig: str) -> None:
        # -STOP 冻结 / -CONT 恢复 该 Nano 的 ADAS.py（持久连接上开新 channel，~十几 ms）
        cmd = "pkill -%s -f '%s' || true" % (sig, self._hosts[role]["pat"])
        try:
            c = self._client(role)
            _in, out, _err = c.exec_command(cmd, timeout=8)
            out.channel.recv_exit_status()
        except Exception:
            # 连接可能失效，丢弃后下次重连
            with self._cli_lock:
                self._clients.pop(role, None)
            raise

    # ── 注入 / 恢复 ──
    @staticmethod
    def _roles_for(target: str) -> List[str]:
        if target in ("both", "dual"):
            return ["primary", "backup"]
        if target == "nano_b":
            return ["backup"]
        return ["primary"]   # nano_a / 默认 = 主控

    def fault(self, fault_type: str, target: str, sim_t: float) -> Dict:
        """冻结目标 Nano（真实断心跳）→ 触发真实接管；到点自动恢复。立即返回事件。

        SSH 是阻塞 I/O，**必须在后台线程执行**——否则会卡住调用方持有的核心锁、拖垮整个后端。
        """
        # 防重复：上一次故障的「冻结+自动恢复」窗口未结束时拒绝再注入，
        # 避免连点把真实双机失效仲裁的 TAKEOVER_COOLDOWN 触发到抑制接管。
        now = time.monotonic()
        if now < self._busy_until:
            return {"time": round(sim_t, 3), "type": "FAULT_BUSY",
                    "target": target,
                    "detail": "上一次故障/恢复进行中（约 %.0fs 后可再注入）" % (self._busy_until - now)}
        self._busy_until = now + self.auto_restore_s + 3.0   # +3s 让真实接管冷却清零

        roles = self._roles_for(target if fault_type != "dual_fail" else "both")

        def _stop():
            for role in roles:
                try:
                    self._signal(role, "STOP")
                except Exception:
                    pass
        threading.Thread(target=_stop, name="nano-fault-stop", daemon=True).start()

        # 定时自动恢复（一键演示：接管后自动切回，链路不残留冻结节点）
        if self.auto_restore_s > 0:
            tm = threading.Timer(self.auto_restore_s, self._auto_restore, args=(roles,))
            tm.daemon = True
            tm.start()
            self._timers.append(tm)
        return {
            "time": round(sim_t, 3), "type": "FAULT_INJECTED",
            "target": target, "detail": "%s(冻结 %s, %.0fs 后自动恢复)" % (
                fault_type, "/".join(roles), self.auto_restore_s),
        }

    def _auto_restore(self, roles: List[str]) -> None:
        for role in roles:
            try:
                self._signal(role, "CONT")
            except Exception:
                pass

    def restore_all(self) -> None:
        """恢复两台 Nano（reset/stop 时的安全兜底，确保不留冻结进程）。"""
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

    def close(self) -> None:
        self.restore_all()
        with self._cli_lock:
            for c in self._clients.values():
                try:
                    c.close()
                except Exception:
                    pass
            self._clients = {}
