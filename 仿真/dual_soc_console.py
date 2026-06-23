#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""双 SoC 主备控制台（带安全联锁）。

给 CARLA 联合仿真提供一个"看得见、切得动、切不坏"的控制台：

  1. 实时面板：主/备 SoC 健康度（OK/卡死/掉线）+ 当前执行源 + 车辆状态。
  2. 主备切换：一键把执行权从主控切到备控、再切回，模拟整机故障。
  3. 安全联锁（核心）：任何会"摘掉最后一个健康 SoC"的操作都会被拒绝，
     从控制台层面**杜绝双控同时失效 → 看门狗紧急制动**，保证车辆不出故障。

健康判定：进程存活 + soc_worker 状态帧持续到达（每 0.1s 一帧，与是否接管无关）。
控制环卡死时整个 run 循环阻塞 → 状态帧停发 → 判为卡死（HUNG）。
注意：不能用 cycle 计数判活——备机 standby 时被 is_active() 门提前返回、
cycle 本就不推进，但它完全健康、随时可接管。

被 run_cosim.run_scenario() / sil_demo.py 复用：每拍调 update()，按键走 handle_key()。
"""

import time

# 健康状态
OK = 'OK'
HUNG = 'HUNG'    # 进程在，但控制环卡死（状态帧停发）
DOWN = 'DOWN'    # 进程已退出

SRC_NAMES = {0: '主控 PRIMARY', 1: '备控 BACKUP', 9: '看门狗 WATCHDOG'}

HANG_STALL_S = 0.6        # 状态帧静默超过此时长判为卡死（正常 0.1s 一帧）
STARTUP_GRACE_S = 2.0     # 进程刚拉起、首帧未到的宽限窗
HELP = (
    '\n双 SoC 主备控制台  ——  指令：\n'
    '  1 = 切到备机（模拟主控故障，备机接管）   0 = 切回主机（重启主控回切）\n'
    '  p = 杀主控   P = 重启主控   b = 杀备控   B = 重启备控   h = 主控卡死\n'
    '  s = 打印状态面板   ? = 帮助   q = 结束场景\n'
    '  注意 安全联锁：会导致双控同时失效的操作将被拒绝，车辆始终留有一个健康 SoC。\n'
)


class DualSocConsole:
    """封装主备健康跟踪 + 安全联锁 + 状态面板渲染。"""

    def __init__(self, workers):
        self.workers = workers
        # 进程首次被本控制台观察到的时刻，用于首帧未到前的启动宽限
        self._first_seen = {'primary': None, 'backup': None}

    # ── 健康判定 ──────────────────────────────────────────────
    def update(self):
        """每拍调用：记录进程首次出现的时刻（用于启动宽限）。"""
        now = time.monotonic()
        for role in ('primary', 'backup'):
            if self.workers.alive(role):
                if self._first_seen[role] is None:
                    self._first_seen[role] = now
            else:
                self._first_seen[role] = None

    def _status_age(self, role):
        """兼容真实 WorkerManager 与单测 fake：取状态帧年龄(秒)。"""
        f = getattr(self.workers, 'last_status_age', None)
        if callable(f):
            return f(role)
        return 0.0   # fake workers 无时戳：视为始终新鲜

    def health(self, role):
        if not self.workers.alive(role):
            return DOWN
        age = self._status_age(role)
        if age is None:
            # 还没收到过状态帧：启动宽限窗内按 OK，超时则判 HUNG
            seen = self._first_seen.get(role)
            if seen is None or (time.monotonic() - seen) <= STARTUP_GRACE_S:
                return OK
            return HUNG
        if age > HANG_STALL_S:
            return HUNG
        return OK

    def healthy(self, role):
        return self.health(role) == OK

    # ── 安全联锁 ──────────────────────────────────────────────
    def _other(self, role):
        return 'backup' if role == 'primary' else 'primary'

    def _guard_disable(self, role, what):
        """拦截"摘掉最后一个健康 SoC"的操作。

        返回 None 表示放行；返回字符串表示拒绝原因。
        """
        other = self._other(role)
        if not self.healthy(other):
            return ('× 已拒绝[%s]：%s 当前为 %s，再让 %s 失效将导致双控同时'
                    '失效→看门狗紧急制动。安全联锁已保护车辆。'
                    % (what, '备控' if other == 'backup' else '主控',
                       self.health(other),
                       '主控' if role == 'primary' else '备控'))
        return None

    # ── 按键处理 ──────────────────────────────────────────────
    def handle_key(self, k, src=None):
        """处理一个控制台按键。返回 (要打印的消息列表, quit?)。"""
        w = self.workers
        msgs = []

        if k == 'q':
            return msgs, True
        if k in ('?', 'H'):
            msgs.append(HELP)
            return msgs, False
        if k == 's':
            msgs.append(self.render_panel(src=src))
            return msgs, False

        # 切到备机（= 安全地让主控失效，备机接管）
        if k in ('1', 'p', 'h'):
            what = {'1': '切到备机', 'p': '杀主控', 'h': '主控卡死'}[k]
            deny = self._guard_disable('primary', what)
            if deny:
                msgs.append(deny)
                return msgs, False
            w.inject('primary', 'HANG' if k == 'h' else 'KILL')
            msgs.append('√ %s：主控已%s，备控将在 ~150ms 内接管（SRC 0→1）。'
                        % (what, '卡死' if k == 'h' else '下线'))
            return msgs, False

        # 切回主机
        if k in ('0', 'P'):
            w.start('primary')
            msgs.append('√ 切回主机：主控重启中，恢复后将回切（SRC 1→0）。')
            return msgs, False

        # 杀/重启备控
        if k == 'b':
            deny = self._guard_disable('backup', '杀备控')
            if deny:
                msgs.append(deny)
                return msgs, False
            w.inject('backup', 'KILL')
            msgs.append('√ 杀备控：失去冗余但主控仍在驾驶（仍允许，因主控健康）。')
            return msgs, False
        if k == 'B':
            w.start('backup')
            msgs.append('√ 重启备控：冗余恢复中。')
            return msgs, False

        return msgs, False

    # ── 状态面板 ──────────────────────────────────────────────
    def _badge(self, role):
        h = self.health(role)
        sym = {OK: '●运行', HUNG: '▲卡死', DOWN: '○掉线'}[h]
        if h == DOWN:
            return sym                      # 掉线：不显示陈旧的 active/cyc
        st = self.workers.status.get(role) or {}
        act = '执行' if st.get('active') else '待命'
        aeb = ' AEB!' if st.get('aeb') else ''
        cyc = st.get('cycle')
        cyc_s = ('cyc=%d' % cyc) if cyc is not None else 'cyc=--'
        return '%s %s %s%s' % (sym, act, cyc_s, aeb)

    def render_panel(self, sim_t=None, frame=None, gap=None, src=None,
                     out=None):
        """渲染一行/多行状态面板字符串。"""
        redundancy = '双活' if (self.healthy('primary')
                                and self.healthy('backup')) else '无冗余'
        lines = []
        lines.append('┌─ 双 SoC 状态 ' + '─' * 46)
        lines.append('│ 主控 PRIMARY : %-32s' % self._badge('primary'))
        lines.append('│ 备控 BACKUP  : %-32s' % self._badge('backup'))
        srcline = SRC_NAMES.get(src, '?') if src is not None else '?'
        lines.append('│ 执行源 SRC   : %-20s 冗余: %s' % (srcline, redundancy))
        if frame is not None:
            v = frame.get('ego_v', 0.0)
            off = frame.get('lane_offset', 0.0)
            gap_s = ('%.1fm' % gap) if (gap not in (None, float('inf'))) else '--'
            ttc = out['state'].ttc if (out and 'state' in out) else None
            ttc_s = ('%.1fs' % min(ttc, 999.9)) if ttc is not None else '--'
            t_s = ('t=%.1fs ' % sim_t) if sim_t is not None else ''
            lines.append('│ 车辆 %sv=%.2fm/s gap=%s off=%+.2fm ttc=%s'
                         % (t_s, v, gap_s, off, ttc_s))
        lines.append('└' + '─' * 60)
        return '\n'.join(lines)
