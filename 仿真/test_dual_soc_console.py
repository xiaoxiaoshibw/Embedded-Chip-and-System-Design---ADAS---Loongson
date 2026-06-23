#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""双 SoC 控制台安全联锁单测（无外部依赖，秒级）。

覆盖：
  - standby 备机（状态帧新鲜）判 OK，主备切换放行；
  - 摘掉最后一个健康 SoC 的操作被拒绝（另一侧 DOWN 或 HUNG）；
  - 状态帧静默 → HUNG；进程退出 → DOWN；启动宽限窗。

运行：python test_dual_soc_console.py   （退出码 0 = 通过）
"""

import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from dual_soc_console import (  # noqa: E402
    DualSocConsole, OK, HUNG, DOWN, STARTUP_GRACE_S, HANG_STALL_S,
)


class FakeWorkers:
    """模拟 WorkerManager 的 alive/inject/start/status/last_status_age。"""

    def __init__(self):
        self.status = {'primary': {'active': True, 'cycle': 10, 'aeb': False},
                       'backup': {'active': False, 'cycle': 0, 'aeb': False}}
        self._alive = {'primary': True, 'backup': True}
        # 状态帧年龄（秒）；None=从未收到
        self._age = {'primary': 0.0, 'backup': 0.0}
        self.calls = []

    def alive(self, r):
        return self._alive[r]

    def last_status_age(self, r):
        return self._age[r]

    def inject(self, r, c):
        self.calls.append(('inject', r, c))

    def start(self, r):
        self.calls.append(('start', r))
        self._alive[r] = True
        self._age[r] = 0.0


def check(cond, msg):
    if not cond:
        print('FAIL:', msg)
        sys.exit(1)
    print('  ok:', msg)


def main():
    w = FakeWorkers()
    c = DualSocConsole(w)
    c.update()

    # 1) 双活：standby 备机状态新鲜 → OK
    check(c.health('primary') == OK and c.health('backup') == OK,
          'standby 备机（帧新鲜）判 OK，不误判 HUNG')

    # 2) 切到备机放行，注入 KILL 给主控
    n = len(w.calls)
    msgs, q = c.handle_key('1', src=0)
    check(('inject', 'primary', 'KILL') in w.calls and not q,
          '双活时"切到备机"放行并杀主控')

    # 3) 主控掉线后，杀备控必须被拒绝（否则双失效）
    w._alive['primary'] = False
    w._age['primary'] = None
    c.update()
    check(c.health('primary') == DOWN, '主控进程退出判 DOWN')
    n = len(w.calls)
    msgs, q = c.handle_key('b', src=1)
    check(len(w.calls) == n and msgs and msgs[0].startswith('×'),
          '主控 DOWN 时拒绝杀备控（保留最后一个健康 SoC）')

    # 4) 恢复主控；备控状态帧静默 → HUNG；此时切到备机应被拒
    c.handle_key('0', src=1)
    w._alive['primary'] = True
    w._age['primary'] = 0.0
    w._age['backup'] = HANG_STALL_S + 0.5      # 备控帧静默
    c.update()
    check(c.health('backup') == HUNG, '备控状态帧静默判 HUNG')
    n = len(w.calls)
    msgs, q = c.handle_key('1', src=0)
    check(len(w.calls) == n and msgs and msgs[0].startswith('×'),
          '备控 HUNG 时拒绝切到备机（接管目标不健康）')

    # 5) 启动宽限：进程刚拉起、首帧未到（age=None）应判 OK
    w2 = FakeWorkers()
    w2._age['backup'] = None
    c2 = DualSocConsole(w2)
    c2.update()
    check(c2.health('backup') == OK, '首帧未到但在启动宽限窗内判 OK')

    # 6) 杀备控（主控健康）允许——失冗余但不致车辆失控
    w3 = FakeWorkers()
    c3 = DualSocConsole(w3)
    c3.update()
    n = len(w3.calls)
    msgs, q = c3.handle_key('b', src=0)
    check(('inject', 'backup', 'KILL') in w3.calls,
          '主控健康时允许杀备控')

    print('PASS: 双 SoC 控制台安全联锁全部通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
