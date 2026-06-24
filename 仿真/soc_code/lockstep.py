#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单板软件双核锁步（software lockstep / DMR + 逐拍比较器）。

把硬件锁步（Master + Checker 影子核 + 专用比较器）的**思想**用软件复刻在一块
Nano 上：主核(core0)算控制管线产出 (delta / lon_cmd / AEB)；**影子/Checker 线程
钉在 LOCKSTEP_CHECKER_CORE(默认 core2)**，对**同一拍输入 + 同一拍前状态**再算一遍，
逐拍比较两份输出，连续 N 拍失配即报故障、由主循环进入安全态。结构对应教学图里
Master/Checker/比较器/encode。

确定性保证（零误报）：影子用主核本拍**前**的状态深拷贝 + 同一输入快照 + 主核本拍
的 ml_result，跑同一份 run_pure_pipeline → 同 CPU 同码必 bit 一致；唯有真实计算
故障（位翻转等）或注入故障才会失配。

诚实边界（非 ISO 26262 硬件锁步）：
- 比较粒度是**每拍(10ms)**而非每时钟周期；检出延迟约 1~N 拍。
- 单板共享 OS / 内存 / 供电 → 有共因失效风险；抓不了纳秒级 SEU。
- Python GIL 下影子计算与主线程分时（核隔离减少缓存污染，但非真正并行）。
所以定位是"锁步思想的软件近似 / 演示"，非硬件锁步。

安全：全程 best-effort，默认关闭（config.LOCKSTEP_ENABLED）；任何内部异常都不影响
主控制路径——只有显式比对失配才触发安全态（这正是要演示的行为）。
"""

import copy
import dataclasses
import logging
import os
import threading

try:
    import queue
except ImportError:  # pragma: no cover - py2 fallback, 不会触发
    import Queue as queue

from pipeline import run_pure_pipeline
from control.context import ControlManagers

logger = logging.getLogger(__name__)


def snapshot_managers(mgr):
    """深拷贝一份 ControlManagers 供影子核独立演算，但 **排除 ml_bridge**
    （异步线程 / ONNX 会话不可深拷贝；影子核改用主核传入的 ml_result）。"""
    fields = {}
    for f in dataclasses.fields(mgr):
        if f.name == 'ml_bridge':
            fields[f.name] = None
        else:
            fields[f.name] = copy.deepcopy(getattr(mgr, f.name))
    return ControlManagers(**fields)


class LockstepChecker:
    """影子核比较器。主循环每拍 submit 前状态 + 主核输出；本线程在 checker 核
    重算并逐拍比较，失配去抖后置 fault。"""

    def __init__(self, checker_core=2, delta_eps=1e-9, lon_eps=1e-9,
                 debounce_n=2, inject=False, inject_delta=0.05):
        self.enabled = True
        self.checker_core = int(checker_core)
        self.delta_eps = float(delta_eps)
        self.lon_eps = float(lon_eps)
        self.debounce_n = max(1, int(debounce_n))
        self._inject = bool(inject)
        self.inject_delta = float(inject_delta)

        self.fault = False
        self.fault_reason = ''
        self.compared = 0
        self.mismatch_total = 0
        self._mismatch_run = 0
        self._q = queue.Queue(maxsize=4)
        self._t = threading.Thread(target=self._run, name='lockstep-checker',
                                   daemon=True)
        self._t.start()

    # ── 控制接口（演示用）──
    def set_inject(self, on):
        self._inject = bool(on)

    def clear_fault(self):
        self.fault = False
        self.fault_reason = ''
        self._mismatch_run = 0

    def submit(self, now, signals, memory, managers, takeover_rate, ml_result,
               main_delta, main_lon, main_aeb):
        """主线程调用：投递本拍前状态（已深拷贝）+ 主核输出。非阻塞，队列满则丢
        （影子落后时跳过该拍，不产生误报）。"""
        try:
            self._q.put_nowait((now, signals, memory, managers, takeover_rate,
                                ml_result, float(main_delta), float(main_lon),
                                bool(main_aeb)))
        except queue.Full:
            pass

    # ── 影子核线程 ──
    def _pin_self(self):
        # 每次循环重钉到 checker 核：胜过 rt_affinity keeper 的低频重扫（3s），
        # 把影子稳定保持在 checker 核。checker 核不在进程允许集时静默放弃（best-effort）。
        try:
            if self.checker_core in os.sched_getaffinity(0):
                os.sched_setaffinity(0, {self.checker_core})
        except (AttributeError, OSError):
            pass

    def _run(self):
        announced = False
        while True:
            item = self._q.get()
            self._pin_self()
            if not announced:
                try:
                    cur = sorted(os.sched_getaffinity(0))
                except (AttributeError, OSError):
                    cur = '?'
                logger.info('[LOCKSTEP] checker 线程就绪，目标核=%d 实际允许=%s inject=%s',
                            self.checker_core, cur, self._inject)
                announced = True
            (now, signals, memory, managers, takeover_rate, ml_result,
             m_delta, m_lon, m_aeb) = item
            try:
                shadow = run_pure_pipeline(now, signals, memory, managers,
                                           takeover_rate, ml_result=ml_result)
            except Exception as exc:
                logger.warning('[LOCKSTEP] 影子计算异常（忽略本拍）：%r', exc)
                self._mismatch_run = 0
                continue
            s_delta = shadow.lateral_ctx.delta
            s_lon = shadow.lon_cmd
            s_aeb = bool(shadow.lon_ctx.aeb_active)
            if self._inject:
                # 注入"故障 checker 核"：让影子 delta 偏移，制造可控失配以演示检出。
                s_delta = s_delta + self.inject_delta

            self.compared += 1
            d_delta = abs(s_delta - m_delta)
            d_lon = abs(s_lon - m_lon)
            mism = (d_delta > self.delta_eps or d_lon > self.lon_eps
                    or s_aeb != m_aeb)
            if mism:
                self.mismatch_total += 1
                self._mismatch_run += 1
                if self._mismatch_run >= self.debounce_n and not self.fault:
                    self.fault = True
                    self.fault_reason = (
                        'Δdelta=%.5f Δlon=%.5f AEB(主=%d/影=%d) 连续=%d拍'
                        % (d_delta, d_lon, int(m_aeb), int(s_aeb),
                           self._mismatch_run))
                    logger.critical('[LOCKSTEP] 比较器失配 → 报故障：%s',
                                    self.fault_reason)
            else:
                self._mismatch_run = 0
