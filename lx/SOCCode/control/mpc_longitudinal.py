#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""可切换的纵向 MPC 控制器（与 LongitudinalController 接口等价的 drop-in）。

可行性结论（Jetson Nano / 100Hz / 8ms 预算）：
  - 在线跑通用 QP 风险高、收益低。采用"有限时域 LQ + 约束投影"等价实现：
    2 状态线性模型 [e=车距误差, vr=相对速度]，离散 LQR 增益在**构造时**
    用纯 Python Riccati 迭代算一次；在线只做 u=-Kx + 前车加速度前馈 +
    钳位 + 安全距离硬投影，全部 O(1)，微秒级，远低于 8ms。
  - 不引入在线 numpy 依赖（纯 Python 标量/小矩阵）。
  - 约束：加减速上限用 clamp；唯一关键不等式（安全距离）用显式 override
    （量产 ACC 常规做法）。
  - 自动回退：见 FallbackLonController；异常或超预算永久回退 PID。

接口与 longitudinal.LongitudinalController 一致：
  compute(dist, ego_v, lead_v_proj, lead_accel, min_safe_dist, curv) -> lon_cmd
  reset(); 属性 i_term; set_gains(**kw)
lon_cmd 约定：正=减速，负=加速（与 PID 版本完全一致）。

config.LON_CONTROLLER='pid'（默认）时本类根本不被实例化，行为零变化。
单线程、Python 3.6 兼容。
"""

import logging
import time

from config import (
    ACC_D0,
    ACC_FF_MAX,
    ACC_TIME_GAP,
    CONTROL_LOOP_BUDGET_S,
    CURV_NO_ACCEL_THRESH,
    LON_CMD_MAX_BRAKE_DECEL,
    LON_CMD_MAX_DRIVE_ACCEL,
    MPC_LEAD_FF_GAIN,
    MPC_Q_E,
    MPC_Q_V,
    MPC_R,
    MPC_RICCATI_ITERS,
    MPC_TS,
)


def _design_lqr_gain(ts, q_e, q_v, r, iters):
    """离散 LQR 增益 K（纯 Python，构造时算一次）。

    模型：x=[e,vr]，A=[[1,Ts],[0,1]]，B=[[0],[-Ts]]，Q=diag(q_e,q_v)，R=r。
    迭代 Riccati：P = Q + A'PA - A'PB (R + B'PB)^-1 B'PA。
    标量化展开（2x2，B'PB 为标量），不依赖任何线代库。
    返回 (k1, k2)：u* = -(k1*e + k2*vr)。
    """
    a11, a12, a21, a22 = 1.0, ts, 0.0, 1.0
    b1, b2 = 0.0, -ts
    p11, p12, p22 = q_e, 0.0, q_v
    k1 = k2 = 0.0
    for _ in range(max(1, iters)):
        # B'PB (标量) + R
        pb1 = p11 * b1 + p12 * b2
        pb2 = p12 * b1 + p22 * b2
        bpb = b1 * pb1 + b2 * pb2
        denom = r + bpb
        if denom <= 1e-12:
            denom = 1e-12
        # B'PA (1x2)
        bpa1 = b1 * (p11 * a11 + p12 * a21) + b2 * (p12 * a11 + p22 * a21)
        bpa2 = b1 * (p11 * a12 + p12 * a22) + b2 * (p12 * a12 + p22 * a22)
        k1 = bpa1 / denom
        k2 = bpa2 / denom
        # A'PA
        apa11 = a11 * (p11 * a11 + p12 * a21) + a21 * (p12 * a11 + p22 * a21)
        apa12 = a11 * (p11 * a12 + p12 * a22) + a21 * (p12 * a12 + p22 * a22)
        apa22 = a12 * (p11 * a12 + p12 * a22) + a22 * (p12 * a12 + p22 * a22)
        # A'PB (2x1)
        apb1 = a11 * pb1 + a21 * pb2
        apb2 = a12 * pb1 + a22 * pb2
        n11 = apb1 * bpa1 / denom
        n12 = apb1 * bpa2 / denom
        n22 = apb2 * bpa2 / denom
        np11 = q_e + apa11 - n11
        np12 = apa12 - n12
        np22 = q_v + apa22 - n22
        if (abs(np11 - p11) + abs(np12 - p12) + abs(np22 - p22)) < 1e-10:
            p11, p12, p22 = np11, np12, np22
            break
        p11, p12, p22 = np11, np12, np22
    return k1, k2


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MpcLongitudinalController:
    """有限时域 LQ + 约束投影的纵向控制器（drop-in）。"""

    def __init__(self, dt):
        self._dt = dt
        self._k1, self._k2 = _design_lqr_gain(
            MPC_TS, MPC_Q_E, MPC_Q_V, MPC_R, MPC_RICCATI_ITERS)
        logging.info('[MPC] longitudinal LQ gain k=(%.4f, %.4f) Ts=%.3f',
                     self._k1, self._k2, MPC_TS)

    def reset(self):
        """无积分状态，空操作（保持与 PID 接口一致）。"""
        return

    @property
    def i_term(self):
        """MPC 无积分项；返回 0.0（telemetry 列在 MPC 模式恒为 0）。"""
        return 0.0

    def set_gains(self, **kw):
        """ACC 增益热更新对 MPC 不适用，安全忽略（仅 debug 记录）。"""
        if kw:
            logging.debug('[MPC] set_gains ignored in MPC mode: %s', kw)

    def compute(self, dist, ego_v, lead_v_proj, lead_accel,
                min_safe_dist, curv):
        """单步纵向控制，返回 lon_cmd（正=减速）。"""
        # 参考间距与 PID 版一致，保证两控制器目标一致、可公平对比
        d_ref = max(ACC_D0 + ACC_TIME_GAP * max(ego_v, 0.0), min_safe_dist)
        e = dist - d_ref                 # 车距误差（>0=偏远，可加速）
        vr = lead_v_proj - ego_v          # 相对速度（<0=正在接近）

        # u* = -(k1 e + k2 vr) 为期望 ego 加速度（+加速）
        a_des = -(self._k1 * e + self._k2 * vr)
        # 前车加速度前馈（限幅，复用 ACC_FF_MAX 量纲）
        a_ff = _clamp(MPC_LEAD_FF_GAIN * lead_accel, -ACC_FF_MAX, ACC_FF_MAX)
        a_cmd = a_des + a_ff

        lon_cmd = -a_cmd                  # 转成"正=减速"
        lon_cmd = _clamp(lon_cmd,
                         -LON_CMD_MAX_DRIVE_ACCEL, LON_CMD_MAX_BRAKE_DECEL)

        # 弯道禁止加速（与 PID 版一致）
        if curv > CURV_NO_ACCEL_THRESH and lon_cmd < 0.0:
            lon_cmd = 0.0

        # 安全距离硬投影：进入最小安全距离内只能减速，越深越重
        if dist < min_safe_dist:
            deficit = (min_safe_dist - dist) / max(min_safe_dist, 1.0)
            floor = _clamp(1.0 + 5.0 * deficit, 1.0,
                           LON_CMD_MAX_BRAKE_DECEL)
            lon_cmd = max(lon_cmd, floor)
        return lon_cmd


class FallbackLonController:
    """主控制器外壳：异常/超预算时一次性永久回退到备份控制器。

    保证即便 MPC 出问题，纵向链路仍由经过验证的 PID 兜底，绝不丢控制。
    """

    def __init__(self, primary, backup, budget_s=None, probe_calls=200):
        self._primary = primary
        self._backup = backup
        self._failed = False
        self._budget = (CONTROL_LOOP_BUDGET_S * 0.5
                        if budget_s is None else budget_s)
        self._probe_left = probe_calls

    def _fallback(self, reason):
        if not self._failed:
            self._failed = True
            logging.error('[MPC] permanent fallback to PID: %s', reason)

    def compute(self, **kw):
        if self._failed:
            return self._backup.compute(**kw)
        try:
            if self._probe_left > 0:
                t0 = time.perf_counter()
                out = self._primary.compute(**kw)
                if (time.perf_counter() - t0) > self._budget:
                    self._fallback('compute over budget')
                    return self._backup.compute(**kw)
                self._probe_left -= 1
                return out
            return self._primary.compute(**kw)
        except Exception as e:
            self._fallback('exception %r' % (e,))
            try:
                return self._backup.compute(**kw)
            except Exception:
                # 备份也异常：返回安全减速，绝不抛回控制环
                return 2.0

    def reset(self):
        self._primary.reset()
        self._backup.reset()

    @property
    def i_term(self):
        return (self._backup if self._failed else self._primary).i_term

    def set_gains(self, **kw):
        # 转发给备份（PID），MPC 侧自行忽略
        self._backup.set_gains(**kw)
        self._primary.set_gains(**kw)


def make_lon_controller(dt):
    """按 config.LON_CONTROLLER 选纵向控制器。

    'pid'（默认）→ 原 LongitudinalController，与改造前逐字节一致。
    'mpc'        → MPC 主控 + PID 备份的自动回退外壳。
    """
    from config import LON_CONTROLLER
    from longitudinal import LongitudinalController
    if str(LON_CONTROLLER).lower() == 'mpc':
        return FallbackLonController(
            primary=MpcLongitudinalController(dt),
            backup=LongitudinalController(dt),
        )
    return LongitudinalController(dt)
