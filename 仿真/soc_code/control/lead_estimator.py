#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""前车纵向状态卡尔曼估计器（恒加速度 CA 模型，drop-in）。

工程动机：
  原 ACC/AEB 链路里前车加速度由"投影速度的有限差分 + 一阶低通"得到
  （longitudinal_policy.py 旧逻辑），并辅以若干手写的速度跳变检测
  （LEAD_DROP_GLITCH_*）。有限差分天生噪声大、相位滞后，且低通越平滑滞后越大，
  这是 ACC 前馈与 AEB 触发时机的质量上限——也是与量产毫米波/视觉栈差距最大的一环：
  量产栈对每个目标跑卡尔曼滤波做状态估计。

本模块实现一个 2 状态恒加速度卡尔曼滤波器，对"前车投影到自车纵向的速度"
做最优估计，同时输出平滑、低滞后的加速度，并用 Mahalanobis 新息门控
做单帧野值剔除（替代手写 glitch 检测）：
  状态 x = [v, a]（投影速度、投影加速度）
  过程模型 F = [[1, dt], [0, 1]]，连续白噪声 jerk 过程噪声 Q
  量测 z = v（H = [1, 0]），量测方差 R
全部为 2x2 标量展开，纯 Python、O(1)、微秒级，无 numpy 依赖，Python 3.6 兼容。

接口（供 longitudinal_policy 在 ACC 分支消费）：
  reset(v0)                          前车重获/航迹切换时，以首帧测量初始化
  update(z, dt) -> (v_est, a_est)    单步预测+校正，返回平滑速度与加速度
  属性 velocity / accel / failed / last_outlier

config.LEAD_ESTIMATOR='legacy'（默认）时本类根本不被实例化，链路逐字节一致。
异常自管理：update() 内部 try/except，任何异常一次性记日志并永久降级为
"直通测量速度 + 零加速度"的安全输出，绝不把异常抛回 100Hz 控制环。
"""

import logging
import math

from config import (
    LEAD_ACCEL_MAX,
    LEAD_KF_GATE_SIGMA,
    LEAD_KF_INIT_A_VAR,
    LEAD_KF_INIT_V_VAR,
    LEAD_KF_JERK_PSD,
    LEAD_KF_MAX_CONSEC_OUTLIERS,
    LEAD_KF_MEAS_VAR,
)


def _finite(v):
    return (not math.isinf(v)) and (not math.isnan(v))


class LeadCaKalman:
    """恒加速度（CA）前车纵向速度/加速度卡尔曼估计器。"""

    def __init__(self):
        self._jerk_psd = max(1e-6, float(LEAD_KF_JERK_PSD))
        self._meas_var = max(1e-6, float(LEAD_KF_MEAS_VAR))
        self._gate2 = float(LEAD_KF_GATE_SIGMA) ** 2
        self._max_consec_outliers = max(0, int(LEAD_KF_MAX_CONSEC_OUTLIERS))
        self._failed = False
        self._primed = False
        self._last_outlier = False
        self._consec_outliers = 0
        # 状态与协方差（2x2 对称，存 p00/p01/p11）
        self._v = 0.0
        self._a = 0.0
        self._p00 = LEAD_KF_INIT_V_VAR
        self._p01 = 0.0
        self._p11 = LEAD_KF_INIT_A_VAR

    # ── 公共接口 ──
    def reset(self, v0=0.0):
        """前车重获/主前车切换时以首帧测量初始化，加速度先验置零。

        初始协方差用较大的先验方差，让前几拍快速收敛到测量，
        避免从 0 低通爬升造成的滞后（与旧 priming 分支同义，但更原则化）。
        """
        v0 = float(v0) if _finite(float(v0)) else 0.0
        self._v = max(0.0, v0)
        self._a = 0.0
        self._p00 = LEAD_KF_INIT_V_VAR
        self._p01 = 0.0
        self._p11 = LEAD_KF_INIT_A_VAR
        self._primed = True
        self._last_outlier = False
        self._consec_outliers = 0
        # reset 不清 _failed：一旦永久降级，保持降级直至进程重启（与 MPC 回退一致）。

    def update(self, z, dt):
        """单步预测 + 校正。返回 (v_est, a_est)。

        z:  本拍前车投影速度测量 (m/s)
        dt: 距上次更新的时间 (s)，内部对非法值做兜底
        失败降级：返回 (max(0,z), 0.0)。
        """
        if self._failed:
            zf = float(z) if _finite(float(z)) else self._v
            return max(0.0, zf), 0.0
        try:
            return self._update_impl(z, dt)
        except Exception as e:  # noqa: BLE001 — 控制环绝不抛异常
            self._failed = True
            logging.error('[LEAD_KF] permanent fallback (pass-through): %r', e)
            zf = float(z) if _finite(float(z)) else self._v
            return max(0.0, zf), 0.0

    # ── 内部实现 ──
    def _update_impl(self, z, dt):
        z = float(z)
        dt = float(dt)
        if not _finite(z):
            # 量测无效：纯预测一步（coast），不校正
            self._predict(self._safe_dt(dt))
            return self._v, self._a
        if not self._primed:
            # 未初始化：直接以首帧测量定位（等价 reset）
            self.reset(z)
            return self._v, self._a

        dt = self._safe_dt(dt)
        self._predict(dt)

        # 新息与门控
        s = self._p00 + self._meas_var          # 新息协方差 S = H P H' + R
        if s <= 1e-9:
            s = 1e-9
        y = z - self._v                          # 新息
        d2 = (y * y) / s                         # Mahalanobis 距离平方
        outlier = d2 > self._gate2
        if outlier and self._consec_outliers < self._max_consec_outliers:
            # 单帧野值：跳过校正，仅保留预测（拒绝 glitch），但已加大协方差
            self._last_outlier = True
            self._consec_outliers += 1
            return self._v, self._a
        # 正常校正（或连续野值超限→接受为真实变化，重新锁定）
        self._last_outlier = False
        self._consec_outliers = 0
        k0 = self._p00 / s
        k1 = self._p01 / s
        self._v = self._v + k0 * y
        self._a = self._a + k1 * y
        # P+ = (I - K H) P-（标量量测短式），对称化 p01
        np00 = (1.0 - k0) * self._p00
        np01 = (1.0 - k0) * self._p01
        np11 = self._p11 - k1 * self._p01
        self._p00 = np00
        self._p01 = np01
        self._p11 = np11
        # 加速度估计限幅（与旧 LEAD_ACCEL_MAX 量纲一致，防极端测量推爆 FF）
        if self._a > LEAD_ACCEL_MAX:
            self._a = LEAD_ACCEL_MAX
        elif self._a < -LEAD_ACCEL_MAX:
            self._a = -LEAD_ACCEL_MAX
        if self._v < 0.0:
            self._v = 0.0
        return self._v, self._a

    def _predict(self, dt):
        """时间更新：x- = F x，P- = F P F' + Q（连续白噪声 jerk 过程噪声）。"""
        # 状态预测
        self._v = self._v + self._a * dt
        # 协方差预测 F P F'
        p00 = self._p00 + 2.0 * dt * self._p01 + dt * dt * self._p11
        p01 = self._p01 + dt * self._p11
        p11 = self._p11
        # 过程噪声 Q = q * [[dt^3/3, dt^2/2],[dt^2/2, dt]]
        q = self._jerk_psd
        dt2 = dt * dt
        dt3 = dt2 * dt
        self._p00 = p00 + q * dt3 / 3.0
        self._p01 = p01 + q * dt2 / 2.0
        self._p11 = p11 + q * dt

    @staticmethod
    def _safe_dt(dt):
        """把 dt 钳到合理区间，防除零/爆裂（与控制环 CTRL_DT 同量级）。"""
        if (not _finite(dt)) or dt <= 0.0:
            return 0.01
        if dt > 0.2:
            return 0.2
        return dt

    # ── 诊断属性 ──
    @property
    def velocity(self):
        return self._v

    @property
    def accel(self):
        return self._a

    @property
    def failed(self):
        return self._failed

    @property
    def last_outlier(self):
        return self._last_outlier


def make_lead_estimator():
    """按 config.LEAD_ESTIMATOR 选前车状态估计器。

    'legacy'（默认）→ 返回 None：longitudinal_policy 走原有限差分路径，
                       与改造前逐字节一致（回滚路径）。
    'kalman'        → 返回 LeadCaKalman 实例。
    """
    from config import LEAD_ESTIMATOR
    if str(LEAD_ESTIMATOR).lower() == 'kalman':
        est = LeadCaKalman()
        logging.info('[LEAD_KF] enabled: jerk_psd=%.3g meas_var=%.3g gate=%.1fσ',
                     est._jerk_psd, est._meas_var, math.sqrt(est._gate2))
        return est
    return None
