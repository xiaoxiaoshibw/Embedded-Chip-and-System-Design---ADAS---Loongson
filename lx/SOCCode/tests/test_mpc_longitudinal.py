#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""MPC 纵向控制器单元测试。

覆盖：
  - _design_lqr_gain 纯函数
  - MpcLongitudinalController.compute 正常/安全距离/弯道
  - FallbackLonController 回退逻辑
"""

import math
import sys
from unittest import mock

import pytest

# 需要先 mock 掉 config 中的常量，再导入被测模块
# （mpc_longitudinal 在 import 时就从 config 拉取常量并计算 LQR 增益）
_MPC_CONFIG_PATCHES = {
    'ACC_D0': 2.5,
    'ACC_FF_MAX': 0.6,
    'ACC_TIME_GAP': 2.0,
    'CONTROL_LOOP_BUDGET_S': 0.008,
    'CURV_NO_ACCEL_THRESH': 0.008,
    'LON_CMD_MAX_BRAKE_DECEL': 6.0,
    'LON_CMD_MAX_DRIVE_ACCEL': 6.0,
    'MPC_LEAD_FF_GAIN': 1.0,
    'MPC_Q_E': 1.0,
    'MPC_Q_V': 2.5,
    'MPC_R': 12.0,
    'MPC_RICCATI_ITERS': 200,
    'MPC_TS': 0.10,
}


def _import_mpc_module():
    """在 mock 环境下导入被测模块，避免依赖 config.py 全局副作用。"""
    with mock.patch.dict('sys.modules', {}):
        # 确保 SOCCode 在 sys.path 中
        import os
        soccode = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if soccode not in sys.path:
            sys.path.insert(0, soccode)
        # mock config 模块
        cfg = mock.MagicMock()
        for k, v in _MPC_CONFIG_PATCHES.items():
            setattr(cfg, k, v)
        sys.modules['config'] = cfg
        sys.modules['common'] = mock.MagicMock()
        sys.modules['runtime'] = mock.MagicMock()

        import importlib
        if 'control.mpc_longitudinal' in sys.modules:
            importlib.reload(sys.modules['control.mpc_longitudinal'])
        else:
            # 确保 control 子包可导入
            if 'control' not in sys.modules:
                sys.modules['control'] = mock.MagicMock()
                sys.modules['control'].__path__ = []
            import control.mpc_longitudinal as mod
            return mod


# ── 为避免反复 mock，直接在模块加载时 patch config ──
# 使用 conftest 已加入的 sys.path
import os
_soccode = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _soccode not in sys.path:
    sys.path.insert(0, _soccode)

# 直接 mock config 模块中的常量（通过 patch 目标模块的命名空间）
# mpc_longitudinal.py 内部使用 from config import ... 所以需要 patch 它的模块属性


class TestDesignLqrGain:
    """_design_lqr_gain 纯函数测试。"""

    @pytest.fixture(autouse=True)
    def _setup(self):
        # 延迟导入，配合 conftest 的 sys.path
        from control import mpc_longitudinal as mod
        self._fn = mod._design_lqr_gain

    def test_returns_finite_tuple(self):
        """正常参数返回两个有限浮点数。"""
        k1, k2 = self._fn(0.10, 1.0, 2.5, 12.0, 200)
        assert math.isfinite(k1)
        assert math.isfinite(k2)

    def test_nonzero_gains(self):
        """正定 Q、R 应产出非零增益。"""
        k1, k2 = self._fn(0.10, 1.0, 2.5, 12.0, 200)
        assert abs(k1) > 1e-6
        assert abs(k2) > 1e-6

    def test_convergence_many_iters(self):
        """迭代 200 次与 500 次结果几乎相同（Riccati 收敛）。"""
        k1_a, k2_a = self._fn(0.10, 1.0, 2.5, 12.0, 200)
        k1_b, k2_b = self._fn(0.10, 1.0, 2.5, 12.0, 500)
        assert abs(k1_a - k1_b) < 1e-6
        assert abs(k2_a - k2_b) < 1e-6

    def test_iters_zero_returns_computed(self):
        """iters=0 时循环仍执行一次（max(1,0)=1），返回计算后的增益。

        注意：代码中 for _ in range(max(1, iters)) 保证至少迭代一次，
        所以 iters=0 不会返回初始的 (0,0)，而是首次迭代结果。
        """
        k1, k2 = self._fn(0.10, 1.0, 2.5, 12.0, 0)
        # 首次迭代后 k1, k2 应已更新为非零值
        assert math.isfinite(k1)
        assert math.isfinite(k2)

    def test_iters_one_single_step(self):
        """iters=1 与 iters=0 结果相同（都只迭代一次）。"""
        k1_a, k2_a = self._fn(0.10, 1.0, 2.5, 12.0, 0)
        k1_b, k2_b = self._fn(0.10, 1.0, 2.5, 12.0, 1)
        assert abs(k1_a - k1_b) < 1e-12
        assert abs(k2_a - k2_b) < 1e-12

    def test_small_ts(self):
        """极小时间步长不会导致数值异常。"""
        k1, k2 = self._fn(0.001, 1.0, 2.5, 12.0, 100)
        assert math.isfinite(k1)
        assert math.isfinite(k2)

    def test_large_q_e(self):
        """加大距离误差权重 → |k1| 应增大（增益对误差更敏感）。"""
        k1_low, _ = self._fn(0.10, 0.1, 2.5, 12.0, 200)
        k1_high, _ = self._fn(0.10, 10.0, 2.5, 12.0, 200)
        assert abs(k1_high) > abs(k1_low)

    def test_denom_guard(self):
        """R 极小时 denom 保护逻辑不崩溃。"""
        k1, k2 = self._fn(0.10, 1.0, 2.5, 1e-15, 50)
        assert math.isfinite(k1)
        assert math.isfinite(k2)


class TestMpcLongitudinalController:
    """MpcLongitudinalController.compute 测试。"""

    @pytest.fixture(autouse=True)
    def _setup(self):
        from control import mpc_longitudinal as mod
        # 构造时会调用 _design_lqr_gain，需要 config 常量已就绪
        self._cls = mod.MpcLongitudinalController
        self._mod = mod

    def _make_ctrl(self, dt=0.01):
        return self._cls(dt)

    def test_normal_following_returns_finite(self):
        """正常跟车场景：返回有限纵向指令。"""
        ctrl = self._make_ctrl()
        # dist=20m, ego_v=15, lead_v=14, accel=0, safe=10, curv=0
        cmd = ctrl.compute(20.0, 15.0, 14.0, 0.0, 10.0, 0.0)
        assert math.isfinite(cmd)

    def test_normal_following_positive_cmd(self):
        """前车更慢 + 距离不足 → 正 lon_cmd（减速）。"""
        ctrl = self._make_ctrl()
        # ego 快于前车，距离刚够 → 应减速
        cmd = ctrl.compute(12.0, 20.0, 10.0, 0.0, 10.0, 0.0)
        assert cmd > 0.0

    def test_far_distance_negative_cmd(self):
        """距离远 + 自车慢于前车 → 负 lon_cmd（加速）。"""
        ctrl = self._make_ctrl()
        cmd = ctrl.compute(80.0, 10.0, 20.0, 0.0, 10.0, 0.0)
        assert cmd < 0.0

    def test_hard_projection_below_min_safe(self):
        """dist < min_safe_dist → 安全距离硬投影强制减速。"""
        ctrl = self._make_ctrl()
        # dist=3m < min_safe=10m → 应触发硬投影
        cmd = ctrl.compute(3.0, 15.0, 15.0, 0.0, 10.0, 0.0)
        # 硬投影 floor = clamp(1 + 5*deficit, 1, 6) 其中 deficit=(10-3)/10=0.7
        # floor = clamp(1+3.5, 1, 6) = 4.5
        assert cmd >= 4.5 - 0.01

    def test_hard_projection_deeper_penalty(self):
        """更近的距离 → 更大的硬投影制动力。"""
        ctrl = self._make_ctrl()
        cmd_shallow = ctrl.compute(8.0, 15.0, 15.0, 0.0, 10.0, 0.0)
        cmd_deep = ctrl.compute(3.0, 15.0, 15.0, 0.0, 10.0, 0.0)
        assert cmd_deep >= cmd_shallow

    def test_curve_suppression_no_accel(self):
        """弯道曲率超阈值 → 禁止加速（负 lon_cmd 归零）。"""
        ctrl = self._make_ctrl()
        # 远距离 + 低速 + 高曲率 → 本应加速，但弯道应抑制
        cmd_curve = ctrl.compute(80.0, 5.0, 10.0, 0.0, 10.0, 0.05)
        assert cmd_curve >= 0.0

    def test_no_curve_acceleration_allowed(self):
        """直道（curv=0）允许加速。"""
        ctrl = self._make_ctrl()
        cmd = ctrl.compute(80.0, 5.0, 10.0, 0.0, 10.0, 0.0)
        assert cmd < 0.0

    def test_output_clamped_upper(self):
        """输出不超过 LON_CMD_MAX_BRAKE_DECEL。"""
        ctrl = self._make_ctrl()
        # 极近距离 + 高速 → 应被钳位
        cmd = ctrl.compute(1.0, 30.0, 0.0, 0.0, 10.0, 0.0)
        assert cmd <= 6.0 + 0.01

    def test_output_clamped_lower(self):
        """输出不低于 -LON_CMD_MAX_DRIVE_ACCEL。"""
        ctrl = self._make_ctrl()
        # 极远距离 + 极低速 + 前车高速 → 应被钳位
        cmd = ctrl.compute(200.0, 1.0, 30.0, 0.0, 10.0, 0.0)
        assert cmd >= -6.0 - 0.01

    def test_lead_accel_feedforward(self):
        """前车加速前馈 → lon_cmd 减小（更少减速/更多加速）。"""
        ctrl = self._make_ctrl()
        cmd_no_ff = ctrl.compute(20.0, 15.0, 14.0, 0.0, 10.0, 0.0)
        cmd_ff = ctrl.compute(20.0, 15.0, 14.0, 2.0, 10.0, 0.0)
        # 前车加速 → 应减少制动
        assert cmd_ff < cmd_no_ff

    def test_i_term_is_zero(self):
        """MPC 无积分项，i_term 恒为 0。"""
        ctrl = self._make_ctrl()
        assert ctrl.i_term == 0.0

    def test_reset_no_error(self):
        """reset 不抛异常。"""
        ctrl = self._make_ctrl()
        ctrl.reset()

    def test_set_gains_no_error(self):
        """set_gains 安全忽略。"""
        ctrl = self._make_ctrl()
        ctrl.set_gains(acc_kd=0.5)


class TestFallbackLonController:
    """FallbackLonController 回退逻辑测试。"""

    @pytest.fixture(autouse=True)
    def _setup(self):
        from control import mpc_longitudinal as mod
        self._cls = mod.FallbackLonController

    def _make_mock_ctrl(self, return_val=1.0, side_effect=None):
        """创建 mock 控制器。"""
        m = mock.MagicMock()
        m.compute.return_value = return_val
        if side_effect is not None:
            m.compute.side_effect = side_effect
        m.i_term = 0.0
        return m

    def test_primary_works_delegates(self):
        """主控制器正常 → 委托主控制器。"""
        primary = self._make_mock_ctrl(return_val=1.5)
        backup = self._make_mock_ctrl(return_val=2.0)
        fb = self._cls(primary, backup, budget_s=1.0, probe_calls=10)
        result = fb.compute(dist=20, ego_v=15, lead_v_proj=14,
                            lead_accel=0, min_safe_dist=10, curv=0)
        assert result == 1.5
        primary.compute.assert_called_once()

    def test_primary_throws_falls_back(self):
        """主控制器抛异常 → 回退到备份。"""
        primary = self._make_mock_ctrl(side_effect=ValueError("boom"))
        backup = self._make_mock_ctrl(return_val=2.5)
        fb = self._cls(primary, backup, budget_s=1.0, probe_calls=10)
        result = fb.compute(dist=20, ego_v=15, lead_v_proj=14,
                            lead_accel=0, min_safe_dist=10, curv=0)
        assert result == 2.5

    def test_both_throw_returns_safe_decel(self):
        """主备都抛异常 → 返回安全减速值 2.0。"""
        primary = self._make_mock_ctrl(side_effect=ValueError("p"))
        backup = self._make_mock_ctrl(side_effect=ValueError("b"))
        fb = self._cls(primary, backup, budget_s=1.0, probe_calls=10)
        result = fb.compute(dist=20, ego_v=15, lead_v_proj=14,
                            lead_accel=0, min_safe_dist=10, curv=0)
        assert result == 2.0

    def test_permanent_fallback_after_exception(self):
        """主控制器异常后永久回退（不再尝试主控制器）。"""
        primary = self._make_mock_ctrl(side_effect=ValueError("boom"))
        backup = self._make_mock_ctrl(return_val=3.0)
        fb = self._cls(primary, backup, budget_s=1.0, probe_calls=10)

        # 第一次：触发回退
        fb.compute(dist=20, ego_v=15, lead_v_proj=14,
                   lead_accel=0, min_safe_dist=10, curv=0)
        # 第二次：应直接走备份
        fb.compute(dist=20, ego_v=15, lead_v_proj=14,
                   lead_accel=0, min_safe_dist=10, curv=0)
        # 主控制器只被调用一次（第二次直接走备份）
        assert primary.compute.call_count == 1

    def test_i_term_delegates_to_backup_when_failed(self):
        """回退后 i_term 取备份的值。"""
        primary = self._make_mock_ctrl(side_effect=ValueError("boom"))
        backup = self._make_mock_ctrl(return_val=2.0)
        backup.i_term = 0.5
        fb = self._cls(primary, backup, budget_s=1.0, probe_calls=10)
        # 触发回退
        fb.compute(dist=20, ego_v=15, lead_v_proj=14,
                   lead_accel=0, min_safe_dist=10, curv=0)
        assert fb.i_term == 0.5

    def test_i_term_uses_primary_before_failure(self):
        """回退前 i_term 取主控制器的值。"""
        primary = self._make_mock_ctrl(return_val=1.0)
        primary.i_term = 0.3
        backup = self._make_mock_ctrl(return_val=2.0)
        backup.i_term = 0.7
        fb = self._cls(primary, backup, budget_s=1.0, probe_calls=10)
        assert fb.i_term == 0.3

    def test_reset_calls_both(self):
        """reset 调用主备两个控制器的 reset。"""
        primary = self._make_mock_ctrl()
        backup = self._make_mock_ctrl()
        fb = self._cls(primary, backup, budget_s=1.0, probe_calls=10)
        fb.reset()
        primary.reset.assert_called_once()
        backup.reset.assert_called_once()

    def test_set_gains_calls_both(self):
        """set_gains 转发给主备两个控制器。"""
        primary = self._make_mock_ctrl()
        backup = self._make_mock_ctrl()
        fb = self._cls(primary, backup, budget_s=1.0, probe_calls=10)
        fb.set_gains(acc_kd=0.5)
        primary.set_gains.assert_called_once()
        backup.set_gains.assert_called_once()

    def test_budget_exceeded_triggers_fallback(self):
        """主控制器超时 → 触发回退。"""

        def slow_compute(**kw):
            import time
            time.sleep(0.05)  # 远超 budget
            return 1.0

        primary = self._make_mock_ctrl()
        primary.compute.side_effect = slow_compute
        backup = self._make_mock_ctrl(return_val=2.5)
        fb = self._cls(primary, backup, budget_s=0.001, probe_calls=100)
        result = fb.compute(dist=20, ego_v=15, lead_v_proj=14,
                            lead_accel=0, min_safe_dist=10, curv=0)
        assert result == 2.5
