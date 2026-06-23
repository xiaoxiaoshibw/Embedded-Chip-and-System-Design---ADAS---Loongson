#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

from control.comfort import JerkComfortLayer
from control.lead_estimator import LeadCaKalman
from control.model_lateral import StanleyLateralController


class TestLeadCaKalman:
    def test_constant_speed_converges_to_small_accel(self):
        kf = LeadCaKalman()
        kf.reset(10.0)
        v = a = 0.0
        for _ in range(80):
            v, a = kf.update(10.0, 0.01)
        assert abs(v - 10.0) < 0.2
        assert abs(a) < 0.5

    def test_outlier_is_gated(self):
        kf = LeadCaKalman()
        kf.reset(10.0)
        kf.update(10.0, 0.01)
        v, _ = kf.update(0.0, 0.01)
        assert v > 5.0
        assert kf.last_outlier is True


class TestStanleyLateralController:
    def test_returns_finite_bounded_delta(self):
        ctrl = StanleyLateralController()
        delta = ctrl.compute(
            psi_err=0.1,
            cte_err=0.5,
            ego_v=8.0,
            filtered_curv=0.005,
            delta_ff=0.02,
        )
        assert math.isfinite(delta)
        assert abs(delta) <= math.radians(25)

    def test_positive_cte_steers_back_right(self):
        ctrl = StanleyLateralController()
        delta = ctrl.compute(
            psi_err=0.0,
            cte_err=1.0,
            ego_v=8.0,
            filtered_curv=0.0,
            delta_ff=0.0,
        )
        assert delta < 0.0


class TestJerkComfortLayer:
    def test_regular_target_is_jerk_limited(self):
        layer = JerkComfortLayer(dt=0.01)
        out = layer.update(target=2.0)
        assert 0.0 < out < 2.0

    def test_aeb_bypasses_comfort_limit(self):
        layer = JerkComfortLayer(dt=0.01)
        out = layer.update(target=5.0, aeb_active=True)
        assert out == 5.0
