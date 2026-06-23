#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Optional model-based lateral path tracking controllers.

The default lateral path remains the proven PID path in
``control.lateral_controller``.  This module provides a small Stanley
drop-in that only runs when ``config.LAT_CONTROLLER='stanley'`` and returns
``None`` on any problem so the caller can fall back to PID for that cycle.
"""

import logging
import math

from common import clamp, wrap_angle
from config import (
    K_FF_CURV,
    LAT_CONTROLLER,
    MAX_DELTA,
    MAX_FF_DELTA,
    STANLEY_CTE_MAX,
    STANLEY_HEADING_GAIN,
    STANLEY_K_CTE,
    STANLEY_SOFTENING_V,
    STEER_SIGN,
    WHEEL_BASE,
)


def _finite(v):
    return (not math.isinf(v)) and (not math.isnan(v))


class StanleyLateralController:
    """Stateless Stanley-style lateral controller.

    Output sign follows the rest of this codebase: positive CTE means ego is
    left of the lane center, so the corrective steering term is negative.
    """

    def compute(self, psi_err, cte_err, ego_v, filtered_curv, delta_ff):
        psi_err = float(psi_err)
        cte_err = clamp(float(cte_err), -STANLEY_CTE_MAX, STANLEY_CTE_MAX)
        ego_v = abs(float(ego_v))
        filtered_curv = float(filtered_curv)
        if not (_finite(psi_err) and _finite(cte_err)
                and _finite(ego_v) and _finite(filtered_curv)):
            return None

        heading_term = STANLEY_HEADING_GAIN * psi_err
        cte_term = -math.atan2(
            STANLEY_K_CTE * cte_err,
            ego_v + max(0.1, STANLEY_SOFTENING_V),
        )
        ff = delta_ff
        if not _finite(ff):
            ff = clamp(
                K_FF_CURV * math.atan(WHEEL_BASE * filtered_curv),
                -MAX_FF_DELTA,
                MAX_FF_DELTA,
            )
        return clamp(STEER_SIGN * (heading_term + cte_term + ff),
                     -MAX_DELTA, MAX_DELTA)


class FallbackLateralController:
    """Small safety wrapper; one exception permanently disables the model path."""

    def __init__(self, primary):
        self._primary = primary
        self._failed = False

    def compute(self, **kw):
        if self._failed:
            return None
        try:
            return self._primary.compute(**kw)
        except Exception as e:  # noqa: BLE001 - control-loop fallback
            self._failed = True
            logging.error('[LAT_MODEL] permanent fallback to PID: %r', e)
            return None


def make_lateral_model_controller():
    """Return an optional model-based lateral controller."""
    if str(LAT_CONTROLLER).lower() == 'stanley':
        logging.info('[LAT_MODEL] Stanley lateral controller enabled')
        return FallbackLateralController(StanleyLateralController())
    return None
