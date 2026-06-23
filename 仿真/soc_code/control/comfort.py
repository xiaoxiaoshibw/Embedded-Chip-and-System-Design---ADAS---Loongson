#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Optional longitudinal comfort shaping.

This is intentionally placed before the existing ``LonSmoothing`` stage.  It
does not replace the safety limiter; it only shapes non-AEB/non-boundary
targets when explicitly enabled.
"""

import logging
import math

from common import clamp
from config import (
    COMFORT_JERK_ACCEL,
    COMFORT_JERK_DECEL,
    COMFORT_JERK_RELEASE,
    COMFORT_LAYER,
    LON_CMD_MAX_BRAKE_DECEL,
    LON_CMD_MAX_DRIVE_ACCEL,
)


def _finite(v):
    return (not math.isinf(v)) and (not math.isnan(v))


class JerkComfortLayer:
    """Jerk-bounded target shaper for regular ACC/cruise commands."""

    def __init__(self, dt):
        self._dt = max(1e-3, float(dt))
        self._prev = 0.0

    def reset(self, value=0.0):
        self._prev = float(value) if _finite(float(value)) else 0.0

    def update(self, target, aeb_active=False, boundary_brake=False):
        target = float(target)
        if not _finite(target):
            return self._prev
        if aeb_active or boundary_brake:
            self._prev = clamp(
                target, -LON_CMD_MAX_DRIVE_ACCEL, LON_CMD_MAX_BRAKE_DECEL)
            return self._prev

        if target > self._prev:
            jerk = COMFORT_JERK_DECEL
        elif self._prev > 0.5 and target < self._prev:
            jerk = COMFORT_JERK_RELEASE
        else:
            jerk = COMFORT_JERK_ACCEL
        max_step = max(0.0, float(jerk)) * self._dt
        self._prev = clamp(
            target,
            self._prev - max_step,
            self._prev + max_step,
        )
        return self._prev


class FallbackComfortLayer:
    """Safety wrapper; failures permanently bypass comfort shaping."""

    def __init__(self, primary):
        self._primary = primary
        self._failed = False

    def reset(self, value=0.0):
        if not self._failed:
            try:
                self._primary.reset(value)
            except Exception as e:  # noqa: BLE001
                self._failed = True
                logging.error('[COMFORT] permanent bypass after reset: %r', e)

    def update(self, **kw):
        if self._failed:
            return kw.get('target', 0.0)
        try:
            return self._primary.update(**kw)
        except Exception as e:  # noqa: BLE001
            self._failed = True
            logging.error('[COMFORT] permanent bypass after update: %r', e)
            return kw.get('target', 0.0)


def make_comfort_layer(dt):
    """Return an optional comfort layer."""
    if str(COMFORT_LAYER).lower() == 'jerk':
        logging.info('[COMFORT] jerk-bounded target shaper enabled')
        return FallbackComfortLayer(JerkComfortLayer(dt))
    return None
