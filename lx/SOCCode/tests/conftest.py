#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""pytest 配置 — SOCCode 测试套件。

确保 SOCCode/ 在 sys.path 中，使 'from config import *' 等导入正常工作。
提供常用 fixture（默认 ControlMemory、VehicleSignals 等）。
"""

import math
import os
import sys

# 确保 SOCCode/ 在 sys.path 中
_soccode_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _soccode_dir not in sys.path:
    sys.path.insert(0, _soccode_dir)

import pytest

from control.context import ControlMemory, VehicleSignals
from control.state import LeadContext, LeadTrackingInputs, LongitudinalContext


@pytest.fixture
def dt():
    """默认控制周期 0.01s (100Hz)。"""
    return 0.01


@pytest.fixture
def memory(dt):
    """默认 ControlMemory，所有字段取默认值。"""
    return ControlMemory(dt=dt)


@pytest.fixture
def signals():
    """默认 VehicleSignals，所有字段取零/False。"""
    return VehicleSignals()


@pytest.fixture
def lead_ctx():
    """默认 LeadContext。"""
    return LeadContext()


@pytest.fixture
def lon_ctx():
    """默认 LongitudinalContext。"""
    return LongitudinalContext()
