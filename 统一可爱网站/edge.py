#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Compatibility wrapper：转发到本地自包含的 edge_engine（原「主控」包已不在仓库）。"""

import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from edge_engine import EdgeEngine  # noqa: E402,F401

__all__ = ['EdgeEngine']
