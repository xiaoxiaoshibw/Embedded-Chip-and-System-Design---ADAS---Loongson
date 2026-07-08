# -*- coding: utf-8 -*-
"""★5 planning→control 接口（trajectory）回归测试。"""

from control.trajectory import summarize_target, TargetPath, SpeedProfile


def test_summarize_target():
    path, speed = summarize_target(0.5, 0.01, 13.9)
    assert isinstance(path, TargetPath) and isinstance(speed, SpeedProfile)
    assert path.lateral_offset_target == 0.5
    assert path.curvature == 0.01
    assert path.source == "reactive"
    assert speed.target_speed == 13.9


def test_defaults_zero():
    assert TargetPath().lateral_offset_target == 0.0
    assert SpeedProfile().target_speed == 0.0
