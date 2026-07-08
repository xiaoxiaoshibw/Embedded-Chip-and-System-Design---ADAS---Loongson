# -*- coding: utf-8 -*-
"""启动期安全配置校验回归测试。"""

from control.config_check import validate_config, log_config_issues


def test_real_config_has_no_issues():
    """真实 config 的所有安全关键参数应在合理范围内（零误报，否则启动会刷告警）。"""
    issues = validate_config()
    assert issues == [], "真实 config 出现安全配置告警: %r" % (issues,)


def test_log_returns_zero_on_clean():
    assert log_config_issues() == 0


def test_out_of_range_detected(monkeypatch):
    import config
    monkeypatch.setattr(config, "MAX_DELTA", 99.0)
    names = [n for _, n, _ in validate_config()]
    assert "MAX_DELTA" in names


def test_nan_detected(monkeypatch):
    import config
    monkeypatch.setattr(config, "LON_CMD_MAX_BRAKE_DECEL", float("nan"))
    names = [n for _, n, _ in validate_config()]
    assert "LON_CMD_MAX_BRAKE_DECEL" in names


def test_ttc_ordering_detected(monkeypatch):
    import config
    monkeypatch.setattr(config, "TTC_BRAKE_FULL", 20.0)  # > TTC_BRAKE_START(15)
    names = [n for _, n, _ in validate_config()]
    assert "TTC_ORDER" in names
