# -*- coding: utf-8 -*-
"""运行参数管理。

集中持有本次实验的可调参数，并支持运行中热更新（/api/parameters/update）。
所有参数都有显式默认值与类型/范围校验，避免前端传入脏数据带乱仿真。
"""

from __future__ import annotations

from typing import Any, Dict


# 参数注册表：name -> (默认值, 类型, 最小, 最大)。范围用于钳制，None 表示不限。
PARAM_REGISTRY: Dict[str, tuple] = {
    "ego_speed":               (50.0, float, 0.0, 200.0),   # 自车目标速度 km/h
    "front_distance":          (40.0, float, 0.0, 300.0),   # 初始前车距离 m
    "front_speed":             (35.0, float, 0.0, 200.0),   # 前车速度 km/h
    "cut_in_speed":            (40.0, float, 0.0, 200.0),   # 切入车速度 km/h
    "cut_in_trigger_distance": (25.0, float, 0.0, 300.0),   # 切入触发距离 m
    "weather":                 ("clear", str, None, None),  # clear/rain/fog/night
    "comm_delay_ms":           (0.0, float, 0.0, 2000.0),   # 通信延迟 ms
    "sensor_noise":            (0.0, float, 0.0, 1.0),      # 传感器噪声比例
    "fault_trigger_time":      (0.0, float, 0.0, 600.0),    # 故障注入时刻 s（0=不预设）
    "fault_type":              ("none", str, None, None),   # 预设故障类型
}

WEATHER_CHOICES = ("clear", "rain", "fog", "night")


class ParameterManager:
    def __init__(self):
        self._params: Dict[str, Any] = self.defaults()

    @staticmethod
    def defaults() -> Dict[str, Any]:
        return {k: v[0] for k, v in PARAM_REGISTRY.items()}

    @property
    def params(self) -> Dict[str, Any]:
        return dict(self._params)

    def reset(self) -> None:
        self._params = self.defaults()

    def _validate_one(self, key: str, value: Any) -> Any:
        if key not in PARAM_REGISTRY:
            raise KeyError("未知参数：%s" % key)
        _default, typ, lo, hi = PARAM_REGISTRY[key]
        if typ is float:
            try:
                v = float(value)
            except (TypeError, ValueError):
                raise ValueError("参数 %s 需为数值，收到 %r" % (key, value))
            if lo is not None:
                v = max(lo, v)
            if hi is not None:
                v = min(hi, v)
            return v
        if typ is str:
            v = str(value)
            if key == "weather" and v not in WEATHER_CHOICES:
                raise ValueError("weather 取值须为 %s" % (WEATHER_CHOICES,))
            return v
        return value

    def apply(self, updates: Dict[str, Any]) -> Dict[str, Any]:
        """批量校验后更新，返回更新后的全量参数。未知键会被忽略并记录。"""
        for k, v in updates.items():
            if v is None:
                continue
            if k not in PARAM_REGISTRY:
                continue  # 静默忽略未知键，避免前端多传字段时报错
            self._params[k] = self._validate_one(k, v)
        return self.params

    def load_scenario_defaults(self, scenario_params: Dict[str, Any]) -> Dict[str, Any]:
        """加载场景时：先重置为全局默认，再叠加场景默认。"""
        self.reset()
        self.apply(scenario_params)
        return self.params
