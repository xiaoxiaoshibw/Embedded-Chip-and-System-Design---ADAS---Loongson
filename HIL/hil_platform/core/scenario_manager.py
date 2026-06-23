# -*- coding: utf-8 -*-
"""场景管理：从 configs/*.yaml 加载场景定义 + 默认参数。

为保持与项目其它模块一致的"零 pip 依赖"风格，YAML 解析优先用 PyYAML，
缺失时回退到内置的极简解析器（仅支持本目录配置用到的两层 key: value 结构）。
"""

from __future__ import annotations

import os
from typing import Any, Dict, List, Optional

CONFIG_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "configs")
)

# 已知场景及其展示名（与 5 个比赛场景对应）
SCENARIO_TITLES = {
    "acc_follow": "ACC 自适应巡航跟车",
    "aeb_brake": "AEB 自动紧急制动",
    "lka_curve": "LKA 车道保持（弯道）",
    "cut_in": "Cut-in 切入",
    "takeover": "主控故障接管",
}


def _coerce(value: str) -> Any:
    """把 YAML 标量字符串转成 bool/int/float/str。"""
    v = value.strip()
    if v == "" or v.lower() in ("null", "none", "~"):
        return None
    if v.lower() in ("true", "yes"):
        return True
    if v.lower() in ("false", "no"):
        return False
    if (v[0] == '"' and v[-1] == '"') or (v[0] == "'" and v[-1] == "'"):
        return v[1:-1]
    try:
        return int(v)
    except ValueError:
        pass
    try:
        return float(v)
    except ValueError:
        pass
    return v


def _parse_simple_yaml(text: str) -> Dict[str, Any]:
    """极简 YAML 解析：支持 # 注释、key: value、两空格缩进的一层嵌套字典。"""
    root: Dict[str, Any] = {}
    cur: Dict[str, Any] = root
    for raw in text.splitlines():
        line = raw.rstrip()
        if not line.strip() or line.strip().startswith("#"):
            continue
        # 去掉行内注释（简单处理，不在引号内）
        if "#" in line and '"' not in line and "'" not in line:
            line = line.split("#", 1)[0].rstrip()
        indent = len(line) - len(line.lstrip())
        key, _, val = line.strip().partition(":")
        key = key.strip()
        val = val.strip()
        if indent == 0:
            if val == "":
                cur = {}
                root[key] = cur
            else:
                root[key] = _coerce(val)
                cur = root
        else:
            # 嵌套项写入最近一次创建的子字典
            if isinstance(root.get(_last_top_key(root)), dict):
                root[_last_top_key(root)][key] = _coerce(val)
    return root


def _last_top_key(d: Dict[str, Any]) -> Optional[str]:
    keys = list(d.keys())
    return keys[-1] if keys else None


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as fh:
        text = fh.read()
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(text)
        return data or {}
    except ImportError:
        return _parse_simple_yaml(text)


class Scenario:
    """一个已加载的场景：名字 + 地图 + 默认参数。"""

    def __init__(self, name: str, data: Dict[str, Any]):
        self.name = name
        self.title = data.get("title") or SCENARIO_TITLES.get(name, name)
        self.map = data.get("map", "Town04")
        self.description = data.get("description", "")
        # 默认参数集中在 params 字段
        self.default_params: Dict[str, Any] = dict(data.get("params", {}))

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "title": self.title,
            "map": self.map,
            "description": self.description,
            "default_params": self.default_params,
        }


class ScenarioManager:
    """负责发现/加载 configs 下的场景。"""

    def __init__(self, config_dir: str = CONFIG_DIR):
        self.config_dir = config_dir

    def list_scenarios(self) -> List[str]:
        if not os.path.isdir(self.config_dir):
            return list(SCENARIO_TITLES.keys())
        names = []
        for fn in sorted(os.listdir(self.config_dir)):
            if fn.endswith((".yaml", ".yml")):
                names.append(os.path.splitext(fn)[0])
        return names

    def load(self, name: str) -> Scenario:
        path = None
        for ext in (".yaml", ".yml"):
            cand = os.path.join(self.config_dir, name + ext)
            if os.path.isfile(cand):
                path = cand
                break
        if path is None:
            raise FileNotFoundError("未找到场景配置：%s" % name)
        data = load_yaml(path)
        return Scenario(name, data)
