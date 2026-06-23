# -*- coding: utf-8 -*-
"""实验记录器：统一负责写入 runs/<run_id>/ 下的所有文件。

stop 时落盘：
    meta.json      run 元信息 + config 快照
    states.csv     逐帧状态（字段顺序见 STATE_FIELDS）
    events.json    事件序列（含派生事件）
    summary.json   实验摘要
    report.md      可读报告（答辩用）
    screenshots/   预留（真实 CARLA 接入后存关键帧截图）
    curves/        预留（离线绘图输出）

历史回放只读 runs/ 目录，不触碰 CARLA。
"""

from __future__ import annotations

import csv
import json
import os
import threading
from datetime import datetime
from typing import Any, Dict, List, Optional

from .types import StateFrame

# states.csv 列顺序（严格按需求）
STATE_FIELDS = [
    "t", "ego_speed", "front_distance", "relative_speed", "ttc",
    "lateral_error", "heading_error", "throttle", "brake", "steer",
    "nano_a_alive", "nano_a_seq", "nano_a_latency_ms", "nano_a_valid_output",
    "nano_b_alive", "nano_b_seq", "nano_b_latency_ms", "nano_b_valid_output",
    "active_controller", "takeover_count", "safe_brake", "event",
]

RUNS_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs")
)


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def generate_run_id(scenario: str, runs_dir: str = RUNS_DIR) -> str:
    """生成 run_id：YYYYMMDD_NNN_scenario，NNN 为当日序号。"""
    _ensure_dir(runs_dir)
    today = datetime.now().strftime("%Y%m%d")
    n = 0
    for name in os.listdir(runs_dir):
        if name.startswith(today + "_"):
            n += 1
    return "%s_%03d_%s" % (today, n + 1, scenario)


class Recorder:
    """缓冲帧/事件，stop 时一次性落盘。"""

    def __init__(self, runs_dir: str = RUNS_DIR):
        self.runs_dir = runs_dir
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self.run_id: Optional[str] = None
            self.scenario: str = ""
            self.map: str = "Town04"
            self.start_time: Optional[datetime] = None
            self.config: Dict[str, Any] = {}
            self._rows: List[Dict[str, Any]] = []
            self._events: List[Dict[str, Any]] = []

    def begin(self, run_id: str, scenario: str, map_name: str,
              config: Dict[str, Any]) -> None:
        with self._lock:
            self.run_id = run_id
            self.scenario = scenario
            self.map = map_name
            self.config = dict(config)
            self.start_time = datetime.now()
            self._rows = []
            self._events = []

    def append_frame(self, frame: StateFrame) -> None:
        with self._lock:
            self._rows.append(frame.to_csv_row())

    def append_event(self, event: Dict[str, Any]) -> None:
        with self._lock:
            self._events.append(event)

    @property
    def frame_count(self) -> int:
        with self._lock:
            return len(self._rows)

    def _run_dir(self) -> str:
        assert self.run_id is not None
        return os.path.join(self.runs_dir, self.run_id)

    def finalize(self, summary: Dict[str, Any],
                 derived_events: List[Dict[str, Any]]) -> Dict[str, Any]:
        """落盘所有文件，返回 meta dict。"""
        with self._lock:
            if self.run_id is None:
                raise RuntimeError("recorder 未 begin，无法 finalize")
            run_dir = self._run_dir()
            _ensure_dir(run_dir)
            _ensure_dir(os.path.join(run_dir, "screenshots"))
            _ensure_dir(os.path.join(run_dir, "curves"))

            duration = round(float(self._rows[-1]["t"]), 2) if self._rows else 0.0

            # events：注入/接管等实时事件 + 派生事件，按时间排序
            all_events = list(self._events) + list(derived_events)
            all_events.sort(key=lambda e: e.get("time", 0.0))

            meta = {
                "run_id": self.run_id,
                "scenario": self.scenario,
                "map": self.map,
                "start_time": (self.start_time or datetime.now()).strftime("%Y-%m-%d %H:%M:%S"),
                "duration": duration,
                "config": self.config,
            }

            # meta.json
            self._write_json(os.path.join(run_dir, "meta.json"), meta)
            # states.csv
            self._write_csv(os.path.join(run_dir, "states.csv"), self._rows)
            # events.json
            self._write_json(os.path.join(run_dir, "events.json"), all_events)
            # summary.json
            self._write_json(os.path.join(run_dir, "summary.json"), summary)
            # report.md
            self._write_report(os.path.join(run_dir, "report.md"), meta, summary, all_events)

            return meta

    # ── 写文件 ──
    @staticmethod
    def _write_json(path: str, data: Any) -> None:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(data, fh, ensure_ascii=False, indent=2)

    @staticmethod
    def _write_csv(path: str, rows: List[Dict[str, Any]]) -> None:
        with open(path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=STATE_FIELDS)
            writer.writeheader()
            for r in rows:
                writer.writerow(r)

    @staticmethod
    def _write_report(path: str, meta: Dict[str, Any], summary: Dict[str, Any],
                      events: List[Dict[str, Any]]) -> None:
        cfg = meta.get("config", {})
        lines: List[str] = []
        lines.append("# HIL 实验报告 — %s" % meta["run_id"])
        lines.append("")
        lines.append("## 基本信息")
        lines.append("")
        lines.append("| 项 | 值 |")
        lines.append("|---|---|")
        lines.append("| 场景 | %s |" % meta["scenario"])
        lines.append("| 地图 | %s |" % meta["map"])
        lines.append("| 开始时间 | %s |" % meta["start_time"])
        lines.append("| 时长 | %.2f s |" % meta["duration"])
        lines.append("")
        lines.append("## 参数配置")
        lines.append("")
        lines.append("| 参数 | 值 |")
        lines.append("|---|---|")
        for k, v in cfg.items():
            lines.append("| %s | %s |" % (k, v))
        lines.append("")
        lines.append("## 实验结论")
        lines.append("")
        result = summary.get("result", "—")
        lines.append("- **结果**：%s" % result)
        lines.append("- 碰撞：%s" % ("是" if summary.get("collision") else "否"))
        lines.append("- 最小 TTC：%s s" % summary.get("min_ttc"))
        lines.append("- 最大横向误差：%s m" % summary.get("max_lateral_error"))
        lines.append("- 是否接管：%s" % ("是" if summary.get("takeover_happened") else "否"))
        lines.append("- 接管时延：%s ms" % summary.get("takeover_latency_ms"))
        lines.append("- 最终控制器：%s" % summary.get("active_controller_final"))
        lines.append("- 安全制动触发：%s" % ("是" if summary.get("safe_brake_triggered") else "否"))
        lines.append("")
        lines.append("> %s" % summary.get("conclusion", ""))
        lines.append("")
        lines.append("## 关键事件")
        lines.append("")
        if events:
            lines.append("| 时间(s) | 类型 | 详情 |")
            lines.append("|---|---|---|")
            for e in events:
                detail = {k: v for k, v in e.items() if k not in ("time", "type")}
                lines.append("| %s | %s | %s |" % (
                    e.get("time"), e.get("type"),
                    ", ".join("%s=%s" % (k, v) for k, v in detail.items())))
        else:
            lines.append("（无）")
        lines.append("")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines))
