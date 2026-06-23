#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全链路综合对比仪表盘 — 场景测试 + ML 推理效果汇总。"""

import csv
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.ticker as mticker
import numpy as np

# ── 中文字体 ──
plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

# ── 配色 ──
C = {
    "bg": "#F8FAFC", "grid": "#E2E8F0",
    "pri": "#2563EB", "sec": "#7C3AED",
    "ok": "#10B981", "warn": "#F59E0B", "err": "#EF4444",
    "info": "#06B6D4", "gray": "#94A3B8",
}
PALETTE = ["#2563EB", "#7C3AED", "#10B981", "#F59E0B", "#EF4444",
           "#06B6D4", "#EC4899", "#8B5CF6", "#14B8A6", "#F97316"]

SCENARIO_CN = {
    "straight_cruise": "直道巡航", "curve_follow": "弯道跟车",
    "high_speed_follow": "高速跟车", "tight_gap": "近距离跟车",
    "lead_hard_brake": "前车急刹", "cut_in": "Cut-In",
    "overtake_stopped": "超车(静止)", "pedestrian_cross": "行人横穿",
    "lead_lost_reacquire": "目标丢失/恢复", "sensor_dropout": "传感器掉线",
}


def _fig(w=20, h=24):
    fig = plt.figure(figsize=(w, h), facecolor=C["bg"])
    return fig


def _ax(fig, rect, title=""):
    ax = fig.add_subplot(rect)
    ax.set_facecolor(C["bg"])
    ax.grid(True, alpha=0.3, color=C["grid"], linestyle="--")
    if title:
        ax.set_title(title, fontsize=13, fontweight="bold", pad=10)
    return ax


def load_scenario_csv(name, csv_dir):
    """读取场景 CSV，返回 dict of numpy arrays。"""
    path = os.path.join(csv_dir, "%s.csv" % name)
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    def col(key, dtype=float):
        vals = []
        for r in rows:
            v = r.get(key, "")
            if v == "" or v is None:
                vals.append(np.nan)
            else:
                try:
                    vals.append(dtype(v))
                except (ValueError, TypeError):
                    vals.append(np.nan)
        return np.array(vals, dtype=float)
    return {
        "t": col("t_wall"),
        "ego_v": col("ego_v"),
        "dist": col("dist"),
        "ttc": col("ttc"),
        "delta": col("delta"),
        "aeb": col("aeb_active", int),
        "cte": col("filtered_cte"),
        "lon_cmd": col("lon_cmd"),
        "lead_detected": col("lead_detected", int),
    }


# ═════════════════════════════════════════════════════════════════════
# Panel 1: 场景 KPI 雷达图
# ═════════════════════════════════════════════════════════════════════
def panel_radar(fig, gs, summary):
    ax = fig.add_subplot(gs, polar=True)
    ax.set_facecolor(C["bg"])

    metrics = ["min_gap", "min_ttc_inv", "aeb_norm", "max_steer_inv", "rms_cte_inv"]
    labels = ["最小车距", "TTC\n(倒数)", "AEB\n激活", "转向\n(倒数)", "CTE\n(倒数)"]

    scenarios = list(summary.keys())
    n = len(scenarios)
    angles = np.linspace(0, 2 * np.pi, len(metrics), endpoint=False).tolist()
    angles += angles[:1]

    for i, sc in enumerate(scenarios):
        s = summary[sc]
        min_gap = s.get("min_gap") or 50
        min_ttc = s.get("min_ttc") or 60
        aeb = s.get("aeb_count", 0)
        max_steer = s.get("max_steer", 0.01)
        rms_cte = s.get("rms_cte", 0.01)

        # normalize to 0-1 (higher = safer)
        vals = [
            min(min_gap / 50, 1.0),
            min(10 / min_ttc, 1.0) if min_ttc and min_ttc > 0 else 0.1,
            max(0, 1.0 - min(aeb / 500, 1.0)),
            max(0, 1.0 - min(max_steer / 0.1, 1.0)),
            max(0, 1.0 - min(rms_cte / 1.0, 1.0)),
        ]
        vals += vals[:1]
        ax.plot(angles, vals, "o-", linewidth=1.5, markersize=4,
                color=PALETTE[i % len(PALETTE)], label=SCENARIO_CN.get(sc, sc), alpha=0.8)
        ax.fill(angles, vals, alpha=0.05, color=PALETTE[i % len(PALETTE)])

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(labels, fontsize=10)
    ax.set_ylim(0, 1.1)
    ax.set_title("场景安全 KPI 雷达图", fontsize=14, fontweight="bold", pad=20, y=1.08)
    ax.legend(loc="upper right", bbox_to_anchor=(1.35, 1.1), fontsize=8, ncol=2, framealpha=0.9)


# ═════════════════════════════════════════════════════════════════════
# Panel 2-3: 速度 & 车距 时间序列 (选 4 个关键场景)
# ═════════════════════════════════════════════════════════════════════
def panel_timeseries(fig, gs, csv_data, scenarios, title, key, ylabel, color):
    ax = fig.add_subplot(gs)
    ax.set_facecolor(C["bg"])
    ax.grid(True, alpha=0.3, color=C["grid"], linestyle="--")
    for i, sc in enumerate(scenarios):
        d = csv_data.get(sc)
        if d is None:
            continue
        t = d["t"] - d["t"][0]  # relative time
        vals = d[key]
        mask = ~np.isnan(vals)
        ax.plot(t[mask] * 1000, vals[mask], linewidth=1.5, color=PALETTE[i],
                label=SCENARIO_CN.get(sc, sc), alpha=0.85)
    ax.set_xlabel("时间 (ms)", fontsize=11)
    ax.set_ylabel(ylabel, fontsize=11)
    ax.set_title(title, fontsize=13, fontweight="bold", pad=10)
    ax.legend(fontsize=9, framealpha=0.9, loc="upper right")


# ═════════════════════════════════════════════════════════════════════
# Panel 4: AEB 激活时间线 (选有 AEB 的场景)
# ═════════════════════════════════════════════════════════════════════
def panel_aeb_timeline(fig, gs, csv_data):
    ax = fig.add_subplot(gs)
    ax.set_facecolor(C["bg"])
    ax.grid(True, alpha=0.3, color=C["grid"], linestyle="--")

    aeb_scenarios = ["tight_gap", "cut_in", "pedestrian_cross", "lead_hard_brake", "overtake_stopped"]
    for i, sc in enumerate(aeb_scenarios):
        d = csv_data.get(sc)
        if d is None:
            continue
        t = d["t"] - d["t"][0]
        aeb = d["aeb"]
        mask = aeb > 0
        if np.any(mask):
            ax.barh(i * 0.3, t[mask][-1] - t[mask][0], left=t[mask][0] * 1000,
                    height=0.2, color=PALETTE[i], alpha=0.8, edgecolor="white")
            ax.text(t[mask][-1] * 1000 + 5, i * 0.3, "%d 帧" % np.sum(mask),
                    va="center", fontsize=9, fontweight="bold", color=PALETTE[i])
        ax.text(-20, i * 0.3, SCENARIO_CN.get(sc, sc), va="center", ha="right",
                fontsize=10, fontweight="bold")

    ax.set_xlabel("时间 (ms)", fontsize=11)
    ax.set_title("AEB 激活时间窗口对比", fontsize=13, fontweight="bold", pad=10)
    ax.set_yticks([])
    ax.set_ylim(-0.3, len(aeb_scenarios) * 0.3)


# ═════════════════════════════════════════════════════════════════════
# Panel 5: 场景结果柱状图 (KPI 对比)
# ═════════════════════════════════════════════════════════════════════
def panel_kpi_bars(fig, gs, summary):
    ax = fig.add_subplot(gs)
    ax.set_facecolor(C["bg"])
    ax.grid(True, alpha=0.3, color=C["grid"], linestyle="--", axis="x")

    scenarios = list(summary.keys())
    labels = [SCENARIO_CN.get(s, s) for s in scenarios]
    min_gaps = [summary[s].get("min_gap") or 0 for s in scenarios]
    min_ttc = [summary[s].get("min_ttc") or 0 for s in scenarios]
    aeb_cnt = [summary[s].get("aeb_count", 0) for s in scenarios]
    max_steer = [summary[s].get("max_steer", 0) * 100 for s in scenarios]  # deg

    x = np.arange(len(scenarios))
    w = 0.2

    ax.bar(x - 1.5*w, min_gaps, w, label="最小车距 (m)", color=C["pri"], alpha=0.85, edgecolor="white")
    ax.bar(x - 0.5*w, min_ttc, w, label="最小 TTC (s)", color=C["ok"], alpha=0.85, edgecolor="white")
    ax.bar(x + 0.5*w, aeb_cnt, w, label="AEB 激活帧数", color=C["err"], alpha=0.85, edgecolor="white")
    ax.bar(x + 1.5*w, max_steer, w, label="最大转向 (°)", color=C["sec"], alpha=0.85, edgecolor="white")

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9, rotation=30, ha="right")
    ax.set_ylabel("数值", fontsize=12)
    ax.set_title("各场景 KPI 柱状图对比", fontsize=14, fontweight="bold", pad=10)
    ax.legend(fontsize=10, framealpha=0.9, ncol=2)


# ═════════════════════════════════════════════════════════════════════
# Panel 6: (已删除——边缘计算已移入 HIL Nano)
# ═════════════════════════════════════════════════════════════════════


# ═════════════════════════════════════════════════════════════════════
# Panel 8: 综合评分卡
# ═════════════════════════════════════════════════════════════════════
def panel_scorecard(fig, gs, summary):
    ax = fig.add_subplot(gs)
    ax.set_facecolor(C["bg"])
    ax.axis("off")

    total_scenarios = len(summary)
    total_rows = sum(s.get("rows", 0) for s in summary.values())
    collisions = sum(1 for s in summary.values() if s.get("collision"))
    aeb_total = sum(s.get("aeb_count", 0) for s in summary.values())
    all_no_collision = collisions == 0

    cards = [
        ("🛡️ 安全性", "零碰撞" if all_no_collision else "%d 次碰撞" % collisions,
         C["ok"] if all_no_collision else C["err"]),
        ("⏱️ AEB 响应", "%d 帧激活" % aeb_total, C["warn"] if aeb_total > 0 else C["ok"]),
        ("📊 场景覆盖", "%d 场景 / %d 步" % (total_scenarios, total_rows), C["pri"]),
    ]

    for i, (title, value, color) in enumerate(cards):
        y = 0.92 - i * 0.15
        ax.text(0.05, y, title, fontsize=13, fontweight="bold", transform=ax.transAxes, va="top")
        ax.text(0.55, y, value, fontsize=14, fontweight="bold", color=color,
                transform=ax.transAxes, va="top",
                bbox=dict(boxstyle="round,pad=0.3", facecolor=color, alpha=0.12))

    ax.set_title("全链路测试评分卡", fontsize=15, fontweight="bold", pad=10)


# ═════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════
def main():
    base = os.path.dirname(os.path.abspath(__file__))
    csv_dir = os.path.join(base, "csv_output")
    summary_path = os.path.join(csv_dir, "summary.json")

    with open(summary_path, "r", encoding="utf-8") as f:
        summary = json.load(f)

    # load CSVs
    csv_data = {}
    for sc in summary:
        csv_data[sc] = load_scenario_csv(sc, csv_dir)

    print("=" * 60)
    print("  全链路综合对比仪表盘")
    print("=" * 60)

    fig = _fig(22, 24)
    gs = gridspec.GridSpec(4, 3, figure=fig, hspace=0.35, wspace=0.3,
                           left=0.06, right=0.94, top=0.95, bottom=0.03)

    # Row 1: radar + scorecard
    panel_radar(fig, gs[0, 0:2], summary)
    panel_scorecard(fig, gs[0, 2], summary)

    # Row 2: speed time-series
    key_scenarios_speed = ["straight_cruise", "high_speed_follow", "tight_gap", "cut_in"]
    panel_timeseries(fig, gs[1, 0:2], csv_data, key_scenarios_speed,
                     "自车速度对比", "ego_v", "速度 (m/s)", C["pri"])
    panel_timeseries(fig, gs[1, 2], csv_data,
                     ["lead_hard_brake", "tight_gap", "cut_in", "pedestrian_cross"],
                     "前车距离对比", "dist", "距离 (m)", C["err"])

    # Row 3: AEB timeline + KPI bars
    panel_aeb_timeline(fig, gs[2, 0:2], csv_data)
    panel_kpi_bars(fig, gs[2, 2], summary)

    # Row 4: CTE + TTC time-series
    panel_timeseries(fig, gs[3, 0:2], csv_data,
                     ["straight_cruise", "curve_follow", "overtake_stopped"],
                     "横向误差 (CTE) 时间序列", "cte", "CTE (m)", C["sec"])
    panel_timeseries(fig, gs[3, 2], csv_data,
                     ["cut_in", "tight_gap", "lead_hard_brake"],
                     "TTC 时间序列", "ttc", "TTC (s)", C["warn"])

    out_dir = os.path.join(base, "csv_output")
    out_path = os.path.join(out_dir, "full_dashboard.png")
    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=C["bg"])
    plt.close(fig)
    print("  Saved: %s" % out_path)

    # ── 单独保存各子图 ──
    # 速度子图
    fig2, axes = plt.subplots(2, 2, figsize=(16, 10), facecolor=C["bg"])
    for i, sc in enumerate(["straight_cruise", "high_speed_follow", "tight_gap", "cut_in"]):
        ax = axes[i // 2][i % 2]
        ax.set_facecolor(C["bg"])
        ax.grid(True, alpha=0.3, color=C["grid"], linestyle="--")
        d = csv_data.get(sc)
        if d is not None:
            t = d["t"] - d["t"][0]
            mask = ~np.isnan(d["ego_v"])
            ax.plot(t[mask] * 1000, d["ego_v"][mask], color=C["pri"], linewidth=1.5)
            # AEB highlight
            aeb_mask = d["aeb"] > 0
            if np.any(aeb_mask):
                ax.axvspan(t[np.where(aeb_mask)[0][0]] * 1000,
                           t[np.where(aeb_mask)[0][-1]] * 1000,
                           alpha=0.2, color=C["err"], label="AEB")
                ax.legend(fontsize=9)
        ax.set_title(SCENARIO_CN.get(sc, sc), fontsize=12, fontweight="bold")
        ax.set_xlabel("时间 (ms)", fontsize=10)
        ax.set_ylabel("速度 (m/s)", fontsize=10)
    fig2.suptitle("各场景自车速度时间序列", fontsize=14, fontweight="bold", y=1.01)
    fig2.tight_layout()
    sp = os.path.join(out_dir, "scenario_speed_timeseries.png")
    fig2.savefig(sp, dpi=150, bbox_inches="tight", facecolor=C["bg"])
    plt.close(fig2)
    print("  Saved: %s" % sp)

    print("\n" + "=" * 60)
    print("  Done!")
    print("=" * 60)


if __name__ == "__main__":
    main()
