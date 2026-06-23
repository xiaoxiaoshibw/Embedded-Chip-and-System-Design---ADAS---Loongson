# -*- coding: utf-8 -*-
"""把 headless CARLA 实测数据绘成对比图（我方系统 vs 无创新基线）。

读取 成果/数据/*.csv → 输出 PNG 到 成果/图片/。所有数据均为本机 Town04 无图形
实测（接近调速器 + 受控接近门控 + 统一仲裁 + ML 预警 对比关闭这些创新的基线）。

字体与配色对齐项目既有图（Microsoft YaHei 在前、√ 代替 ✓）。
"""
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 150
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.alpha"] = 0.25
plt.rcParams["grid.linestyle"] = "--"

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DATA = os.path.join(ROOT, "成果", "数据")
OUT = os.path.join(ROOT, "成果", "图片")

C_OURS = "#4f6f8f"     # muted blue-gray, paper-style
C_BASE = "#9aa0a6"     # neutral gray
C_RED = "#8b4a4a"      # muted red-brown
C_GREEN = "#6f8a74"    # muted green-gray
C_ORANGE = "#9a7b45"   # muted ochre


def load(name):
    path = os.path.join(DATA, name)
    cols = {}
    with open(path, encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            for k, v in row.items():
                cols.setdefault(k, []).append(v)
    out = {}
    for k, vals in cols.items():
        try:
            out[k] = np.array([float(x) for x in vals])
        except ValueError:
            out[k] = np.array(vals)
    return out


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    fig.savefig(p, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(p)


# ──────────────────────────────────────────────────────────
# 图1：ACC↔AEB 打架对比（核心创新）
# ──────────────────────────────────────────────────────────
def fig_arbitration():
    b = load("acc_approach_baseline.csv")
    o = load("acc_approach_ours.csv")
    n_aeb_b = int(b["rule_aeb"].sum())
    n_aeb_o = int(o["rule_aeb"].sum())

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    fig.suptitle("核心创新：ACC / AEB 协同仲裁 —— 从“加速—急刹”打架到平滑停车",
                 fontsize=15, fontweight="bold")

    ax = axes[0]
    ax.plot(b["t"], b["v"] * 3.6, color=C_BASE, lw=2.0,
            label="无创新基线（原始 TTC-AEB）")
    ax.plot(o["t"], o["v"] * 3.6, color=C_OURS, lw=2.4,
            label="本系统（接近调速器+受控门控）")
    # 标出基线触发 AEB 的时刻
    mb = b["rule_aeb"] > 0.5
    ax.scatter(b["t"][mb], b["v"][mb] * 3.6, s=14, color=C_RED, zorder=3,
               label="基线 AEB 急刹 (%d 次)" % n_aeb_b)
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("车速 (km/h)")
    ax.set_title("1. 车速：基线反复加速—急刹，本系统单调平滑", fontsize=11, fontweight="bold")
    ax.legend(fontsize=9, loc="upper right")

    ax = axes[1]
    ax.plot(b["t"], b["gap"], color=C_BASE, lw=2.0, label="无创新基线")
    ax.plot(o["t"], o["gap"], color=C_OURS, lw=2.4, label="本系统")
    ax.axhline(6.0, color=C_GREEN, lw=1.3, ls="--")
    ax.text(o["t"][-1] * 0.62, 6.6, "期望停车间距 ≈ 6 m", color=C_GREEN, fontsize=9)
    ax.annotate("基线卡在 %.0f m\n始终未驶达" % b["gap"][-1],
                xy=(b["t"][-1], b["gap"][-1]), xytext=(b["t"][-1] * 0.5, b["gap"][-1] + 3),
                color=C_BASE, fontsize=9, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color=C_BASE))
    ax.annotate("本系统平滑停在 %.1f m" % o["gap"][-1],
                xy=(o["t"][-1], o["gap"][-1]), xytext=(o["t"][-1] * 0.35, o["gap"][-1] - 4),
                color=C_OURS, fontsize=9, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color=C_OURS))
    ax.set_xlabel("时间 (s)")
    ax.set_ylabel("与前车间距 (m)")
    ax.set_title("2. 接近静止前车：本系统稳停车后，基线打架停不下", fontsize=11, fontweight="bold")
    ax.legend(fontsize=9, loc="upper right")

    fig.text(0.5, -0.02,
             "Town04 无图形实测：接近正前方静止车。基线 AEB 误触发 %d 次、车辆蠕动且无法接近；"
             "本系统 AEB 误触发 %d 次（√），平滑驶近并停稳。" % (n_aeb_b, n_aeb_o),
             ha="center", fontsize=9.5, color="#374151")
    fig.tight_layout(rect=[0, 0.02, 1, 0.93])
    save(fig, "图1_ACC-AEB协同仲裁对比.png")


# ──────────────────────────────────────────────────────────
# 图2：速度稳定边界
# ──────────────────────────────────────────────────────────
def fig_boundary():
    s = load("speed_sweep.csv")
    tv = s["target_v"] * 3.6
    mv = s["mean_v"] * 3.6
    rms = s["rms_off"]

    stable_bound = 0.5
    stable = rms <= stable_bound
    # Vcrit is an empirical boundary estimated from the first stable-to-unstable crossing.
    vcrit = None
    for i in range(len(tv) - 1):
        if stable[i] and not stable[i + 1]:
            ratio = (stable_bound - rms[i]) / (rms[i + 1] - rms[i])
            vcrit = tv[i] + ratio * (tv[i + 1] - tv[i])
            break
    if vcrit is None:
        vcrit = tv[stable].max()

    fig, ax = plt.subplots(figsize=(12.8, 5.8))
    fig.suptitle("车道保持系统稳定性边界分析", fontsize=15, fontweight="bold")

    xmin, xmax = tv.min() - 5, tv.max() + 5
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(0, max(1.55, rms.max() * 1.16))

    ax.axvspan(xmin, vcrit, color=C_GREEN, alpha=0.13, label="稳定控制区")
    ax.axvspan(vcrit, xmax, color=C_RED, alpha=0.08, label="失稳区")
    ax.axhspan(0, stable_bound, color=C_GREEN, alpha=0.055)
    ax.axhline(stable_bound, color=C_GREEN, lw=1.4, ls="--",
               label="工程稳定阈值 RMS = 0.5 m")
    ax.axvline(vcrit, color=C_RED, lw=1.5, ls="--",
               label="Vcrit（interpolated estimate）")

    ax.plot(tv, rms, "-o", color=C_OURS, lw=2.2, ms=6.5,
            label="lateral error RMS（simulation sweep）")
    ax.scatter([vcrit], [stable_bound], marker="D", s=56, color=C_RED,
               edgecolor="#333", zorder=5, label="Vcrit interpolation point")
    for x, y in zip(tv, rms):
        status = "bounded" if y <= stable_bound else "divergent"
        ax.annotate("%.3f\n%s" % (y, status), (x, y), textcoords="offset points",
                    xytext=(0, 8), ha="center", fontsize=8.0, color="#333")

    ax2 = ax.twinx()
    ax2.grid(False)
    ax2.set_ylim(0, max(82, mv.max() * 1.18))
    ax2.plot(tv, mv, "s--", color=C_BASE, lw=1.7, ms=5.5,
             label="actual mean speed（simulation sweep）")
    for x, y in zip(tv, mv):
        ax2.annotate("%.0f" % y, (x, y), textcoords="offset points",
                     xytext=(0, 8), ha="center", fontsize=8.0, color="#555")

    ax.text(xmin + 1.5, stable_bound * 0.63,
            "稳定控制区\nRMS ≤ 0.5 m\n无发散振荡 ≥10 s",
            color="#405f45", fontsize=9.2, fontweight="bold")
    ax.text(vcrit + 1.4, ax.get_ylim()[1] * 0.78,
            "Vcrit ≈ %.0f km/h\nempirical stability boundary" % vcrit,
            color=C_RED, fontsize=9.2, fontweight="bold")
    ax.text(vcrit + 1.4, ax.get_ylim()[1] * 0.18,
            "高于 Vcrit：\nlateral error divergence\n或振荡发散",
            color=C_RED, fontsize=8.8)

    ax.set_xlabel("目标车速 (km/h)")
    ax.set_ylabel("车道横向偏移误差 RMS (m)")
    ax2.set_ylabel("系统实际稳定可达均速 (km/h)")
    ax.set_title("simulation sweep / interpolated estimate；0.5 m 为工程性能边界，非物理常数",
                 fontsize=10.8, fontweight="bold")

    lines, labels = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines + lines2, labels + labels2, fontsize=8.4, loc="upper left",
              framealpha=0.92)

    fig.text(0.5, -0.015,
             "数据来源类型：离散速度点为 simulation sweep；Vcrit 由 RMS=0.5 m 阈值穿越处线性插值得到，属于 interpolated estimate；"
             "本图不包含 MIL experiment 点。Vcrit 不是理论解析解，而是该控制器/场景/采样设置下的经验稳定边界。",
             ha="center", fontsize=9.1, color="#374151")
    fig.tight_layout(rect=[0, 0.035, 1, 0.93])
    save(fig, "图2_车道保持速度稳定边界.png")


# ──────────────────────────────────────────────────────────
# 图3：三大场景时序
# ──────────────────────────────────────────────────────────
def fig_scenarios():
    ov = load("overtake_ours.csv")
    ci = load("cutin_ours.csv")
    pd = load("pedestrian_ours.csv")

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    fig.suptitle("三大功能场景实测时序（CARLA Town04，无图形）", fontsize=15, fontweight="bold")

    # 超车：速度 + 横向偏移
    ax = axes[0]
    ax.plot(ov["t"], ov["v"] * 3.6, color=C_OURS, lw=2.2, label="车速")
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("车速 (km/h)", color=C_OURS)
    ax.tick_params(axis="y", labelcolor=C_OURS)
    ax2 = ax.twinx(); ax2.grid(False)
    ax2.plot(ov["t"], np.abs(ov["eff_off"]), color=C_ORANGE, lw=2.0, label="横向偏移")
    ax2.set_ylabel("横向偏移 |m|", color=C_ORANGE)
    ax2.tick_params(axis="y", labelcolor=C_ORANGE)
    ax.set_title("超车：减速停车→向左变道(偏移↑)→超越回正", fontsize=10.5, fontweight="bold")

    # 加塞：速度 + 间距 + ML 风险
    ax = axes[1]
    ax.plot(ci["t"], ci["v"] * 3.6, color=C_OURS, lw=2.2, label="车速")
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("车速 (km/h)", color=C_OURS)
    ax.tick_params(axis="y", labelcolor=C_OURS)
    ax2 = ax.twinx(); ax2.grid(False)
    gap = np.where(ci["gap"] > 0, ci["gap"], np.nan)
    ax2.plot(ci["t"], gap, color=C_GREEN, lw=2.0, label="车间距")
    ax2.set_ylabel("车间距 (m)", color=C_GREEN)
    ax2.tick_params(axis="y", labelcolor=C_GREEN)
    ax.set_title("加塞 Cut-in：邻道车并入→自车减速保距", fontsize=10.5, fontweight="bold")

    # 行人：速度 + 行人横穿阴影
    ax = axes[2]
    ax.plot(pd["t"], pd["v"] * 3.6, color=C_OURS, lw=2.2, label="车速")
    ped = pd["ped"] > 0.5
    if ped.any():
        t0 = pd["t"][ped][0]; t1 = pd["t"][ped][-1]
        ax.axvspan(t0, t1, color=C_RED, alpha=0.15)
        ax.text((t0 + t1) / 2, ax.get_ylim()[1] * 0.9, "行人横穿",
                ha="center", color=C_RED, fontsize=9, fontweight="bold")
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("车速 (km/h)")
    ax.set_title("行人横穿：巡航→识别横穿→制动避让", fontsize=10.5, fontweight="bold")

    fig.tight_layout(rect=[0, 0, 1, 0.93])
    save(fig, "图3_三大功能场景时序.png")


# ──────────────────────────────────────────────────────────
# 图4：ML 风险预警提前量
# ──────────────────────────────────────────────────────────
def fig_ml():
    ci = load("cutin_ours.csv")
    t = ci["t"]; risk = ci["ml_risk"]; rule_aeb = ci["rule_aeb"]
    gap = np.where(ci["gap"] > 0, ci["gap"], np.nan)
    # 并线时刻：车间距首次出现（前车进入本车道被识别）
    merge_idx = np.where(ci["gap"] > 0)[0]
    t_merge = t[merge_idx[0]] if len(merge_idx) else None
    risk_peak = float(np.nanmax(risk))

    fig, ax = plt.subplots(figsize=(11.5, 4.9))
    fig.suptitle("机器学习补盲：加塞并线瞬间，规则 AEB 未触发，ML 仍精确捕捉碰撞风险",
                 fontsize=13.5, fontweight="bold")
    ax.plot(t, risk, color="#7c3aed", lw=2.4, label="ML-LSTM 风险概率 (warning+emergency)")
    ax.fill_between(t, 0, risk, color="#7c3aed", alpha=0.12)
    ax.plot(t, rule_aeb, color=C_RED, lw=2.2, ls="--",
            label="规则 AEB 触发标志 (全程为 0 → 未触发)")
    if t_merge is not None:
        ax.axvline(t_merge, color=C_GREEN, lw=1.4, ls="--")
        ax.text(t_merge + 0.2, 0.06, "并线时刻 %.1fs" % t_merge,
                color=C_GREEN, fontsize=9.5)
    # 标注 ML 峰值
    pk = int(np.nanargmax(risk))
    ax.annotate("ML 风险峰值 %.2f" % risk_peak, xy=(t[pk], risk[pk]),
                xytext=(t[pk] + 2.5, 0.86), fontsize=10, fontweight="bold",
                color="#7c3aed", arrowprops=dict(arrowstyle="->", color="#7c3aed"))
    ax.set_xlabel("时间 (s)"); ax.set_ylabel("风险概率 / 触发标志")
    ax.set_ylim(-0.03, 1.08)
    ax.legend(fontsize=9.5, loc="upper right")

    # 右轴：车间距，显示并线后骤降
    ax2 = ax.twinx(); ax2.grid(False)
    ax2.plot(t, gap, color="#9ca3af", lw=1.6, alpha=0.8)
    ax2.set_ylabel("车间距 (m)", color="#6b7280")
    ax2.tick_params(axis="y", labelcolor="#6b7280")

    fig.text(0.5, -0.02,
             "加塞场景：邻道车并入本车道瞬间车间距骤降，纯规则 TTC-AEB 因阈值/确认周期未触发；"
             "ML-LSTM（ONNX）输出风险峰值 ≈%.2f，作为学习型冗余预警并入保守仲裁（√ 只会更稳地减速）。"
             % risk_peak,
             ha="center", fontsize=9.5, color="#374151")
    fig.tight_layout(rect=[0, 0.02, 1, 0.92])
    save(fig, "图4_ML风险补盲预警.png")


# ──────────────────────────────────────────────────────────
# 图5：系统 vs 无创新基线 综合对比
# ──────────────────────────────────────────────────────────
def fig_overview():
    b = load("acc_approach_baseline.csv")
    o = load("acc_approach_ours.csv")

    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.2))
    fig.suptitle("综合对比：本系统 vs 无创新基线", fontsize=15, fontweight="bold")

    # 左：关键指标条形（真实实测）
    ax = axes[0]
    metrics = ["AEB 误触发\n(次)", "接近静止前车\n最终车距(m)"]
    base_vals = [int(b["rule_aeb"].sum()), float(b["gap"][-1])]
    our_vals = [int(o["rule_aeb"].sum()), float(o["gap"][-1])]
    x = np.arange(len(metrics)); w = 0.36
    bb = ax.bar(x - w / 2, base_vals, w, color=C_BASE, edgecolor="#333", label="无创新基线")
    bo = ax.bar(x + w / 2, our_vals, w, color=C_OURS, edgecolor="#333", label="本系统")
    for rects in (bb, bo):
        for r in rects:
            ax.text(r.get_x() + r.get_width() / 2, r.get_height() + 0.6,
                    "%.0f" % r.get_height() if r.get_height() >= 2 else "%.1f" % r.get_height(),
                    ha="center", fontsize=10, fontweight="bold")
    ax.set_xticks(x); ax.set_xticklabels(metrics)
    ax.set_title("关键指标（实测，越接近目标越好）", fontsize=11, fontweight="bold")
    ax.legend(fontsize=9.5)
    ax.text(1.0, max(base_vals) * 0.5, "基线停不下\n本系统稳停6m", fontsize=9, color="#6b7280")

    # 右：能力清单 √/×
    ax = axes[1]; ax.axis("off")
    rows = [
        ("车道保持 LKA", "√", "√"),
        ("自适应巡航 ACC", "√", "√"),
        ("自动紧急制动 AEB", "√", "√"),
        ("ACC/AEB 防打架仲裁", "×", "√"),
        ("受控接近静止车平滑停", "×", "√"),
        ("自动超车（绕行静止车）", "×", "√"),
        ("加塞 Cut-in 识别避让", "×", "√"),
        ("行人横穿制动", "×", "√"),
        ("机器学习风险预警", "×", "√"),
        ("主备双冗余无感切换", "×", "√"),
    ]
    ax.set_title("功能能力对比", fontsize=11, fontweight="bold")
    y = 0.93; dy = 0.088
    ax.text(0.46, y + 0.06, "基线", ha="center", fontsize=10, fontweight="bold", color=C_BASE)
    ax.text(0.78, y + 0.06, "本系统", ha="center", fontsize=10, fontweight="bold", color=C_OURS)
    for name, bv, ov in rows:
        ax.text(0.02, y, name, fontsize=10.5, va="center")
        ax.text(0.46, y, bv, ha="center", va="center", fontsize=13,
                color=C_GREEN if bv == "√" else C_RED, fontweight="bold")
        ax.text(0.78, y, ov, ha="center", va="center", fontsize=13,
                color=C_GREEN if ov == "√" else C_RED, fontweight="bold")
        y -= dy
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)

    fig.text(0.5, -0.01,
             "“无创新基线”= 关闭接近调速器/受控门控/仲裁/ML 等创新点的同一控制栈；指标均为本机实测。",
             ha="center", fontsize=9.5, color="#374151")
    fig.tight_layout(rect=[0, 0.01, 1, 0.93])
    save(fig, "图5_系统综合对比.png")


def main():
    fig_arbitration()
    fig_boundary()
    fig_scenarios()
    fig_ml()
    fig_overview()
    print("done ->", OUT)


if __name__ == "__main__":
    main()
