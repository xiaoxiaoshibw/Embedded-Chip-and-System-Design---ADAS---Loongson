#!/usr/bin/env python3
"""生成 SOC 单元测试效果可视化图表"""

import matplotlib.pyplot as plt
import matplotlib
import numpy as np

matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

# ── 测试数量数据 ──
test_modules = {
    'common':          60,
    'serial_protocol': 16,
    'health':          17,
    'longitudinal':    67,
    'lateral':         36,
    'lateral_ctrl':    17,
    'lead_tracking':   25,
    'aeb_alert':       17,
    'curve_hold':      15,
    'overtake':        25,
    'mpc_longitudinal':31,
    'perception':      28,
    'pipeline':        21,
}

# ── 覆盖率数据（源码模块，排除 tests/ 和 0% 的 I/O 模块）──
cov_modules = {
    'aeb_alert':       98,
    'context':         99,
    'curve_hold':      100,
    'health':          100,
    'lateral_ctrl':    98,
    'lead_tracking':   87,
    'lon_policy':      69,
    'mpc_longitudinal':94,
    'overtake':        99,
    'perception':      98,
    'serial_protocol': 100,
    'state':           100,
    'common':          65,
    'config':          81,
    'lateral':         85,
    'longitudinal':    98,
    'pipeline':        87,
    'runtime':         52,
}

# ── 测试层级分布 ──
tier_counts = {
    'Tier 1\n纯函数':   93,
    'Tier 2\n有状态类': 196,
    'Tier 3\n集成测试': 86,
}

# ── 开始绘图 ──
fig = plt.figure(figsize=(20, 14), facecolor='#1a1a2e')
fig.suptitle('SOC 单元测试效果总览', fontsize=24, fontweight='bold', color='white', y=0.97)
fig.text(0.5, 0.94, '375 tests · 0.27s · 全部通过', ha='center', fontsize=14, color='#a0a0a0')

# 配色
CYAN    = '#00d4ff'
GREEN   = '#00e676'
ORANGE  = '#ff9100'
RED     = '#ff1744'
PURPLE  = '#b388ff'
BG_AX   = '#16213e'

# ── 图1: 各模块测试数量（水平条形图）──
ax1 = fig.add_axes([0.05, 0.52, 0.42, 0.38])
ax1.set_facecolor(BG_AX)
names = list(test_modules.keys())
vals  = list(test_modules.values())
colors = [CYAN if v >= 30 else GREEN if v >= 20 else ORANGE for v in vals]
bars = ax1.barh(range(len(names)), vals, color=colors, edgecolor='none', height=0.7)
ax1.set_yticks(range(len(names)))
ax1.set_yticklabels(names, fontsize=10, color='white')
ax1.invert_yaxis()
ax1.set_xlabel('测试数量', fontsize=11, color='white')
ax1.set_title('各模块测试数量', fontsize=14, fontweight='bold', color='white', pad=10)
ax1.tick_params(colors='white')
ax1.spines['top'].set_visible(False)
ax1.spines['right'].set_visible(False)
ax1.spines['bottom'].set_color('#444')
ax1.spines['left'].set_color('#444')
for bar, val in zip(bars, vals):
    ax1.text(bar.get_width() + 1, bar.get_y() + bar.get_height()/2,
             str(val), va='center', ha='left', fontsize=10, color='white', fontweight='bold')

# ── 图2: 源码模块覆盖率（水平条形图，按覆盖率排序）──
ax2 = fig.add_axes([0.55, 0.52, 0.42, 0.38])
ax2.set_facecolor(BG_AX)
sorted_cov = sorted(cov_modules.items(), key=lambda x: x[1])
c_names = [x[0] for x in sorted_cov]
c_vals  = [x[1] for x in sorted_cov]
c_colors = [GREEN if v >= 90 else CYAN if v >= 80 else ORANGE if v >= 60 else RED for v in c_vals]
bars2 = ax2.barh(range(len(c_names)), c_vals, color=c_colors, edgecolor='none', height=0.7)
ax2.set_yticks(range(len(c_names)))
ax2.set_yticklabels(c_names, fontsize=9, color='white')
ax2.invert_yaxis()
ax2.set_xlabel('覆盖率 %', fontsize=11, color='white')
ax2.set_title('源码模块行覆盖率', fontsize=14, fontweight='bold', color='white', pad=10)
ax2.set_xlim(0, 110)
ax2.axvline(x=80, color='#ff9100', linestyle='--', alpha=0.5, linewidth=1)
ax2.axvline(x=90, color='#00e676', linestyle='--', alpha=0.5, linewidth=1)
ax2.text(80.5, len(c_names)-0.5, '80%', color='#ff9100', fontsize=8, alpha=0.7)
ax2.text(90.5, len(c_names)-0.5, '90%', color='#00e676', fontsize=8, alpha=0.7)
ax2.tick_params(colors='white')
ax2.spines['top'].set_visible(False)
ax2.spines['right'].set_visible(False)
ax2.spines['bottom'].set_color('#444')
ax2.spines['left'].set_color('#444')
for bar, val in zip(bars2, c_vals):
    ax2.text(bar.get_width() + 1, bar.get_y() + bar.get_height()/2,
             f'{val}%', va='center', ha='left', fontsize=9, color='white', fontweight='bold')

# ── 图3: 测试层级分布（环形图）──
ax3 = fig.add_axes([0.05, 0.05, 0.28, 0.38])
ax3.set_facecolor(BG_AX)
tier_labels = list(tier_counts.keys())
tier_vals   = list(tier_counts.values())
tier_colors = [CYAN, GREEN, PURPLE]
wedges, texts, autotexts = ax3.pie(
    tier_vals, labels=tier_labels, autopct='%1.0f%%',
    colors=tier_colors, startangle=90, pctdistance=0.75,
    wedgeprops=dict(width=0.4, edgecolor=BG_AX, linewidth=2),
    textprops=dict(color='white', fontsize=11)
)
for t in autotexts:
    t.set_fontsize(12)
    t.set_fontweight('bold')
    t.set_color('white')
ax3.set_title('测试层级分布', fontsize=14, fontweight='bold', color='white', pad=10)
# 中心数字
ax3.text(0, 0, f'{sum(tier_vals)}', ha='center', va='center', fontsize=28,
         fontweight='bold', color='white')
ax3.text(0, -0.12, 'tests', ha='center', va='center', fontsize=10, color='#a0a0a0')

# ── 图4: 关键指标卡片 ──
ax4 = fig.add_axes([0.38, 0.05, 0.60, 0.38])
ax4.set_facecolor(BG_AX)
ax4.axis('off')

cards = [
    ('375',  '总测试数',    CYAN),
    ('0.27s','运行耗时',    GREEN),
    ('100%', '通过率',      GREEN),
    ('69%',  '总行覆盖率',  ORANGE),
    ('13',   '测试文件',    PURPLE),
    ('17',   '覆盖源码模块', CYAN),
]

for i, (num, label, color) in enumerate(cards):
    col = i % 3
    row = i // 3
    x = 0.05 + col * 0.33
    y = 0.55 - row * 0.50

    # 卡片背景
    rect = plt.Rectangle((x-0.02, y-0.18), 0.30, 0.40, transform=ax4.transAxes,
                          facecolor='#0f3460', edgecolor=color, linewidth=2,
                          joinstyle='round', clip_on=False, zorder=1)
    ax4.add_patch(rect)

    # 数字
    ax4.text(x + 0.13, y + 0.07, num, transform=ax4.transAxes,
             ha='center', va='center', fontsize=28 if len(num) <= 4 else 22,
             fontweight='bold', color=color, zorder=2)
    # 标签
    ax4.text(x + 0.13, y - 0.10, label, transform=ax4.transAxes,
             ha='center', va='center', fontsize=11, color='#a0a0a0', zorder=2)

plt.savefig(r'C:\Users\30680\Desktop\lx\SOCCode\tests\test_report.png',
            dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
plt.close()
print('[OK] SOCCode/tests/test_report.png')
