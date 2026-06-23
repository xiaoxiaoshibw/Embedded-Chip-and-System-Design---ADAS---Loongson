# -*- coding: utf-8 -*-
"""Render LaTeX $$ blocks from the markdown to clean PNG images via matplotlib mathtext.

mathtext does not support aligned/cases environments, so we split a block into
logical rows (on \\\\) and render each row, keeping '=' column alignment for
aligned blocks via a two-column placement trick.
"""
import os, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# mathtext-safe substitutions applied to every math fragment
_SUBS = [
    (r"\lor", r"\vee"),
    (r"\land", r"\wedge"),
    (r"\bigl", ""),
    (r"\bigr", ""),
    (r"\text{-}", "-"),
]

def _clean(frag):
    s = frag.strip()
    for a, b in _SUBS:
        s = s.replace(a, b)
    # \le / \ge are not aliases in this mathtext build -> use \leq / \geq
    s = re.sub(r"\\le(?![a-zA-Z])", r"\\leq", s)
    s = re.sub(r"\\ge(?![a-zA-Z])", r"\\geq", s)
    return s

def _parse_block(latex):
    body = latex.strip()
    m = re.search(r"\\begin\{(aligned|cases)\}(.*)\\end\{\1\}", body, re.S)
    env = None
    pre = ""
    if m:
        env = m.group(1)
        pre = body[:m.start()].strip()
        inner = m.group(2)
    else:
        inner = body
    rows = [r.strip() for r in re.split(r"\\\\", inner) if r.strip()]
    return env, pre, rows

def render_formula(latex, outpath, fontsize=21, dpi=200):
    env, pre, rows = _parse_block(latex)
    FIGW = 8.0                       # 测量基准宽（英寸）
    GAP = 0.014                      # 左量与 = 之间的小间隔（图宽分数）
    ROW_H = 0.42                     # 每行高度（英寸），收紧行距

    # 解析为带类型的单元
    cells = []  # ("split", lhs, rhs) | ("case", val, cond) | ("single", text)
    if env == "cases" and pre:
        cells.append(("single", _clean(pre) + r"\ \{"))
    for r in rows:
        if env == "cases":
            if "&" in r:
                v, c = r.split("&", 1); cells.append(("case", _clean(v), _clean(c)))
            else:
                cells.append(("single", _clean(r)))
        elif "&" in r:
            l, rr = r.split("&", 1); cells.append(("split", _clean(l), _clean(rr)))
        else:
            cells.append(("single", _clean(r)))

    # 把 “= rhs” 拆成 “lhs =”（含等号，右对齐到等号列）+ “rhs”（紧跟其后）
    def split_eq(lhs, rhs):
        rhs = rhs.strip()
        if rhs.startswith("="):
            return (lhs.strip() + r"\,=").strip(), rhs[1:].strip()
        return lhs.strip(), rhs

    n = len(cells)
    fig = plt.figure(figsize=(FIGW, 1.0))
    ax = fig.add_axes([0, 0, 1, 1]); ax.axis("off")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    fig.canvas.draw()
    rnd = fig.canvas.get_renderer()
    figw_px = fig.get_figwidth() * fig.dpi

    def W(s):
        t = ax.text(0, -9, "$%s$" % s, fontsize=fontsize)
        bb = t.get_window_extent(rnd); t.remove()
        return bb.width / figw_px

    # 计算对齐基准 x（= 号右缘所在列），使整组水平居中
    splits = [c for c in cells if c[0] == "split"]
    cases = [c for c in cells if c[0] == "case"]
    if splits:
        le = [split_eq(c[1], c[2]) for c in splits]
        max_l = max(W(a) for a, b in le)
        max_r = max(W(b) for a, b in le)
        block_w = max_l + GAP + max_r
        x_eq = max(0.02, (1 - block_w) / 2) + max_l
    else:
        x_eq = 0.5
    if cases:
        max_v = max(W(c[1]) for c in cases)
        x_cond = 0.085 + max_v + 0.045

    fig.set_size_inches(FIGW, max(0.5, ROW_H * n))
    ys = [1.0 - (i + 0.5) / n for i in range(n)]
    for cell, y in zip(cells, ys):
        kind = cell[0]
        if kind == "split":
            lhs_eq, rhs_expr = split_eq(cell[1], cell[2])
            ax.text(x_eq, y, "$%s$" % lhs_eq, fontsize=fontsize, ha="right", va="center")
            ax.text(x_eq + GAP, y, "$%s$" % rhs_expr, fontsize=fontsize, ha="left", va="center")
        elif kind == "case":
            ax.text(0.085, y, "$%s$" % cell[1], fontsize=fontsize, ha="left", va="center")
            ax.text(x_cond, y, "$%s$" % cell[2], fontsize=fontsize, ha="left", va="center")
        else:
            ha = "left" if env == "cases" else "center"
            x = 0.03 if env == "cases" else 0.5
            ax.text(x, y, "$%s$" % cell[1], fontsize=fontsize, ha=ha, va="center")
    fig.savefig(outpath, dpi=dpi, transparent=True, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)

if __name__ == "__main__":
    import sys
    md = open(sys.argv[1] if len(sys.argv) > 1 else
              r"D:\Code\自动辅助驾驶仿真平台\文档\龙芯定稿\作品报告_龙芯定稿.md",
              encoding="utf-8").read()
    formulas = re.findall(r"\$\$(.*?)\$\$", md, re.S)
    print("found %d formula blocks" % len(formulas))
    os.makedirs("_test_formula", exist_ok=True)
    fails = []
    for i, f in enumerate(formulas):
        try:
            render_formula(f, "_test_formula/f%02d.png" % i)
        except Exception as e:
            fails.append((i, str(e)[:160], f[:80]))
    if fails:
        print("FAILURES:")
        for i, e, snip in fails:
            print(" #%d: %s | %s" % (i, e, snip.replace(chr(10), " ")))
    else:
        print("ALL %d formulas rendered OK" % len(formulas))
