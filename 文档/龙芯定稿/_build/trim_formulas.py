# -*- coding: utf-8 -*-
"""按用户选定范围删减《作品报告_龙芯定稿.md》中的非核心公式（锚点法）。

保留：创新1 失效检测、创新2 CRC+src_exec+AEB地板、创新3 TTC/DRAC+Class-Aware+协同仲裁、
      LKA 横向控制律、弯道前瞻限速。
删除：创新1 接管评价(T_takeover/jerk)、2.3.2 组2(RMS/V_crit)、组4(ACC)、组5(Class-Aware
      AEB 重复)、组6(协同仲裁 重复)、组7(主备接管 重复)、组8(MCU 重复)、组9(ML 指标)。
"""
import io, os

MD = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '作品报告_龙芯定稿.md'))
txt = io.open(MD, encoding='utf-8').read()
orig_len = len(txt)


def line_start(s, idx):
    return s.rfind('\n', 0, idx) + 1


def assert_one(marker):
    n = txt.count(marker)
    assert n == 1, '标记出现 %d 次（应 1 次）: %r' % (n, marker[:30])


def cut(start_marker, end_marker):
    """删除从 start_marker 所在行起、到 end_marker 所在行前的整段。"""
    global txt
    assert_one(start_marker)
    assert_one(end_marker)
    s = line_start(txt, txt.index(start_marker))
    e = line_start(txt, txt.index(end_marker))
    assert s < e, '区间反了: %r .. %r' % (start_marker[:20], end_marker[:20])
    txt = txt[:e] and (txt[:s] + txt[e:])


# 1) 创新1：删接管评价指标 T_takeover/jerk_step（保留 F1 失效检测 + 代码）
cut('接管效果用两个量核验', '实现上只保留接管判定与状态继承')

# 2) 2.3.2 组2：横向 RMS 与临界速度（统计类，删）
cut('2. **横向 RMS 与临界速度', '3. **弯道前瞻限速：说明为什么目标设到 144')

# 3) 2.3.2 组4~组9 一并删除（ACC/Class-Aware重复/协同仲裁重复/接管重复/MCU重复/ML指标）
cut('4. **ACC 跟驰与停车门控', '**机器学习辅助（学习型辅助）。**')

# 4) 弯道前瞻限速从 “3.” 改 “2.”（因组2删除后它成为第二条）
old = '3. **弯道前瞻限速：说明为什么目标设到 144 km/h 仍可过弯有界。**'
new = '2. **弯道前瞻限速：说明为什么目标设到 144 km/h 仍可过弯有界。**'
assert txt.count(old) == 1
txt = txt.replace(old, new)

io.open(MD, 'w', encoding='utf-8').write(txt)
print('删减完成：%d -> %d 字符（-%d）' % (orig_len, len(txt), orig_len - len(txt)))
print('剩余 $$ 公式块数:', txt.count('$$') // 2)
