#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把历史运行 CSV 归档进「数据仓库」并重建索引（纯标准库）。

用法：
  python archive_csv.py 某次运行.csv                  # 归档单个 CSV
  python archive_csv.py a.csv b.csv --category acc     # 归档多个并指定类型
  python archive_csv.py 某次.csv --name 雨天跟车        # 指定展示名
  python archive_csv.py --from-logs <目录路径>  # 批量导入目录所有 CSV（例：..\\仿真\\logs）
  python archive_csv.py --rebuild                       # 只重建索引（不导入新文件）

归档后即可在统一可爱网站「🚗 实时驾驶舱 · 🗂️ 数据仓库」里按类型/日期选择回放。
"""

import os
import sys
import time
import glob
import shutil
import argparse

try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import repo_index

REPO = os.path.join(HERE, '数据仓库')


def archive_one(src, category=None, name=None):
    if not os.path.isfile(src):
        print('  跳过（不存在）：%s' % src)
        return None
    stem = os.path.splitext(os.path.basename(src))[0]
    parts = []
    if category:
        parts.append(category)
    parts.append(name or stem)
    parts.append(time.strftime('%Y%m%d_%H%M%S'))
    dst = os.path.join(REPO, '_'.join(parts) + '.csv')
    # 避免同秒重名
    k = 1
    while os.path.exists(dst):
        dst = os.path.join(REPO, '_'.join(parts) + ('_%d' % k) + '.csv')
        k += 1
    shutil.copy2(src, dst)
    print('  归档：%s → %s' % (os.path.basename(src), os.path.basename(dst)))
    return dst


def main(argv=None):
    p = argparse.ArgumentParser(description='归档历史 CSV 进数据仓库并重建索引')
    p.add_argument('sources', nargs='*', help='要归档的 CSV 文件')
    p.add_argument('--category', default=None,
                   help='指定场景类型（overtake/cutin/pedestrian/acc/highspeed/cruise）')
    p.add_argument('--name', default=None, help='展示名（默认用文件名）')
    p.add_argument('--from-logs', default=None, help='批量导入该目录下所有 *.csv')
    p.add_argument('--rebuild', action='store_true', help='只重建索引')
    args = p.parse_args(argv)

    os.makedirs(REPO, exist_ok=True)
    n = 0
    srcs = list(args.sources)
    if args.from_logs:
        srcs += sorted(glob.glob(os.path.join(args.from_logs, '*.csv')))
    if srcs:
        print('[归档] 导入 %d 个 CSV 到数据仓库 ...' % len(srcs))
        for s in srcs:
            if archive_one(s, args.category, args.name):
                n += 1

    print('[归档] 重建索引 ...')
    idx = repo_index.build_index(REPO)
    print('[归档] 完成：仓库共 %d 条记录（本次新增 %d）。' % (idx['count'], n))
    print('[归档] 索引：%s' % os.path.join(REPO, repo_index.INDEX_NAME))


if __name__ == '__main__':
    main()
