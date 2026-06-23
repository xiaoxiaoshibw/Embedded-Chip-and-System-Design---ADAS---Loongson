#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从 unpacked_report2/word/document.xml 提取报告正文与表格 → report_full.json。

纯标准库、UTF-8 输出。用于从竞赛报告 docx 抽取「亮点总览」素材，避免手工转写中文出错。
"""

import json
import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_DOC = os.path.normpath(os.path.join(_HERE, '..', 'unpacked_report2', 'word', 'document.xml'))


def _para_text(block):
    # 只匹配 <w:t> / <w:t xml:space=...>，不能误匹配 <w:tcPr>/<w:tcW>/<w:tbl> 等以 w:t 开头的标签
    return ''.join(re.findall(r'<w:t(?:\s[^>]*)?>(.*?)</w:t>', block, re.S)).strip()


def _unescape(s):
    return (s.replace('&amp;', '&').replace('&lt;', '<').replace('&gt;', '>')
            .replace('&quot;', '"').replace('&apos;', "'"))


def extract(doc_path=_DOC):
    xml = open(doc_path, encoding='utf-8').read()
    body = re.search(r'<w:body>(.*)</w:body>', xml, re.S)
    body = body.group(1) if body else xml

    # 顺序遍历段落与表格
    items = []
    for m in re.finditer(r'<w:tbl>.*?</w:tbl>|<w:p[ >].*?</w:p>', body, re.S):
        chunk = m.group(0)
        if chunk.startswith('<w:tbl'):
            rows = []
            for tr in re.findall(r'<w:tr[ >].*?</w:tr>', chunk, re.S):
                cells = [_unescape(_para_text(tc))
                         for tc in re.findall(r'<w:tc>.*?</w:tc>', tr, re.S)]
                rows.append(cells)
            if rows:
                items.append({'type': 'table', 'rows': rows})
        else:
            txt = _unescape(_para_text(chunk))
            if txt:
                items.append({'type': 'p', 'text': txt})
    return items


def main():
    items = extract()
    out = os.path.join(_HERE, 'report_full.json')
    with open(out, 'w', encoding='utf-8') as fh:
        json.dump(items, fh, ensure_ascii=False, indent=1)
    paras = sum(1 for i in items if i['type'] == 'p')
    tables = sum(1 for i in items if i['type'] == 'table')
    print('extracted %d items (%d paragraphs, %d tables) -> %s' %
          (len(items), paras, tables, out))


if __name__ == '__main__':
    main()
