#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""遥测生命周期回归：懒建文件 + 历史保留清理。

对应 2026-07-03 改动：
  - 空闲（无 record）会话不再留下纯表头 CSV；
  - 首行数据到达才创建文件并写表头；
  - 写盘线程启动时按 TELEMETRY_KEEP 清理本 role 旧 CSV。
"""

import glob
import os
import time

import pytest

import telemetry


@pytest.fixture()
def tmp_telemetry_dir(tmp_path, monkeypatch):
    monkeypatch.setenv('TELEMETRY', '1')
    monkeypatch.setenv('TELEMETRY_DIR', str(tmp_path))
    monkeypatch.setenv('TELEMETRY_KEEP', '20')
    return tmp_path


def _csvs(d, role='testrole'):
    return sorted(glob.glob(os.path.join(str(d), 'adas_%s_telemetry_*.csv' % role)))


def _wait(cond, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return True
        time.sleep(0.02)
    return cond()


def test_no_file_without_rows(tmp_telemetry_dir):
    """空闲会话（零 record）不创建 CSV。"""
    t = telemetry.Telemetry('testrole')
    # 给写盘线程一点时间跑完清理段
    time.sleep(0.3)
    assert _csvs(tmp_telemetry_dir) == []
    t.close()
    assert _csvs(tmp_telemetry_dir) == []


def test_file_created_on_first_row(tmp_telemetry_dir):
    """首行数据到达即创建文件：表头 + 数据行。"""
    t = telemetry.Telemetry('testrole')
    t.record({'cycle': 1, 'ego_v': 5.0})
    assert _wait(lambda: len(_csvs(tmp_telemetry_dir)) == 1)
    t.close()
    files = _csvs(tmp_telemetry_dir)
    assert len(files) == 1
    with open(files[0]) as fh:
        lines = fh.read().strip().splitlines()
    assert lines[0].split(',')[:3] == ['t_wall', 't_mono', 'cycle']
    assert len(lines) == 2          # 表头 + 1 行数据


def test_close_drains_queued_rows(tmp_telemetry_dir):
    """close() 时若有残留行而文件未建，补建并落盘（不丢数据）。"""
    t = telemetry.Telemetry('testrole')
    # 直接塞队列不等写盘线程消费，立即 close
    t.record({'cycle': 7})
    t.close()
    files = _csvs(tmp_telemetry_dir)
    assert len(files) == 1
    with open(files[0]) as fh:
        lines = fh.read().strip().splitlines()
    assert len(lines) >= 2


def test_cleanup_keeps_recent(tmp_telemetry_dir, monkeypatch):
    """启动清理只保留最近 TELEMETRY_KEEP 个，且不碰其它 role。"""
    monkeypatch.setenv('TELEMETRY_KEEP', '3')
    old = []
    for i in range(6):
        p = os.path.join(str(tmp_telemetry_dir),
                         'adas_testrole_telemetry_2026010%d_000000.csv' % i)
        with open(p, 'w') as fh:
            fh.write('header\n')
        ts = time.time() - 3600 + i * 60
        os.utime(p, (ts, ts))
        old.append(p)
    other = os.path.join(str(tmp_telemetry_dir),
                         'adas_otherrole_telemetry_20260101_000000.csv')
    with open(other, 'w') as fh:
        fh.write('header\n')

    t = telemetry.Telemetry('testrole')
    assert _wait(lambda: len(_csvs(tmp_telemetry_dir)) == 3)
    t.close()
    kept = _csvs(tmp_telemetry_dir)
    # 留的是 mtime 最新的 3 个
    assert kept == sorted(old[-3:])
    assert os.path.exists(other)    # 其它 role 不受影响


def test_disabled_never_touches_disk(tmp_telemetry_dir, monkeypatch):
    """TELEMETRY=0：record/close 全程零文件。"""
    monkeypatch.setenv('TELEMETRY', '0')
    t = telemetry.Telemetry('testrole')
    t.record({'cycle': 1})
    t.close()
    assert _csvs(tmp_telemetry_dir) == []
