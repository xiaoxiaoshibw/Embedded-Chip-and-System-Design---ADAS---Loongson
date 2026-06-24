#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""PerceptionLayer 单元测试。

覆盖：
  - 无目标 → 空 frame
  - 单目标注入 → 出现在 frame.tracks
  - NaN 位姿 → 拒绝
  - 主前车选举：最近前向 in_lane 目标
  - get_cls / ingest_cls / has_target
"""

import math
from unittest import mock

import pytest

# perception.py 在 import 时从 config 拉取常量，需要确保它们存在。
# conftest.py 已将 SOCCode/ 加入 sys.path，直接 import 即可。
from control.perception import PerceptionLayer, TrackRel, PerceptionFrame, _finite


# ── 辅助常量（与 config 默认值一致） ──
LEAD_TIMEOUT_S = 0.5
MULTI_TARGET_FWD_MIN = 0.5
MULTI_TARGET_FWD_MAX = 60.0


class TestFinite:
    """_finite 辅助函数。"""

    def test_normal_values(self):
        assert _finite(1.0, 2.0, 3.0) is True

    def test_nan_rejected(self):
        assert _finite(float('nan'), 1.0) is False

    def test_inf_rejected(self):
        assert _finite(float('inf'), 1.0) is False

    def test_none_rejected(self):
        assert _finite(None, 1.0) is False

    def test_empty_returns_true(self):
        assert _finite() is True


class TestIngestion:
    """ingest_pose / ingest_v / ingest_cls / get_cls / has_target。"""

    @pytest.fixture
    def layer(self):
        return PerceptionLayer()

    def test_ingest_pose_creates_target(self, layer):
        """ingest_pose 后 has_target 返回 True。"""
        layer.ingest_pose(tid=2, x=10.0, y=0.0, yaw=0.0, now=1.0)
        assert layer.has_target(2)

    def test_ingest_pose_nan_rejected(self, layer):
        """NaN 坐标 → 不创建目标。"""
        layer.ingest_pose(tid=2, x=float('nan'), y=0.0, yaw=0.0, now=1.0)
        assert not layer.has_target(2)

    def test_ingest_pose_inf_rejected(self, layer):
        """inf 坐标 → 不创建目标。"""
        layer.ingest_pose(tid=2, x=float('inf'), y=0.0, yaw=0.0, now=1.0)
        assert not layer.has_target(2)

    def test_ingest_pose_none_rejected(self, layer):
        """None 坐标 → 不创建目标。"""
        layer.ingest_pose(tid=2, x=None, y=0.0, yaw=0.0, now=1.0)
        assert not layer.has_target(2)

    def test_ingest_v_creates_target(self, layer):
        """ingest_v 也能创建目标条目。"""
        layer.ingest_v(tid=3, v=15.0)
        assert layer.has_target(3)

    def test_ingest_v_nan_rejected(self, layer):
        """NaN 速度 → 不更新。"""
        layer.ingest_v(tid=3, v=float('nan'))
        assert not layer.has_target(3)

    def test_ingest_cls_valid(self, layer):
        """有效 cls 值被接受。"""
        layer.ingest_cls(tid=2, cls=1)
        assert layer.get_cls(2) == 1

    def test_ingest_cls_zero(self, layer):
        """cls=0（UNKNOWN）也是有效值。"""
        layer.ingest_cls(tid=2, cls=0)
        assert layer.get_cls(2) == 0

    def test_ingest_cls_negative_rejected(self, layer):
        """负 cls → 拒绝，保持默认 UNKNOWN=0。"""
        layer.ingest_cls(tid=2, cls=-1)
        assert layer.get_cls(2) == 0  # ACTOR_CLASS_UNKNOWN

    def test_ingest_cls_over_255_rejected(self, layer):
        """cls>255 → 拒绝。"""
        layer.ingest_cls(tid=2, cls=256)
        assert layer.get_cls(2) == 0

    def test_ingest_cls_string_accepted(self, layer):
        """字符串 cls 能被 int() 转换则接受。"""
        layer.ingest_cls(tid=2, cls="3")
        assert layer.get_cls(2) == 3

    def test_ingest_cls_invalid_string_rejected(self, layer):
        """无法转 int 的字符串 → 拒绝。"""
        layer.ingest_cls(tid=2, cls="abc")
        assert layer.get_cls(2) == 0

    def test_get_cls_unknown_tid(self, layer):
        """查询不存在的 tid → ACTOR_CLASS_UNKNOWN=0。"""
        assert layer.get_cls(99) == 0

    def test_has_target_nonexistent(self, layer):
        """不存在的 tid → False。"""
        assert not layer.has_target(99)


class TestBuildFrame:
    """build_frame 感知帧构建。"""

    @pytest.fixture
    def layer(self):
        return PerceptionLayer()

    def _ingest(self, layer, tid, x, y, yaw, v, now, cls=0):
        """便捷注入一个完整目标。"""
        layer.ingest_pose(tid, x, y, yaw, now)
        layer.ingest_v(tid, v)
        layer.ingest_cls(tid, cls)

    def test_no_targets_empty_frame(self, layer):
        """无任何目标 → 空 tracks，primary_tid=None。"""
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=1.0,
        )
        assert isinstance(frame, PerceptionFrame)
        assert len(frame.tracks) == 0
        assert frame.primary_tid is None
        assert frame.n_fresh == 0

    def test_single_target_in_frame(self, layer):
        """单目标注入 → 出现在 frame.tracks 中。"""
        now = 1.0
        self._ingest(layer, tid=2, x=20.0, y=0.0, yaw=0.0, v=10.0, now=now)
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=now,
        )
        assert 2 in frame.tracks
        tr = frame.tracks[2]
        assert isinstance(tr, TrackRel)
        assert tr.tid == 2
        assert tr.fresh is True
        assert abs(tr.x_rel - 20.0) < 0.1

    def test_single_target_is_primary(self, layer):
        """唯一 fresh + 前向 + in_lane 目标 → 成为主前车。"""
        now = 1.0
        self._ingest(layer, tid=2, x=20.0, y=0.0, yaw=0.0, v=10.0, now=now)
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=now,
        )
        assert frame.primary_tid == 2

    def test_stale_target_not_fresh(self, layer):
        """超时目标 → fresh=False。"""
        now_old = 1.0
        now_new = 1.0 + LEAD_TIMEOUT_S + 0.1
        self._ingest(layer, tid=2, x=20.0, y=0.0, yaw=0.0, v=10.0, now=now_old)
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=now_new,
        )
        assert 2 in frame.tracks
        assert frame.tracks[2].fresh is False

    def test_stale_target_not_primary(self, layer):
        """超时目标不参与主前车选举。"""
        now_old = 1.0
        now_new = 1.0 + LEAD_TIMEOUT_S + 0.1
        self._ingest(layer, tid=2, x=20.0, y=0.0, yaw=0.0, v=10.0, now=now_old)
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=now_new,
        )
        assert frame.primary_tid is None
        assert frame.n_fresh == 0

    def test_behind_target_not_primary(self, layer):
        """后方目标（x_rel < FWD_MIN）不选为主前车。"""
        now = 1.0
        # ego_yaw=0 → x 轴正方向为前方；目标 x=-5 在后方
        self._ingest(layer, tid=2, x=-5.0, y=0.0, yaw=0.0, v=10.0, now=now)
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=now,
        )
        assert frame.primary_tid is None

    def test_nearest_forward_selected(self, layer):
        """多个 in_lane 目标 → 选最近的。"""
        now = 1.0
        self._ingest(layer, tid=2, x=30.0, y=0.0, yaw=0.0, v=10.0, now=now)
        self._ingest(layer, tid=3, x=15.0, y=0.0, yaw=0.0, v=10.0, now=now)
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=now,
        )
        # tid=3 更近 → 主前车
        assert frame.primary_tid == 3

    def test_lateral_out_of_lane_not_primary(self, layer):
        """横向偏移过大 → 不在 in_lane → 不选为主前车（除非是 car2 兜底）。"""
        now = 1.0
        # y=5.0 远超车道半宽 → 不在 in_lane
        self._ingest(layer, tid=3, x=20.0, y=5.0, yaw=0.0, v=10.0, now=now)
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=now,
        )
        # tid=3 不是 car2 且不在 in_lane → 不选
        assert frame.primary_tid is None

    def test_car2_fallback_when_no_in_lane(self, layer):
        """无 in_lane 目标时 car2（fresh + 前向）作为兜底。"""
        now = 1.0
        # car2 在车道边缘（不算 in_lane 但是 car2 兜底候选）
        # y=2.0 可能超出 in_lane 但仍在前向范围
        self._ingest(layer, tid=2, x=20.0, y=2.0, yaw=0.0, v=10.0, now=now)
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=now,
        )
        # car2 兜底：forward_ok=True 的 car2 总是候选
        # 即使不在 in_lane，car2 仍可作为兜底
        assert frame.primary_tid == 2

    def test_n_fresh_count(self, layer):
        """n_fresh 统计 fresh 目标数。"""
        now = 1.0
        now_old = now - LEAD_TIMEOUT_S - 1.0
        self._ingest(layer, tid=2, x=20.0, y=0.0, yaw=0.0, v=10.0, now=now)
        self._ingest(layer, tid=3, x=30.0, y=0.0, yaw=0.0, v=10.0, now=now)
        self._ingest(layer, tid=4, x=40.0, y=0.0, yaw=0.0, v=10.0, now=now_old)
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=now,
        )
        # tid=2,3 fresh; tid=4 stale
        assert frame.n_fresh == 2

    def test_frame_metadata(self, layer):
        """frame 保存 ego 位姿和参数。"""
        now = 2.5
        frame = layer.build_frame(
            ego_x=1.0, ego_y=2.0, ego_yaw=0.3,
            lane_width=4.0, filtered_curv=0.01, now=now,
        )
        assert frame.now == now
        assert frame.ego_x == 1.0
        assert frame.ego_y == 2.0
        assert frame.ego_yaw == 0.3
        assert frame.lane_width == 4.0
        assert frame.filtered_curv == 0.01

    def test_track_cls_transmitted(self, layer):
        """目标 cls 值透传到 TrackRel。"""
        now = 1.0
        self._ingest(layer, tid=2, x=20.0, y=0.0, yaw=0.0, v=10.0,
                     now=now, cls=3)
        frame = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=now,
        )
        assert frame.tracks[2].cls == 3

    def test_two_calls_same_target_updates(self, layer):
        """两次 build_frame 对同一目标，第二次会更新滤波状态。"""
        t1 = 1.0
        self._ingest(layer, tid=2, x=20.0, y=0.0, yaw=0.0, v=10.0, now=t1)
        frame1 = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=t1,
        )
        t2 = t1 + 0.01
        layer.ingest_pose(2, x=19.9, y=0.0, yaw=0.0, now=t2)
        frame2 = layer.build_frame(
            ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lane_width=3.8, filtered_curv=0.0, now=t2,
        )
        # 第二帧 x_rel 应接近 19.9（低通滤波后）
        assert abs(frame2.tracks[2].x_rel - 19.9) < 0.5
