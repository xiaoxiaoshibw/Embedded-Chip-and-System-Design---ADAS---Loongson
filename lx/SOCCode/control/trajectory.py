#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""规划→控制接口（planning→control interface）—— 对标 Apollo `ControlCommand` /
Autoware `trajectory_follower`。

诊断建议把“控制意图”从“最终可执行命令”中显式分离。本模块用两个结构表达控制意图：
横向 `TargetPath`（期望横向偏移 + 参考曲率），纵向 `SpeedProfile`（期望速度）。

当前控制是反应式的（直接吃车道线误差 + 前车），本模块提供其**只读结构化投影**
（与 `aeb_overlay` 同思路）——确立 planning→control 接口边界、让“控制意图”可观测/可取证；
未来 planner 可填充更丰富的多点 path / 速度序列，控制层只读本结构即可。**不改任何控制数值。**

Python 3.6 兼容（JetPack 4.x）。中文注释，遵循 SOCCode 约定。
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class TargetPath:
    """横向规划目标（planning→control 接口·横向）。"""
    lateral_offset_target: float = 0.0   # 期望横向偏移 (m)，含超车借道 target_lane_offset
    curvature: float = 0.0               # 参考路径曲率 (1/m)
    source: str = "reactive"             # reactive（当前反应式投影）/ planner（未来）


@dataclass(frozen=True)
class SpeedProfile:
    """纵向规划目标（planning→control 接口·纵向）。"""
    target_speed: float = 0.0            # 期望速度 (m/s)
    source: str = "reactive"


def summarize_target(target_lane_offset, curvature, set_speed):
    """从当前反应式控制状态投影出 planning→control 接口结构（只读，不改控制）。

    - TargetPath：横向期望偏移（含超车借道）+ 参考曲率；
    - SpeedProfile：纵向期望速度（巡航设定）。
    """
    path = TargetPath(
        lateral_offset_target=float(target_lane_offset),
        curvature=float(curvature),
        source="reactive")
    speed = SpeedProfile(target_speed=float(set_speed), source="reactive")
    return path, speed
