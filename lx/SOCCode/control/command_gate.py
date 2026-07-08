#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""控制命令门控层（CommandGate）—— ESP32 下发前的唯一安全裁决点。

设计动机（对标 Autoware `control_command_gate` / Apollo `ControlValidator`）：
原先「发往 ESP32 的最终裁决」分散在 ADAS.py 多处——NaN 守护、串口范围 clamp、
安全回退帧构造（NaN / 急停 / 感知超时 / 锁步失配各一份）。本模块把这些收敛成
**一个权威**：任何一帧 ESP32 控制量，要么经 `evaluate()` 校验+限幅后下发，要么经
`build_safe_fallback_payload()` 构造安全回退帧下发——两条路都带 `reason`/`severity`，
便于结构化诊断、报告取证与“证明最终输出一定安全”。

**关键约束：默认行为与原 ADAS.py 内联逻辑字节级一致。**
- `evaluate()` 的 NaN 候选字段、clamp 区间、安全回退帧的硬编码值，逐一对齐原实现
  （`_find_nonfinite_tx_fields` / `_control_loop_impl` 1109-1115 / `_build_safe_fallback_payload`）。
- 横向最外层限幅（`lat_smooth`）现由 `filter_lateral()` 在 gate 内施加（★2：gate 为
  唯一横向限幅权威）。lat_smooth 是有状态对象，由 AdasNode 持有并传入，gate 只决定
  “以何速率限幅”——默认与原 `lat_smooth.update(...)` 字节级一致；扩展限幅
  （`GATE_FILTER_EXT_ENABLED`，默认关）开启才改变控制。纵向接管保险网
  （`_apply_takeover_guard`）仍在 AdasNode。
- `evaluate()` 仍只做无状态的“校验 + clamp + 分类”，不持任何状态。
- `reason` 仅为新增元数据（写遥测 / 日志），不影响下发字节。

Python 3.6 兼容（JetPack 4.x）。中文注释，遵循 SOCCode 约定。
"""

from dataclasses import dataclass
from typing import List, Tuple

from common import clamp, is_finite
from config import (
    LANE_DEFAULT_WIDTH,
    LON_CMD_MAX_BRAKE_DECEL,
    LON_CMD_MAX_DRIVE_ACCEL,
    TAKEOVER_DELTA_RATE,
)
try:
    from config import (
        GATE_FILTER_EXT_ENABLED,
        GATE_STEER_DIFF_MAX,
        GATE_STEER_DIFF_MAX_TRANSITION,
    )
except ImportError:  # 老 config 无这些项时回退到“扩展限幅关闭”（仍字节级一致）
    GATE_FILTER_EXT_ENABLED = False
    GATE_STEER_DIFF_MAX = 0.20944            # math.radians(12)
    GATE_STEER_DIFF_MAX_TRANSITION = 0.10472  # math.radians(6)
from control.serial_protocol import Esp32ControlFrame, build_esp32_payload


# ── reason 码：每帧 ESP32 输出的来由（结构化诊断 / 报告）──
GATE_NORMAL = "normal"                 # 常规控制下发
GATE_AEB_OVERRIDE = "aeb_override"     # 本帧 lon 由 AEB 路径产生
GATE_BACKUP_TAKEOVER = "backup_takeover"  # 处于主备接管保护窗
GATE_NAN_BLOCK = "nan_block"           # 控制量含 NaN/Inf → 拦截，走安全回退
GATE_SENSOR_STALE = "sensor_stale"     # 感知中断 → 安全轻制动
GATE_LOCKSTEP_FAULT = "lockstep_fault"  # 双核锁步失配 → 受控制动
GATE_CTRL_ERROR = "ctrl_error"         # 连续异常 → 紧急停车
GATE_STANDBY = "standby"               # 静默待机（无下发）

# ── 严重度：0 正常 / 1 告警 / 2 严重 ──
SEVERITY_NORMAL = 0
SEVERITY_WARN = 1
SEVERITY_CRITICAL = 2


@dataclass(frozen=True)
class GateDecision:
    """gate 对一帧控制量的裁决结果。"""
    blocked: bool                       # True=拦截常规帧，调用方须改走安全回退
    reason: str
    severity: int
    bad_fields: Tuple = ()              # blocked=True(NaN) 时列出非有限字段名
    # 以下为 clamp 后可下发标量，仅 blocked=False 时有效（与原内联 clamp 一致）
    ttc: float = 999.99
    dist: float = 0.0
    psi: float = 0.0
    delta: float = 0.0
    speed: float = 0.0
    lon: float = 0.0


class CommandGate:
    """ESP32 下发前的统一裁决器（无状态、可单测）。"""

    # 串口协议字段范围（集中常量，原先散在 ADAS.py 内联）
    TTC_FALLBACK = 999.99
    DIST_MAX = 999.99
    PSI_LIMIT = 9.9999
    DELTA_LIMIT = 9.9999
    SPEED_LIMIT = 99.99

    @staticmethod
    def nonfinite_fields(upd_psi, delta, cur_off, ego_v, lon_cmd, dist,
                         lead_v_proj, min_safe_dist, cur_lane_width,
                         lane_warn_margin, lane_hard_margin, filtered_curv):
        # type: (...) -> List[str]
        """返回所有将进入 ESP32 帧的原始标量中非有限（NaN/Inf）字段名。

        必须在 clamp 之前查源值：clamp(v,lo,hi)=max(lo,min(hi,v)) 对 NaN 返回上界，
        会把 NaN 静默放大成最大转角/最大制动。候选清单与原 `_find_nonfinite_tx_fields`
        逐字段一致；ttc 不在此列（inf=无前车正常哨兵，由 evaluate 单独落到 999.99）。
        """
        candidates = (
            ("upd_psi", upd_psi),
            ("delta", delta),
            ("cur_off", cur_off),
            ("ego_v", ego_v),
            ("lon_cmd", lon_cmd),
            ("dist", dist),
            ("lead_v_proj", lead_v_proj),
            ("min_safe_dist", min_safe_dist),
            ("cur_lane_width", cur_lane_width),
            ("lane_warn_margin", lane_warn_margin),
            ("lane_hard_margin", lane_hard_margin),
            ("filtered_curv", filtered_curv),
        )
        return [name for name, v in candidates if not is_finite(v)]

    def evaluate(self, upd_psi, delta, cur_off, ego_v, lon_cmd, ttc, dist,
                 lead_v_proj, min_safe_dist, cur_lane_width,
                 lane_warn_margin, lane_hard_margin, filtered_curv,
                 aeb_active=False, takeover_active=False):
        # type: (...) -> GateDecision
        """校验 + 限幅 + 分类。不改任何控制数值（与原 1099-1115 内联一致）。

        - 任一源字段非有限 → blocked=True, reason=nan_block（调用方走安全回退）。
        - 否则把 6 个串口字段 clamp 到约定区间，并按 aeb/takeover 分类 reason。
        """
        bad = self.nonfinite_fields(
            upd_psi, delta, cur_off, ego_v, lon_cmd, dist, lead_v_proj,
            min_safe_dist, cur_lane_width, lane_warn_margin,
            lane_hard_margin, filtered_curv)
        if bad:
            return GateDecision(
                blocked=True, reason=GATE_NAN_BLOCK,
                severity=SEVERITY_CRITICAL, bad_fields=tuple(bad))

        ttc_tx = ttc if is_finite(ttc) else self.TTC_FALLBACK
        dist_tx = clamp(dist, 0.0, self.DIST_MAX)
        psi_tx = clamp(upd_psi, -self.PSI_LIMIT, self.PSI_LIMIT)
        delta_tx = clamp(delta, -self.DELTA_LIMIT, self.DELTA_LIMIT)
        speed_tx = clamp(ego_v, -self.SPEED_LIMIT, self.SPEED_LIMIT)
        lon_tx = clamp(lon_cmd, -LON_CMD_MAX_DRIVE_ACCEL, LON_CMD_MAX_BRAKE_DECEL)

        # reason 分类：AEB 优先于接管（AEB 是更强的安全干预语义）
        if aeb_active:
            reason, severity = GATE_AEB_OVERRIDE, SEVERITY_WARN
        elif takeover_active:
            reason, severity = GATE_BACKUP_TAKEOVER, SEVERITY_WARN
        else:
            reason, severity = GATE_NORMAL, SEVERITY_NORMAL

        return GateDecision(
            blocked=False, reason=reason, severity=severity,
            ttc=ttc_tx, dist=dist_tx, psi=psi_tx, delta=delta_tx,
            speed=speed_tx, lon=lon_tx)

    def filter_lateral(self, delta, lat_smooth, takeover_active, esp_delta=None):
        # type: (float, object, bool, float) -> float
        """横向输出限幅权威（★2）：最外层坡度限速 + 一阶低通（接管窗收紧到
        TAKEOVER_DELTA_RATE），可选叠加 Autoware 式扩展限幅。

        阶段 A（默认）：仅 `lat_smooth.update` —— 与原 ADAS.py 内联
        `lat_smooth.update(delta, max_rate_override=...)` **字节级一致**（同一 lat_smooth
        实例、同一调用、同一顺序）。
        阶段 B（`GATE_FILTER_EXT_ENABLED`，默认关）：叠加 steer_cmd_diff_from_current
        等新限幅 —— **开启会改变控制数值**，须实机回归确认后再开。

        lat_smooth 是有状态对象（由 AdasNode 持有并传入）；gate 只决定“以何速率限幅”。
        """
        lat_rate = TAKEOVER_DELTA_RATE if takeover_active else None
        delta = lat_smooth.update(delta, max_rate_override=lat_rate)
        if GATE_FILTER_EXT_ENABLED:
            delta = self._filter_ext_lateral(delta, esp_delta, takeover_active)
        return delta

    @staticmethod
    def _filter_ext_lateral(delta, esp_delta, takeover_active):
        # type: (float, float, bool) -> float
        """Autoware steer_cmd_diff_lim_from_current_steer：限制指令与 ESP32 实际回读
        转角之差（nominal/transition 两套：接管/过渡期收紧），防执行器跟不上时指令
        大幅领先实际。仅 GATE_FILTER_EXT_ENABLED 生效。"""
        diff_max = (GATE_STEER_DIFF_MAX_TRANSITION if takeover_active
                    else GATE_STEER_DIFF_MAX)
        if esp_delta is not None and is_finite(esp_delta):
            delta = clamp(delta, esp_delta - diff_max, esp_delta + diff_max)
        return delta

    @staticmethod
    def build_safe_fallback_payload(lon_cmd):
        # type: (float) -> bytes
        """构造完全脱离 self.memory 的安全帧（所有值硬编码有限数）。

        NaN 守护 / 急停 / 感知超时 / 锁步失配四条路径共用此构造器，确保
        ESP32 解析不会因 'nan'/'inf' 字符串失败。与原 `_build_safe_fallback_payload`
        逐字段一致。
        """
        return build_esp32_payload(
            Esp32ControlFrame(
                ttc=999.99,
                dist=999.99,
                psi=0.0,
                delta=0.0,
                speed=0.0,
                lon=float(lon_cmd),
                offset=0.0,
                lead_v_proj=0.0,
                min_safe_dist=0.0,
                lane_warn_margin=LANE_DEFAULT_WIDTH * 0.5 * 0.6,
                lane_hard_margin=LANE_DEFAULT_WIDTH * 0.5 * 0.4,
                filtered_curv=0.0,
            )
        )
