"""边缘计算引擎（自包含，纯标准库）。

在 ADAS 主控进程内**就近**对每帧运行信息做边缘计算：
  1. 逐帧特征：相对/接近速度、TTC、DRAC（避撞所需减速度）、THW（车头时距）、
     加速度（缺省由速度差分得到）、jerk（加加速度）。
  2. 事件检测：TTC 预警/危险、急刹、AEB 触发、车道偏离、行人危险、边界制动、
     大 jerk、数据质量异常。
  3. 5s 滑动窗口 KPI：min_ttc / min_gap / 均速 / 最高速 / CTE-RMS / jerk-RMS /
     急刹次数 / AEB 次数 / 风险事件数，并据此做窗口级风险分级。
  4. "上云"：按固定间隔把窗口 KPI 摘要写入 output/cloud_outbox/*.jsonl，
     模拟边缘节点向云端上报（离线可查、可回放）。

零第三方依赖。本文件原为统一网站从已删除的「主控」包内联自带，使网站自包含可独立运行。
"""

import json
import math
import os
import threading
import time


EPS = 1e-6

# ── 事件阈值（与根仓库边缘计算模块保持一致量级）──
FOLLOW_WARNING_TTC_S = 5.0
FOLLOW_CRITICAL_TTC_S = 2.5
HARD_BRAKE_ACCEL = -3.5          # m/s²，纵向减速超过此值算急刹
LANE_WARNING_OFFSET_M = 0.55
LANE_CRITICAL_OFFSET_M = 0.90
JERK_WARNING = 8.0               # m/s³
BAD_DATA_WARNING_RATIO = 0.20

# 事件类型 → 中文标签（前端展示用）
EVENT_LABELS = {
    'ttc_warning': 'TTC 预警',
    'ttc_critical': 'TTC 危险',
    'hard_brake': '急刹车',
    'aeb_activation': 'AEB 触发',
    'aeb_active': 'AEB 制动中',
    'lane_offset_warning': '车道偏移预警',
    'lane_departure': '车道偏离',
    'pedestrian_warning': '行人预警',
    'pedestrian_critical': '行人危险',
    'boundary_brake': '边界制动',
    'jerk_warning': '顿挫(大 jerk)',
    'overtake': '超车中',
    'failover': '主备接管',
    'data_quality': '数据质量异常',
}

RISK_RANK = {'normal': 0, 'warning': 1, 'critical': 2}


def _is_finite(v):
    return v is not None and isinstance(v, (int, float)) and math.isfinite(v)


def _round(v, ndigits=3):
    if not _is_finite(v):
        return None
    return round(float(v), ndigits)


def _min_finite(values):
    items = [v for v in values if _is_finite(v)]
    return min(items) if items else None


def _max_finite(values):
    items = [v for v in values if _is_finite(v)]
    return max(items) if items else None


def _avg_finite(values):
    items = [v for v in values if _is_finite(v)]
    return sum(items) / float(len(items)) if items else None


def _rms(values):
    items = [v for v in values if _is_finite(v)]
    if not items:
        return None
    return math.sqrt(sum(v * v for v in items) / float(len(items)))


class _Feature(object):
    """单帧计算结果。"""
    __slots__ = ('t', 'ego_v', 'gap', 'lane_offset', 'accel', 'jerk',
                 'ttc', 'closing', 'drac', 'thw', 'events', 'risk')

    def __init__(self, t):
        self.t = t
        self.ego_v = None
        self.gap = None
        self.lane_offset = None
        self.accel = None
        self.jerk = None
        self.ttc = None
        self.closing = None
        self.drac = None
        self.thw = None
        self.events = []
        self.risk = 'normal'


class EdgeEngine(object):
    """ADAS 进程内的边缘计算引擎（线程安全读取）。"""

    def __init__(self, window_s=5.0, emit_interval_s=1.0,
                 outbox_dir=None, max_recent_events=40):
        self.window_s = float(window_s)
        self.emit_interval_s = float(emit_interval_s)
        self.max_recent_events = int(max_recent_events)

        self._lock = threading.Lock()
        self._features = []          # 滑动窗口内的 _Feature
        self._prev = None            # 上一帧 _Feature
        self._recent_events = []     # 最近事件（前端事件流）
        self._latest_summary = None  # 最新窗口摘要 dict
        self._last_emit_t = None

        # 计数器
        self.total_events = 0
        self.cloud_uploads = 0
        self.event_type_counts = {}

        # "上云" 输出文件
        self._fh = None
        self._outbox_path = None
        if outbox_dir:
            try:
                os.makedirs(outbox_dir, exist_ok=True)
                name = 'edge_%d.jsonl' % int(time.time())
                self._outbox_path = os.path.join(outbox_dir, name)
                self._fh = open(self._outbox_path, 'w', encoding='utf-8')
            except Exception:
                self._fh = None
                self._outbox_path = None

    # ── 逐帧输入 ──
    def feed(self, t, ego_v, gap, lead_v, lead_detected,
             lane_offset=None, accel=None, aeb_active=False,
             ped_warn=False, ped_ttc=None, boundary_brake=0.0,
             overtake_active=False, failover_src=0):
        """喂入一帧运行信息，返回该帧 _Feature（含事件）。"""
        f = _Feature(t)
        f.ego_v = ego_v if _is_finite(ego_v) else None
        f.lane_offset = lane_offset if _is_finite(lane_offset) else None
        f.gap = gap if (lead_detected and _is_finite(gap)) else None

        # 接近速度 / TTC / DRAC / THW
        if lead_detected and _is_finite(ego_v) and _is_finite(lead_v):
            closing = max(ego_v - lead_v, 0.0)
            f.closing = closing
            if _is_finite(gap) and gap > EPS:
                if closing > EPS:
                    f.ttc = gap / closing
                    f.drac = (closing * closing) / max(2.0 * gap, EPS)
                if ego_v > EPS:
                    f.thw = gap / ego_v

        # 加速度（外部未给则由速度差分）
        a = accel
        if not _is_finite(a) and self._prev is not None and \
                _is_finite(ego_v) and _is_finite(self._prev.ego_v):
            dt = t - self._prev.t
            if dt > EPS:
                a = (ego_v - self._prev.ego_v) / dt
        f.accel = a if _is_finite(a) else None

        # jerk
        if _is_finite(f.accel) and self._prev is not None and \
                _is_finite(self._prev.accel):
            dt = t - self._prev.t
            if dt > EPS:
                f.jerk = (f.accel - self._prev.accel) / dt

        # 事件检测
        f.events = self._detect_events(
            f, aeb_active, ped_warn, ped_ttc, boundary_brake,
            overtake_active, failover_src)
        f.risk = self._classify(f.events)

        with self._lock:
            self._features.append(f)
            self._prune(t)
            for ev in f.events:
                self.total_events += 1
                et = ev['type']
                self.event_type_counts[et] = self.event_type_counts.get(et, 0) + 1
                self._recent_events.append(ev)
            if len(self._recent_events) > self.max_recent_events:
                self._recent_events = self._recent_events[-self.max_recent_events:]
            self._prev = f

            # 按间隔生成窗口摘要 + 上云
            if self._last_emit_t is None or \
                    (t - self._last_emit_t) >= self.emit_interval_s:
                self._last_emit_t = t
                self._latest_summary = self._summarize_locked()
                self._upload(self._latest_summary)

        return f

    # ── 事件检测 ──
    def _detect_events(self, f, aeb_active, ped_warn, ped_ttc,
                       boundary_brake, overtake_active, failover_src):
        events = []

        if _is_finite(f.ttc):
            if f.ttc <= FOLLOW_CRITICAL_TTC_S:
                events.append(self._ev(f.t, 'ttc_critical', 'critical', f.ttc))
            elif f.ttc <= FOLLOW_WARNING_TTC_S:
                events.append(self._ev(f.t, 'ttc_warning', 'warning', f.ttc))

        if _is_finite(f.accel) and f.accel <= HARD_BRAKE_ACCEL:
            events.append(self._ev(f.t, 'hard_brake', 'warning', f.accel))

        if aeb_active:
            # 仅在上一帧未激活时记一次"触发"，避免每帧刷屏
            was = self._prev is not None and any(
                e['type'] in ('aeb_active', 'aeb_activation')
                for e in self._prev.events)
            etype = 'aeb_active' if was else 'aeb_activation'
            events.append(self._ev(f.t, etype, 'critical', 1.0))

        if ped_warn:
            sev = 'critical' if (_is_finite(ped_ttc) and ped_ttc < 2.5) else 'warning'
            etype = 'pedestrian_critical' if sev == 'critical' else 'pedestrian_warning'
            events.append(self._ev(f.t, etype, sev,
                                   ped_ttc if _is_finite(ped_ttc) else 1.0))

        if _is_finite(f.lane_offset):
            off = abs(f.lane_offset)
            if off >= LANE_CRITICAL_OFFSET_M:
                events.append(self._ev(f.t, 'lane_departure', 'critical', f.lane_offset))
            elif off >= LANE_WARNING_OFFSET_M:
                events.append(self._ev(f.t, 'lane_offset_warning', 'warning', f.lane_offset))

        if _is_finite(boundary_brake) and boundary_brake > 0.1:
            events.append(self._ev(f.t, 'boundary_brake', 'warning', boundary_brake))

        if _is_finite(f.jerk) and abs(f.jerk) >= JERK_WARNING:
            events.append(self._ev(f.t, 'jerk_warning', 'warning', f.jerk))

        if failover_src in (1, 9):
            was = self._prev is not None and any(
                e['type'] == 'failover' for e in self._prev.events)
            if not was:
                sev = 'critical' if failover_src == 9 else 'warning'
                events.append(self._ev(f.t, 'failover', sev, float(failover_src)))

        return events

    def _classify(self, events):
        if any(e['severity'] == 'critical' for e in events):
            return 'critical'
        if events:
            return 'warning'
        return 'normal'

    def _ev(self, t, etype, severity, value):
        return {
            't': round(float(t), 2),
            'type': etype,
            'label': EVENT_LABELS.get(etype, etype),
            'severity': severity,
            'value': _round(value),
        }

    # ── 窗口摘要 ──
    def _prune(self, now):
        cutoff = now - self.window_s
        while self._features and self._features[0].t < cutoff:
            self._features.pop(0)

    def _summarize_locked(self):
        feats = self._features
        if not feats:
            return None
        events = []
        for f in feats:
            events.extend(f.events)
        kpis = {
            'min_ttc_s': _round(_min_finite([f.ttc for f in feats])),
            'min_gap_m': _round(_min_finite([f.gap for f in feats])),
            'avg_speed_kmh': _round((_avg_finite([f.ego_v for f in feats]) or 0.0) * 3.6, 1),
            'max_speed_kmh': _round((_max_finite([f.ego_v for f in feats]) or 0.0) * 3.6, 1),
            'cte_rms_m': _round(_rms([f.lane_offset for f in feats])),
            'jerk_rms': _round(_rms([f.jerk for f in feats])),
            'hard_brake_count': sum(1 for e in events if e['type'] == 'hard_brake'),
            'aeb_count': sum(1 for e in events if e['type'] == 'aeb_activation'),
            'risk_event_count': len(events),
        }
        risk = 'normal'
        if any(e['severity'] == 'critical' for e in events):
            risk = 'critical'
        elif events:
            risk = 'warning'
        return {
            'window_start': round(feats[0].t, 2),
            'window_end': round(feats[-1].t, 2),
            'sample_count': len(feats),
            'kpis': kpis,
            'risk_level': risk,
        }

    # ── 上云 ──
    def _upload(self, summary):
        if summary is None or self._fh is None:
            return
        try:
            rec = dict(summary)
            rec['ts'] = time.time()
            self._fh.write(json.dumps(rec, ensure_ascii=False) + '\n')
            self._fh.flush()
            self.cloud_uploads += 1
        except Exception:
            pass

    # ── 对外快照（供 Web / 控制台读取，线程安全）──
    def snapshot(self):
        with self._lock:
            summary = self._latest_summary
            return {
                'summary': summary,
                'recent_events': list(reversed(self._recent_events[-12:])),
                'total_events': self.total_events,
                'cloud_uploads': self.cloud_uploads,
                'event_type_counts': dict(self.event_type_counts),
                'outbox': os.path.basename(self._outbox_path) if self._outbox_path else None,
            }

    def reset(self):
        """清空滑窗与事件流（切换回放场景时调用，使边缘计算立即反映新场景）。

        保留 total_events / cloud_uploads 等累计计数。"""
        with self._lock:
            self._features = []
            self._prev = None
            self._recent_events = []
            self._latest_summary = None
            self._last_emit_t = None

    def close(self):
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.close()
                except Exception:
                    pass
                self._fh = None
