#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AI 智能监控分析器（自包含，纯标准库）。

把边缘计算的 5s 滑窗 KPI + 事件流，周期性转成「自然语言评估 + 调参建议」：
  • 默认走内置规则分析（零依赖，离线可用，演示稳定）。
  • 若检测到本地 Ollama（默认 http://127.0.0.1:11434，模型 qwen2.5:3b）可达，
    则调用大模型给出更自然的总评与建议；任何异常自动回退到规则分析。

设计参考根仓库 ollama模型调用/analyzer.py 的协议与判读经验，裁剪为自包含线程版。
建议只能从下方 PARAM_TABLE（非安全关键、带允许范围）中选择，越界丢弃。
"""

import json
import os
import threading
import time
import urllib.request
from collections import deque

OLLAMA_URL = os.environ.get('OLLAMA_URL', 'http://127.0.0.1:11434')
OLLAMA_MODEL = os.environ.get('OLLAMA_MODEL', 'qwen2.5:3b')

# 可调参数表（param | 当前参考值 | [min,max] | 单位 | 说明）——仅非安全关键项
PARAM_TABLE = [
    {'name': 'ACC_TIME_GAP', 'value': 2.0, 'min': 1.2, 'max': 3.0, 'unit': 's', 'desc': 'ACC 跟车时距，增大更安全更保守'},
    {'name': 'SYSTEM_MAX_CRUISE', 'value': 10.0, 'min': 5.0, 'max': 16.0, 'unit': 'm/s', 'desc': '系统最高巡航速度'},
    {'name': 'LAT_OUTPUT_ALPHA', 'value': 0.50, 'min': 0.2, 'max': 0.8, 'unit': '', 'desc': '横向输出平滑系数，减小更平稳抗画龙'},
    {'name': 'LON_OUTPUT_ALPHA', 'value': 0.25, 'min': 0.1, 'max': 0.6, 'unit': '', 'desc': '纵向输出平滑系数，减小降低顿挫'},
    {'name': 'CORNERING_MAX_LAT_ACCEL', 'value': 2.2, 'min': 1.5, 'max': 3.0, 'unit': 'm/s²', 'desc': '弯道横向加速度上限，减小过弯更稳'},
]
_PARAM_BY_NAME = {p['name']: p for p in PARAM_TABLE}

SYSTEM_PROMPT = (
    "你是 ADAS（LKA 车道保持 / ACC 自适应巡航 / AEB 紧急制动）双冗余系统的运行质量分析专家。"
    "用户给你车端边缘计算 5s 滑窗 KPI 与事件流，你负责：判断风险等级；指出具体问题；"
    "必要时从「可调参数表」中给出调参建议（值必须落在允许范围内，没把握就不建议）。"
    "判读参考：TTC<4s 偏危险、<2.5s 危急；车道偏移 RMS>0.3m 偏大；jerk RMS 偏高舒适性差；"
    "出现 AEB/主备接管/看门狗事件需重点说明。"
    "必须只输出严格 JSON：{\"risk_level\":\"normal|warning|critical\",\"summary\":\"一句话中文\","
    "\"findings\":[\"…\"],\"adjustments\":[{\"param\":\"…\",\"value\":数值,\"reason\":\"…\"}]}。")


def _validate(name, value):
    p = _PARAM_BY_NAME.get(name)
    if p is None:
        return False, '未知参数'
    try:
        v = float(value)
    except (TypeError, ValueError):
        return False, '非数值'
    if not (p['min'] <= v <= p['max']):
        return False, '超出范围 [%g, %g]' % (p['min'], p['max'])
    return True, v


# ── 内置 Ollama 极简客户端（urllib，无第三方依赖）──
def _ollama_chat_json(system, user, timeout=30.0):
    body = json.dumps({
        'model': OLLAMA_MODEL,
        'messages': [{'role': 'system', 'content': system},
                     {'role': 'user', 'content': user}],
        'stream': False,
        'format': 'json',
        'options': {'temperature': 0.2},
    }).encode('utf-8')
    req = urllib.request.Request(OLLAMA_URL.rstrip('/') + '/api/chat',
                                 data=body, headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        data = json.loads(resp.read().decode('utf-8'))
    content = (data.get('message') or {}).get('content', '')
    return json.loads(content)


def _ollama_alive(timeout=1.0):
    try:
        with urllib.request.urlopen(OLLAMA_URL.rstrip('/') + '/api/tags', timeout=timeout):
            return True
    except Exception:
        return False


# ── 规则分析（默认，离线可用）──
def rule_analyze(kpis, events, counts):
    findings = []
    risk = 'normal'
    adj = []

    min_ttc = kpis.get('min_ttc_s')
    cte = kpis.get('cte_rms_m')
    jerk = kpis.get('jerk_rms')
    hb = kpis.get('hard_brake_count', 0)
    aeb = counts.get('aeb_activation', 0)
    ped = counts.get('pedestrian_critical', 0) + counts.get('pedestrian_warning', 0)
    fo = counts.get('failover', 0)

    if min_ttc is not None and min_ttc < 2.5:
        risk = 'critical'
        findings.append('最小 TTC 仅 %.1fs，跟车过近、碰撞风险高' % min_ttc)
        adj.append(('ACC_TIME_GAP', 2.4, '增大跟车时距以拉开安全距离'))
    elif min_ttc is not None and min_ttc < 4.0:
        risk = max(risk, 'warning', key=lambda x: {'normal': 0, 'warning': 1, 'critical': 2}[x])
        findings.append('最小 TTC %.1fs 偏低，建议适当保守' % min_ttc)

    if aeb > 0:
        risk = 'critical'
        findings.append('窗口内发生 %d 次 AEB 紧急制动' % aeb)
    if ped > 0:
        risk = 'critical' if risk != 'critical' else risk
        findings.append('检测到行人风险事件，已制动避让')
    if fo > 0:
        findings.append('发生主备接管事件，冗余机制已生效')
        if risk == 'normal':
            risk = 'warning'

    if cte is not None and cte > 0.3:
        if risk == 'normal':
            risk = 'warning'
        findings.append('车道偏移 RMS %.2fm 偏大' % cte)
        adj.append(('LAT_OUTPUT_ALPHA', 0.40, '减小横向平滑系数，抑制画龙'))
    if jerk is not None and jerk > 6.0:
        if risk == 'normal':
            risk = 'warning'
        findings.append('急动度偏高，纵向不够平顺')
        adj.append(('LON_OUTPUT_ALPHA', 0.20, '减小纵向平滑系数，降低顿挫'))
    if hb and hb > 0:
        findings.append('窗口内 %d 次急刹' % hb)

    summary = {
        'normal': '运行平稳，各项指标正常，无明显风险。',
        'warning': '总体可控，存在需关注的指标，建议适度保守。',
        'critical': '出现高风险事件，安全机制已介入，请重点复盘。',
    }[risk]
    if not findings:
        findings.append('车道居中良好，跟车与制动表现正常。')

    adjustments = []
    seen = set()
    for name, val, reason in adj:
        if name in seen:
            continue
        seen.add(name)
        ok, v = _validate(name, val)
        adjustments.append({'param': name, 'value': v if ok else val,
                            'reason': reason, 'valid': ok})
    return {'risk_level': risk, 'summary': summary,
            'findings': findings[:4], 'adjustments': adjustments}


def _build_user_prompt(kpis, events, counts):
    def n(x, s=''):
        return ('%.2f%s' % (x, s)) if isinstance(x, (int, float)) else '无'
    lines = ['## 当前 5s 滑窗 KPI',
             '速度均值%s 峰值%s | 最小车距%s 最小TTC%s | 车道偏移RMS%s | jerkRMS%s | 急刹%d AEB%d 风险事件%d' % (
                 n(kpis.get('avg_speed_kmh'), 'km/h'), n(kpis.get('max_speed_kmh'), 'km/h'),
                 n(kpis.get('min_gap_m'), 'm'), n(kpis.get('min_ttc_s'), 's'),
                 n(kpis.get('cte_rms_m'), 'm'), n(kpis.get('jerk_rms')),
                 kpis.get('hard_brake_count', 0), kpis.get('aeb_count', 0),
                 kpis.get('risk_event_count', 0))]
    if events:
        lines.append('## 最近事件')
        lines += ['- %s (t=%.1fs, %s)' % (e.get('label', e.get('type')), e.get('t', 0), e.get('severity'))
                  for e in events[:6]]
    lines.append('## 可调参数表（param | 当前 | 范围 | 说明）')
    for p in PARAM_TABLE:
        lines.append('%s | %g%s | [%g,%g] | %s' % (p['name'], p['value'], p['unit'], p['min'], p['max'], p['desc']))
    lines.append('请按约定 JSON 输出。')
    return '\n'.join(lines)


def _sanitize_llm(raw):
    risk = str(raw.get('risk_level', 'normal')).lower()
    if risk not in ('normal', 'warning', 'critical'):
        risk = 'normal'
    findings = [str(x) for x in (raw.get('findings') or [])][:4]
    adjustments = []
    seen = set()
    for a in (raw.get('adjustments') or []):
        if not isinstance(a, dict):
            continue
        name = str(a.get('param', ''))
        if name in seen:
            continue
        seen.add(name)
        ok, v = _validate(name, a.get('value'))
        adjustments.append({'param': name, 'value': v if ok else a.get('value'),
                            'reason': str(a.get('reason', ''))[:120], 'valid': ok})
    return {'risk_level': risk, 'summary': str(raw.get('summary', ''))[:200] or '（无总评）',
            'findings': findings or ['（模型未给出发现）'], 'adjustments': adjustments}


class AiAnalyzer(object):
    """后台线程：周期性产出 AI 分析，存最新结果 + 历史，供 Web 读取。"""

    def __init__(self, get_edge_snapshot, interval_s=12.0, history=8):
        self._get_edge = get_edge_snapshot
        self.interval_s = float(interval_s)
        self._lock = threading.Lock()
        self._latest = None
        self._history = deque(maxlen=history)
        self.backend = 'rule'
        self.model = OLLAMA_MODEL
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name='ai-analyzer')

    def start(self):
        self._thread.start()

    def _loop(self):
        # 启动时探测一次 Ollama
        use_ollama = _ollama_alive()
        self.backend = 'ollama' if use_ollama else 'rule'
        while not self._stop.is_set():
            try:
                self._analyze_once()
            except Exception:
                pass
            self._stop.wait(self.interval_s)

    def _analyze_once(self):
        snap = self._get_edge() or {}
        summary = snap.get('summary') or {}
        kpis = dict(summary.get('kpis') or {})
        kpis.setdefault('risk_event_count', 0)
        events = snap.get('recent_events') or []
        counts = snap.get('event_type_counts') or {}
        t0 = time.monotonic()
        result = None
        backend = 'rule'
        if self.backend == 'ollama':
            try:
                raw = _ollama_chat_json(SYSTEM_PROMPT, _build_user_prompt(kpis, events, counts))
                result = _sanitize_llm(raw)
                backend = 'ollama'
            except Exception:
                result = None
        if result is None:
            result = rule_analyze(kpis, events, counts)
            backend = 'rule'
        result['backend'] = backend
        result['model'] = self.model if backend == 'ollama' else '内置规则引擎'
        result['elapsed_s'] = round(time.monotonic() - t0, 2)
        result['ts'] = round(time.time(), 1)
        with self._lock:
            self._latest = result
            self._history.appendleft({'ts': result['ts'], 'risk_level': result['risk_level'],
                                      'summary': result['summary']})

    def snapshot(self):
        with self._lock:
            return {'latest': self._latest, 'history': list(self._history),
                    'backend': self.backend, 'model': self.model,
                    'params': PARAM_TABLE}

    def stop(self):
        self._stop.set()
