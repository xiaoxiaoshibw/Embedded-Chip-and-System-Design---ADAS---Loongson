/* 🌈 萌驾舱 MoeDrive 前端：SPA 路由 + SSE 实时 + 报告渲染。零依赖、零 CDN。 */
'use strict';
const $ = (id) => document.getElementById(id);
const fmt = (v, d = 1) => (v === null || v === undefined || Number.isNaN(v)) ? '—' : Number(v).toFixed(d);
const RISK_CN = { normal: '正常', warning: '注意', critical: '危险' };
const RISK_CLS = { normal: '', warning: 'warn', critical: 'crit' };
const SRC_NAME = { 0: 'PRIMARY', 1: 'BACKUP', 9: 'WATCHDOG' };
const SRC_CLS = { 0: '', 1: 'backup', 9: 'watchdog' };

/* ── 导航 ── */
function showView(name) {
  document.querySelectorAll('.view').forEach(v => v.classList.toggle('active', v.dataset.view === name));
  document.querySelectorAll('.tab').forEach(t => t.classList.toggle('active', t.dataset.view === name));
  window.scrollTo({ top: 0, behavior: 'smooth' });
}
document.querySelectorAll('.tab').forEach(t => t.onclick = () => showView(t.dataset.view));
document.querySelectorAll('[data-goto]').forEach(b => b.onclick = () => showView(b.dataset.goto));

/* ── 趋势缓冲 ── */
const MAXPTS = 360, hist = { v: [], ttc: [] };
function setBar(id, r) { const e = $(id); if (e) e.style.width = (Math.max(0, Math.min(1, r || 0)) * 100).toFixed(1) + '%'; }

/* ── 主渲染 ── */
function render(s) {
  if (!s || !s.connected) {
    $('conn-dot').classList.remove('on');
    $('conn-text').textContent = s && s.error ? s.error : '等待数据…';
    return;
  }
  $('conn-dot').classList.add('on');
  $('conn-text').textContent = '已连接';

  const ego = s.ego || {}, lead = s.lead || {}, c = s.control || {};
  const src = s.failover_src ?? 0;
  const mode = s.mode || 'LKA';
  const risk = ((s.edge && s.edge.summary) || {}).risk_level || 'normal';

  // 顶栏 + 总览 live
  $('top-mode').textContent = mode;
  $('top-speed').textContent = Math.round(ego.v_kmh || 0);
  $('ov-mode').textContent = mode;
  $('ov-speed').textContent = Math.round(ego.v_kmh || 0);
  $('ov-risk').textContent = RISK_CN[risk];

  // 驾驶舱
  $('speed-kmh').textContent = Math.round(ego.v_kmh || 0);
  $('speed-ms').textContent = fmt(ego.v_ms, 1);
  setBar('bar-speed', (ego.v_kmh || 0) / 120);
  const mb = $('mode-badge'); mb.textContent = mode;
  mb.className = 'mode-badge ' + (mode.includes('AEB') ? 'aeb' : mode.includes('OVERTAKE') ? 'overtake' : mode.includes('ACC') ? 'acc' : '');
  const feats = ['LKA', 'ACC', 'AEB', 'OVERTAKE', 'PEDESTRIAN', 'FAILOVER', 'BOUNDARY'];
  const on = new Set(s.features || []);
  $('feature-chips').innerHTML = feats.map(f => `<span class="chip ${on.has(f) ? 'on' : ''}">${f}</span>`).join('');

  const lc = $('lead-card');
  if (lead.detected) {
    lc.classList.remove('empty');
    $('lead-gap').textContent = fmt(lead.gap, 1);
    const ttc = (lead.ttc !== null && lead.ttc < 99) ? lead.ttc : null;
    $('lead-ttc').textContent = ttc === null ? '∞' : fmt(ttc, 1) + ' s';
    $('lead-rel').textContent = fmt(lead.rel_speed, 2);
    $('lead-v').textContent = fmt(lead.lead_v, 2);
    setBar('bar-gap', (lead.gap || 0) / 60);
    setBar('bar-ttc', ttc === null ? 1 : Math.min(1, ttc / 15));
  } else lc.classList.add('empty');

  $('lane-offset').textContent = ((ego.lane_offset >= 0 ? '+' : '') + fmt(ego.lane_offset, 2)) + ' m';
  $('lane-width').textContent = fmt(ego.lane_width, 1);
  $('curv').textContent = fmt(ego.curvature, 4);
  const off = Math.max(-2, Math.min(2, ego.lane_offset || 0));
  $('ego-dot').style.left = (50 + off / 2 * 30) + '%';

  $('v-steer').textContent = fmt(c.steer, 2);
  $('v-throttle').textContent = fmt(c.throttle, 2);
  $('v-brake').textContent = fmt(c.brake, 2);
  const sf = $('bar-steer'), st = Math.max(-1, Math.min(1, c.steer || 0));
  sf.style.width = Math.abs(st) * 50 + '%'; sf.style.left = st >= 0 ? '50%' : (50 + st * 50) + '%';
  setBar('bar-throttle', c.throttle); setBar('bar-brake', c.brake);
  $('lon-cmd').textContent = fmt(c.lon_cmd, 2) + ' m/s²';
  $('lon-src').textContent = c.lon_src || '—';

  $('src').textContent = SRC_NAME[src] || '?';
  $('src-pill').className = 'src-chip ' + (SRC_CLS[src] || '');

  // 告警
  const al = [];
  if (s.aeb) al.push(['crit', '🛑 AEB 紧急制动']);
  if (s.ped && s.ped.warn) al.push(['warn', '🚶 行人横穿 · 制动避让']);
  if (src === 1) al.push(['warn', '🔁 主控失效 · 备机接管']);
  if (src === 9) al.push(['crit', '🐶 看门狗紧急制动']);
  if (s.overtake_state && s.overtake_state !== 'idle') al.push(['warn', '↩️ 超车中：' + s.overtake_state]);
  $('alert-row').innerHTML = al.map(([k, t]) => `<div class="alert ${k}">${t}</div>`).join('');

  renderEdge(s.edge || {});
  renderAi(s.ai);

  // 趋势
  hist.v.push(ego.v_kmh || 0);
  hist.ttc.push((lead.detected && lead.ttc < 99) ? lead.ttc : null);
  if (hist.v.length > MAXPTS) { hist.v.shift(); hist.ttc.shift(); }
  drawChart();
}

function renderEdge(edge) {
  const k = ((edge.summary || {}).kpis) || {};
  $('k-minttc').textContent = fmt(k.min_ttc_s, 1);
  $('k-mingap').textContent = fmt(k.min_gap_m, 1);
  $('k-avgspd').textContent = fmt(k.avg_speed_kmh, 0);
  $('k-maxspd').textContent = fmt(k.max_speed_kmh, 0);
  $('k-cte').textContent = fmt(k.cte_rms_m, 3);
  $('k-jerk').textContent = fmt(k.jerk_rms, 1);
  $('k-hb').textContent = k.hard_brake_count ?? 0;
  $('k-aeb').textContent = k.aeb_count ?? 0;
  $('k-ev').textContent = k.risk_event_count ?? 0;
  $('k-total').textContent = edge.total_events ?? 0;
  $('cloud-n').textContent = edge.cloud_uploads ?? 0;
  const risk = (edge.summary || {}).risk_level || 'normal';
  const rb = $('risk-badge'); rb.textContent = RISK_CN[risk]; rb.className = 'risk-badge ' + RISK_CLS[risk];
  const evs = edge.recent_events || [], list = $('event-list');
  list.innerHTML = evs.length ? evs.map(e => {
    const cls = RISK_CLS[e.severity === 'critical' ? 'critical' : e.severity === 'warning' ? 'warning' : 'normal'];
    const val = (e.value !== null && e.value !== undefined) ? `<span class="ev-val">${e.value}</span>` : '';
    return `<li class="${cls}"><span class="et">${e.label || e.type}</span>${val}<span class="ev-t">t=${fmt(e.t, 1)}s</span></li>`;
  }).join('') : '<li class="ev-empty">暂无风险事件 · 运行平稳 🌿</li>';
}

function renderAi(ai) {
  if (!ai) return;
  $('ai-backend').textContent = ai.backend === 'ollama' ? '🟢 Ollama ' + (ai.model || '') : '🧩 内置规则引擎';
  const l = ai.latest;
  if (!l) { $('ai-summary').textContent = '正在收集数据，首轮分析即将给出～'; return; }
  $('ai-summary').textContent = l.summary || '—';
  const rb = $('ai-risk'); rb.textContent = RISK_CN[l.risk_level] || '正常'; rb.className = 'risk-badge ' + RISK_CLS[l.risk_level];
  $('ai-findings').innerHTML = (l.findings || []).map(f => `<li>${f}</li>`).join('') || '<li>运行正常</li>';
  const adj = l.adjustments || [];
  $('ai-adjust').innerHTML = adj.length ? adj.map(a =>
    `<div class="adj ${a.valid ? '' : 'invalid'}"><b>${a.param} → ${a.value}</b>${a.valid ? '' : ' ⚠️越界'}<span class="adj-r">${a.reason || ''}</span></div>`
  ).join('') : '<div class="adj-empty">暂无建议 · 运行平稳 ✅</div>';
  $('ai-model').textContent = l.model || '—';
  $('ai-elapsed').textContent = fmt(l.elapsed_s, 2);
  const hl = ai.history || [], hist = $('ai-history');
  hist.innerHTML = hl.length ? hl.map(h =>
    `<li class="${RISK_CLS[h.risk_level]}"><span class="et">${RISK_CN[h.risk_level] || ''}</span> ${h.summary}<span class="ev-t">${tfmt(h.ts)}</span></li>`
  ).join('') : '<li class="ev-empty">暂无历史</li>';
}
function tfmt(ts) { try { return new Date(ts * 1000).toLocaleTimeString('zh-CN', { hour12: false }); } catch (_) { return ''; } }

/* ── 趋势图 ── */
const cv = $('chart'), ctx = cv.getContext('2d');
function drawChart() {
  const W = cv.width, H = cv.height, pad = 24;
  ctx.clearRect(0, 0, W, H);
  ctx.strokeStyle = 'rgba(176,123,240,.16)'; ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) { const y = pad + (H - 2 * pad) * i / 4; ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(W - pad, y); ctx.stroke(); }
  const plot = (arr, max, color) => {
    ctx.strokeStyle = color; ctx.lineWidth = 3; ctx.lineJoin = 'round'; ctx.beginPath();
    let started = false;
    arr.forEach((val, i) => {
      const x = pad + (W - 2 * pad) * i / Math.max(1, MAXPTS - 1);
      if (val === null) { started = false; return; }
      const y = H - pad - (H - 2 * pad) * Math.min(1, val / max);
      if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
    });
    ctx.stroke();
  };
  plot(hist.v, 120, '#43d9b8');
  plot(hist.ttc, 15, '#ffab33');
  ctx.fillStyle = 'rgba(154,138,163,.8)'; ctx.font = '11px sans-serif';
  ctx.fillText('120', 2, pad + 4); ctx.fillText('0', 8, H - pad + 4);
}

/* ── 报告渲染（拉取一次）── */
fetch('/api/report').then(r => r.json()).then(rp => {
  if (rp.title) { $('ov-title').textContent = rp.title; document.title = '🚗 ' + rp.title; }
  if (rp.subtitle) $('ov-subtitle').textContent = rp.subtitle;
  if (rp.intro) $('ov-intro').textContent = rp.intro;
  $('rp-meta').innerHTML = `<b>${rp.title || ''}</b> · ${rp.domain || ''} · ${rp.date || ''}<br>${rp.intro || ''}`;
  $('ov-stats').innerHTML = (rp.stats || []).map(s =>
    `<div class="stat"><div class="st-ic">${s.icon}</div><div class="st-v">${s.value}<small> ${s.unit}</small></div><div class="st-l">${s.label}</div><div class="st-sub">${s.sub}</div></div>`
  ).join('');
  $('rp-features').innerHTML = (rp.features || []).map(f =>
    `<div class="fr"><div class="fr-ic">${f.icon}</div><div class="fr-id">${f.id}</div><div class="fr-name">${f.name}</div><div class="fr-desc">${f.desc}</div></div>`
  ).join('');
  $('rp-nfr').innerHTML = (rp.nfr || []).map(n =>
    `<div class="nfr"><div class="nfr-top"><span class="nfr-id">${n.id}</span><span class="nfr-cat">${n.cat}</span></div><div class="nfr-req">${n.req}</div></div>`
  ).join('');
  $('rp-innov').innerHTML = (rp.innovations || []).map(i =>
    `<div class="innov"><span class="iv-tag">${i.tag || ''}</span><div class="iv-ic">${i.icon}</div><div class="iv-title">${i.title}</div><div class="iv-desc">${i.desc}</div></div>`
  ).join('');
}).catch(() => {});

/* ── 连接 SSE，失败回退轮询 ── */
function connect() {
  let poll = false;
  try {
    const es = new EventSource('/api/stream');
    es.onmessage = e => { try { render(JSON.parse(e.data)); } catch (_) {} };
    es.onerror = () => { if (!poll) { poll = true; es.close(); startPoll(); } };
  } catch (_) { startPoll(); }
}
function startPoll() { setInterval(() => fetch('/api/state').then(r => r.json()).then(render).catch(() => {}), 250); }

/* ── 历史数据仓库：按类型归类 + 日期 + KPI 摘要，自由选择回放（实时边缘处理）── */
let curScn = null;
function riskClass(r) { return r === 'critical' ? 'risk-crit' : (r === 'warning' ? 'risk-warn' : 'risk-ok'); }
function highlight(box) {
  box.querySelectorAll('.scn-card,.scn-btn').forEach(x => x.classList.toggle('on', x.dataset.scn === curScn));
}
function selectScn(id, box) {
  fetch('/api/select?scenario=' + encodeURIComponent(id)).then(r => r.json()).then(res => {
    if (res && res.ok) { curScn = res.current; highlight(box); }
  }).catch(() => {});
}
function kpiChips(k) {
  const f1 = (v) => Number(v).toFixed(1);
  return [
    (k.max_kmh != null) ? '最高 ' + Math.round(k.max_kmh) + ' km/h' : null,
    (k.min_gap_m != null) ? '最小车距 ' + f1(k.min_gap_m) + ' m' : null,
    (k.min_ttc_s != null) ? '最小 TTC ' + f1(k.min_ttc_s) + ' s' : null,
    (k.hard_brake) ? '急刹 ' + k.hard_brake + ' 次' : null,
    (k.aeb) ? 'AEB ' + k.aeb + ' 次' : null,
  ].filter(Boolean).map(s => `<span class="scn-kpi">${s}</span>`).join('');
}
function loadScenarios() {
  const box = $('scn-picker');
  if (!box) return;
  fetch('/api/scenarios').then(r => r.json()).then(d => {
    if (d && d.repo && (d.records || []).length) {
      curScn = d.current;
      const order = d.category_order || [], cn = d.category_cn || {}, recs = d.records || [];
      const byCat = {};
      recs.forEach(r => { (byCat[r.category] = byCat[r.category] || []).push(r); });
      const cats = order.filter(c => byCat[c]).concat(Object.keys(byCat).filter(c => order.indexOf(c) < 0));
      let html = `<div class="scn-head">🗂️ 历史数据仓库 · 共 ${recs.length} 条 · 点卡片回放（1:1 真实时间，边缘计算实时处理）</div>`;
      html += cats.map(c => {
        const items = byCat[c];
        const cards = items.map(r => `<button class="scn-card ${riskClass(r.risk)}${r.id === curScn ? ' on' : ''}" data-scn="${r.id}">
            <div class="scn-cn">${r.name}<span class="scn-risk">${r.risk_cn}</span></div>
            <div class="scn-meta">${r.date_str} · ${r.duration_s}s</div>
            <div class="scn-kpis">${kpiChips(r.kpi || {})}</div></button>`).join('');
        return `<div class="scn-group"><div class="scn-gtitle">${cn[c] || c}<span class="scn-gn">${items.length}</span></div><div class="scn-cards">${cards}</div></div>`;
      }).join('');
      box.innerHTML = html;
      box.style.display = 'block';
      box.querySelectorAll('.scn-card').forEach(b => b.onclick = () => selectScn(b.dataset.scn, box));
      return;
    }
    // 回退：扁平场景列表
    const list = (d && d.scenarios) || [];
    if (!list.length) { box.style.display = 'none'; return; }
    box.style.display = 'flex';
    curScn = d.current;
    box.innerHTML = '<span class="scn-label">🎬 回溯场景</span>' +
      list.map(s => `<button class="scn-btn${s.key === curScn ? ' on' : ''}" data-scn="${s.key}">${s.name}</button>`).join('');
    box.querySelectorAll('.scn-btn').forEach(b => b.onclick = () => selectScn(b.dataset.scn, box));
  }).catch(() => { box.style.display = 'none'; });
}

/* ── HMI 记录 / CARLA-HIL 试验监控：WebSocket + mock 演示 ── */
const HMI_WS_URL = 'ws://localhost:8765';
const HMI_MAXPTS = 120;
const HMI_MAX_EVENTS = 80;
const hmiHist = { t: [], ttc: [], speed: [], lateral: [], brake: [], steer: [] };
const hmiEvents = [];
let hmiWs = null;
let hmiMode = 'mock';
let hmiMockTimer = null;
let hmiReconnectTimer = null;
let hmiMockStart = Date.now();
let hmiLast = null;
let hmiLastEventAt = {};

function hmiClass(level) {
  return level === 'critical' ? 'crit' : level === 'warning' ? 'warn' : 'ok';
}
function hmiLevelText(level) {
  return level === 'critical' ? '高风险' : level === 'warning' ? '注意' : '记录';
}
function hmiSetText(id, text) {
  const el = $(id);
  if (el) el.textContent = text;
}
function hmiSetBadge(id, text, cls) {
  const el = $(id);
  if (!el) return;
  el.textContent = text;
  el.className = 'hmi-badge ' + cls;
}
function hmiMetricLevel(key, v) {
  if (key === 'ttc_s') return v <= 2.2 ? 'critical' : v <= 4.5 ? 'warning' : 'normal';
  if (key === 'target_distance_m') return v < 12 ? 'critical' : v < 25 ? 'warning' : 'normal';
  if (key === 'relative_speed_mps') return v > 7 ? 'critical' : v > 4 ? 'warning' : 'normal';
  if (key === 'lateral_error_m') return Math.abs(v) > .65 ? 'critical' : Math.abs(v) > .32 ? 'warning' : 'normal';
  if (key === 'heading_error_deg') return Math.abs(v) > 7 ? 'critical' : Math.abs(v) > 3 ? 'warning' : 'normal';
  if (key === 'ego_speed_kmh') return v > 115 ? 'warning' : 'normal';
  return 'normal';
}
function hmiStateClass(label, value, sample) {
  if (label === 'AEB状态' && ['BRAKE', 'HOLD'].includes(value)) return 'crit';
  if (label === 'AEB状态' && value === 'WARNING') return 'warn';
  if (label === 'ACC状态' && value === 'BRAKE') return 'warn';
  if (label === 'LKA状态' && value === 'WARNING') return 'warn';
  if (label === '主控状态' && value !== 'ACTIVE') return 'crit';
  if (label === '备控状态' && value === 'FAULT') return 'crit';
  if (label === '备控状态' && value === 'ACTIVE') return 'warn';
  if (label === 'MCU接管' && sample.mcu_takeover) return 'crit';
  return value === 'OFF' || value === 'false' ? '' : 'ok';
}
function hmiAddEvent(type, level, desc, sample) {
  const now = sample.time_s || 0;
  const key = type + level;
  if (hmiLastEventAt[key] !== undefined && now - hmiLastEventAt[key] < 2.5) return;
  hmiLastEventAt[key] = now;
  hmiEvents.unshift({
    time: now,
    type,
    level,
    desc,
    ttc: sample.ttc_s,
    distance: sample.target_distance_m,
    speed: sample.ego_speed_kmh
  });
  if (hmiEvents.length > HMI_MAX_EVENTS) hmiEvents.pop();
}
function hmiDetectEvents(sample) {
  if (!hmiLast) {
    if (sample.acc_state === 'FOLLOW') hmiAddEvent('ACC_FOLLOW', 'normal', 'ACC 进入跟车巡航状态', sample);
    if (sample.lka_state === 'ACTIVE') hmiAddEvent('LKA_ACTIVE', 'normal', 'LKA 保持车道居中控制', sample);
    return;
  }
  if (sample.aeb_state === 'WARNING' && hmiLast.aeb_state !== 'WARNING') hmiAddEvent('AEB_WARNING', 'warning', 'TTC 降低，AEB 发出预警', sample);
  if (sample.aeb_state === 'BRAKE' && hmiLast.aeb_state !== 'BRAKE') hmiAddEvent('AEB_BRAKE', 'critical', 'AEB 触发紧急制动', sample);
  if (sample.aeb_state === 'HOLD' && hmiLast.aeb_state !== 'HOLD') hmiAddEvent('AEB_HOLD', 'critical', 'AEB 保持制动状态', sample);
  if (sample.acc_state === 'FOLLOW' && hmiLast.acc_state !== 'FOLLOW') hmiAddEvent('ACC_FOLLOW', 'normal', 'ACC 切换至跟车控制', sample);
  if (sample.lka_state === 'ACTIVE' && hmiLast.lka_state !== 'ACTIVE') hmiAddEvent('LKA_ACTIVE', 'normal', 'LKA 激活并输出横向控制', sample);
  if (sample.lane_invasion && !hmiLast.lane_invasion) hmiAddEvent('LANE_INVASION', 'warning', '检测到车道入侵或越线风险', sample);
  if (sample.collision && !hmiLast.collision) hmiAddEvent('COLLISION', 'critical', '检测到碰撞事件', sample);
  if (sample.target_lost && !hmiLast.target_lost) hmiAddEvent('TARGET_LOST', 'warning', '目标车跟踪短暂丢失', sample);
  if (sample.main_controller_state === 'FAULT' && hmiLast.main_controller_state !== 'FAULT') hmiAddEvent('MAIN_CONTROLLER_FAULT', 'critical', '主控状态异常，进入冗余判断', sample);
  if (sample.backup_controller_state === 'ACTIVE' && hmiLast.backup_controller_state !== 'ACTIVE') hmiAddEvent('BACKUP_TAKEOVER', 'warning', '备控接管控制链路', sample);
  if (sample.mcu_takeover && !hmiLast.mcu_takeover) hmiAddEvent('MCU_TAKEOVER', 'critical', 'MCU 安全接管触发', sample);
}
function hmiNormalize(raw) {
  return {
    time_s: Number(raw.time_s ?? 0),
    ego_speed_kmh: Number(raw.ego_speed_kmh ?? 0),
    target_distance_m: Number(raw.target_distance_m ?? 0),
    relative_speed_mps: Number(raw.relative_speed_mps ?? 0),
    ttc_s: Number(raw.ttc_s ?? 99),
    lateral_error_m: Number(raw.lateral_error_m ?? 0),
    heading_error_deg: Number(raw.heading_error_deg ?? 0),
    throttle: Number(raw.throttle ?? 0),
    brake: Number(raw.brake ?? 0),
    steer: Number(raw.steer ?? 0),
    aeb_state: String(raw.aeb_state ?? 'OFF'),
    acc_state: String(raw.acc_state ?? 'CRUISE'),
    lka_state: String(raw.lka_state ?? 'ACTIVE'),
    main_controller_state: String(raw.main_controller_state ?? 'ACTIVE'),
    backup_controller_state: String(raw.backup_controller_state ?? 'STANDBY'),
    mcu_takeover: Boolean(raw.mcu_takeover),
    collision: Boolean(raw.collision),
    lane_invasion: Boolean(raw.lane_invasion),
    target_lost: Boolean(raw.target_lost)
  };
}
function hmiMockSample() {
  const t = (Date.now() - hmiMockStart) / 1000;
  const phase = t % 72;
  let speed = 72 + Math.sin(t / 4) * 2.2;
  let distance = 46 - Math.max(0, phase - 10) * 1.35;
  let relative = Math.max(0, 2.2 + (phase - 18) * .08);
  let aeb = 'OFF', acc = phase > 8 ? 'FOLLOW' : 'CRUISE', lka = 'ACTIVE';
  let main = 'ACTIVE', backup = 'STANDBY', mcu = false, brake = 0, throttle = .22;
  const lane = Math.sin(t * .72) * .12 + Math.sin(t * .19) * .08;
  if (phase > 24) distance = Math.max(6, 38 - (phase - 24) * 1.72);
  const ttc = Math.max(.75, distance / Math.max(.1, relative + (phase > 24 ? 4.2 : 1.1)));
  if (ttc < 4.2) { aeb = 'WARNING'; brake = .18; throttle = .05; }
  if (ttc < 2.4) { aeb = 'BRAKE'; acc = 'BRAKE'; brake = .78; throttle = 0; speed -= (2.5 - ttc) * 10; }
  if (phase > 46 && phase < 52) { main = 'FAULT'; backup = 'ACTIVE'; }
  if (phase > 56 && phase < 61) { mcu = true; main = 'LOST'; backup = 'ACTIVE'; brake = Math.max(brake, .55); }
  if (phase > 62) { distance = 35 + Math.sin(t) * 3; relative = 2; aeb = 'OFF'; acc = 'FOLLOW'; brake = 0; throttle = .18; }
  return hmiNormalize({
    time_s: t,
    ego_speed_kmh: Math.max(0, speed),
    target_distance_m: distance,
    relative_speed_mps: relative,
    ttc_s: ttc,
    lateral_error_m: lane + ((phase > 34 && phase < 38) ? .42 : 0),
    heading_error_deg: lane * 7 + Math.sin(t / 2) * .7,
    throttle,
    brake,
    steer: Math.max(-.8, Math.min(.8, -lane * .7 + Math.sin(t / 1.7) * .03)),
    aeb_state: aeb,
    acc_state: acc,
    lka_state: (phase > 34 && phase < 38) ? 'WARNING' : lka,
    main_controller_state: main,
    backup_controller_state: backup,
    mcu_takeover: mcu,
    collision: false,
    lane_invasion: phase > 35 && phase < 36,
    target_lost: phase > 66 && phase < 68
  });
}
function hmiPushHistory(sample) {
  hmiHist.t.push(sample.time_s);
  hmiHist.ttc.push(Math.min(12, sample.ttc_s));
  hmiHist.speed.push(sample.ego_speed_kmh);
  hmiHist.lateral.push(sample.lateral_error_m);
  hmiHist.brake.push(sample.brake);
  hmiHist.steer.push(sample.steer);
  Object.keys(hmiHist).forEach(k => { if (hmiHist[k].length > HMI_MAXPTS) hmiHist[k].shift(); });
}
function hmiRenderMetrics(sample) {
  const metrics = [
    ['ego_speed_kmh', '自车速度', sample.ego_speed_kmh, 'km/h', '车辆纵向速度，用于判断巡航与制动效果'],
    ['target_distance_m', '前车距离', sample.target_distance_m, 'm', '目标车相对距离，支撑 ACC/AEB 判定'],
    ['relative_speed_mps', '相对速度', sample.relative_speed_mps, 'm/s', '自车相对目标车的闭合速度'],
    ['ttc_s', 'TTC', sample.ttc_s, 's', '预计碰撞时间，越低风险越高'],
    ['lateral_error_m', '横向误差', sample.lateral_error_m, 'm', '车辆相对车道中心的偏移'],
    ['heading_error_deg', '航向误差', sample.heading_error_deg, 'deg', '车身航向与目标轨迹的夹角误差']
  ];
  const box = $('hmi-metrics');
  if (!box) return;
  box.innerHTML = metrics.map(([key, name, value, unit, desc]) => {
    const level = hmiMetricLevel(key, value);
    const cls = hmiClass(level);
    return `<div class="hmi-metric ${cls}">
      <div class="hmi-metric-top"><span class="hmi-metric-name">${name}</span><span class="hmi-metric-state">${RISK_CN[level]}</span></div>
      <div class="hmi-metric-value">${fmt(value, key === 'ego_speed_kmh' ? 1 : 2)}<small>${unit}</small></div>
      <div class="hmi-metric-desc">${desc}</div>
    </div>`;
  }).join('');
}
function hmiRenderStates(sample) {
  const states = [
    ['ACC状态', sample.acc_state],
    ['AEB状态', sample.aeb_state],
    ['LKA状态', sample.lka_state],
    ['主控状态', sample.main_controller_state],
    ['备控状态', sample.backup_controller_state],
    ['MCU接管', String(sample.mcu_takeover)]
  ];
  const box = $('hmi-states');
  if (!box) return;
  box.innerHTML = states.map(([label, value]) => {
    const cls = hmiStateClass(label, value, sample);
    return `<div class="hmi-state"><div class="hmi-state-label">${label}</div><span class="hmi-state-value ${cls}">${value}</span></div>`;
  }).join('');
}
function hmiRenderEvents() {
  const body = $('hmi-events');
  if (!body) return;
  hmiSetText('hmi-event-count', String(hmiEvents.length));
  if (!hmiEvents.length) {
    body.innerHTML = '<tr class="hmi-empty"><td colspan="7">暂无事件记录</td></tr>';
    return;
  }
  body.innerHTML = hmiEvents.slice(0, 24).map(e => {
    const cls = hmiClass(e.level);
    return `<tr class="${cls}"><td>${fmt(e.time, 2)}s</td><td><b>${e.type}</b></td><td><span class="level">${hmiLevelText(e.level)}</span></td><td>${e.desc}</td><td>${fmt(e.ttc, 2)}s</td><td>${fmt(e.distance, 1)}m</td><td>${fmt(e.speed, 1)}km/h</td></tr>`;
  }).join('');
}
function hmiDrawCanvas(id, series, options) {
  const cvx = $(id);
  if (!cvx) return;
  const ctx2 = cvx.getContext('2d');
  const W = cvx.width, H = cvx.height, pad = 28;
  ctx2.clearRect(0, 0, W, H);
  ctx2.fillStyle = 'rgba(255,255,255,.55)';
  ctx2.fillRect(0, 0, W, H);
  ctx2.strokeStyle = 'rgba(176,123,240,.16)';
  ctx2.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = pad + (H - 2 * pad) * i / 4;
    ctx2.beginPath(); ctx2.moveTo(pad, y); ctx2.lineTo(W - pad, y); ctx2.stroke();
  }
  const draw = (arr, color, min, max) => {
    ctx2.strokeStyle = color; ctx2.lineWidth = 3; ctx2.lineJoin = 'round'; ctx2.beginPath();
    arr.forEach((val, i) => {
      const x = pad + (W - 2 * pad) * i / Math.max(1, HMI_MAXPTS - 1);
      const ratio = (val - min) / Math.max(.001, max - min);
      const y = H - pad - (H - 2 * pad) * Math.max(0, Math.min(1, ratio));
      if (i === 0) ctx2.moveTo(x, y); else ctx2.lineTo(x, y);
    });
    ctx2.stroke();
  };
  series.forEach(s => draw(s.data, s.color, options.min, options.max));
  ctx2.fillStyle = 'rgba(91,74,99,.72)';
  ctx2.font = '12px sans-serif';
  ctx2.fillText(options.label, pad, 18);
  ctx2.fillText(String(options.max), 4, pad + 4);
  ctx2.fillText(String(options.min), 8, H - pad + 4);
}
function hmiDrawCharts() {
  hmiDrawCanvas('hmi-chart-ttc', [{ data: hmiHist.ttc, color: '#ffab33' }], { min: 0, max: 12, label: '最近 60 秒 · TTC(s)' });
  hmiDrawCanvas('hmi-chart-speed', [{ data: hmiHist.speed, color: '#43d9b8' }], { min: 0, max: 120, label: '最近 60 秒 · km/h' });
  hmiDrawCanvas('hmi-chart-lateral', [{ data: hmiHist.lateral, color: '#b07bf0' }], { min: -1, max: 1, label: '最近 60 秒 · lateral_error_m' });
  hmiDrawCanvas('hmi-chart-control', [
    { data: hmiHist.brake, color: '#ff6b8b' },
    { data: hmiHist.steer, color: '#5aa9ff' }
  ], { min: -1, max: 1, label: '最近 60 秒 · brake(红) / steer(蓝)' });
}
function hmiRender(sample) {
  const risk = sample.collision || sample.mcu_takeover || sample.aeb_state === 'BRAKE' || sample.main_controller_state !== 'ACTIVE'
    ? 'critical' : (sample.aeb_state === 'WARNING' || sample.lane_invasion || sample.ttc_s < 4.5 ? 'warning' : 'normal');
  hmiSetText('hmi-data-mode', hmiMode === 'live' ? '实时数据模式' : '演示数据模式');
  hmiSetText('hmi-hero-time', fmt(sample.time_s, 1));
  hmiSetText('hmi-hero-risk', RISK_CN[risk]);
  hmiSetText('hmi-time', fmt(sample.time_s, 2));
  hmiSetBadge('hmi-carlas', 'CARLA连接：' + (hmiMode === 'live' ? '已连接' : '等待连接'), hmiMode === 'live' ? 'ok' : 'warn');
  hmiSetBadge('hmi-hils', 'HIL闭环：' + (sample.main_controller_state === 'LOST' ? '未启动' : '运行中'), sample.main_controller_state === 'LOST' ? 'warn' : 'ok');
  hmiSetBadge('hmi-logs', '日志记录：记录中', 'ok');
  hmiSetBadge('hmi-wss', 'WebSocket：' + (hmiMode === 'live' ? '已连接' : '断开'), hmiMode === 'live' ? 'ok' : 'crit');
  hmiDetectEvents(sample);
  hmiPushHistory(sample);
  hmiRenderMetrics(sample);
  hmiRenderStates(sample);
  hmiRenderEvents();
  hmiDrawCharts();
  hmiLast = sample;
}
function hmiStartMock() {
  hmiMode = 'mock';
  if (!hmiMockTimer) hmiMockTimer = setInterval(() => hmiRender(hmiMockSample()), 500);
  if (!hmiReconnectTimer) hmiReconnectTimer = setInterval(hmiConnect, 30000);
}
function hmiStopMock() {
  if (hmiMockTimer) clearInterval(hmiMockTimer);
  hmiMockTimer = null;
}
function hmiConnect() {
  if (hmiWs && (hmiWs.readyState === WebSocket.OPEN || hmiWs.readyState === WebSocket.CONNECTING)) return;
  try {
    hmiWs = new WebSocket(HMI_WS_URL);
    hmiWs.onopen = () => {
      hmiMode = 'live';
      hmiStopMock();
      if (hmiReconnectTimer) clearInterval(hmiReconnectTimer);
      hmiReconnectTimer = null;
    };
    hmiWs.onmessage = e => {
      try { hmiRender(hmiNormalize(JSON.parse(e.data))); } catch (_) {}
    };
    hmiWs.onclose = () => hmiStartMock();
    hmiWs.onerror = () => {
      try { hmiWs.close(); } catch (_) {}
      hmiStartMock();
    };
  } catch (_) {
    hmiStartMock();
  }
}
function hmiInit() {
  hmiRender(hmiMockSample());
  hmiConnect();
  setTimeout(() => { if (hmiMode !== 'live') hmiStartMock(); }, 900);
}

loadScenarios();
connect();
hmiInit();
