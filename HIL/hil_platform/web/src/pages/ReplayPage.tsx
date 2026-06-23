// 历史回放页：选择 run → 时间轴复现历史状态。只读，不影响仿真。
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { api } from "../api/client";
import { RunList, type RunFilters } from "../components/RunList";
import { ReplayTimeline, type JumpTargets } from "../components/ReplayTimeline";
import { EventList } from "../components/EventList";
import { StatusBar } from "../components/StatusBar";
import { MetricCards, type MetricValues } from "../components/MetricCards";
import { NanoPanel, Esp32Panel } from "../components/ControllerStatus";
import { EChart } from "../components/EChart";
import { fmt, fmtTtc, resultBadge, boolBadge } from "../lib/format";
import type {
  RunListItem, RunMeta, Summary, RunEvent, StateRow, ActiveController,
  Controller, Esp32, Ego, Target,
} from "../types/hil";

const ACTIVE_CODE: Record<string, number> = { nano_a: 0, nano_b: 1, safe_brake: 2, none: 0 };

export default function ReplayPage() {
  const [runs, setRuns] = useState<RunListItem[]>([]);
  const [filters, setFilters] = useState<RunFilters>({});
  const [scenarioOptions, setScenarioOptions] = useState<string[]>([]);
  const [selId, setSelId] = useState<string | null>(null);

  const [meta, setMeta] = useState<RunMeta | null>(null);
  const [summary, setSummary] = useState<Summary | null>(null);
  const [events, setEvents] = useState<RunEvent[]>([]);
  const [states, setStates] = useState<StateRow[]>([]);

  const [t, setT] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);

  // ── 拉取列表 ──
  const fetchRuns = useCallback(async (f: RunFilters) => {
    const q: Record<string, string> = {};
    (Object.keys(f) as (keyof RunFilters)[]).forEach((k) => { if (f[k]) q[k] = f[k] as string; });
    const r = await api.listRuns(q);
    setRuns(r.runs);
  }, []);

  useEffect(() => {
    (async () => {
      const r = await api.listRuns();
      setRuns(r.runs);
      setScenarioOptions([...new Set(r.runs.map((x) => x.scenario).filter(Boolean) as string[])]);
    })();
  }, []);

  const onFilterChange = (f: RunFilters) => { setFilters(f); fetchRuns(f); };

  // ── 选择 run → 载入全部数据 ──
  const selectRun = async (id: string) => {
    setSelId(id);
    setPlaying(false);
    setT(0);
    const [m, s, ev, st] = await Promise.all([
      api.runMeta(id), api.runSummary(id), api.runEvents(id), api.runStates(id),
    ]);
    setMeta(m); setSummary(s); setEvents(ev); setStates(st.states);
  };

  const duration = meta?.duration ?? (states.length ? states[states.length - 1].t : 0);

  // ── 播放推进 ──
  const raf = useRef<number | null>(null);
  const lastWall = useRef<number>(0);
  useEffect(() => {
    if (!playing) return;
    lastWall.current = performance.now();
    const tick = () => {
      const now = performance.now();
      const dt = (now - lastWall.current) / 1000;
      lastWall.current = now;
      setT((prev) => {
        const nt = prev + dt * speed;
        if (nt >= duration) { setPlaying(false); return duration; }
        return nt;
      });
      raf.current = requestAnimationFrame(tick);
    };
    raf.current = requestAnimationFrame(tick);
    return () => { if (raf.current) cancelAnimationFrame(raf.current); };
  }, [playing, speed, duration]);

  // ── 当前时刻最近的状态行 ──
  const curIdx = useMemo(() => nearestIndex(states, t), [states, t]);
  const row: StateRow | null = states[curIdx] ?? null;

  // 跳转目标
  const jumps: JumpTargets = useMemo(() => ({
    takeover: events.find((e) => e.type === "TAKEOVER")?.time ?? null,
    minTtc: events.find((e) => e.type === "MIN_TTC")?.time ?? null,
    maxLat: events.find((e) => e.type === "MAX_LATERAL_ERROR")?.time ?? null,
  }), [events]);

  // 当前事件（时间窗 ±0.2s）
  const curEvent = events.find((e) => Math.abs((e.time ?? 0) - t) < 0.2);

  // ── 快照对象（复用 /live 组件）──
  const metricValues: MetricValues = {
    speed_kmh: row?.ego_speed ?? null,
    front_distance: row?.front_distance ?? null,
    relative_speed: row?.relative_speed ?? null,
    ttc: row?.ttc ?? null,
    lateral_error: row?.lateral_error ?? null,
    heading_error: row?.heading_error ?? null,
    throttle: row?.throttle ?? null,
    brake: row?.brake ?? null,
    steer: row?.steer ?? null,
  };
  const nanoA: Controller | null = row && {
    alive: row.nano_a_alive === 1, seq: row.nano_a_seq, latency_ms: row.nano_a_latency_ms,
    valid_output: row.nano_a_valid_output === 1, last_control_time: null,
    throttle: null, brake: null, steer: null,
  };
  const nanoB: Controller | null = row && {
    alive: row.nano_b_alive === 1, seq: row.nano_b_seq, latency_ms: row.nano_b_latency_ms,
    valid_output: row.nano_b_valid_output === 1, last_control_time: null,
    throttle: null, brake: null, steer: null,
  };
  const esp: Esp32 | null = row && {
    active_controller: row.active_controller, takeover_count: row.takeover_count,
    last_takeover_reason: null, safe_brake: row.safe_brake === 1,
    throttle: row.throttle, brake: row.brake, steer: row.steer,
  };
  const activeCtrl: ActiveController = row?.active_controller ?? "none";

  const onSeek = (nt: number) => { setPlaying(false); setT(nt); };

  return (
    <div className="page">
      <RunList
        runs={runs} selectedId={selId} filters={filters} scenarioOptions={scenarioOptions}
        onFilterChange={onFilterChange} onSelect={selectRun}
      />

      {!selId && <div className="card faint">从上方列表选择一次实验进行回放。</div>}

      {selId && (
        <>
          <StatusBar
            runId={selId} scenario={meta?.scenario ?? null} state="STOPPED"
            scenarioTime={t} activeController={activeCtrl}
            takeover={activeCtrl === "nano_b" || activeCtrl === "safe_brake"}
            safeBrake={row?.safe_brake === 1}
          />

          <ReplayTimeline
            duration={duration} currentTime={t} playing={playing} speed={speed} jumps={jumps}
            onSeek={onSeek} onTogglePlay={() => setPlaying((v) => !v)} onSpeed={setSpeed}
          />

          {/* 实验摘要 */}
          {summary && (
            <div className="card">
              <h3>实验摘要</h3>
              <div className="row" style={{ gap: 18 }}>
                <span className={"badge " + resultBadge(summary.result)}><span className="dot" />结果 {summary.result}</span>
                <span className={"badge " + boolBadge(summary.collision)}><span className="dot" />碰撞 {summary.collision ? "是" : "否"}</span>
                <span className="badge"><span className="dot" />最小 TTC {fmtTtc(summary.min_ttc)}s</span>
                <span className="badge"><span className="dot" />最大横向误差 {fmt(summary.max_lateral_error, 2)}m</span>
                <span className={"badge " + (summary.takeover_happened ? "warn" : "ok")}><span className="dot" />接管 {summary.takeover_happened ? "是" : "否"}</span>
                <span className="badge"><span className="dot" />接管时延 {summary.takeover_latency_ms ?? "--"}ms</span>
                <span className={"badge " + boolBadge(summary.safe_brake_triggered)}><span className="dot" />安全制动 {summary.safe_brake_triggered ? "是" : "否"}</span>
              </div>
              <div className="muted" style={{ marginTop: 10 }}>{summary.conclusion}</div>
            </div>
          )}

          {/* 当前时刻快照 */}
          <MetricCards v={metricValues} />
          <div className="row">
            <div style={{ flex: "1 1 240px" }}><NanoPanel name="Nano A（主控）" ctrl={nanoA} active={activeCtrl === "nano_a"} /></div>
            <div style={{ flex: "1 1 240px" }}><NanoPanel name="Nano B（备控）" ctrl={nanoB} active={activeCtrl === "nano_b"} /></div>
            <div style={{ flex: "1 1 240px" }}><Esp32Panel esp={esp} /></div>
          </div>

          {/* 当前事件 */}
          <div className="card">
            <h3>当前事件</h3>
            {curEvent
              ? <div className={"event-item " + curEvent.type}>
                  <span className="etime">{curEvent.time.toFixed(2)}s</span>
                  <span className="etype">{curEvent.type}</span>
                </div>
              : <span className="faint">此刻无事件</span>}
          </div>

          {/* 历史曲线（与时间轴联动，点击跳转） */}
          <HistoryCharts states={states} currentTime={t}
            onSeek={(i) => onSeek(states[i]?.t ?? 0)} />

          {/* 事件列表 */}
          <EventList events={events} currentTime={t} onSelect={onSeek} />
        </>
      )}
    </div>
  );
}

// ── 历史曲线组 ──
function HistoryCharts({ states, currentTime, onSeek }: {
  states: StateRow[]; currentTime: number; onSeek: (idx: number) => void;
}) {
  const mk = (pick: (r: StateRow) => number | null, color: string,
    opts: { ttc?: boolean; step?: boolean; yMin?: number; yMax?: number } = {}) => ({
    animation: false,
    grid: { left: 44, right: 12, top: 22, bottom: 22 },
    tooltip: { trigger: "axis" },
    xAxis: {
      type: "value", min: "dataMin", max: "dataMax",
      axisLabel: { color: "#5c6b7d", formatter: (v: number) => v.toFixed(0) + "s" },
      axisLine: { lineStyle: { color: "#243343" } },
    },
    yAxis: {
      type: "value", scale: !opts.step, min: opts.yMin, max: opts.yMax,
      axisLabel: {
        color: "#5c6b7d",
        formatter: opts.step ? (v: number) => (["A", "B", "SAFE"][v] ?? "") : undefined,
      },
      splitLine: { lineStyle: { color: "#1b2733" } },
    },
    series: [{
      type: "line",
      data: states.map((r) => [r.t, opts.ttc ? finite(pick(r)) : pick(r)]),
      showSymbol: false, step: opts.step ? "end" : false,
      lineStyle: { color, width: 2 },
      areaStyle: opts.step ? undefined : { color, opacity: 0.08 },
      markLine: {
        silent: true, symbol: "none",
        data: [{ xAxis: currentTime }],
        lineStyle: { color: "#fff", type: "dashed", width: 1 },
        label: { show: false },
      },
    }],
  });

  const charts: { title: string; opt: object }[] = [
    { title: "Ego 速度 (km/h)", opt: mk((r) => r.ego_speed, "#2f81f7") },
    { title: "TTC (s)", opt: mk((r) => r.ttc, "#9b59b6", { ttc: true }) },
    { title: "前车距离 (m)", opt: mk((r) => r.front_distance, "#1abc9c") },
    { title: "横向误差 (m)", opt: mk((r) => r.lateral_error, "#f1c40f") },
    { title: "刹车", opt: mk((r) => r.brake, "#e74c3c", { yMin: 0, yMax: 1 }) },
    { title: "生效控制器", opt: mk((r) => ACTIVE_CODE[r.active_controller] ?? 0, "#e67e22", { step: true, yMin: 0, yMax: 2 }) },
  ];

  return (
    <div className="grid" style={{ gridTemplateColumns: "repeat(auto-fit, minmax(300px, 1fr))" }}>
      {charts.map((c) => (
        <div className="card" key={c.title} style={{ padding: "8px 10px" }}>
          <h3 style={{ marginBottom: 4 }}>{c.title}</h3>
          <EChart option={c.opt} height={150} onPointClick={(idx) => onSeek(idx)} />
        </div>
      ))}
    </div>
  );
}

function finite(v: number | null): number | null {
  return v != null && Number.isFinite(v) ? v : null;
}

function nearestIndex(states: StateRow[], t: number): number {
  if (states.length === 0) return -1;
  // 线性扫描足够（单 run 数据量有限）
  let lo = 0, hi = states.length - 1;
  // 二分到首个 >= t
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (states[mid].t < t) lo = mid + 1; else hi = mid;
  }
  if (lo > 0 && Math.abs(states[lo - 1].t - t) <= Math.abs(states[lo].t - t)) return lo - 1;
  return lo;
}
