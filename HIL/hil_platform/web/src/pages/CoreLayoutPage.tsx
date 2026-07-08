import { useEffect, useMemo, useState } from "react";
import { api } from "../api/client";
import type { CoreLayout, CoreLayoutTarget, CoreProcess, CoreThread } from "../types/hil";

const CORE_LABELS: Record<string, string> = {
  "0": "100 Hz 控制主循环",
  "1": "DDS / UART / ML 辅助",
  "2": "Gateway / Lockstep Checker",
  "3": "Edge / Telemetry 后台",
};

const KIND_LABEL: Record<CoreThread["kind"], string> = {
  adas: "ADAS",
  ml: "ML",
  gateway: "Gateway",
  edge: "Edge",
  lockstep: "Lockstep",
  other: "Other",
};

function fmtCpu(v: number | undefined): string {
  if (v === undefined || Number.isNaN(v)) return "--";
  return `${v.toFixed(1)}%`;
}

function roleName(key: "primary" | "backup"): string {
  return key === "primary" ? "Primary Nano B（主控）" : "Backup Nano A（备控）";
}

function procName(p: CoreProcess): string {
  if (p.args.includes("--role primary")) return "ADAS primary";
  if (p.args.includes("--role backup")) return "ADAS backup";
  if (p.args.includes("hil_ros_gateway.py")) return "HIL gateway";
  if (p.args.includes("edge_result_collector.py")) return "Edge collector";
  return p.args.split(" ").slice(-1)[0] || `PID ${p.pid}`;
}

function mlSummary(processes: CoreProcess[]): string {
  const adas = processes.find((p) => p.args.includes("ADAS.py"));
  if (!adas) return "未发现 ADAS 进程";
  const enabled = adas.env.ADAS_ML_ENABLED === "1";
  const backend = adas.env.ADAS_ML_BACKEND ?? "--";
  const async = adas.env.ADAS_ML_ASYNC ?? "--";
  const threads = adas.env.ADAS_ML_THREADS ?? "--";
  return enabled ? `ML=ON · ${backend} · async=${async} · threads=${threads}` : "ML=OFF";
}

function lockstepSummary(processes: CoreProcess[], threads: CoreThread[]): string {
  const adas = processes.find((p) => p.args.includes("ADAS.py"));
  const checker = threads.find((t) => t.kind === "lockstep");
  if (checker) return `Lockstep checker 在线 · core${checker.core}`;
  if (adas?.env.LOCKSTEP_ENABLED === "1") return `Lockstep 已启用 · checker core ${adas.env.LOCKSTEP_CHECKER_CORE ?? "2"}`;
  return "Lockstep 默认关闭";
}

function CoreCard({ coreId, target }: { coreId: string; target?: CoreLayoutTarget }) {
  const data = target?.data;
  const threads = (data?.core_threads?.[coreId] ?? []).slice(0, 6);
  const cpu = data?.system_cpu?.[coreId];
  return (
    <div className="core-card">
      <div className="core-head">
        <div>
          <div className="core-id">CPU{coreId}</div>
          <div className="core-role">{CORE_LABELS[coreId] ?? "辅助核心"}</div>
        </div>
        <div className="core-cpu">{fmtCpu(cpu)}</div>
      </div>
      <div className="thread-list">
        {threads.length === 0 && <div className="thread-empty">当前未采样到 HIL 线程</div>}
        {threads.map((t) => (
          <div className={"thread-row " + t.kind} key={`${t.pid}-${t.tid}-${t.comm}`}>
            <span className="thread-kind">{KIND_LABEL[t.kind]}</span>
            <span className="thread-name">{t.comm}</span>
            <span className="thread-cpu">{fmtCpu(t.cpu)}</span>
          </div>
        ))}
      </div>
    </div>
  );
}

function NanoPanel({ name, target }: { name: string; target?: CoreLayoutTarget }) {
  const data = target?.data;
  const processRows = useMemo(() => data?.processes ?? [], [data]);
  return (
    <section className="card core-panel">
      <div className="core-panel-title">
        <div>
          <h3>{name}</h3>
          <div className="muted">
            {target?.host ?? "--"} · {data?.hostname ?? "--"} · {data?.nproc ?? 0} 核 · SSH {target?.elapsed_ms ?? "--"} ms
          </div>
        </div>
        <span className={"badge " + (target?.ok ? "ok" : "danger")}>
          <span className="dot" />{target?.ok ? "在线" : "异常"}
        </span>
      </div>

      {!target?.ok && <pre className="hw-log">{target?.stderr || "无法读取 Nano 状态"}</pre>}

      <div className="core-grid">
        {["0", "1", "2", "3"].map((id) => <CoreCard key={id} coreId={id} target={target} />)}
      </div>

      <div className="evidence-row">
        <div className="evidence-box">
          <div className="evidence-label">ML 推理</div>
          <div className="evidence-value">{mlSummary(processRows)}</div>
        </div>
        <div className="evidence-box">
          <div className="evidence-label">软件锁步</div>
          <div className="evidence-value">{lockstepSummary(processRows, data?.threads ?? [])}</div>
        </div>
        <div className="evidence-box">
          <div className="evidence-label">系统负载</div>
          <div className="evidence-value">{data?.loadavg?.map((v) => v.toFixed(2)).join(" / ") || "--"}</div>
        </div>
      </div>

      <table className="core-proc-table">
        <thead>
          <tr><th>进程</th><th>PID</th><th>Affinity</th><th>命令</th></tr>
        </thead>
        <tbody>
          {processRows.map((p) => (
            <tr key={p.pid}>
              <td>{procName(p)}</td>
              <td className="mono">{p.pid}</td>
              <td className="mono">{p.affinity.length ? p.affinity.map((c) => `CPU${c}`).join(", ") : "--"}</td>
              <td className="mono muted">{p.args}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </section>
  );
}

export default function CoreLayoutPage() {
  const [layout, setLayout] = useState<CoreLayout | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [auto, setAuto] = useState(true);

  const refresh = async () => {
    try {
      setError(null);
      setLayout(await api.hardware.coreLayout());
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    }
  };

  useEffect(() => { refresh(); }, []);
  useEffect(() => {
    if (!auto) return;
    const id = window.setInterval(refresh, 1000);
    return () => window.clearInterval(id);
  }, [auto]);

  const updated = layout?.primary?.data?.timestamp ?? layout?.backup?.data?.timestamp;
  return (
    <div className="page">
      {error && <div className="disconnected-banner">{error}</div>}
      <div className="statusbar">
        <div className="item">
          <span className="k">页面</span>
          <span className="v">双 Nano 四核调度</span>
        </div>
        <div className="item">
          <span className="k">刷新</span>
          <span className="v">{auto ? "1 Hz 自动" : "手动"}</span>
        </div>
        <div className="item">
          <span className="k">更新时间</span>
          <span className="v">{updated ? new Date(updated * 1000).toLocaleTimeString() : "--"}</span>
        </div>
        <div className="spacer" />
        <button className="sm" onClick={() => setAuto(!auto)}>{auto ? "暂停刷新" : "自动刷新"}</button>
        <button className="sm primary" onClick={refresh}>立即刷新</button>
      </div>

      <section className="card">
        <h3>运行时调度机制</h3>
        <div className="core-plan">
          {["0", "1", "2", "3"].map((id) => (
            <div className="plan-step" key={id}>
              <span className="plan-core">CPU{id}</span>
              <span>{layout?.core_plan?.[id] ?? CORE_LABELS[id]}</span>
            </div>
          ))}
        </div>
      </section>

      <div className="dual-core-layout">
        <NanoPanel name={roleName("primary")} target={layout?.primary} />
        <NanoPanel name={roleName("backup")} target={layout?.backup} />
      </div>
    </div>
  );
}
