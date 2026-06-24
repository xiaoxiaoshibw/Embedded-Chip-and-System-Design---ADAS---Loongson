import { useMemo, useState } from "react";
import { api } from "../api/client";
import type { HardwareCommandResult, HardwareHealth } from "../types/hil";

function shortLine(text?: string) {
  return (text || "").split(/\r?\n/).filter(Boolean).slice(-1)[0] || "--";
}

function hasProcess(r?: HardwareCommandResult, token = "ADAS.py --role") {
  return !!r?.stdout?.includes(token);
}

function ResultBox({ title, result }: { title: string; result?: HardwareCommandResult }) {
  const ok = result?.ok ?? false;
  const proc = title.includes("Gateway")
    ? hasProcess(result, "hil_ros_gateway.py")
    : hasProcess(result);
  return (
    <div className="hw-box">
      <div className="ctrl-head">
        <span className="title">{title}</span>
        <span className={"badge " + (ok && proc ? "ok" : ok ? "warn" : "danger")}>
          <span className="dot" />{ok && proc ? "RUNNING" : ok ? "CHECK" : "ERROR"}
        </span>
      </div>
      <div className="kv"><span className="k">host</span><span className="v">{result?.host ?? "--"}</span></div>
      <div className="kv"><span className="k">elapsed</span><span className="v">{result?.elapsed_ms ?? "--"} ms</span></div>
      <pre className="hw-log">{result ? trimOutput(result.stdout || result.stderr) : "--"}</pre>
    </div>
  );
}

function trimOutput(text: string) {
  const lines = text.split(/\r?\n/).filter(Boolean);
  return lines.slice(-18).join("\n") || "--";
}

export function HardwareControlPanel({ controlSource }: { controlSource?: string }) {
  const [health, setHealth] = useState<HardwareHealth | null>(null);
  const [busy, setBusy] = useState("");
  const [err, setErr] = useState("");
  const [last, setLast] = useState("");

  const primary = health?.primary as HardwareCommandResult | undefined;
  const backup = health?.backup as HardwareCommandResult | undefined;

  const summary = useMemo(() => {
    if (!health) return "尚未检查";
    const parts = [
      hasProcess(primary) ? "primary ADAS ok" : "primary ADAS missing",
      hasProcess(backup) ? "backup ADAS ok" : "backup ADAS missing",
      primary?.stdout?.includes("hil_ros_gateway.py") ? "gateway ok" : "gateway missing",
    ];
    return parts.join(" / ");
  }, [health, primary, backup]);

  const run = async (label: string, fn: () => Promise<HardwareHealth>) => {
    setBusy(label);
    setErr("");
    setLast("");
    try {
      const r = await fn();
      const preparedHealth = (r.steps as { health?: HardwareHealth } | undefined)?.health;
      if (preparedHealth || r.primary || r.backup) {
        setHealth(preparedHealth ?? r);
      } else {
        const fresh = await api.hardware.health();
        setHealth(fresh);
      }
      const failedStep = (r as { failed_step?: string }).failed_step;
      setLast(`${label} done: ${shortLine(JSON.stringify({ ok: r.ok, failed_step: failedStep }))}`);
    } catch (e) {
      setErr((e as Error).message);
    } finally {
      setBusy("");
    }
  };

  return (
    <div className="card">
      <div className="ctrl-head">
        <h3 style={{ margin: 0 }}>硬件在环控制</h3>
        <span className={"badge " + (controlSource === "nano" ? "ok" : "warn")}>
          <span className="dot" />{controlSource === "nano" ? "Nano 控制" : `当前 ${controlSource ?? "--"}`}
        </span>
      </div>
      <div className="faint" style={{ margin: "8px 0" }}>
        Web 直接操作两台 Jetson Nano 与主控网关；CARLA 只负责真值感知输入和接收 Nano/ESP32 输出。
      </div>

      <div className="btn-row">
        <button className="primary" disabled={!!busy} onClick={() => run("Prepare HIL ESP32", () => api.hardware.prepareHil("esp32", true, true))}>
          一键准备 HIL(ESP32)
        </button>
        <button className="primary" disabled={!!busy} onClick={() => run("Prepare HIL Jetson", () => api.hardware.prepareHil("jetson", true, true))}>
          一键准备 HIL(Jetson)
        </button>
        <button disabled={!!busy} onClick={() => run("Start CARLA", api.hardware.startCarla)}>
          启动 CARLA
        </button>
        <button disabled={!!busy} onClick={() => run("Deploy Gateway", api.hardware.deployGateway)}>
          部署 Nano Gateway
        </button>
        <button disabled={!!busy} onClick={() => run("Deploy ADAS", api.hardware.deployAdas)}>
          部署 ADAS
        </button>
        <button disabled={!!busy} onClick={() => run("Apply CPU affinity", api.hardware.applyCpu)}>
          CPU绑核
        </button>
        <button disabled={!!busy} onClick={() => run("Nano resources", api.hardware.resources)}>
          Nano资源
        </button>
        <button disabled={!!busy} onClick={() => run("Start edge on 124", api.hardware.startEdge)}>
          124边缘计算
        </button>
        <button disabled={!!busy} onClick={() => run("Sync edge results", api.hardware.syncEdge)}>
          回传边缘结果
        </button>
        <button disabled={!!busy} onClick={() => run("Stop perception_sim", api.hardware.stopPerception)}>
          停止旧感知
        </button>
        <button className="primary" disabled={!!busy} onClick={() => run("健康检查", api.hardware.health)}>
          刷新硬件状态
        </button>
        <button disabled={!!busy} onClick={() => run("重启主控 ADAS", () => api.hardware.restartAdas("primary"))}>
          重启主控 Nano
        </button>
        <button disabled={!!busy} onClick={() => run("重启备控 ADAS", () => api.hardware.restartAdas("backup"))}>
          重启备控 Nano
        </button>
        <button disabled={!!busy} onClick={() => run("重启双 Nano", () => api.hardware.restartAdas("both"))}>
          重启双 Nano
        </button>
        <button disabled={!!busy} onClick={() => run("Gateway=ESP32", () => api.hardware.startGateway("esp32"))}>
          网关接收 ESP32 仲裁
        </button>
        <button disabled={!!busy} onClick={() => run("Gateway=Jetson", () => api.hardware.startGateway("jetson"))}>
          网关接收 Jetson 调试
        </button>
        <button className="danger" disabled={!!busy} onClick={() => run("恢复 Nano", api.hardware.restoreNanos)}>
          恢复 SIGCONT
        </button>
      </div>

      <div className="muted" style={{ marginTop: 8 }}>
        {busy ? `执行中：${busy}` : `状态：${summary}`}
      </div>
      {last && <div className="muted" style={{ marginTop: 6 }}>{last}</div>}
      {err && <div style={{ color: "var(--danger)", marginTop: 6 }}>失败：{err}</div>}

      <div className="hw-grid">
        <ResultBox title="Primary Nano" result={primary} />
        <ResultBox title="Backup Nano" result={backup} />
        <ResultBox title="Gateway on Primary" result={primary} />
      </div>
    </div>
  );
}
