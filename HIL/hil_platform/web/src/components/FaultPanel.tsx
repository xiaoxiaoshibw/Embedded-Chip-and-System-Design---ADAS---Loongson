// 故障注入面板（/live 专用，可控制仿真）。
import { useState } from "react";
import { api } from "../api/client";
import type { FaultType } from "../types/hil";

interface FaultBtn { label: string; type: FaultType; target: string; danger?: boolean; }

const FAULTS: FaultBtn[] = [
  { label: "Nano A 断心跳", type: "heartbeat_loss", target: "nano_a" },
  { label: "Nano A seq 停止", type: "seq_stuck", target: "nano_a" },
  { label: "Nano A 输出 NaN", type: "nan_output", target: "nano_a" },
  { label: "Nano A 控制延迟", type: "control_delay", target: "nano_a" },
  { label: "Nano B 接管失败", type: "backup_fail", target: "nano_b" },
  { label: "双路失败安全制动", type: "dual_fail", target: "both", danger: true },
];

export function FaultPanel({ disabled }: { disabled: boolean }) {
  const [last, setLast] = useState<string>("");
  const [err, setErr] = useState<string>("");

  const inject = async (f: FaultBtn) => {
    setErr("");
    try {
      const r = await api.injectFault(f.type, f.target);
      setLast(`${f.label} @ ${r.event.time}s`);
    } catch (e) {
      setErr((e as Error).message);
    }
  };

  return (
    <div className="card">
      <h3>故障注入</h3>
      <div className="btn-row">
        {FAULTS.map((f) => (
          <button key={f.type} className={"sm " + (f.danger ? "danger" : "")}
            disabled={disabled} onClick={() => inject(f)}>
            {f.label}
          </button>
        ))}
      </div>
      {last && <div className="muted" style={{ marginTop: 8 }}>已注入：{last}</div>}
      {err && <div style={{ color: "var(--danger)", marginTop: 8 }}>注入失败：{err}</div>}
      {disabled && <div className="faint" style={{ marginTop: 8 }}>仿真未运行，无法注入故障。</div>}
    </div>
  );
}
