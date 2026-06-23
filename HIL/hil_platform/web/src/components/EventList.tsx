// 事件列表：FAULT_INJECTED / TAKEOVER / SAFE_BRAKE / COLLISION / MIN_TTC / MAX_LATERAL_ERROR。
// 点击事件跳转到对应时间点。
import type { RunEvent } from "../types/hil";

const TYPE_LABEL: Record<string, string> = {
  FAULT_INJECTED: "故障注入",
  TAKEOVER: "主备接管",
  SAFE_BRAKE: "安全制动",
  COLLISION: "碰撞",
  MIN_TTC: "最小 TTC",
  MAX_LATERAL_ERROR: "最大横向误差",
};

function detailText(e: RunEvent): string {
  const skip = new Set(["time", "type"]);
  const parts = Object.entries(e)
    .filter(([k]) => !skip.has(k))
    .map(([k, v]) => `${k}=${v}`);
  return parts.join("  ");
}

export function EventList({ events, currentTime, onSelect }: {
  events: RunEvent[];
  currentTime: number;
  onSelect: (t: number) => void;
}) {
  return (
    <div className="card">
      <h3>事件列表（{events.length}）</h3>
      <div className="scroll" style={{ maxHeight: 320, display: "flex", flexDirection: "column", gap: 4 }}>
        {events.map((e, i) => {
          const near = Math.abs((e.time ?? 0) - currentTime) < 0.2;
          return (
            <div key={i} className={"event-item " + e.type}
              style={near ? { background: "var(--bg-elev)" } : undefined}
              onClick={() => onSelect(e.time)}>
              <span className="etime">{e.time?.toFixed(2)}s</span>
              <span className="etype">{TYPE_LABEL[e.type] ?? e.type}</span>
              <span className="faint" style={{ marginLeft: "auto", fontFamily: "var(--mono)" }}>
                {detailText(e)}
              </span>
            </div>
          );
        })}
        {events.length === 0 && <div className="faint">（无事件）</div>}
      </div>
    </div>
  );
}
