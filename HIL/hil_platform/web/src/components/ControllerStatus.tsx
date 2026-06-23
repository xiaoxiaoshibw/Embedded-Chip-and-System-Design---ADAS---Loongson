// Nano A / Nano B 状态面板 + ESP32 仲裁状态面板。
import { fmt, fmtInt, controllerLabel, controllerBadge } from "../lib/format";
import type { Controller, Esp32, ActiveController } from "../types/hil";

function Row({ k, v, cls }: { k: string; v: React.ReactNode; cls?: string }) {
  return (
    <div className="kv">
      <span className="k">{k}</span>
      <span className={"v " + (cls ?? "")}>{v}</span>
    </div>
  );
}

export function NanoPanel({ name, ctrl, active }: {
  name: string; ctrl: Controller | null; active: boolean;
}) {
  const alive = ctrl?.alive ?? null;
  const valid = ctrl?.valid_output ?? null;
  return (
    <div className="card ctrl-panel">
      <div className="ctrl-head">
        <span className="title">{name}</span>
        <span className={"badge " + (active ? "ok live" : alive === false ? "danger" : "idle")}>
          <span className="dot" />{active ? "生效中" : alive === false ? "失活" : "热备"}
        </span>
      </div>
      <Row k="alive" v={alive == null ? "--" : alive ? "是" : "否"} cls={alive === false ? "bad" : alive ? "good" : ""} />
      <Row k="seq" v={fmtInt(ctrl?.seq)} />
      <Row k="latency_ms" v={fmt(ctrl?.latency_ms, 0)} cls={(ctrl?.latency_ms ?? 0) > 300 ? "bad" : ""} />
      <Row k="valid_output" v={valid == null ? "--" : valid ? "是" : "否"} cls={valid === false ? "bad" : ""} />
      <Row k="last_control_time" v={fmt(ctrl?.last_control_time, 2)} />
      <Row k="throttle / brake / steer"
        v={`${fmt(ctrl?.throttle, 2)} / ${fmt(ctrl?.brake, 2)} / ${fmt(ctrl?.steer, 3)}`} />
    </div>
  );
}

export function Esp32Panel({ esp }: { esp: Esp32 | null }) {
  const active: ActiveController = esp?.active_controller ?? "none";
  return (
    <div className="card ctrl-panel">
      <div className="ctrl-head">
        <span className="title">ESP32 仲裁器</span>
        <span className={"badge " + controllerBadge(active)}>
          <span className="dot" />{controllerLabel[active]}
        </span>
      </div>
      <Row k="active_controller" v={controllerLabel[active]} />
      <Row k="takeover_count" v={fmtInt(esp?.takeover_count)}
        cls={(esp?.takeover_count ?? 0) > 0 ? "bad" : ""} />
      <Row k="last_takeover_reason" v={esp?.last_takeover_reason ?? "—"} />
      <Row k="safe_brake" v={esp?.safe_brake ? "触发" : "未触发"}
        cls={esp?.safe_brake ? "bad" : "good"} />
      <Row k="output throttle / brake / steer"
        v={`${fmt(esp?.throttle, 2)} / ${fmt(esp?.brake, 2)} / ${fmt(esp?.steer, 3)}`} />
    </div>
  );
}
