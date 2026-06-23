// 场景参数面板（/live 专用）：选择场景 + 编辑参数 + 加载/热更新。
import { useEffect, useState } from "react";
import type { ScenarioDef } from "../types/hil";

// 可编辑参数定义（label, key, 类型）
const NUM_FIELDS: { key: string; label: string; step?: number }[] = [
  { key: "ego_speed", label: "自车目标速度 (km/h)" },
  { key: "front_distance", label: "前车初始距离 (m)" },
  { key: "front_speed", label: "前车速度 (km/h)" },
  { key: "cut_in_speed", label: "切入车速度 (km/h)" },
  { key: "cut_in_trigger_distance", label: "切入触发距离 (m)" },
  { key: "comm_delay_ms", label: "通信延迟 (ms)" },
  { key: "sensor_noise", label: "传感器噪声", step: 0.01 },
  { key: "fault_trigger_time", label: "故障触发时刻 (s)" },
];
const WEATHER = ["clear", "rain", "fog", "night"];

interface Props {
  scenarios: ScenarioDef[];
  params: Record<string, number | string>;
  selectedScenario: string;
  onScenarioChange: (s: string) => void;
  onLoad: (params: Record<string, number | string>) => void;
  onUpdate: (params: Record<string, number | string>) => void;
  canEdit: boolean;       // load 仅在非 RUNNING 可用
  running: boolean;       // update 在 RUNNING 时也可用（热更新）
}

export function ParameterPanel(p: Props) {
  const [local, setLocal] = useState<Record<string, number | string>>(p.params);

  // 后端参数变化（加载场景/复位后）同步到本地编辑态
  useEffect(() => { setLocal(p.params); }, [p.params]);

  const set = (k: string, v: number | string) =>
    setLocal((s) => ({ ...s, [k]: v }));

  return (
    <div className="card">
      <h3>场景参数</h3>
      <div className="field" style={{ marginBottom: 10 }}>
        <label>场景</label>
        <select value={p.selectedScenario} onChange={(e) => p.onScenarioChange(e.target.value)}>
          {p.scenarios.map((s) => (
            <option key={s.name} value={s.name}>{s.title ?? s.name}</option>
          ))}
        </select>
      </div>

      <div className="form-grid">
        {NUM_FIELDS.map((f) => (
          <div className="field" key={f.key}>
            <label>{f.label}</label>
            <input
              type="number"
              step={f.step ?? 1}
              value={local[f.key] ?? ""}
              onChange={(e) => set(f.key, e.target.value === "" ? "" : Number(e.target.value))}
            />
          </div>
        ))}
        <div className="field">
          <label>天气 weather</label>
          <select value={String(local.weather ?? "clear")} onChange={(e) => set("weather", e.target.value)}>
            {WEATHER.map((w) => <option key={w} value={w}>{w}</option>)}
          </select>
        </div>
      </div>

      <div className="btn-row" style={{ marginTop: 12 }}>
        <button className="primary" disabled={!p.canEdit} onClick={() => p.onLoad(local)}>
          加载场景
        </button>
        <button disabled={!p.running} onClick={() => p.onUpdate(local)} title="运行中热更新非安全关键参数">
          应用参数（热更新）
        </button>
      </div>
      {!p.canEdit && <div className="faint" style={{ marginTop: 8 }}>运行中不可重新加载场景，请先停止。</div>}
    </div>
  );
}
