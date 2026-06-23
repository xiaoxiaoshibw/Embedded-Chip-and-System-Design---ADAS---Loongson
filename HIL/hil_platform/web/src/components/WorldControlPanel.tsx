// 自由操控 CARLA 世界（仅真实模式有效）：天气 / NPC 交通流 / 前车接管 / 手动驾驶。
import { useState } from "react";
import { api } from "../api/client";

const WEATHER = ["clear", "rain", "fog", "night"];

export function WorldControlPanel({ mock }: { mock: boolean }) {
  const [weather, setWeather] = useState("clear");
  const [npc, setNpc] = useState(8);
  const [leadKmh, setLeadKmh] = useState<number | "">("");
  const [manualOn, setManualOn] = useState(false);
  const [msg, setMsg] = useState("");
  const [err, setErr] = useState("");

  const run = async (label: string, fn: () => Promise<unknown>) => {
    setErr(""); setMsg("");
    try { const r = await fn(); setMsg(`${label} ✓ ${JSON.stringify(r)}`); }
    catch (e) { setErr((e as Error).message); }
  };

  const toggleManual = async () => {
    const next = !manualOn;
    setManualOn(next);
    await run(next ? "手动接管开" : "手动接管关", () => api.world.manual(next));
  };

  return (
    <div className="card">
      <h3>CARLA 世界辅助</h3>
      <div className="faint" style={{ marginBottom: 8 }}>
        仅用于配置仿真世界和可视化；真实控制链路由上方硬件面板操作 Nano/Gateway。
      </div>
      {mock && <div className="faint" style={{ marginBottom: 8 }}>
        当前 mock 模式——以 <code>HIL_MOCK=0</code> 启动后端并连上 CARLA 后此面板生效。
      </div>}

      <div className="form-grid">
        <div className="field">
          <label>天气</label>
          <div className="btn-row">
            <select value={weather} onChange={(e) => setWeather(e.target.value)}>
              {WEATHER.map((w) => <option key={w} value={w}>{w}</option>)}
            </select>
            <button className="sm" disabled={mock} onClick={() => run("天气", () => api.world.weather(weather))}>应用</button>
          </div>
        </div>

        <div className="field">
          <label>NPC 交通流（辆）</label>
          <div className="btn-row">
            <input type="number" min={0} max={80} value={npc}
              onChange={(e) => setNpc(Number(e.target.value))} style={{ width: 70 }} />
            <button className="sm" disabled={mock} onClick={() => run("生成NPC", () => api.world.spawnNpc(npc))}>生成</button>
            <button className="sm" disabled={mock} onClick={() => run("清除NPC", () => api.world.clearNpc())}>清除</button>
          </div>
        </div>

        <div className="field">
          <label>前车速度接管 (km/h)</label>
          <div className="btn-row">
            <input type="number" value={leadKmh}
              onChange={(e) => setLeadKmh(e.target.value === "" ? "" : Number(e.target.value))}
              placeholder="留空=脚本" style={{ width: 80 }} />
            <button className="sm" disabled={mock}
              onClick={() => run("前车接管", () => api.world.leadSpeed(leadKmh === "" ? null : leadKmh))}>应用</button>
            <button className="sm" disabled={mock}
              onClick={() => { setLeadKmh(""); run("前车恢复脚本", () => api.world.leadSpeed(null)); }}>恢复</button>
          </div>
        </div>

        <div className="field">
          <label>手动驾驶</label>
          <button className={"sm " + (manualOn ? "danger" : "")} disabled={mock} onClick={toggleManual}>
            {manualOn ? "■ 退出手动接管" : "▶ 手动接管 ego"}
          </button>
        </div>
      </div>

      {manualOn && !mock && <ManualPad />}

      {msg && <div className="muted" style={{ marginTop: 8, wordBreak: "break-all" }}>{msg}</div>}
      {err && <div style={{ color: "var(--danger)", marginTop: 8 }}>失败：{err}</div>}
    </div>
  );
}

// 手动驾驶简易控制（点按下发一帧控制量）
function ManualPad() {
  const send = (throttle: number, brake: number, steer: number) =>
    api.world.manualCmd(throttle, brake, steer).catch(() => undefined);
  return (
    <div className="btn-row" style={{ marginTop: 10 }}>
      <button className="sm" onMouseDown={() => send(0.6, 0, 0)} onMouseUp={() => send(0, 0, 0)}>↑ 油门</button>
      <button className="sm" onMouseDown={() => send(0, 0.8, 0)} onMouseUp={() => send(0, 0, 0)}>↓ 刹车</button>
      <button className="sm" onMouseDown={() => send(0.3, 0, -0.4)} onMouseUp={() => send(0, 0, 0)}>← 左</button>
      <button className="sm" onMouseDown={() => send(0.3, 0, 0.4)} onMouseUp={() => send(0, 0, 0)}>→ 右</button>
      <span className="faint" style={{ alignSelf: "center" }}>按住下发，松开回中</span>
    </div>
  );
}
