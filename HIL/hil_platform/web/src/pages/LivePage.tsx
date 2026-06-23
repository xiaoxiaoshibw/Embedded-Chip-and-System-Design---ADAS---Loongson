// 实时监控页：比赛现场展示 + 控制仿真。
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { api } from "../api/client";
import { LiveSocket } from "../api/websocket";
import { useLiveStore } from "../store";
import { StatusBar } from "../components/StatusBar";
import { MetricCards, type MetricValues } from "../components/MetricCards";
import { NanoPanel, Esp32Panel } from "../components/ControllerStatus";
import { ParameterPanel } from "../components/ParameterPanel";
import { FaultPanel } from "../components/FaultPanel";
import { LiveCharts } from "../components/LiveCharts";
import { CameraView } from "../components/CameraView";
import { WorldControlPanel } from "../components/WorldControlPanel";
import { HardwareControlPanel } from "../components/HardwareControlPanel";
import type { ScenarioDef, Status } from "../types/hil";

export default function LivePage() {
  const { connected, frame, status, setConnected, setStatus, ingest } = useLiveStore();
  const [scenarios, setScenarios] = useState<ScenarioDef[]>([]);
  const [selected, setSelected] = useState<string>("");
  const [formParams, setFormParams] = useState<Record<string, number | string>>({});
  const [error, setError] = useState<string>("");

  // ── WebSocket 连接（自动重连）──
  useEffect(() => {
    const sock = new LiveSocket({ onFrame: ingest, onStatus: setConnected });
    sock.connect();
    return () => sock.close();
  }, [ingest, setConnected]);

  // ── 拉取状态（params/故障）──
  const refreshStatus = useCallback(async () => {
    try {
      const s = await api.getStatus();
      setStatus(s);
      return s;
    } catch { return null; }
  }, [setStatus]);

  // 初始化：场景列表 + 状态
  const inited = useRef(false);
  useEffect(() => {
    (async () => {
      try {
        const [sc, st] = await Promise.all([api.getScenarios(), api.getStatus()]);
        setScenarios(sc.scenarios);
        setStatus(st);
        const initSel = st.scenario || sc.scenarios[0]?.name || "";
        setSelected(initSel);
        setFormParams(st.scenario ? st.params : defaultsOf(sc.scenarios, initSel));
        inited.current = true;
      } catch (e) { setError((e as Error).message); }
    })();
  }, [setStatus]);

  // 轮询状态（参数/故障变化）
  useEffect(() => {
    const id = window.setInterval(refreshStatus, 2000);
    return () => window.clearInterval(id);
  }, [refreshStatus]);

  // ── 控制动作 ──
  const guarded = async (fn: () => Promise<unknown>) => {
    setError("");
    try { await fn(); await refreshStatus(); }
    catch (e) { setError((e as Error).message); }
  };
  const onLoad = (params: Record<string, number | string>) =>
    guarded(async () => {
      const r = await api.loadScenario(selected, params);
      setFormParams(r.status.params);
    });
  const onUpdate = (params: Record<string, number | string>) =>
    guarded(() => api.updateParams(params));
  const onScenarioChange = (s: string) => {
    setSelected(s);
    setFormParams(defaultsOf(scenarios, s));
  };

  const state = frame?.state ?? status?.state ?? "IDLE";
  const running = state === "RUNNING";
  const canEdit = state !== "RUNNING";

  const metricValues: MetricValues = useMemo(() => ({
    speed_kmh: frame?.ego?.speed_kmh ?? null,
    front_distance: frame?.target?.front_distance ?? null,
    relative_speed: frame?.target?.relative_speed ?? null,
    ttc: frame?.target?.ttc ?? null,
    lateral_error: frame?.ego?.lateral_error ?? null,
    heading_error: frame?.ego?.heading_error ?? null,
    throttle: frame?.ego?.throttle ?? null,
    brake: frame?.ego?.brake ?? null,
    steer: frame?.ego?.steer ?? null,
  }), [frame]);

  const esp = frame?.esp32 ?? null;
  const mock = status?.mock ?? true;

  return (
    <div className="page">
      {!connected && <div className="disconnected-banner">⚠ 实时链路断开，正在自动重连…</div>}
      {status?.error && <div className="disconnected-banner">⚠ {status.error}（请检查 CARLA，必要时复位）</div>}

      <StatusBar
        runId={frame?.run_id ?? status?.run_id ?? null}
        scenario={frame?.scenario ?? status?.scenario ?? null}
        state={state}
        scenarioTime={frame?.timestamp ?? status?.scenario_time ?? null}
        activeController={esp?.active_controller ?? status?.active_controller ?? "none"}
        takeover={esp ? (esp.active_controller === "nano_b" || esp.active_controller === "safe_brake") : (status?.takeover ?? false)}
        safeBrake={esp?.safe_brake ?? status?.safe_brake ?? false}
        connected={connected}
      />

      {/* 控制按钮 */}
      <div className="card">
        <div className="btn-row">
          <button className="primary" disabled={state !== "READY" && state !== "PAUSED"}
            onClick={() => guarded(api.start)}>▶ 开始</button>
          <button disabled={state !== "RUNNING"} onClick={() => guarded(api.pause)}>⏸ 暂停</button>
          <button className="danger" disabled={state !== "RUNNING" && state !== "PAUSED"}
            onClick={() => guarded(api.stop)}>⏹ 停止并保存</button>
          <button className="ghost" onClick={() => guarded(api.reset)}>↺ 复位</button>
          {error && <span style={{ color: "var(--danger)", alignSelf: "center" }}>错误：{error}</span>}
        </div>
      </div>

      <MetricCards v={metricValues} />

      <HardwareControlPanel controlSource={status?.control_source} />

      {/* CARLA 只作为真值世界、感知输入和闭环执行显示 */}
      <div className="row">
        <div style={{ flex: "2 1 460px" }}><CameraView active={!mock} /></div>
        <div style={{ flex: "1 1 320px" }}><WorldControlPanel mock={mock} /></div>
      </div>

      <div className="row">
        <div style={{ flex: "1 1 240px" }}><NanoPanel name="Nano A（主控）" ctrl={frame?.nano_a ?? null} active={esp?.active_controller === "nano_a"} /></div>
        <div style={{ flex: "1 1 240px" }}><NanoPanel name="Nano B（备控）" ctrl={frame?.nano_b ?? null} active={esp?.active_controller === "nano_b"} /></div>
        <div style={{ flex: "1 1 240px" }}><Esp32Panel esp={esp} /></div>
      </div>

      <div className="row">
        <div style={{ flex: "2 1 420px" }}>
          <ParameterPanel
            scenarios={scenarios} params={formParams} selectedScenario={selected}
            onScenarioChange={onScenarioChange} onLoad={onLoad} onUpdate={onUpdate}
            canEdit={canEdit} running={running}
          />
        </div>
        <div style={{ flex: "1 1 320px" }}><FaultPanel disabled={!running} /></div>
      </div>

      <LiveCharts />
    </div>
  );
}

function defaultsOf(scenarios: ScenarioDef[], name: string): Record<string, number | string> {
  const s = scenarios.find((x) => x.name === name);
  return s?.default_params ? { ...s.default_params } : {};
}
