// 统一 REST 客户端：/live 与 /replay 共用。
// 默认相对路径（开发期由 Vite 代理到 8000，构建后由 FastAPI 同源托管）。
import type {
  RunListItem, RunMeta, RunStates, RunEvent, Summary,
  Status, ScenarioDef, FaultType, StateRow, HardwareHealth,
} from "../types/hil";

const BASE = import.meta.env.VITE_API_BASE ?? "";

async function req<T>(method: string, path: string, body?: unknown): Promise<T> {
  const init: RequestInit = { method, headers: { Accept: "application/json" } };
  if (body !== undefined) {
    init.headers = { ...init.headers, "Content-Type": "application/json" };
    init.body = JSON.stringify(body);
  }
  const resp = await fetch(BASE + path, init);
  if (!resp.ok) {
    let detail = `${resp.status} ${resp.statusText}`;
    try {
      const j = await resp.json();
      if (j?.detail) detail = j.detail;
    } catch { /* ignore */ }
    throw new Error(detail);
  }
  const ctype = resp.headers.get("Content-Type") || "";
  return (ctype.includes("application/json") ? await resp.json() : await resp.text()) as T;
}

export const api = {
  // ── 实时控制 ──
  getStatus: () => req<Status>("GET", "/api/status"),
  getMetrics: () => req<Summary>("GET", "/api/metrics"),
  getScenarios: () => req<{ scenarios: ScenarioDef[] }>("GET", "/api/scenarios"),
  loadScenario: (scenario: string, params?: Record<string, unknown>) =>
    req<{ ok: boolean; status: Status }>("POST", "/api/scenario/load", { scenario, params }),
  start: () => req<{ ok: boolean; status: Status }>("POST", "/api/simulation/start"),
  pause: () => req<{ ok: boolean; status: Status }>("POST", "/api/simulation/pause"),
  stop: () => req<{ ok: boolean; status: Status; meta: RunMeta }>("POST", "/api/simulation/stop"),
  reset: () => req<{ ok: boolean; status: Status }>("POST", "/api/simulation/reset"),
  updateParams: (params: Record<string, unknown>) =>
    req<{ ok: boolean; params: Record<string, number | string> }>(
      "POST", "/api/parameters/update", { params }),
  injectFault: (fault_type: FaultType, target: string) =>
    req<{ ok: boolean; event: RunEvent }>("POST", "/api/fault/inject", { fault_type, target }),

  hardware: {
    health: () => req<HardwareHealth>("GET", "/api/hardware/health"),
    restartAdas: (target: "primary" | "backup" | "both") =>
      req<HardwareHealth>("POST", "/api/hardware/adas/restart", { target }),
    deployAdas: () => req<HardwareHealth>("POST", "/api/hardware/adas/deploy"),
    startGateway: (source: "esp32" | "jetson") =>
      req<HardwareHealth>("POST", "/api/hardware/gateway/start", { source }),
    deployGateway: () => req<HardwareHealth>("POST", "/api/hardware/gateway/deploy"),
    resources: () => req<HardwareHealth>("GET", "/api/hardware/resources"),
    applyCpu: () => req<HardwareHealth>("POST", "/api/hardware/cpu/apply"),
    startEdge: () => req<HardwareHealth>("POST", "/api/hardware/edge/start"),
    syncEdge: () => req<HardwareHealth>("POST", "/api/hardware/edge/sync"),
    stopPerception: () => req<HardwareHealth>("POST", "/api/hardware/perception/stop"),
    startCarla: () => req<HardwareHealth>("POST", "/api/hardware/carla/start"),
    prepareHil: (source: "esp32" | "jetson", deploy = true, carla = true) =>
      req<HardwareHealth>("POST", "/api/hardware/hil/prepare", { source, deploy, carla }),
    restoreNanos: () => req<HardwareHealth>("POST", "/api/hardware/nanos/restore"),
  },

  // ── 历史回放（只读）──
  listRuns: (filters?: Record<string, string>) => {
    const qs = filters ? "?" + new URLSearchParams(filters).toString() : "";
    return req<{ runs: RunListItem[] }>("GET", "/api/runs" + qs);
  },
  runMeta: (id: string) => req<RunMeta>("GET", `/api/runs/${id}/meta`),
  runSummary: (id: string) => req<Summary>("GET", `/api/runs/${id}/summary`),
  runEvents: (id: string) => req<RunEvent[]>("GET", `/api/runs/${id}/events`),
  runStates: (id: string, stride = 1) =>
    req<RunStates>("GET", `/api/runs/${id}/states?stride=${stride}`),
  runStateAt: (id: string, t: number) =>
    req<StateRow>("GET", `/api/runs/${id}/state?t=${t}`),
  runReport: (id: string) => req<string>("GET", `/api/runs/${id}/report`),

  // ── 自由操控世界（仅真实 CARLA 模式有效）──
  world: {
    weather: (weather: string) =>
      req("POST", "/api/world/weather", { weather }),
    spawnNpc: (count: number) =>
      req("POST", "/api/world/npc", { count }),
    clearNpc: () => req("POST", "/api/world/npc/clear"),
    leadSpeed: (kmh: number | null) =>
      req("POST", "/api/world/lead_speed", { kmh }),
    manual: (on: boolean) => req("POST", "/api/world/manual", { on }),
    manualCmd: (throttle: number, brake: number, steer: number) =>
      req("POST", "/api/world/manual_cmd", { throttle, brake, steer }),
  },
};

// 摄像头帧 URL（带时间戳防缓存）；BASE 为空表示同源
export function cameraUrl(): string {
  return `${BASE}/api/world/camera?ts=${Date.now()}`;
}
