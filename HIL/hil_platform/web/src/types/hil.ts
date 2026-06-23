// 与后端 core/types.py、server 接口一一对应的类型定义。
// /live 与 /replay 共用本文件。

export type SimStateName =
  | "IDLE" | "READY" | "RUNNING" | "PAUSED" | "STOPPED" | "ERROR";

export type ActiveController = "nano_a" | "nano_b" | "safe_brake" | "none";

export type FaultType =
  | "heartbeat_loss" | "seq_stuck" | "nan_output"
  | "control_delay" | "backup_fail" | "dual_fail";

export interface Ego {
  speed_kmh: number | null;
  throttle: number | null;
  brake: number | null;
  steer: number | null;
  lateral_error: number | null;
  heading_error: number | null;
}

export interface Target {
  front_distance: number | null;
  relative_speed: number | null;
  ttc: number | null;
}

export interface Controller {
  alive: boolean;
  seq: number;
  latency_ms: number | null;
  valid_output: boolean;
  last_control_time: number | null;
  throttle: number | null;
  brake: number | null;
  steer: number | null;
}

export interface Esp32 {
  active_controller: ActiveController;
  takeover_count: number;
  last_takeover_reason: string | null;
  safe_brake: boolean;
  throttle: number | null;
  brake: number | null;
  steer: number | null;
}

// /ws/live 推送的一帧
export interface LiveFrame {
  run_id: string | null;
  timestamp: number | null;
  scenario: string | null;
  state: SimStateName;
  ego: Ego | null;
  target: Target | null;
  nano_a: Controller | null;
  nano_b: Controller | null;
  esp32: Esp32 | null;
  event: string | null;
}

// /api/status
export interface Status {
  state: SimStateName;
  mock: boolean;
  control_source?: "mock" | "internal" | "nano" | "unknown";
  run_id: string | null;
  scenario: string | null;
  scenario_title: string | null;
  map: string | null;
  scenario_time: number;
  active_controller: ActiveController;
  takeover: boolean;
  safe_brake: boolean;
  params: Record<string, number | string>;
  active_faults: { type: string; target: string; trigger_time: number }[];
  frame_count: number;
  error?: string | null;
}

// /api/metrics 与 summary.json
export interface Summary {
  result: "PASS" | "FAIL" | null;
  collision: boolean;
  min_ttc: number | null;
  max_lateral_error: number | null;
  takeover_happened: boolean;
  takeover_latency_ms: number | null;
  active_controller_final: ActiveController;
  safe_brake_triggered: boolean;
  conclusion: string;
  scenario_time?: number;
}

export interface ScenarioDef {
  name: string;
  title?: string;
  map?: string;
  description?: string;
  default_params?: Record<string, number | string>;
}

// 历史列表项
export interface RunListItem {
  run_id: string;
  start_time: string | null;
  scenario: string | null;
  duration: number | null;
  result: "PASS" | "FAIL" | null;
  collision: boolean | null;
  takeover_happened: boolean | null;
  min_ttc: number | null;
  max_lateral_error: number | null;
}

export interface RunMeta {
  run_id: string;
  scenario: string;
  map: string;
  start_time: string;
  duration: number;
  config: Record<string, number | string>;
}

export interface RunEvent {
  time: number;
  type: string;
  [k: string]: unknown;
}

// states.csv 解析后的一行
export interface StateRow {
  t: number;
  ego_speed: number | null;
  front_distance: number | null;
  relative_speed: number | null;
  ttc: number | null;
  lateral_error: number | null;
  heading_error: number | null;
  throttle: number | null;
  brake: number | null;
  steer: number | null;
  nano_a_alive: number;
  nano_a_seq: number;
  nano_a_latency_ms: number | null;
  nano_a_valid_output: number;
  nano_b_alive: number;
  nano_b_seq: number;
  nano_b_latency_ms: number | null;
  nano_b_valid_output: number;
  active_controller: ActiveController;
  takeover_count: number;
  safe_brake: number;
  event: string;
}

export interface RunStates {
  fields: string[];
  states: StateRow[];
}

export interface HardwareCommandResult {
  target: string;
  host: string;
  ok: boolean;
  rc: number | null;
  stdout: string;
  stderr: string;
  elapsed_ms: number;
}

export interface HardwareHealth {
  ok: boolean;
  primary?: HardwareCommandResult;
  backup?: HardwareCommandResult;
  [key: string]: unknown;
}
