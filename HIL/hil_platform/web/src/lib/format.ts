// 共用格式化工具：所有数值都做空值保护，无数据显示 "--"。
import type { ActiveController, SimStateName } from "../types/hil";

export function fmt(v: number | null | undefined, digits = 2): string {
  if (v == null || Number.isNaN(v) || !Number.isFinite(v)) return "--";
  return v.toFixed(digits);
}

export function fmtInt(v: number | null | undefined): string {
  if (v == null || Number.isNaN(v)) return "--";
  return String(Math.round(v));
}

export function fmtTtc(v: number | null | undefined): string {
  if (v == null || !Number.isFinite(v)) return "∞";
  return v.toFixed(2);
}

// 控制器 → 中文标签 + 徽章样式
export const controllerLabel: Record<ActiveController, string> = {
  nano_a: "Nano A 主控",
  nano_b: "Nano B 接管",
  safe_brake: "安全制动",
  none: "未运行",
};

export function controllerBadge(c: ActiveController): string {
  if (c === "nano_a") return "ok";
  if (c === "nano_b") return "warn";
  if (c === "safe_brake") return "danger";
  return "idle";
}

// 仿真状态 → 徽章样式
export function stateBadge(s: SimStateName): string {
  switch (s) {
    case "RUNNING": return "ok live";
    case "PAUSED": return "warn";
    case "ERROR": return "danger";
    case "READY": return "ok";
    default: return "idle";
  }
}

export function resultBadge(r: string | null | undefined): string {
  if (r === "PASS") return "ok";
  if (r === "FAIL") return "danger";
  return "idle";
}

export function boolBadge(v: boolean | null | undefined, dangerWhenTrue = true): string {
  if (v == null) return "idle";
  if (v) return dangerWhenTrue ? "danger" : "ok";
  return dangerWhenTrue ? "ok" : "idle";
}
