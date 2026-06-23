// /live 页的实时状态管理（Zustand）。
// 维护：连接状态、最新帧、最近 60s 历史采样、/api/status 快照。
import { create } from "zustand";
import type { LiveFrame, Status } from "./types/hil";

// 实时曲线只保留最近 60 秒，避免浏览器卡死
const WINDOW_S = 60;

export interface Sample {
  t: number;
  speed: number | null;
  ttc: number | null;
  front_distance: number | null;
  lateral_error: number | null;
  brake: number | null;
  active: number; // 0=nano_a 1=nano_b 2=safe_brake
}

const ACTIVE_CODE: Record<string, number> = { nano_a: 0, nano_b: 1, safe_brake: 2, none: 0 };

interface LiveState {
  connected: boolean;
  frame: LiveFrame | null;
  status: Status | null;
  history: Sample[];
  lastTimestamp: number | null;
  setConnected: (c: boolean) => void;
  setStatus: (s: Status) => void;
  ingest: (f: LiveFrame) => void;
  clearHistory: () => void;
}

export const useLiveStore = create<LiveState>((set) => ({
  connected: false,
  frame: null,
  status: null,
  history: [],
  lastTimestamp: null,
  setConnected: (c) => set({ connected: c }),
  setStatus: (s) => set({ status: s }),
  clearHistory: () => set({ history: [], lastTimestamp: null }),
  ingest: (f) =>
    set((st) => {
      const next: Partial<LiveState> = { frame: f };
      const t = f.timestamp;
      // 仅在 RUNNING 且有有效时间戳时累计曲线；时间回退（新 run/复位）则清空
      if (f.state === "RUNNING" && t != null && f.ego) {
        let history = st.history;
        if (st.lastTimestamp != null && t < st.lastTimestamp) history = [];
        const sample: Sample = {
          t,
          speed: f.ego.speed_kmh,
          ttc: f.target?.ttc ?? null,
          front_distance: f.target?.front_distance ?? null,
          lateral_error: f.ego.lateral_error,
          brake: f.esp32?.brake ?? null,
          active: ACTIVE_CODE[f.esp32?.active_controller ?? "none"] ?? 0,
        };
        const trimmed = [...history, sample].filter((s) => s.t >= t - WINDOW_S);
        next.history = trimmed;
        next.lastTimestamp = t;
      }
      return next;
    }),
}));
