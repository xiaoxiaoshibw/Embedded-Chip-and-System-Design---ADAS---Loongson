// 自动重连的 /ws/live 客户端。
// 断线时回调 onStatus(false)，并按退避重连；不因临时无数据而抛错。
import type { LiveFrame } from "../types/hil";

export interface LiveSocketHandlers {
  onFrame: (f: LiveFrame) => void;
  onStatus: (connected: boolean) => void;
}

function wsUrl(): string {
  const override = import.meta.env.VITE_WS_URL as string | undefined;
  if (override) return override;
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${window.location.host}/ws/live`;
}

export class LiveSocket {
  private ws: WebSocket | null = null;
  private closedByUser = false;
  private retry = 0;
  private timer: number | null = null;

  constructor(private handlers: LiveSocketHandlers) {}

  connect() {
    this.closedByUser = false;
    this.open();
  }

  private open() {
    try {
      this.ws = new WebSocket(wsUrl());
    } catch {
      this.scheduleReconnect();
      return;
    }
    this.ws.onopen = () => {
      this.retry = 0;
      this.handlers.onStatus(true);
    };
    this.ws.onmessage = (ev) => {
      try {
        const frame = JSON.parse(ev.data) as LiveFrame;
        this.handlers.onFrame(frame);
      } catch {
        /* 忽略坏帧，绝不崩溃 */
      }
    };
    this.ws.onclose = () => {
      this.handlers.onStatus(false);
      if (!this.closedByUser) this.scheduleReconnect();
    };
    this.ws.onerror = () => {
      // onclose 会随后触发，这里不重复处理
      try { this.ws?.close(); } catch { /* ignore */ }
    };
  }

  private scheduleReconnect() {
    this.retry = Math.min(this.retry + 1, 6);
    const delay = Math.min(500 * 2 ** (this.retry - 1), 8000); // 0.5s→8s 退避
    if (this.timer) window.clearTimeout(this.timer);
    this.timer = window.setTimeout(() => this.open(), delay);
  }

  close() {
    this.closedByUser = true;
    if (this.timer) window.clearTimeout(this.timer);
    try { this.ws?.close(); } catch { /* ignore */ }
    this.ws = null;
  }
}
