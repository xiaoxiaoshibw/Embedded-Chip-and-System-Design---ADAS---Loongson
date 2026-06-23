// 回放时间轴：play/pause + 0.5x/1x/2x/4x + 拖动 + 跳转（接管 / 最小TTC / 最大横向误差）。
const SPEEDS = [0.5, 1, 2, 4];

export interface JumpTargets {
  takeover: number | null;
  minTtc: number | null;
  maxLat: number | null;
}

interface Props {
  duration: number;
  currentTime: number;
  playing: boolean;
  speed: number;
  jumps: JumpTargets;
  onSeek: (t: number) => void;
  onTogglePlay: () => void;
  onSpeed: (s: number) => void;
}

export function ReplayTimeline(p: Props) {
  return (
    <div className="card timeline">
      <div className="controls">
        <button className="primary sm" onClick={p.onTogglePlay}>
          {p.playing ? "⏸ 暂停" : "▶ 播放"}
        </button>
        <span className="tlabel">{p.currentTime.toFixed(2)} / {p.duration.toFixed(2)} s</span>
        <span className="faint">倍速</span>
        {SPEEDS.map((s) => (
          <button key={s} className={"sm " + (p.speed === s ? "primary" : "")}
            onClick={() => p.onSpeed(s)}>{s}x</button>
        ))}
        <div className="spacer" style={{ flex: 1 }} />
        <button className="sm" disabled={p.jumps.takeover == null}
          onClick={() => p.jumps.takeover != null && p.onSeek(p.jumps.takeover)}>跳到接管时刻</button>
        <button className="sm" disabled={p.jumps.minTtc == null}
          onClick={() => p.jumps.minTtc != null && p.onSeek(p.jumps.minTtc)}>跳到最小 TTC</button>
        <button className="sm" disabled={p.jumps.maxLat == null}
          onClick={() => p.jumps.maxLat != null && p.onSeek(p.jumps.maxLat)}>跳到最大横向误差</button>
      </div>
      <input
        type="range" min={0} max={p.duration || 0} step={0.05}
        value={Math.min(p.currentTime, p.duration || 0)}
        onChange={(e) => p.onSeek(Number(e.target.value))}
      />
    </div>
  );
}
