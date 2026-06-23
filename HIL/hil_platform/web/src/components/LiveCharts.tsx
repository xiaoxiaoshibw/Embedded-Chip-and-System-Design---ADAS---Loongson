// 实时曲线：speed / TTC / front_distance / lateral_error / brake / active_controller。
// 数据源为 store 的最近 60s 采样，避免浏览器卡死。
import { useMemo } from "react";
import { EChart } from "./EChart";
import { useLiveStore, type Sample } from "../store";

const AXIS = { color: "#5c6b7d" };
const GRID = { left: 44, right: 12, top: 24, bottom: 22 };

function lineOption(
  samples: Sample[],
  pick: (s: Sample) => number | null,
  color: string,
  opts: { ttc?: boolean; step?: boolean; yMax?: number; yMin?: number } = {},
) {
  const data = samples.map((s) => {
    let y = pick(s);
    if (opts.ttc && (y == null || !Number.isFinite(y))) y = null; // ∞ 不画
    return [s.t, y];
  });
  return {
    animation: false,
    grid: GRID,
    tooltip: { trigger: "axis" },
    xAxis: {
      type: "value", min: "dataMin", max: "dataMax",
      axisLabel: { color: AXIS.color, formatter: (v: number) => v.toFixed(0) + "s" },
      axisLine: { lineStyle: { color: "#243343" } },
    },
    yAxis: {
      type: "value", scale: !opts.step,
      min: opts.yMin, max: opts.yMax,
      axisLabel: {
        color: AXIS.color,
        formatter: opts.step
          ? (v: number) => (["A", "B", "SAFE"][v] ?? "")
          : undefined,
      },
      splitLine: { lineStyle: { color: "#1b2733" } },
    },
    series: [{
      type: "line", data, showSymbol: false,
      step: opts.step ? "end" : false,
      lineStyle: { color, width: 2 },
      areaStyle: opts.step ? undefined : { color, opacity: 0.08 },
    }],
  };
}

function Mini({ title, children }: { title: string; children: React.ReactNode }) {
  return (
    <div className="card" style={{ padding: "8px 10px" }}>
      <h3 style={{ marginBottom: 4 }}>{title}</h3>
      {children}
    </div>
  );
}

export function LiveCharts() {
  const history = useLiveStore((s) => s.history);

  const opts = useMemo(() => ({
    speed: lineOption(history, (s) => s.speed, "#2f81f7"),
    ttc: lineOption(history, (s) => s.ttc, "#9b59b6", { ttc: true }),
    dist: lineOption(history, (s) => s.front_distance, "#1abc9c"),
    lat: lineOption(history, (s) => s.lateral_error, "#f1c40f"),
    brake: lineOption(history, (s) => s.brake, "#e74c3c", { yMin: 0, yMax: 1 }),
    active: lineOption(history, (s) => s.active, "#e67e22", { step: true, yMin: 0, yMax: 2 }),
  }), [history]);

  return (
    <div className="grid" style={{ gridTemplateColumns: "repeat(auto-fit, minmax(280px, 1fr))" }}>
      <Mini title="Ego 速度 (km/h)"><EChart option={opts.speed} height={150} /></Mini>
      <Mini title="TTC (s)"><EChart option={opts.ttc} height={150} /></Mini>
      <Mini title="前车距离 (m)"><EChart option={opts.dist} height={150} /></Mini>
      <Mini title="横向误差 (m)"><EChart option={opts.lat} height={150} /></Mini>
      <Mini title="刹车"><EChart option={opts.brake} height={150} /></Mini>
      <Mini title="生效控制器"><EChart option={opts.active} height={150} /></Mini>
    </div>
  );
}
