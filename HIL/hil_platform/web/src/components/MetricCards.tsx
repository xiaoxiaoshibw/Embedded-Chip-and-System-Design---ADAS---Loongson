// 实时指标卡片：Ego 速度 / 前车距离 / 相对速度 / TTC / 横向误差 / 航向误差 / 油门 / 刹车 / 转向。
// /live 用 LiveFrame，/replay 用某时刻的 StateRow，故统一成入参字段。
import { fmt, fmtTtc } from "../lib/format";

export interface MetricValues {
  speed_kmh: number | null;
  front_distance: number | null;
  relative_speed: number | null;
  ttc: number | null;
  lateral_error: number | null;
  heading_error: number | null;
  throttle: number | null;
  brake: number | null;
  steer: number | null;
}

function Card({ label, value, unit, cls }: {
  label: string; value: string; unit?: string; cls?: string;
}) {
  return (
    <div className={"metric " + (cls ?? "")}>
      <span className="label">{label}</span>
      <span className="value">{value}{unit && <span className="unit">{unit}</span>}</span>
    </div>
  );
}

export function MetricCards({ v }: { v: MetricValues }) {
  // TTC < 2s 标红，< 4s 标黄；横向误差 > 0.5 标黄
  const ttcCls = v.ttc != null && Number.isFinite(v.ttc)
    ? (v.ttc < 2 ? "alert" : v.ttc < 4 ? "warnv" : "")
    : "";
  const latCls = v.lateral_error != null && Math.abs(v.lateral_error) > 0.5 ? "warnv" : "";
  const brakeCls = v.brake != null && v.brake > 0.6 ? "alert" : "";

  return (
    <div className="metrics-grid">
      <Card label="Ego 速度" value={fmt(v.speed_kmh, 1)} unit="km/h" />
      <Card label="前车距离" value={fmt(v.front_distance, 1)} unit="m" />
      <Card label="相对速度" value={fmt(v.relative_speed, 1)} unit="km/h" />
      <Card label="TTC" value={fmtTtc(v.ttc)} unit="s" cls={ttcCls} />
      <Card label="横向误差" value={fmt(v.lateral_error, 3)} unit="m" cls={latCls} />
      <Card label="航向误差" value={fmt(v.heading_error, 3)} unit="rad" />
      <Card label="油门" value={fmt(v.throttle, 2)} cls="" />
      <Card label="刹车" value={fmt(v.brake, 2)} cls={brakeCls} />
      <Card label="转向" value={fmt(v.steer, 3)} />
    </div>
  );
}
