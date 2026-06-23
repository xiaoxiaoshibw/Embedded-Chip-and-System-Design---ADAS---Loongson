// 历史实验列表 + 筛选（场景 / 日期 / 是否接管 / 是否碰撞 / PASS-FAIL）。
import { fmt, fmtTtc, resultBadge } from "../lib/format";
import type { RunListItem } from "../types/hil";

export interface RunFilters {
  scenario?: string;
  date?: string;
  takeover?: string;   // "" | "true" | "false"
  collision?: string;
  result?: string;     // "" | "PASS" | "FAIL"
}

interface Props {
  runs: RunListItem[];
  selectedId: string | null;
  filters: RunFilters;
  scenarioOptions: string[];
  onFilterChange: (f: RunFilters) => void;
  onSelect: (id: string) => void;
}

export function RunList(p: Props) {
  const set = (k: keyof RunFilters, v: string) =>
    p.onFilterChange({ ...p.filters, [k]: v || undefined });

  return (
    <div className="card">
      <h3>历史实验（{p.runs.length}）</h3>
      <div className="form-grid" style={{ marginBottom: 10 }}>
        <div className="field">
          <label>场景</label>
          <select value={p.filters.scenario ?? ""} onChange={(e) => set("scenario", e.target.value)}>
            <option value="">全部</option>
            {p.scenarioOptions.map((s) => <option key={s} value={s}>{s}</option>)}
          </select>
        </div>
        <div className="field">
          <label>日期 (YYYYMMDD)</label>
          <input value={p.filters.date ?? ""} placeholder="如 20260622"
            onChange={(e) => set("date", e.target.value)} />
        </div>
        <div className="field">
          <label>是否接管</label>
          <select value={p.filters.takeover ?? ""} onChange={(e) => set("takeover", e.target.value)}>
            <option value="">全部</option><option value="true">已接管</option><option value="false">未接管</option>
          </select>
        </div>
        <div className="field">
          <label>是否碰撞</label>
          <select value={p.filters.collision ?? ""} onChange={(e) => set("collision", e.target.value)}>
            <option value="">全部</option><option value="true">碰撞</option><option value="false">无碰撞</option>
          </select>
        </div>
        <div className="field">
          <label>结果</label>
          <select value={p.filters.result ?? ""} onChange={(e) => set("result", e.target.value)}>
            <option value="">全部</option><option value="PASS">PASS</option><option value="FAIL">FAIL</option>
          </select>
        </div>
      </div>

      <div className="scroll" style={{ maxHeight: 360 }}>
        <table>
          <thead>
            <tr>
              <th>Run ID</th><th>开始时间</th><th>场景</th><th className="right">时长</th>
              <th>结果</th><th>碰撞</th><th>接管</th>
              <th className="right">minTTC</th><th className="right">maxLatErr</th>
            </tr>
          </thead>
          <tbody>
            {p.runs.map((r) => (
              <tr key={r.run_id} className={p.selectedId === r.run_id ? "selected" : ""}
                onClick={() => p.onSelect(r.run_id)}>
                <td className="mono">{r.run_id}</td>
                <td className="mono">{r.start_time ?? "--"}</td>
                <td>{r.scenario}</td>
                <td className="right mono">{fmt(r.duration, 1)}s</td>
                <td><span className={"badge " + resultBadge(r.result)}><span className="dot" />{r.result ?? "--"}</span></td>
                <td>{r.collision ? <span className="badge danger"><span className="dot" />是</span> : "否"}</td>
                <td>{r.takeover_happened ? <span className="badge warn"><span className="dot" />是</span> : "否"}</td>
                <td className="right mono">{fmtTtc(r.min_ttc)}</td>
                <td className="right mono">{fmt(r.max_lateral_error, 2)}</td>
              </tr>
            ))}
            {p.runs.length === 0 && (
              <tr><td colSpan={9} className="faint" style={{ textAlign: "center", cursor: "default" }}>
                暂无记录（先在实时监控页跑一次并停止保存）
              </td></tr>
            )}
          </tbody>
        </table>
      </div>
    </div>
  );
}
