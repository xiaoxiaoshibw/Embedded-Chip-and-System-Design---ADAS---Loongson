// 顶部状态栏：run_id / 场景 / 仿真状态 / scenario_time / 生效控制器 / 是否接管 / 是否安全制动。
// /live 与 /replay 都能用（传入对应字段即可）。
import { controllerBadge, controllerLabel, stateBadge } from "../lib/format";
import type { ActiveController, SimStateName } from "../types/hil";

interface Props {
  runId: string | null;
  scenario: string | null;
  state: SimStateName | null;
  scenarioTime: number | null;
  activeController: ActiveController;
  takeover: boolean;
  safeBrake: boolean;
  connected?: boolean;
}

function Item({ k, children }: { k: string; children: React.ReactNode }) {
  return (
    <div className="item">
      <span className="k">{k}</span>
      <span className="v">{children}</span>
    </div>
  );
}

export function StatusBar(p: Props) {
  return (
    <div className="statusbar">
      <Item k="Run ID">{p.runId ?? "--"}</Item>
      <Item k="场景">{p.scenario ?? "--"}</Item>
      <Item k="仿真状态">
        <span className={"badge " + stateBadge(p.state ?? "IDLE")}>
          <span className="dot" />{p.state ?? "--"}
        </span>
      </Item>
      <Item k="场景时间">{p.scenarioTime != null ? p.scenarioTime.toFixed(2) + " s" : "--"}</Item>
      <Item k="生效控制器">
        <span className={"badge " + controllerBadge(p.activeController)}>
          <span className="dot" />{controllerLabel[p.activeController]}
        </span>
      </Item>
      <Item k="是否接管">
        <span className={"badge " + (p.takeover ? "warn" : "ok")}>
          <span className="dot" />{p.takeover ? "已接管" : "正常"}
        </span>
      </Item>
      <Item k="安全制动">
        <span className={"badge " + (p.safeBrake ? "danger" : "ok")}>
          <span className="dot" />{p.safeBrake ? "触发" : "未触发"}
        </span>
      </Item>
      {p.connected !== undefined && (
        <Item k="链路">
          <span className={"badge " + (p.connected ? "ok live" : "danger")}>
            <span className="dot" />{p.connected ? "已连接" : "断开"}
          </span>
        </Item>
      )}
    </div>
  );
}
