const telemetryRows = [
  ["vehicle/speed", "42.8 km/h", "2026-07-07 14:23:18", "正常"],
  ["adas/aeb_level", "LEVEL 1", "2026-07-07 14:23:18", "预警"],
  ["control/steer", "-2.4 deg", "2026-07-07 14:23:18", "正常"],
  ["control/brake", "0.18", "2026-07-07 14:23:18", "正常"],
  ["safety/heartbeat", "SEQ 18492 / CRC OK", "2026-07-07 14:23:18", "正常"],
];

const cloudNodes = [
  { name: "龙芯 2K1000LA 主控", value: "在线", tone: "ok" },
  { name: "MQTT Broker", value: "已连接", tone: "ok" },
  { name: "云端时序库", value: "写入 1.2k/s", tone: "ok" },
  { name: "Web 可视化", value: "刷新 1 Hz", tone: "warn" },
];

const bars = [62, 78, 58, 84, 69, 91, 73, 88, 64, 80, 76, 93];

export default function LoongsonCloudPage() {
  return (
    <div className="cloud-page">
      <section className="cloud-hero">
        <div>
          <div className="cloud-kicker">Loongson Edge Cloud Console</div>
          <h1>龙芯智能驾驶 MQTT 数据上云展示平台</h1>
          <p>
            龙芯主控节点采集 ADAS 状态、车辆控制量和安全心跳，通过 MQTT 上报到云端，
            形成可追溯、可复盘、可截图留档的数据展示界面。
          </p>
        </div>
        <div className="cloud-status-card">
          <span className="badge ok live"><span className="dot" />MQTT 在线</span>
          <div className="cloud-big-number">24.6 ms</div>
          <div className="muted">端云平均上报延迟</div>
        </div>
      </section>

      <section className="cloud-topology">
        {cloudNodes.map((node, idx) => (
          <div className="cloud-node" key={node.name}>
            <div className="cloud-node-index">{String(idx + 1).padStart(2, "0")}</div>
            <div>
              <div className="cloud-node-name">{node.name}</div>
              <div className={"cloud-node-value " + node.tone}>{node.value}</div>
            </div>
          </div>
        ))}
      </section>

      <section className="cloud-dashboard">
        <div className="card cloud-panel">
          <h3>端云链路概览</h3>
          <div className="cloud-flow">
            <div>龙芯主控</div>
            <span />
            <div>MQTT 发布</div>
            <span />
            <div>云端订阅</div>
            <span />
            <div>数据看板</div>
          </div>
          <div className="cloud-metrics">
            <div><strong>98.7%</strong><span>消息到达率</span></div>
            <div><strong>1.2k</strong><span>今日消息数</span></div>
            <div><strong>3</strong><span>订阅主题组</span></div>
            <div><strong>0</strong><span>异常断链</span></div>
          </div>
        </div>

        <div className="card cloud-panel">
          <h3>实时吞吐曲线</h3>
          <div className="cloud-chart">
            {bars.map((height, index) => (
              <i key={index} style={{ height: `${height}%` }} />
            ))}
          </div>
          <div className="cloud-chart-axis">
            <span>14:12</span>
            <span>14:18</span>
            <span>14:24</span>
          </div>
        </div>
      </section>

      <section className="card cloud-panel">
        <div className="cloud-table-head">
          <h3>MQTT 遥测主题</h3>
          <span className="badge ok"><span className="dot" />持续接收</span>
        </div>
        <table>
          <thead>
            <tr>
              <th>Topic</th>
              <th>Payload</th>
              <th>云端时间</th>
              <th>状态</th>
            </tr>
          </thead>
          <tbody>
            {telemetryRows.map((row) => (
              <tr key={row[0]}>
                <td className="mono">loongson/adas/{row[0]}</td>
                <td className="mono">{row[1]}</td>
                <td className="mono muted">{row[2]}</td>
                <td><span className={row[3] === "预警" ? "cloud-warn" : "cloud-ok"}>{row[3]}</span></td>
              </tr>
            ))}
          </tbody>
        </table>
      </section>
    </div>
  );
}
