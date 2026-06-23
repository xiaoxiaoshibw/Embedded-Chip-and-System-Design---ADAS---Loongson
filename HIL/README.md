# HIL — ADAS 硬件在环（一站式目录）

> 本目录把**所有 HIL（硬件在环）相关代码集中在一处**，不再分散。
> 一句话：**Web 直接操作两台 Jetson Nano（真实 ADAS 主备控制），CARLA 只负责真实世界仿真 + 感知输入 + 接收 Nano 计算结果做闭环。**

```
HIL/
├── hil_platform/     ← 上层：监控 / 回放 / 控制平台（WebUI + 后端 + CLI）
│   ├── web/              WebUI —— React + TS + Vite（/live 实时可控，/replay 历史回放）
│   ├── core/            后端核心 —— SimulationCore 唯一持有底层；mock / 真实CARLA / 真实Nano 三种桥
│   ├── server/         FastAPI —— REST + WebSocket(/ws/live) + 世界/硬件控制接口
│   ├── cli/            hilctl —— 命令行客户端（与 WebUI 共用同一套 API）
│   ├── configs/       5 个演示场景 YAML（acc/aeb/lka/cut_in/takeover）
│   ├── runs/          历史录制（每次 stop 落盘，供 /replay）
│   └── README.md      ← 平台详细说明、API 列表、三种模式切换
│
├── carla_bridge/     ← 底层：CARLA 世界端 + Nano 网关桥（被 hil_platform 复用）
│   ├── pc/              Windows PC 侧（CARLA 客户端 Py3.12）：carla_link / bridge_config / scenarios / hil_carla_bridge
│   ├── nano/           部署到 Nano 的网关与进程脚本：hil_ros_gateway / start_hil_adas / restart_adas / stop_gateway
│   ├── launch/         Windows 一键 .bat + 7 步编排 .ps1
│   ├── tools/          nano_ssh / upload（SSH 执行 + sftp 上传）
│   ├── logs/           桥运行 CSV
│   └── CLAUDE.md / README.md  ← 桥详细说明、网络与节点
│
└── _legacy/          ← 归档：被取代的旧原型（HIL闭环/），可删
```

## 三种运行模式（由后端 `core/simulation_core.py` 选桥）

| 模式 | 环境变量 | 谁在算控制 | 用途 |
|---|---|---|---|
| **mock** | `HIL_MOCK=1`（默认） | 进程内模拟 | 无任何硬件即可演示全套 WebUI / 回放 |
| **真实 CARLA + 内置控制** | `HIL_MOCK=0`, `HIL_CONTROL=internal` | PC 上内置 Controller | 有 CARLA、无 Nano 时的闭环 |
| **真实 Nano（完整 HIL）** | `HIL_MOCK=0`, `HIL_CONTROL=nano` | **两台 Jetson Nano 真实 ADAS** | 比赛现场：Web 控两台 Nano，CARLA 做世界/感知/闭环 |

> 数据面走 **ZeroTier**（PC `10.218.44.190` ↔ primary Nano `10.218.44.10:42110`）；NAT 跳板 `10.18.52.130` 只转发 SSH。详见 `carla_bridge/CLAUDE.md` 网络章节。

## 快速开始

### A. 纯演示（mock，无硬件）
```powershell
cd HIL\hil_platform
python -m uvicorn server.api_server:app --port 8000      # 后端（默认 mock）
# 另开窗口：前端开发服务器
npm --prefix web run dev                                  # http://127.0.0.1:5173
```
构建后前端由后端同源托管：`/live`（实时控制）、`/replay`（历史回放）。

### B. 完整 HIL（真实双 Nano + CARLA）
```powershell
# 1) 起 CARLA（仓库根 CALRA\CarlaUE4.exe）并让 Nano 上 gateway+ADAS 就绪
cd HIL\carla_bridge
.\launch\一键启动HIL闭环.bat esp32 acc      # 或分步见 carla_bridge/README.md
# 2) 起平台后端（真实 Nano 模式），WebUI 即直接操作两台 Nano
cd ..\hil_platform
$env:HIL_MOCK="0"; $env:HIL_CONTROL="nano"
python -m uvicorn server.api_server:app --port 8000
```

## 谁复用谁

- `hil_platform/core/carla_world.py` 把 `carla_bridge/pc/` 加入 `sys.path`，**直接复用** `CarlaLink`（不重写场景代码）。
- `hil_platform/core/nano_link.py` 复用 `carla_bridge/nano/hil_ros_gateway.py` 的 **TCP 42110** 协议读回 ESP32 仲裁后的最终控制 + 主备角色 + 接管信息。
- 所以两半不是并列的两个项目，而是**同一套 HIL 系统的「平台层 / 桥层」**。

## 详细文档

- 平台（WebUI/后端/CLI/API）：[`hil_platform/README.md`](hil_platform/README.md)
- 桥（CARLA/Nano/网络/一键启动）：[`carla_bridge/README.md`](carla_bridge/README.md)、[`carla_bridge/CLAUDE.md`](carla_bridge/CLAUDE.md)
