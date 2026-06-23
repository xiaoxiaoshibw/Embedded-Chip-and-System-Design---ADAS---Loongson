# ADAS HIL 实时监控与历史回放平台

把固定场景演示升级为**可参数化、可控制、可复位、可记录、可回放**的 ADAS HIL 实验平台。
统一后端（FastAPI）+ 单前端两页面（`/live` 实时监控、`/replay` 历史回放）+ 命令行（`hilctl`），
三者**共用同一套 API**。底层 CARLA / 双 Jetson Nano / ESP32 仲裁器由 `SimulationCore` 唯一管理。

> **两种运行模式（上层零差异，前端/CLI/API 完全相同）**：
> - **mock 模式**（默认）：无需 CARLA/Nano/ESP32 即可跑通全链路演示。
> - **真实 CARLA 模式**（`HIL_MOCK=0`）：平台直接驱动本机 CARLA（复用 `carla_bridge/pc/carla_link.py`），
>   支持**自由操控世界**（天气/NPC/前车接管/手动驾驶）与**车载摄像头**实时画面。当前控制核
>   为平台内置 ACC/AEB/LKA（无 Nano 也能看 CARLA 闭环）；把控制源切到真实双 Nano+ESP32 是
>   下一步，见「接入真实硬件」。

## 架构

```
            ┌─────────────── 唯一控制入口 ───────────────┐
  CLI  ───▶ │                FastAPI (server/)             │ ◀── Web /live, /replay
            │                      │                       │
            │             SimulationCore (core/)           │  ← 唯一持有底层、唯一 tick
            │   状态机 / 场景 / 参数 / 故障注入 / 指标 / 记录  │
            │                      │                       │
            │              HilBridge（抽象）                │
            │        ┌─────────────┴──────────────┐        │
            │   MockHilBridge              RealHilBridge    │
            │   (纯软件仿真)         (CARLA + 双Nano + ESP32) │
            └─────────────────────────────────────────────┘
```

**状态机**：`IDLE`（已连接未加载）→ `READY`（已加载场景/参数）→ `RUNNING` ⇄ `PAUSED` → `STOPPED`（已保存）；异常进入 `ERROR`。

**目录**

| 路径 | 说明 |
|---|---|
| `core/` | 纯 Python 核心（无 FastAPI 依赖，可单测）。`simulation_core` 编排，`hil_bridge` 桥接 + mock + ESP32 仲裁器，`fault_injector` 集中故障逻辑，`metrics` 指标，`recorder` 落盘，`scenario_manager`/`parameter_manager` 场景与参数，`state_machine` 状态机，`carla_world` 真实 CARLA 占位 |
| `server/` | FastAPI：`api_server`（REST + `/ws/live` + 可托管前端构建产物）、`schemas`（Pydantic） |
| `cli/` | `hilctl.py`（纯标准库 urllib，调用后端 API） |
| `web/` | React + TS + Vite 前端，`/live` 与 `/replay` 共用组件/类型/API 客户端 |
| `configs/` | 5 个场景：`acc_follow` / `aeb_brake` / `lka_curve` / `cut_in` / `takeover` |
| `runs/` | 每次实验记录（`meta.json` / `states.csv` / `events.json` / `summary.json` / `report.md`） |

## 快速开始

### 1. 后端（mock 模式，默认）

```bash
cd hil_platform
python -m pip install -r requirements.txt
python -m server.api_server            # 默认 8000；HIL_PORT 可改端口
# 或开发热重载： uvicorn server.api_server:app --reload --port 8000
```
- 接口文档：http://127.0.0.1:8000/docs

### 1b. 真实 CARLA 模式（已在 rig 跑通）

CARLA 客户端需 Python 3.12（与本平台一致）。首次装依赖：
```powershell
python -m pip install "..\CALRA\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl"
python -m pip install numpy pillow      # 车载摄像头内存编码 JPEG（绕开中文路径问题）
```
启动 CARLA（先让 `..\CALRA\CarlaUE4.exe` 跑起来、2000 端口就绪；地图运行时自动切 Town04）：
```powershell
$env:HIL_MOCK="0"          # 关键：用真实 CARLA 桥
$env:CARLA_HOST="127.0.0.1"; $env:CARLA_PORT="2000"; $env:CARLA_TOWN="Town04"
$env:CARLA_TM_PORT="8010"  # TrafficManager 端口（默认 8010，避开 HIL 后端 8000）
$env:HIL_CAMERA="1"        # 1=开车载摄像头画面（0 可省开销）
python -m server.api_server
```
然后 `load → start`：平台在 CARLA 里生成 ego+前车、进同步模式 20Hz、贴车载摄像头 + 碰撞传感器，
CARLA 窗口自带旁观者跟车视角，浏览器 `/live` 里有车载画面 + 世界操控面板。

**生产健壮性**：① 摄像头用 numpy+PIL 内存编码 JPEG（CARLA `save_to_disk` 在中文路径下会静默失败，
故不落盘）；② `stop/reset` 有序拆除（TM 退同步 → 世界转 async → 停传感器 → 批量销毁 NPC → 关 link），
反复 load/reset 不残留 actor；③ 碰撞用 CARLA 碰撞传感器判定；④ CARLA 中途崩溃不会拖垮后端——
仿真循环捕获异常后进 `ERROR` 态，`/api/status` 的 `error` 字段与 `/live` 顶部横幅给出原因，复位即可恢复。

> 已验证：`takeover` 场景在真实 CARLA 中 12s 注入 seq 卡死 → ESP32 检测 → 切 Nano B，
> **接管时延 150ms**，全程车载画面实时、reset 后 0 actor 残留。

### 2. 前端

**开发模式**（推荐，热重载；Vite 把 `/api`、`/ws` 代理到 8000）：
```bash
cd web
npm install
npm run dev                             # http://localhost:5173 ，自动跳 /live
```
**生产模式**（构建后由 FastAPI 同源托管，单进程即可）：
```bash
cd web && npm run build                 # 产物在 web/dist
# 回到 hil_platform 启动后端后，直接访问 http://127.0.0.1:8000/live 与 /replay
```

### 3. 命令行 hilctl

```bash
cd hil_platform
export HIL_API=http://127.0.0.1:8000    # 可选，默认即此（Windows: set HIL_API=...）
python -m cli.hilctl load acc_follow --ego-speed 50 --front-distance 30 --weather clear
python -m cli.hilctl start
python -m cli.hilctl update --front-speed 20 --comm-delay 100
python -m cli.hilctl inject-fault seq_stuck --target nano_a
python -m cli.hilctl status
python -m cli.hilctl pause
python -m cli.hilctl stop
python -m cli.hilctl reset
python -m cli.hilctl report latest      # 加 --full 打印完整 report.md
```

## 页面功能

**`/live` 实时监控（可控制仿真）**：顶部状态栏（run_id/场景/状态/时间/生效控制器/接管/安全制动/链路）、
9 项实时指标卡片、Nano A / Nano B / ESP32 三面板、场景参数面板（加载场景 + 热更新）、
控制按钮（开始/暂停/停止/复位）、6 个故障注入按钮、6 条实时曲线（仅保留最近 60s）。
WebSocket 10Hz 推送；断线显示横幅并自动重连；无数据时所有字段显示 `--` 而不崩溃。

**`/replay` 历史回放（只读）**：历史实验列表 + 多维筛选（场景/日期/接管/碰撞/PASS-FAIL）、
时间轴（播放/暂停、0.5×/1×/2×/4×、拖动、一键跳转到接管/最小TTC/最大横向误差时刻）、
当前时刻快照（Ego/Target/Nano/ESP32/事件）、与时间轴联动的历史曲线（点击曲线点跳转）、
事件列表（点击跳转）、实验摘要。

## mock 模式能演示什么

5 个场景 + 真实的主备仲裁链路：速度/TTC 变化、ACC 跟车、AEB 急刹、LKA 弯道、Cut-in 切入；
**Nano A seq 卡死（假活）→ ESP32 检测 → 切换 Nano B**（接管时延约 150ms，与实车一致）、
双路失败 → **安全制动**兜底。`takeover` 场景默认在 12s 预设 `seq_stuck`，最适合演示接管。

## 历史记录格式（`runs/<run_id>/`）

`run_id` = `YYYYMMDD_NNN_<scenario>`。`stop` 时自动生成：
`meta.json`（元信息 + 参数快照）、`states.csv`（逐帧，22 列）、`events.json`（事件序列）、
`summary.json`（结论：result/collision/min_ttc/max_lateral_error/takeover_latency_ms 等）、
`report.md`（答辩用可读报告）、`screenshots/`、`curves/`（预留）。

## API 一览

实时控制：`POST /api/scenario/load` · `/api/simulation/{start,pause,stop,reset}` ·
`/api/parameters/update` · `/api/fault/inject` · `GET /api/status` · `/api/metrics` · `WS /ws/live`
（额外：`GET /api/scenarios` 供前端 Load 面板）。

历史回放（只读）：`GET /api/runs`（支持 `?scenario=&date=&takeover=&collision=&result=` 过滤）·
`/api/runs/{id}/{meta,summary,events,states,report}` · `/api/runs/{id}/state?t=12.15`。

## 搭建与操控 CARLA 世界

**怎么搭建**：`core/carla_world.py` `CarlaWorld` 复用 `carla_bridge/pc/carla_link.CarlaLink` 完成
连接 / `load_world(Town04)` / 同步模式 20Hz / 生成 ego+前车 / 参考中心线 / 真值感知 / 执行器映射
（**不重写**已验证的场景代码）。`params_to_scenario()` 把平台参数适配成 carla_link 场景字典。

**怎么自由操控**（`/api/world/*`，或 `/live` 的「世界自由操控」面板）：
- 天气：`weather` = clear/rain/fog/night（运行中可改）
- NPC 交通流：`spawn_npc(count)` 用 TrafficManager 自动驾驶 / `clear_npc`
- 前车接管：`lead_speed(kmh)` 直接接管前车速度，`null` 恢复场景脚本
- 手动驾驶：`manual(on)` + `manual_cmd(throttle,brake,steer)` 人工接管 ego（"自由开"）

**怎么展示**：① CARLA 窗口自带旁观者跟车视角（投影现场最直观，零成本）；
② 车载 RGB 摄像头 → 限频存 PNG → `GET /api/world/camera` → `/live` 内嵌实时画面；
③ `/live` 数据驾驶舱（指标/双 Nano/ESP32/曲线）+ `/replay` 历史回放。

## 真实双 Nano + ESP32 在环（已跑通）

CARLA ego 由**真实 ADAS.py（双 Jetson Nano）+ 真实 ESP32 仲裁**驱动，**未改一行 Nano/SOC 代码**。

```powershell
$env:HIL_MOCK="0"; $env:HIL_CONTROL="nano"   # 关键：真实双 Nano 控制源
$env:GATEWAY_HOST="192.168.3.125"            # 主控 Nano（跑 carla_bridge/nano/hil_ros_gateway.py）
$env:BACKUP_HOST="192.168.3.124"             # 备控 Nano
$env:NANO_FAULT_RESTORE_S="8"                # 故障自动恢复秒数
$env:CARLA_TOWN="Town04"; python -m server.api_server
```

**链路**（`core/nano_link.py`）：复用 `carla_bridge/nano/hil_ros_gateway.py`（已运行在主控 Nano 上，TCP 42110）现有协议——
上行把 `CarlaWorld.sense()` 的真值感知发给网关（→ ROS2 `/car1_*` → 两台 ADAS.py → ESP32），
下行读回 ESP32 仲裁后的最终控制 + `active_role` + `failover_available`。`active_role` 映射：
`primary`/`secondary_standby` → 主控驾驶(nano_a)，`secondary_active` → **备控接管(nano_b)**；
链路丢失 → 安全制动。active_controller 做了 3 拍去抖，滤掉启动/恢复瞬态。

**真实故障注入 → 真实接管**（`core/nano_fault.py`）：故障按钮经 **SSH SIGSTOP** 冻结目标 Nano 的
ADAS.py（真断心跳 / SEQ 停滞）→ 真实 ESP32 切换到备控 → CARLA 不停 → 到点 **SIGCONT** 自动恢复。
完全可逆、无 sudo、不重启、不改 SOC 代码。实测：注入 → **~300ms** 真实接管 → 8s 后自动切回。
持久 SSH 连接 + 防连点窗口（避免触发真实双机失效的 `TAKEOVER_COOLDOWN`）。

**注意**（真实硬件特性）：① 单次注入即可演示，**勿连点**（真实失效仲裁有冷却，连点会抑制接管）；
② ego 纵向速度由**真实 ADAS 的 ACC 决定**——当前 `esp32` 源回读的纵向命令偏温和（~0.3 m/s²，ego 巡航较慢），
若要更明显的车速可在 Nano 端调 ADAS 巡航/ACC 配置，或把网关 `--actuation-source` 切到 `jetson`（用 Jetson 直出命令）。

> 切回纯软件演示：`HIL_CONTROL=internal`（CARLA + 平台内置控制器，无需 Nano）；或 `HIL_MOCK=1`（纯 mock）。
> **前端 / CLI / 回放在三种模式下完全一致。**

## 设计约束（答辩要点）

- 所有控制入口统一走 FastAPI；CLI 与 Web 共用同一套 API。
- 底层只由 `SimulationCore` 持有，单线程单 tick，`reset` 清理 actor/故障/指标/记录缓冲，避免状态不同步与 actor 残留。
- `/replay` 只读 `runs/`，绝不连接 CARLA，不影响在跑的仿真。
- 故障逻辑集中在 `fault_injector`，参数校验集中在 `parameter_manager`（安全关键参数有范围钳制）。
- 前端所有字段空值保护，PASS/FAIL、碰撞、安全制动、接管均有明显颜色区分。
