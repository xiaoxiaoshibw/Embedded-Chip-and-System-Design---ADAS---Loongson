# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ADAS（高级驾驶辅助系统）仿真与开发平台，实现 LKA / ACC / AEB 双冗余容错架构。用户界面文字为中文，代码标识符为英文。

**顶层组件：**

| 目录 | 用途 |
|---|---|
| `lx/` | 核心源码 — SOC 控制栈、ESP32 固件、ML 模型、CARLA 桥。有独立 `.git` 仓库 |
| `CALRA/` | CARLA 0.9.16 预编译 Windows 二进制分发，提供仿真环境。**不可编译 C++**（gitignored） |
| `仿真/` | CLI 驱动的全链路联合仿真（CARLA + 真实 SOC 栈 + 虚拟 ESP32）。支持**双模式**：A 本机自跑（单机闭环）/ B 连真实 Nano（UDP 桥到 .125/.124） |
| `HIL/` | **所有 HIL（硬件在环）相关代码集中于此**（见 `HIL/README.md` 总览），含两个子目录：<br>· `HIL/hil_platform/` — ADAS HIL 实时监控与历史回放平台。FastAPI 统一后端（`SimulationCore` 唯一持有底层）+ React/TS 双页面（`/live` 实时、`/replay` 回放）+ `hilctl` CLI，三者共用 API。默认 mock 模式（无 CARLA/Nano/ESP32 即可演示），真实硬件经 `HilBridge` 适配器接入；**Web 直接操作两台 Nano，CARLA 仅作真值世界/感知输入/闭环执行**。详见 `HIL/hil_platform/README.md`<br>· `HIL/carla_bridge/` — HIL 闭环底层桥（原 `集成HIL/`，已按 `pc/ nano/ launch/ tools/` 归类）：Windows 跑 CARLA + PC 侧 TCP 桥，真实 SOC 控制栈跑在两块物理 Jetson Nano 上（被 `hil_platform` 复用）。详见 `HIL/carla_bridge/CLAUDE.md` |
| `统一可爱网站/` | 萌驾舱 MoeDrive — 实时驾驶舱 + AI 监控 + 边缘计算 + 项目报告整合 SPA。纯 stdlib，零 CDN |
| `文档/` | 竞赛交付文档与成果物汇总：`龙芯定稿/`（最终报告 docx+md）、`成果/`（对比图表+采集工具）、`CARLA录制视频/`（演示录像 mp4）。详见各子目录 README |
| `IOT_TI/` | TI 板卡申请稿（非源码，可忽略） |

`deploy/`（龙芯部署包）、`ollama模型调用/`（AI 监控台）、`tools/`（文档生成脚本）不在当前 checkout 中——需时从其它分支/备份获取。（原顶层 `HIL闭环/` 旧原型已被 `HIL/carla_bridge/` 取代，归档到 `HIL/_legacy/`，可删。）

**子项目文档：** `lx/`、`lx/SOCCode/`、`lx/MCUcode/ADAS_Test/`、`lx/ml/ml/`、`CALRA/`、`HIL/carla_bridge/` 各有独立 `CLAUDE.md`——编辑对应子目录前**先读其 CLAUDE.md**。`仿真/`、`统一可爱网站/`、`HIL/hil_platform/`、`文档/成果/` 的指导文档在各自 `README.md` 中。`deploy/`、`ollama模型调用/`、`tools/` 不在当前 checkout 中。


## System Architecture

```
感知层 (Simulink/ROS2)
    │  ROS2 Topics
    ▼
Jetson Nano (Primary)  ◄──UDP 心跳──►  Jetson Nano (Backup)
    │  UART1 (GPIO 16/17)                   │  UART2 (GPIO 18/19)
    ▼                                        ▼
ESP32 微控制器（实时执行器 + 安全地板）
```

**安全层（由内到外）：**
1. Jetson AEB — SOC 上基于 TTC 的制动计算
2. ESP32 AEB 地板 — 硬件兜底全力制动
3. 通信看门狗 — 双 Jetson 静默 200ms → 紧急制动
4. TWDT — 控制任务卡死 3s → 硬件复位

**联合仿真链路**（`仿真/`目录）：
```
CarlaUE4.exe（同步模式 20Hz，真值感知）
    │ Python API
run_cosim.py（桥主进程）
    ├── 感知帧 UDP JSON → soc_worker primary/backup（100Hz 真实 run_pure_pipeline()）
    │                       ↕ UDP 心跳（heartbeat.py 原线格式，SEQ 停滞检测 + 接管种子 → 无感降级）
    ├── 虚拟 ESP32（virtual_esp32.py，lx/MCUcode main.c 逐行移植：
    │     CRC-8/MAXIM 校验 / 150ms 主备仲裁 / AEB 硬件地板 / 200ms 看门狗）
    └── 执行器映射（delta→steer，lon_cmd→throttle/brake）→ CARLA 自车
```

控制算法**零修改**复用 `lx/SOCCode`；worker 发送链与 `ADAS._control_loop_impl` 对齐。感知端做**参考线跟踪**避免 `map.get_waypoint` 吸附导致超车变道时偏移突跳。

**共享桥模块**：`仿真/` 与 `HIL/` 共享的 CARLA 桥基础设施（`bridge_config.py`、`carla_link.py`、`scenarios.py`）**统一在 `HIL/carla_bridge/pc/`** 维护，`仿真/` 通过 `paths.py` 导入（不再各自维护独立副本）。

## Common Commands



### SOC 控制节点（Jetson Nano 上运行）
```bash
python3 -m pip install -r requirements.txt
source /opt/ros/<distro>/setup.bash
python3 ADAS.py --role primary          # 主控
python3 ADAS.py --role backup           # 备控

# 离线测试（无 ROS 依赖，同一控制内核）：
python3 replay.py <telemetry.csv>
python3 run_scenario.py scenarios/straight_cruise.yaml
python3 run_scenario.py scenarios/curve_follow.yaml --plot

# 语法/兼容性检查（SOC 目标 Python 3.6）：
python3 -m compileall -q .
grep -rnE "\| None\b|: list\[|: dict\[|: tuple\[|: set\[" --include="*.py" .
```

### ESP32 固件
```bash
# 需先 source ESP-IDF v5.5.3 环境
idf.py build
idf.py flash
idf.py monitor
idf.py build flash monitor
```

### ML 模块
```bash
cd lx/ml/ml
pip install -r requirements.txt
python train.py --model all                             # 训练全部模型
python train.py --model all --data-source ngsim-subset  # 用 NGSIM 真实数据
python demo.py                                          # 端到端演示
```

### AI 质量监控台（`ollama模型调用/`）
```powershell
ollama pull qwen2.5:3b                  # 一次性准备模型
cd ollama模型调用
python server.py                        # 自动取 仿真/logs 最新联合仿真日志回放；浏览器开 http://127.0.0.1:8765
python server.py --csv ..\仿真\logs\cosim_lka_xxx.csv --speed 2
python server.py --csv C:\tmp\adas_primary_telemetry_xxx.csv --mode tail   # 实时跟踪 SOC 遥测

# 自检
python -m compileall -q .
python server.py --speed 10             # 高倍速回放过链路
```
环境变量：`OLLAMA_URL`（默认 http://127.0.0.1:11434）、`OLLAMA_MODEL`（默认 qwen2.5:3b）、`MONITOR_PORT`（默认 8765）。Ollama 未启动时图表/KPI/规则风险分级仍可用。浏览器 `/cockpit.html` 可打开实时驾驶舱（双 SoC + ESP32 节点状态 3D 可视化）。

### 龙芯 LoongArch 部署（`deploy/`，纯 stdlib SIL 链路）
```sh
cd deploy && ./package.sh        # 开发机打包 → dist/adas-platform-<日期>.tar.gz（~7M，不含 CARLA/权重）
# 龙芯上解包后：
sudo ./install.sh                # 系统级 systemd（adas-sil + adas-monitor），开机自启
./install.sh --user              # 用户级 systemd（无 root）
./install.sh --no-systemd        # 仅配置，手动 ./run_demo.sh 起
./run_demo.sh                    # 前台一键演示（Ctrl-C 全停），读 stdin 注入故障/主备切换
./healthcheck.sh                 # 健康检查（遥测在更新? 端口在听? 服务 active?）
```
交付到龙芯实机的是 **SIL 软件在环**链路：单车运动学模型替代 CARLA 世界端，但走与联合仿真**完全相同**的进程间链路与控制栈（`run_pure_pipeline` 真实控制内核）。配置在 `/etc/adas/adas.env`（被 shell `source`，多词值须加引号）。详见 `deploy/README.md`、`deploy/CARLA_REALTIME.md`。

### 测试
```powershell
# SOC 测试（pytest，tests/ 目录，15 个测试文件覆盖 pipeline/横向/纵向/AEB/超车/心跳/感知/健康/串口协议等）
cd lx\SOCCode; python -m pytest
python -m pytest tests\test_pipeline.py              # 单个文件
python -m pytest tests\test_pipeline.py -k <名称>    # 单个测试

# ML 测试（pytest）
cd lx\ml\ml; python -m pytest

# SIL 降级回归测试（无需 CARLA）
python 仿真\test_failover_sil.py
python 仿真\test_dual_soc_console.py     # 双 SoC 控制台安全联锁
```

### 探测 Jetson Nano 板卡
```powershell
# probe_nanos.py — SSH 探测三块 Jetson（192.168.3.125/124/123）的在线状态
# 该脚本不在仓库中，需单独获取或在 deploy/pc_demo/ 环境下使用
```

## UART Frame Protocol（MCU↔SOC 关键接口）

Jetson→ESP32 标签格式：`TTC:8.00 DIST:15.50 PSI:0.1234 DELTA:0.0500 SPEED:16.70 ACC:-2.50 OFFSET:0.100 LEADV:14.00 DSAFE:10.00 WMRN:1.98 WHRD:3.06 CURV:0.01 CRC:AB`

ESP32→Jetson 固定格式：`P:+0.1234` / `D:+0.05` / `B:-2.50` / `SRC:0|1|9`（0=主控, 1=备控, 9=看门狗紧急）

CRC-8/MAXIM（多项式 0x31, 初始值 0x00）覆盖 ` CRC:` 之前所有字节。编码器：`control/serial_protocol.py`；解码器：`main.c`（`parse_jetson_line`）。两者必须保持字节级兼容。

## SOC 数据模型与控制流

**单节点单定时器**：`ADAS.py` `AdasNode` 是唯一 ROS2 节点，`_control_loop_impl()` 为完整管线，每个 tick 内顺序执行，无线程并行。

**数据模型分层**（`control/context.py`, `control/state.py`）：
- `VehicleSignals` — 可变，仅 ROS 回调写入，循环读取。通过 `_safe_float` 校验（有限 + 范围）
- `ControlMemory` — 可变跨 tick 状态（积分器、滤波器、计数器）。`dt` 由 `LOOP_HZ` 推导
- `LateralContext` / `LeadContext` / `LongitudinalContext` — 冻结结果对象，tick 内传递
- `ControlManagers` — 有状态算法对象集合（`LaneWidthEstimator`, `LeadTracker`, `AebAlertManager`, `CurveHoldManager`, `LongitudinalController`, `LonSmoothing`, `OvertakeManager`）

**纯计算内核**（`pipeline.py`）：`run_pure_pipeline()` 提取无 ROS 依赖的控制逻辑，供 `replay.py` 和 `run_scenario.py` 离线回归测试。

**特性开关**（`config.py`）：
- `MULTI_TARGET_COUNT`: `1`（默认）= 单前车；`>1` = 多目标跟踪 + 行人 class=3
- `LON_CONTROLLER`: `'pid'`（默认）= 标准 PID；`'mpc'` = LQR 替代，异常自动回退

**`params.yaml` 运行时覆盖**（可选，与 `config.py` 同目录）：启动时覆盖非安全关键参数。安全关键参数（`MAX_DELTA`、`LON_CMD_MAX_BRAKE_DECEL`、`AEB_EMERGENCY_DIST` 等）被白名单拦截。PyYAML 可选——回退到扁平 `KEY: value` 解析。AI 监控台的调参输出同样写 `params.yaml`，拷贝到 `lx/SOCCode/` 后下次启动 `ADAS.py` 生效。

## Key Constraints

**Python 3.6 兼容性（SOCCode）：** JetPack 4.x 仅提供 Python 3.6。禁用 PEP 604 联合类型（`X | None`）、PEP 585 泛型（`list[int]`）、海象运算符 `:=`、`match`。使用 `typing.Optional`/`typing.List`。`dataclasses` 通过 backport 提供。

**硬实时（SOCCode）：** 100Hz / 10ms 预算。所有阻塞 I/O（串口、日志、遥测）在守护线程运行。主循环只做非阻塞队列写入和原子读取。

**配置集中化（SOCCode）：** 所有可调参数在 `config.py`，各模块通过 `from config import *` 引用。不要在算法文件中散布字面量。



**非根目录 Git 仓库**——`lx/` 有独立 `.git`；根目录无 `.git`。无 CI/CD、无 linter 配置。

**CARLA Python 版本注意**：SOCCode 目标 Python 3.6，CARLA 客户端需 Python 3.12。两者不能共用同一 Python 环境。

**`runtime` 模块陷阱**：`ADAS.main()` 在启动时写入 `os.environ['NANO_ROLE']` 并调用 `runtime.configure_runtime()` 后才构造 `AdasNode`。其它模块必须 `import runtime` 并在使用时读取 `runtime.NANO_ROLE` / `runtime.IS_PRIMARY` —— 切勿 `from runtime import IS_PRIMARY`（会捕获过时的预配置值）。

**心跳 AEB 标记必选**：主控广播的 `AEB` 字段为必选；缺失该字段的旧版主控心跳，其 seed 会被拒绝（回退到零初始化）。新增心跳字段须保持必选，滚动升级需同时重启主备。

**注释和文档为中文**（SOCCode）：编辑时匹配周围风格。

## 补充组件

### 萌驾舱 MoeDrive（`统一可爱网站/`）

把【实时驾驶舱】+【AI 智能监控】+【边缘计算】+【项目报告】整合进同一个 SPA（pastel / 圆角 / 吉祥物风格）。纯 Python 标准库，**零第三方依赖、零 CDN**。

```powershell
python server.py            # 浏览器自动开 http://127.0.0.1:8099
python server.py --speed 4  # 4 倍速演示
```

**数据来源三模式：** ① 默认历史数据仓库回溯（`数据仓库/index.json`，按场景归类，`archive_csv.py` 归档）；② MQTT 订阅（内置纯 stdlib MQTT broker，`--mqtt-broker`）；③ 代理真实 ADAS 后端（`--adas-url`）。Ollama 未启动时 AI 页签回退到内置规则引擎。

**核心文件：** `server.py`（统一服务）、`mqtt_lite.py`（纯 stdlib MQTT 3.1.1 客户端 + broker）、`adas_core.py`（脚本仿真核心）、`edge_engine.py`（边缘计算引擎）、`csv_replay.py`（CSV 回放器）、`web/`（前端）。


### 竞赛成果与工具（`文档/成果/`）

位于 `文档/成果/` 下。工具代码**按用途分类存放**：`工具/数据采集/`、`工具/绘图/`。图片在 `图片/`，数据在 `数据/`。约定：新增工具脚本必须放进对应分类子目录，不能散落根目录。复现命令见 `文档/成果/README.md`。

### HIL 网络与节点（`HIL/carla_bridge/`）

局域网/ZeroTier 硬件在环闭环。两块 Nano 同时挂在 LAN（192.168.3.x）与 ZeroTier（10.218.44.x）上。

| 角色 | LAN IP | ZeroTier IP | SSH |
|---|---|---|---|
| Windows / CARLA PC | 192.168.3.8 | 10.218.44.190 | — |
| Primary Nano B | 192.168.3.125 | 10.218.44.10 | `jetson`/`yahboom` |
| Backup Nano A | 192.168.3.124 | 10.218.44.155 | `jetson`/`jetson` |

ROS_DOMAIN_ID=43，PC↔Nano 控制链路：单条 TCP 42110 双向复用。详见 `HIL/carla_bridge/CLAUDE.md`。

## 关键实验里程碑

经记忆文件记录的实验基准（部分可复现）：

| 实验 | 结果 | 复现入口 |
|---|---|---|
| 主备接管时延极限 | 最小 15ms / 中位 47ms / 最坏 63ms，110 次 0 碰撞；备控热待机消除 10m/s² 冲击 | `仿真/experiment_takeover_limit.py` |
| 车道保持稳定边界 | 开环稳定 108km/h（RMS≤0.5m），弯道限速后 144km/h 仍车道有界 | 纯 SIL 高速弯道扫描 |
| 接近静止前车 | AEB 误触发 57→0 次，蠕动 17→5.9m | 协同仲裁+受控接近门控 |
| 三功能重复试验 | LKA/ACC/AEB 各 6 次重复，统计 RMS/MAE/碰撞次数 | `experiment_three_functions.py` |

## Directory Quick Reference

**SOC (`lx/SOCCode/`)：** `ADAS.py`（ROS2 节点入口）、`config.py`（500+ 行参数）、`pipeline.py`（纯计算内核 `run_pure_pipeline()`，无 ROS 依赖，用于离线回归）、`control/` 包（各阶段算法）、`heartbeat.py`（双冗余心跳）、`serial_link.py`（ESP32 UART）、`scenarios/`（10+ YAML 场景）。

**MCU (`lx/MCUcode/ADAS_Test/`)：** `main/main.c`（~700 行 FreeRTOS 固件，4 个实时任务、UART 协议、AEB 硬件地板）。构建系统：ESP-IDF CMake。

**ML (`lx/ml/ml/`)：** 三种边缘模型（<1M 参数）：AccLstmModel（加速预测）、AebLstmModel（碰撞风险分类）、AebXgbClassifier（XGBoost 快速分类）。`inference.py` 运行时 API，`train.py` 统一训练脚本，`checkpoints/` 训练权重。

**联合仿真 (`仿真/`)：** `cli.py`（入口，双模式菜单）、`scenarios.py`（6 场景库）、`run_cosim.py`（联合仿真运行器）、`soc_worker.py`（主备 SOC 进程，100Hz `run_pure_pipeline()`）、`virtual_esp32.py`（MCU `main.c` 逐行移植）、`carla_link.py`（CARLA 世界端：生成/感知/执行/前车脚本/旁观视角）、`bridge_config.py`（端口/符号约定/执行器映射）、`dual_soc_console.py`（主备交互控制台，安全联锁）、`test_failover_sil.py`（SIL 降级回归）。实验脚本：`experiment_takeover_limit.py`、`experiment_three_functions.py`、`experiment_lane_keeping_sweep.py`、`experiment_failover.py`。详见 `仿真/README.md`。

**文档与成果 (`文档/`)：** 包含 `龙芯定稿/`（最终报告 docx + md 生成脚本 `_build/`）、`成果/`（对比图表 + 采集/绘图工具，见 `成果/README.md`）、`CARLA录制视频/`（演示录像 mp4）、`Skill/`（已归档的学术图表生成 skill）。

**AI 监控台 (`ollama模型调用/`)：** 不在当前 checkout 中。详见记忆文件 `[[ollama模型调用]]`。（原架构：`server.py` ThreadingHTTPServer + SSE + REST、`csv_source.py` 两种 CSV 格式自动识别、`kpi.py` 10s 滑窗 KPI + 规则风险分级兜底、`analyzer.py` 每 12s 调 Ollama 分析、`tuning.py` 写 `output/params.yaml`。AI 只能建议 `config.py` `PARAM_REGISTRY` 注册的非安全关键参数且须在范围内。）

**CARLA (`CALRA/`)：** 预编译二进制分发。`PythonAPI/`（Python 客户端 + agents 导航 + 30+ 示例脚本）、`CarlaUE4/`（UE4 项目）、`Co-Simulation/`（Sumo/Vissim/Carsim/Chrono 集成）、`HDMaps/`（高清点云地图）。详见 `CALRA/CLAUDE.md`。

## .claude 基础设施

**Workflows（`/workflows` 可列出）：**

| 工作流 | 文件 | 功能 |
|---|---|---|
| `phase1-tests` | `.claude/workflows/phase1-tests.js` | 顺序跑 SOCCode pytest → 场景 MIL（10 YAML）→ SIL 降级回归 |
| `phase2-3-4` | `.claude/workflows/phase2-3-4.js` | CSV 结构校验 + ML 分析（pytest/demo/模型加载） |

**Skills（`/` 前缀触发）：**

| Skill | 文件 | 功能 |
|---|---|---|
| `frontend-design` | `.claude/skills/SKILL.md` | Web UI/UX 设计准则：设计系统、排版、间距、玻璃拟态、动效、数据可视化、兼容性清单。构建或修改任何 Web 界面时遵循 |

**Launch 配置（`.claude/launch.json`）：** `adas-web-demo`（4070 演示包，端口 8092）、`moe-portal`（萌驾舱，端口 8099）、`hil-web`（HIL 前端 dev server，端口 5173）。

**`CLAUDE_CODEX_MEMORY/`：** Claude 与 Codex 共享项目工作记忆目录——`worklog/` 按日期记录改动/风险/部署/验证结果。
