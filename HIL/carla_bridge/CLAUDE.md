# CLAUDE.md

本文件为 Claude Code（claude.ai/code）在 `HIL/carla_bridge/` 目录工作时提供指导。先读根目录 `CLAUDE.md` 与 `HIL/README.md`，再读本文件。

## What this is

**局域网 / ZeroTier HIL（硬件在环）闭环**的底层桥。把根目录 `仿真/` 的"进程内联合仿真"升级为**真实硬件在环**：Windows PC 只跑 CARLA（真值世界端）+ PC 侧 TCP 桥；**真实的 SOC 控制栈（`lx/SOCCode/ADAS.py`）跑在两块物理 Jetson Nano 上**，主备冗余 + ROS2 Gateway 都在 Nano 上完成。Windows 不装 ROS2、不跑控制代码。

本目录被上层 `HIL/hil_platform/`（监控/控制平台）复用：`hil_platform/core/carla_world.py` 把本目录 `pc/carla_link.py` 加进 `sys.path` 直接复用其 `CarlaLink`；`hil_platform/core/nano_link.py` 复用本目录 `nano/hil_ros_gateway.py` 的 TCP 42110 协议。

界面/注释中文，代码标识符英文。

## 目录结构（按运行位置归类）

```
carla_bridge/
├── pc/        # Windows PC 侧（CARLA 客户端，Python 3.12）
│   ├── hil_carla_bridge.py   桥主进程：连 gateway TCP，上行感知帧、下行执行量、写 ego、记 logs
│   ├── carla_link.py         CARLA 世界端：spawn/感知/执行器映射/前车脚本（与 仿真/ 同源）
│   ├── bridge_config.py      桥侧基础参数（CARLA 连接、Town04、FIXED_DT、增益、坐标符号约定）
│   └── scenarios.py          场景库：lka/acc/aeb/overtake/failover/free（与 仿真/ 同源）
├── nano/      # 部署到 Nano 的"网关 + 进程管理"单元（deploy_gateway 扁平上传到 /home/jetson/adas/hil）
│   ├── hil_ros_gateway.py    ROS2 Gateway：TCP 感知帧→ROS2 话题；订阅 /jetson|/esp32 执行量→TCP 回执
│   ├── start_hil_adas.py     停 systemd、杀旧 ADAS、用隔离 domain 把 ADAS 注册为 **systemd transient 单元** adas-hil-<role>.service（Restart=always 自愈，如真实部署）
│   ├── restart_adas.py       SIGINT→SIGTERM→SIGKILL 优雅重启（走 deploy/nano/adas-run.sh）
│   └── stop_gateway.py       杀 hil_ros_gateway.py
├── launch/    # Windows 编排脚本（双击 .bat 或命令行）
│   ├── 一键启动HIL闭环.bat / _ESP32.bat / _调通.bat   一键入口（chcp 65001）
│   └── *.ps1                  7 步编排的各步（见下）
├── tools/     # SSH / 上传工具
│   ├── nano_ssh.py           paramiko SSH 执行（A / B / both）
│   └── upload.py             递归 sftp 上传（跳过 __pycache__/.git/logs/.pyc）
├── logs/      # 每次运行的 hil_<场景>_<时间戳>.csv
├── README.md
└── CLAUDE.md（本文件）
```

> 所有 `launch/*.ps1` 用 `$PSScriptRoot` 锚定，跨 PC/tools/nano 子目录引用，可从任意 CWD 调用。

## 闭环数据流

```
Windows PC                                       Primary Nano B
┌─────────────────────────┐                     ┌──────────────────────────────────┐
│ CarlaUE4.exe (同步20Hz)  │                     │ nano/hil_ros_gateway.py (TCP srv) │
│   ↕ Python API           │   sensor JSON ──►   │   ├ 发布 /car1_* /car2* /road_psi  │
│ pc/hil_carla_bridge.py   │  ──TCP 42110──      │   │        /heng_error (感知话题)   │
│  (TCP client)            │   ◄── actuation     │   ▼                                │
│  pc/carla_link.py        │     JSON @50Hz      │ ADAS.py --role primary (真实控制)  │
│  (spawn/sense/执行映射)   │                     │   订阅感知 → 算 LKA/ACC/AEB        │
│  → 写 steer/throttle/brake│                    │   发布 /jetson/* 或 /esp32/* 执行   │
│  → logs/hil_*.csv        │                     │        + /jetson/active_role/...    │
└─────────────────────────┘                     └────────────┬───────────────────────┘
                                                    ROS2 DOMAIN 43 │ DDS
                                                 ┌────────────────▼───────────────────┐
                                                 │ Backup Nano A                        │
                                                 │ ADAS.py --role backup（主备心跳/接管）│
                                                 └──────────────────────────────────────┘
```

**方向要点**：单条 TCP 连接双向复用。**Nano gateway 是 TCP server**（`bind` 42110 监听），**Windows bridge 是 client**（主动连接）。Windows 上行发感知帧 JSON 行，Nano 下行以 `--status-hz`（默认 50Hz）回执行量。新连接会顶掉旧连接（gateway 只保留一个 client）。

## 网络与节点

两块 Nano 同时挂在 **LAN（192.168.3.x，eth0）** 与 **ZeroTier 覆盖网（10.218.44.x）** 上；另有 NAT 跳板 `10.18.52.130`（只转发 SSH：primary=52125/yahboom，backup=52124/jetson）。

| 角色 | LAN IP | ZeroTier IP | SSH | 说明 |
|---|---|---|---|---|
| Windows / CARLA PC | 192.168.3.8 | 10.218.44.190 | — | 跑 CARLA + PC 桥 |
| Primary Nano **B** | 192.168.3.125 | 10.218.44.10 | `jetson`/`yahboom` | 主控 + ROS2 gateway |
| Backup Nano **A** | 192.168.3.124 | 10.218.44.155 | `jetson`/`jetson` | 备控 |

- 当 PC 不在 192.168.3.x 同网段时（常见），**走 ZeroTier**：gateway 用 `--pc-host 10.218.44.190`，PC 桥用 `--gateway-host 10.218.44.10`。
- **ROS_DOMAIN_ID = 43**：把本 HIL 闭环与旧 `/perception_sim`（domain 42）隔离。`ROS_LOCALHOST_ONLY=0`：跨网段 DDS 必需。
- **PC↔Nano 控制链路：单条 TCP `42110`**（双向复用）。`--sensor-port/--actuation-port`（42100/42101）是 `仿真/` 继承的遗留参数，HIL TCP 路径不使用。
- Nano 上 HIL 代码部署路径：`/home/jetson/adas/hil`（= `nano/` 子目录内容）；SOC 仓库：`/home/jetson/adas/lx/SOCCode`。

## Common Commands

从 `carla_bridge/` 目录运行（脚本已 `$PSScriptRoot` 锚定，CWD 不敏感）。

### 一键启动（Windows，双击或命令行）

```powershell
.\launch\一键启动HIL闭环.bat jetson acc      # 通用入口：[actuation-source] [scenario]
.\launch\一键启动HIL闭环_调通.bat            # = jetson acc：先验证 CARLA→Nano→CARLA 链路
.\launch\一键启动HIL闭环_ESP32.bat           # = esp32 acc：完整 HIL（ESP32 仲裁）
```
`一键启动HIL闭环.bat` 是 7 步编排：① `check_lan_ros2` ② `deploy_gateway`（上传 `nano/` 到双 Nano）③ `stop_perception_sim_lan` ④ `start_hil_adas_lan`（双 Nano 起 ADAS@domain43）⑤ `start_carla_if_needed`（起 `..\..\..\CALRA\CarlaUE4.exe` 等 2000 端口）⑥ 新窗口起 gateway ⑦ 本窗口起 CARLA 桥。

### 分步手动（调试时）

```powershell
.\launch\check_lan_ros2.ps1                              # ping + SSH ros2 node/topic list
.\launch\deploy_gateway.ps1                              # 上传 nano/ 到双 Nano /home/jetson/adas/hil + py_compile 校验
.\launch\stop_perception_sim_lan.ps1                     # 杀旧 /perception_sim（避免抢发 /car1_*）
.\launch\start_hil_adas_lan.ps1                          # B=primary, A=backup, domain 43
.\launch\start_gateway_lan.ps1 -ActuationSource jetson   # 前台 SSH 跑 gateway（关窗口=停 gateway）
.\launch\start_carla_bridge.ps1 -ActuationSource jetson -Scenario acc
```

### 收尾 / 环境

```powershell
.\launch\restore_system_adas_lan.ps1   # 杀 HIL ADAS + 恢复 systemd adas-node.service（回常规模式）
.\launch\install_carla_python.ps1      # 装 CARLA 0.9.16 cp312 wheel（首次/缺 carla 包时）
```

### SSH / 上传工具（直接调）

```powershell
python tools\nano_ssh.py A "命令"        # 备控 Nano A
python tools\nano_ssh.py B "命令"        # 主控 Nano B
python tools\nano_ssh.py both "命令"     # 两块都跑
python tools\upload.py B <本地目录> <远程目录>   # 递归 sftp 上传
```

## Key Constraints / 陷阱

- **两套 Python 互不兼容**：Windows 侧 CARLA 客户端需 Python **3.12**（`install_carla_python.ps1` 装 `..\..\..\CALRA\...\carla-0.9.16-cp312-cp312-win_amd64.whl`）；Nano 侧 gateway 需 ROS2 **foxy** 的 `rclpy`（`source /opt/ros/foxy/setup.bash`）。`hil_ros_gateway.py` 在无 `rclpy` 的环境里会直接 `SystemExit`。
- **CARLA 本体不复制进本目录**：始终用仓库根 `CALRA\CarlaUE4.exe`（脚本从 `$PSScriptRoot` 上溯三级定位）。
- **`deploy_gateway` 只上传 `nano/`**：扁平铺到 Nano `/home/jetson/adas/hil`，所以 Nano 上仍是 `hil_ros_gateway.py` 等平铺文件（与 PC 侧子目录无关）。
- **`start_gateway_lan.ps1` 是前台 SSH**：关窗口或 Ctrl+C 会停掉 Nano 上的 gateway。
- **场景 `timeline`（故障注入时间线）在 HIL 路径中不生效**：gateway 没有故障注入接线，主备接管要靠**真的在 Nano 上杀/暂停 `ADAS.py`** 来演示；`lead` 前车脚本与 `spawn_index` 生效。
- **HIL ADAS 现在受 systemd 监管自愈（`adas-hil-<role>.service`, `Restart=always`）**：被 `kill` 后约 2s 自动拉起（与生产 `adas-node.service` 一致），不再是裸 `nohup` 孤儿。推论：① 演示**持续**主备接管须用 `kill -STOP`（冻结，心跳静默触发接管，systemd 不重启）+ `kill -CONT` 恢复，或 `systemctl stop adas-hil-<role>`——单纯 `kill` 只会得到约 2s 的瞬态接管后主控自愈；② 收尾**必须** `restore_system_adas_lan.ps1`（先 `systemctl stop adas-hil-*` 再恢复 `adas-node`），否则 `Restart=always` 会把 pkill 掉的 ADAS 立刻拉回、根本停不掉；③ 日志走 shell 内 `>> /tmp/adas_hil_<role>.log 2>&1`（不用 systemd `StandardOutput=append:`——`User=` 下日志已存在会 EACCES 209/STDOUT）。
- **坐标符号约定（`pc/carla_link.py` / `pc/bridge_config.py`）**：CARLA 左手系下自洽——`yaw(rad)` 直接作 `psi`，`lane_offset` 右正，`steer` 与 `delta` 同号，全程不翻转坐标系。推论：超车向【右】借道，出生点须保证右侧有车道。
- **务必先停旧 `/perception_sim`**（domain 42）：否则它会和 CARLA 桥抢发 `/car1_*`。
- **滚动升级注意**：心跳/接管相关改动须同时重启主备（见根 `CLAUDE.md` 心跳约束）。

## 关联

- `HIL/hil_platform/`：上层监控/回放/控制平台，真实模式复用本目录（`pc/carla_link`、`nano/hil_ros_gateway`）。
- `仿真/`：进程内联合仿真（虚拟 SOC worker + 虚拟 ESP32）。本目录是其**真实双 Nano** 的 HIL 变体，复用 `pc/scenarios.py` / `pc/carla_link.py` 约定。
- `lx/SOCCode/`：HIL 下 Nano 上真正运行的控制内核（`ADAS.py` / `config.py` / `pipeline.py`）。
- `deploy/`：Nano 上的 `adas-node.service` / `adas-run.sh` 由本目录脚本停启。
