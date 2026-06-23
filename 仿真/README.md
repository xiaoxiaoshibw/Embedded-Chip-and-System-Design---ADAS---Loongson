# 仿真 — ADAS 双冗余联合仿真演示系统

CLI 控制端 + CARLA(`CALRA/`) × 真实 SOC 控制栈(`lx/SOCCode`) × 虚拟 ESP32 的
全链路联合仿真，用于系统展示：**LKA / ACC / AEB / 静止前车超车 / 安全无感降级**。

## 快速开始

```powershell
# 0) 一次性：Python 3.12 安装 CARLA wheel
pip install "..\CALRA\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl"

# 1) 一站式控制台（CARLA 启停 / 场景 / 自检 / KPI 报告全在菜单里）
cd 仿真
python cli.py
```

CLI 集成功能（菜单 / 子命令双入口）：

| 菜单键 | 子命令 | 功能 |
|---|---|---|
| 1–6 | `python cli.py <场景key>` | 运行演示场景 |
| c / k | `start-carla` / `stop-carla` | 启动（含等待就绪、可选 Low 画质）/ 关闭 CARLA |
| s | `sil` | SIL 链路自检（无需 CARLA，~40s，断言接管/回切/看门狗/无感性） |
| r | `report` | 解析最近一次运行 CSV：车速/最小车距/车道偏移 RMS/切换无感性 |
| o | — | 运行参数设置（host/地图/出生点/渲染/画质） |

## 场景

| 序号 | key | 场景 | 内容 |
|---|---|---|---|
| 1 | `lka` | LKA 车道保持 | 无前车高速巡航，弯道曲率前馈+CTE 修正 |
| 2 | `acc` | ACC 自适应巡航 | 前车 7→3.5→8→5 m/s 变速，安全时距跟随 |
| 3 | `aeb` | AEB 紧急制动 | t=15s 前车急刹；SOC AEB + ESP32 硬件地板双保险 |
| 4 | `overtake` | 静止前车超车 | 前车驻车 >8s 判定长期阻塞，借道-超越-回道 |
| 5 | `failover` | 安全无感降级 | 杀主控→备机接管→恢复回切→双杀→看门狗制动 |
| 6 | `free` | 自由交互 | 不预设故障，现场手动注入 |

### 双 SoC 主备控制台（`dual_soc_console.py`）

任意场景运行中都通过同一个控制台操作（输入字符后回车）：

| 键 | 动作 | 说明 |
|---|---|---|
| `1` | 切到备机 | 模拟主控整机故障，备机 ~150ms 接管（SRC 0→1） |
| `0` | 切回主机 | 重启主控，恢复后自动回切（SRC 1→0） |
| `p` / `P` | 杀 / 重启主控 | |
| `b` / `B` | 杀 / 重启备控 | |
| `h` | 主控卡死 | HANG，演示 cycle 停滞接管路径 |
| `s` / `?` | 状态面板 / 帮助 | 实时显示主备健康度、执行源、冗余状态 |
| `q` | 结束本场景 | |

**安全联锁（保证车辆不出故障）**：控制台会拒绝任何"摘掉最后一个健康 SoC"的
操作——若另一控制器已掉线或卡死，杀/卡当前控制器的指令将被拦截并提示，
从操作层面杜绝双控同时失效 → 看门狗紧急制动。健康判定 = 进程存活 +
soc_worker 上报的 `cycle` 计数持续推进（进程在但 cycle 停滞即判卡死）。

## 链路架构

```
CarlaUE4.exe（同步模式 20Hz，真值感知）
    │ Python API
run_cosim.py 桥主进程
    ├── 感知帧 UDP JSON ──→ soc_worker --role primary ┐ 100Hz 跑真实
    │                  ──→ soc_worker --role backup  ┘ run_pure_pipeline()
    │                        ↕ UDP 心跳（heartbeat.py 原线格式，
    │                          SEQ 停滞检测 + 接管种子 → 无感降级）
    ├── 虚拟 ESP32（virtual_esp32.py，lx/MCUcode main.c 逐行移植：
    │     CRC-8/MAXIM 校验 / 150ms 主备仲裁 / AEB 硬件地板 / 200ms 看门狗）
    │     ←── 两条"虚拟 UART"（UDP）：worker 发 build_esp32_payload() 真实帧
    └── 执行器映射（delta→steer，lon_cmd→throttle/brake）→ CARLA 自车
```

- 控制算法**零修改**复用 `lx/SOCCode`；worker 发送链与 `ADAS._control_loop_impl`
  逐行对齐（clamp → lat_smooth → 接管守护 → 组帧 → 心跳）。
- 感知端做了**参考线跟踪**：lane_offset/road_psi 相对自车初始车道中心线
  （按纵向投影推进）计算，避免 `map.get_waypoint` 吸附最近车道导致超车
  变道时偏移突跳；并对 yaw ±180° 回绕做连续角展开。

## 如实交代的设计点

- **降级间隙**：主控宕机到备机接管约 0.5s（`HEARTBEAT_TIMEOUT_S`），期间虚拟
  ESP32 按实机语义输出全制动。"无感"指接管后控制量从主机最后一帧种子连续
  衔接、无阶跃，不是零中断。
- **超车方向**：当前符号约定（CARLA 左手系自洽）下 `+OVT_LANE_OFFSET_M`
  表现为向**右**借道，出生点需右侧有车道；停在路肩请换 `--spawn-index`。
- 首跑可调：`bridge_config.py` 的转向归一化（`STEER_MODE`）、油门/刹车增益、
  各场景 `spawn_index`。

## 文件

- `cli.py` — 演示控制台（入口）
- `scenarios.py` — 场景库（前车脚本/故障时间线/讲解要点）
- `run_cosim.py` — 联合仿真运行器（可独立 `--scenario` 运行）
- `soc_worker.py` — 主/备 SOC 节点进程（感知 UDP 入、100Hz 管线、心跳、故障注入）
- `virtual_esp32.py` — MCU `main.c` 的 Python 移植
- `carla_link.py` — CARLA 世界端（生成/感知/执行/前车脚本/旁观视角）
- `bridge_config.py` — 端口/符号约定/执行器映射
- `test_failover_sil.py` — 无 CARLA 的链路回归（接管/回切/看门狗/无感性断言）
- `logs/` — 每次运行的 CSV 遥测

## 一个 CLI 两种模式（A 本机自跑 / B 连真实 Nano）

`python cli.py` 同一入口支持两种运行模式：

| 模式 | 含义 | 入口 | 控制在哪 |
|---|---|---|---|
| **A 本机自跑** | CARLA + 本机真实 SOC 控制栈 + 虚拟 ESP32，单机闭环 | 菜单选场景 1-6 / `python cli.py aeb` | 本机进程 |
| **B 连真实 Nano** | CARLA 当前端，感知/控制经 UDP 桥到真实双 Nano + 边缘盒 | 菜单 `[n]` / `python cli.py nano --edge-host <IP>` | 真实 .125/.124 |

菜单里 `[o]` 可改边缘盒 IP（模式 B）。模式 B 底层调用 `deploy/pc_demo/carla_demo.py`。

### 移植到另一台「有 CARLA」的电脑
两种模式都要本机装 carla 客户端（一次性）：
```powershell
pip install <CARLA>\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl   # Python 3.12
```
然后按需拷目录（保持仓库相对结构）：
- **只跑模式 A**：拷 `仿真/` + `lx/SOCCode/` + `CALRA/`（你的 CARLA）。
- **要带模式 B**：再加 `deploy/pc_demo/`（cli.py 用 `../deploy/pc_demo/carla_demo.py`）。
- 模式 B 还要求边缘盒(.123)在跑 `udp_to_ros2 + ros2_to_udp`、两台 Nano `adas-node` 在线、同一局域网（详见 `deploy/pc_demo/README.md`）。
