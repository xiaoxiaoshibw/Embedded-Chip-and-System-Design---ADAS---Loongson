# carla_bridge — HIL 闭环底层桥

局域网 / ZeroTier HIL（硬件在环）闭环的底层桥。Windows 本机只运行 CARLA 和 PC 侧 TCP 桥，**不安装 ROS2、不运行本机主控代码**；ROS2 发布/订阅与真实控制（`ADAS.py`）都在两块 Jetson Nano 上完成。被上层 `HIL/hil_platform/` 复用。

日常操作进入本目录：

```powershell
cd D:\Code\自动辅助驾驶仿真平台\HIL\carla_bridge
```

## 目录结构

| 子目录 | 跑在哪 | 内容 |
|---|---|---|
| `pc/` | Windows (Py 3.12) | `hil_carla_bridge.py`（桥主进程）、`carla_link.py`、`bridge_config.py`、`scenarios.py` |
| `nano/` | Jetson Nano | `hil_ros_gateway.py`（ROS2 网关）、`start_hil_adas.py`、`restart_adas.py`、`stop_gateway.py` |
| `launch/` | Windows | 一键 `.bat` + 7 步编排 `.ps1`（均 `$PSScriptRoot` 锚定，CWD 不敏感） |
| `tools/` | Windows | `nano_ssh.py`（SSH 执行）、`upload.py`（sftp 上传） |
| `logs/` | Windows | `hil_<场景>_<时间戳>.csv` |

## 网络与节点

两块 Nano 同挂 **LAN（192.168.3.x）** 与 **ZeroTier（10.218.44.x）**；NAT 跳板 `10.18.52.130` 只转发 SSH（primary=52125、backup=52124）。

- Windows / CARLA PC: LAN `192.168.3.8` / ZeroTier `10.218.44.190`
- Primary Nano **B**: LAN `192.168.3.125` / ZT `10.218.44.10`, `jetson/yahboom`
- Backup Nano **A**: LAN `192.168.3.124` / ZT `10.218.44.155`, `jetson/jetson`
- HIL ROS2 domain: `43`（与旧 `/perception_sim`@domain42 隔离）
- PC↔Nano 控制链路：单条 TCP `42110`（双向复用）。PC 不在 LAN 同网段时走 ZeroTier。

## 一键启动

```powershell
.\launch\一键启动HIL闭环_调通.bat            # jetson/acc：先验证 CARLA→Nano→CARLA 链路
.\launch\一键启动HIL闭环_ESP32.bat           # esp32/acc：完整 HIL（ESP32 仲裁）
.\launch\一键启动HIL闭环.bat jetson acc      # 通用：[actuation-source] [scenario]
```

批处理自动启动仓库根 `CALRA\CarlaUE4.exe` 并等待 2000 端口就绪。

## 分步（调试）

```powershell
.\launch\check_lan_ros2.ps1                              # 1. ping + ROS2 图
.\launch\deploy_gateway.ps1                              # 2. 上传 nano/ 到双 Nano
.\launch\stop_perception_sim_lan.ps1                     # 3. 停旧 /perception_sim
.\launch\start_hil_adas_lan.ps1                          # 4. 双 Nano 起 ADAS@domain43
.\launch\start_gateway_lan.ps1 -ActuationSource jetson   # 5. 前台 SSH 起 gateway
.\launch\start_carla_bridge.ps1 -ActuationSource jetson -Scenario acc   # 6. 另开窗口起 PC 桥
```

ESP32 串口回读正常后，把 `-ActuationSource` 换成 `esp32` 走完整 HIL。

## 注意

- CARLA 本体仍用仓库根 `CALRA\CarlaUE4.exe`，不复制进本目录。
- 缺 `carla` Python 包时：`.\launch\install_carla_python.ps1`。
- `start_gateway_lan.ps1` 是前台 SSH，关窗口/Ctrl+C 会停掉 Nano gateway。
- `deploy_gateway.ps1` 只上传 `nano/` 子目录（扁平铺到 Nano `/home/jetson/adas/hil`）。
