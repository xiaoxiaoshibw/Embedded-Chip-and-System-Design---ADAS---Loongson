# HIL 迁移到 Ubuntu 24.04 + ROS2（原生 DDS 传输）

把原来「Windows 跑 CARLA → TCP 桥 → Nano 上 `hil_ros_gateway.py` 翻译成 ROS2」的两层，
合并成 **Ubuntu 上一个原生 rclpy 节点 `pc/carla_ros_node.py`**。CARLA 真值直接 publish 成
ROS2 话题，两台 Jetson Nano 上的 `ADAS.py` 跨网 DDS 直接订阅——**Nano 端不再需要 gateway**。

> 适用拓扑：**这台 Ubuntu(Jazzy) 只跑 CARLA + 桥节点；两块真实 Nano(Foxy) 跑控制**，
> 跨网 ROS2 DDS 直连。`import carla` 与 `import rclpy` 在同一个 Python（Jazzy 自带 3.12）。

## 为什么能去掉 gateway

`hil_ros_gateway.py` 当初存在的唯一原因是 **Windows 跑不了 ROS2**，需要一个 TCP↔ROS2
翻译器跑在 Nano 上。Ubuntu 上 CARLA 端能原生 `import rclpy`，这层翻译就多余了：

| 旧（Windows） | 新（Ubuntu） |
|---|---|
| `pc/hil_carla_bridge.py`（TCP client） | `pc/carla_ros_node.py`（rclpy 节点） |
| `nano/hil_ros_gateway.py`（TCP↔ROS2 翻译） | **删除，不再需要** |
| PC↔Nano 单条 TCP 42110 | ROS2 DDS（domain 43） |
| `.ps1` / `.bat` 编排 | `launch/start_hil_ubuntu.sh` |

发布/订阅的话题名、QoS（感知话题 `qos_profile_sensor_data`）与原 gateway 字节级一致，
所以 **Nano 上的 `ADAS.py` 一行不用改**。

## 闭环数据流

```
Ubuntu 24.04 (Jazzy, Py3.12)                 Primary Nano B (Foxy, .125/.10)
┌────────────────────────────┐              ┌──────────────────────────────┐
│ CarlaUE4 (同步 20Hz)        │              │ ADAS.py --role primary        │
│   ↕ Python API              │   ROS2 DDS   │  订阅 /car1_* /car2* /road_psi │
│ pc/carla_ros_node.py        │ ◄══════════► │       /heng_error             │
│  pub  /car1_xy /car1_psi    │  domain 43   │  发布 /jetson/* 或 /esp32/*    │
│       /car1_v /car2xy ...    │              │       /active_role /failover  │
│  sub  /jetson/{psi,delta,    │              └───────────────┬──────────────┘
│       brake} 或 /esp32/*     │                ROS2 DOMAIN 43 │ DDS
│  → 驱动 CARLA 自车           │              ┌───────────────▼──────────────┐
└────────────────────────────┘              │ Backup Nano A (.124/.155)     │
                                             │ ADAS.py --role backup（主备）  │
                                             └──────────────────────────────┘
```

## 一、准备（一次性）

1. **CARLA 客户端**装进 Jazzy 的 Python（与 rclpy 同环境）：
   ```bash
   pip install <CALRA>/PythonAPI/carla/dist/carla-0.9.16-cp3xx-linux_x86_64.whl
   python3 -c "import carla, rclpy; print('ok')"   # 两个都要 import 成功
   ```
   （`carla_ros_node.py` 也会自动到 `CALRA/PythonAPI/carla/dist/` 找 Linux egg/wheel 兜底。）

2. **两台 Nano** 照常跑 `ADAS.py`（domain 43），**不要再起 gateway**：
   ```bash
   # primary（.125）
   source /opt/ros/foxy/setup.bash
   export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=0
   cd ~/adas/lx/SOCCode && python3 ADAS.py --role primary
   # backup（.124）同理 --role backup
   ```
   仓库里 `nano/start_hil_adas.py` 已用 domain 43 起 ADAS（systemd transient 自愈），
   仍可直接用；只是**第 6 步起 gateway 的环节省掉**。

## 二、启动闭环

```bash
# 1) 起 CARLA（无显卡可加 -RenderOffScreen）
./CarlaUE4.sh -quality-level=Low &

# 2) 起桥节点（自动 source jazzy、设 domain 43）
cd HIL/carla_bridge
chmod +x launch/start_hil_ubuntu.sh        # 首次
./launch/start_hil_ubuntu.sh acc jetson    # 场景 acc，读 Jetson 直出执行量
#   完整 HIL（ESP32 仲裁后）： ./launch/start_hil_ubuntu.sh acc esp32
```

或直接调节点（自定义参数）：
```bash
source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=0
cd HIL/carla_bridge/pc
python3 carla_ros_node.py --scenario acc --actuation-source jetson \
        --carla-host 127.0.0.1 --carla-port 2000 --town Town04
```

## 三、验证链路（关键）

跨机 ROS2 是否打通，用标准工具看（在 Ubuntu 上、同 domain）：
```bash
export ROS_DOMAIN_ID=43
ros2 node list          # 应看到 /carla_hil_bridge + 两台 Nano 的 /adas_primary /adas_backup
ros2 topic list         # 应有 /car1_xy /car2xy /jetson/delta /jetson/active_role ...
ros2 topic echo /jetson/delta     # 桥发感知后，Nano 在算 → 这里应有数
ros2 topic hz  /car1_xy           # 桥发布频率应 ~20Hz
```
桥节点终端每秒打印一行 `t=.. src=.. role=.. delta=.. brake=.. age=..ms`；
`age` 是收到执行量的新鲜度（正常几十 ms），`src=stale` 表示没收到 Nano 回包 → 看 DDS 发现。

## 四、DDS 互通（跨机最容易卡的地方）

- **同一 LAN 子网**（PC 与 Nano 都在 `192.168.3.x`）：多播发现开箱即用，无需任何额外配置。
- **跨子网 / ZeroTier**（PC↔Nano 走 `10.218.44.x`）：多播常不通，启用单播发现（二选一）：
  - Jazzy 原生：`export ROS_STATIC_PEERS="10.218.44.10;10.218.44.155"`（仅 PC 端）
  - Fast DDS profile：两端都 `export FASTRTPS_DEFAULT_PROFILES_FILE=launch/fastdds_peers.xml`
    （按实际 IP 改 `fastdds_peers.xml`）。
- **Jazzy(Fast DDS 2.14) ↔ Foxy(Fast DDS 2.0) 跨 4 个发行版**：基础 RTPS 互通通常没问题，
  但若 `ros2 topic list` 跨机看不全/收不到，最稳妥是**两端统一改用 CycloneDDS**：
  ```bash
  # Ubuntu:  sudo apt install ros-jazzy-rmw-cyclonedds-cpp
  # Nano:    sudo apt install ros-foxy-rmw-cyclonedds-cpp
  # 两端都： export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```
  CycloneDDS 跨版本 RTPS 互通更稳，单播对等体用 `CYCLONEDDS_URI` 配 `<Peers>`。

## 五、与旧 Windows 路径的关系

- `pc/hil_carla_bridge.py` + `nano/hil_ros_gateway.py` + `launch/*.ps1` **保留不动**，
  Windows 用户仍可用（两套并存，互不影响）。
- Ubuntu 路径只新增了 3 个文件：`pc/carla_ros_node.py`、`launch/start_hil_ubuntu.sh`、
  `launch/fastdds_peers.xml`。复用 `pc/carla_link.py`、`pc/scenarios.py`、`pc/bridge_config.py`
  （纯 Python，无改动）。
- `hil_platform/`（监控/回放平台）若也要在 Ubuntu 接真实 Nano，其 `core/nano_link.py`
  现在走 TCP 42110 网关协议，需改成订阅 ROS2 话题（下一步，按需再做）。
```
