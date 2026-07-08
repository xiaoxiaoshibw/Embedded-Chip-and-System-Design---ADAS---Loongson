# 龙芯 UDP 桥（无 ROS2 的边缘控制节点接入）

让**龙芯 2K1000（loongarch64）**在**不安装 ROS2** 的前提下，作为实时网络 ADAS 控制节点接入 HIL / 联合仿真：经 UDP 收感知帧 → 跑真实控制内核 `run_pure_pipeline` → 回控制帧 + 广播心跳。

## 为什么不在龙芯上装 ROS2

2026-07-02 实测：龙芯板系统为 **Loongnix-Embedded 20（Debian10 代 userland）**，Python **3.7.3** / gcc **8.3** / cmake **3.13**；而 ROS2 Humble 需 Python 3.10 / gcc 11 / cmake 3.22，**代际差约 4 年**，源码编译不可行（且 /home 仅 4.3 GB 装不下、Loongnix 无预编译 ROS2）。详见 `文档/龙芯定稿/md/龙芯2K1000实机部署实测报告_2026-07-02.md`。

**但这不是问题**：本项目控制内核 `run_pure_pipeline` 本就**零 ROS 依赖**（联合仿真、SIL、锁步影子核、离线回归全复用它）。用 UDP 桥即可让龙芯作为控制节点接入 ROS2 世界——对端由**带 rclpy 的机器**（Jetson Nano / Ubuntu PC）做「ROS2 话题 ↔ UDP」翻译，龙芯本身不碰 ROS2。这与项目既有的「控制/ROS 解耦」架构一致。

## 组成

| 文件 | 运行位置 | 作用 |
|---|---|---|
| `loongson_udp_bridge.py` | **龙芯板**（放进 `lx/SOCCode` 或其部署副本内，需能 import `pipeline`/`replay`） | UDP 收感知 → `run_pure_pipeline` → 回控制帧 + 心跳；可选 `--serial` 下发 ESP32 |
| `pc_udp_peer.py` | 任意 PC（Windows/Linux，纯 stdlib） | 发脚本化感知（稳态跟车→前车急刹），收控制、测往返时延、验 AEB |

## 协议（JSON 行，UTF-8）

```
感知帧 (对端→板): {"t":float,"ego_v":..,"road_psi":..,"lane_offset":..,
                   "lead":{"x":..,"y":..,"v":..,"cls":int}|null}
控制帧 (板→对端): {"type":"ctrl","seq":n,"delta":..,"lon_cmd":..,"aeb":0/1,"ttc":..|null,"compute_us":..}
心跳   (板→对端): {"type":"hb","seq":n,"role":"primary","delta":..,"acc":..,"aeb":0/1}
```

`cls`：前车类别（1=车 2=障碍 3=行人），驱动 class-aware AEB。`lon_cmd` 正=减速 m/s²。

## 用法

**板上**（把 `loongson_udp_bridge.py` 放进已部署的 `SOCCode` 目录）：
```bash
cd ~/adas/SOCCode
python3 loongson_udp_bridge.py --port 9101 --role primary
# 接了 ESP32 时加： --serial /dev/ttyUSB0
```

**PC 上**：
```bash
python pc_udp_peer.py --board 192.168.137.13 --port 9101 --hz 100 --secs 8
```

## 实测结果（2026-07-02，龙芯 2K1000 loongarch64）

> ⚠ 下列数字是在**龙芯同时满载编译 OpenBLAS（双核 make -j2）**的极端争抢条件下取得，纯净环境会更好。

| 指标 | 值 |
|---|---|
| 感知帧 / 控制回执 | 468 / 468，**0 丢失** |
| 心跳帧接收 | 46 |
| 往返时延 mean / p50 / p99 / max | **6.36 / 6.26 / 9.03 / 15.58 ms** |
| AEB 正确性 | 前车 t=3s 急刹 → TTC 14→…→0.1s，**AEB 于 t=4.61s（TTC=3.52）正确触发** |

结论：**龙芯无 ROS2 也能作为实时网络 ADAS 控制节点**，收感知 / 发控制 / 心跳 / AEB 全链路正确，往返时延满足 100 Hz 级需求。

## 接入真实 ROS2 世界（后续）

当前 `pc_udp_peer.py` 是测试对端。要接真实 CARLA/Nano，把对端换成一个跑在 **带 rclpy 的机器**上的「ROS2↔UDP 适配器」：订阅 `/car1_*`/`/road_psi`/`/heng_error` 等感知话题 → 打包成本协议 UDP 发给龙芯；收龙芯控制帧 → 发布 `/jetson/*` 执行话题。可复用 `HIL/carla_bridge/nano/hil_ros_gateway.py` 的话题约定。
