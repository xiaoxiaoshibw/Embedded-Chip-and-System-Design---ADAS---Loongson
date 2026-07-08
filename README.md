# 边缘智能双冗余 ADAS 仿真与开发平台

高级驾驶辅助系统（ADAS）仿真与开发平台，实现 **LKA（车道保持）/ ACC（自适应巡航）/ AEB（自动紧急制动）** 双冗余容错架构，覆盖从纯软件仿真（SIL）、CARLA 联合仿真到真实硬件在环（HIL）的完整验证链路。

## 系统架构

```
感知层 (Simulink/ROS2)
    │  ROS2 Topics
    ▼
主控（龙芯 / Jetson Nano）  ◄──UDP 心跳──►  备控（Jetson Nano）
    │  UART1                                    │  UART2
    ▼                                            ▼
ESP32 微控制器（实时执行器 + 硬件安全地板）
```

**安全层（由内到外）：**
1. 主控 SOC 上基于 TTC 的 AEB 制动计算
2. ESP32 AEB 地板 —— 硬件兜底全力制动
3. 通信看门狗 —— 双控静默 60ms → 紧急制动
4. TWDT —— 控制任务卡死 3s → 硬件复位

## 目录结构

| 目录 | 用途 |
|---|---|
| `lx/SOCCode/` | SOC 控制栈核心源码——单节点单定时器 ROS2 节点，100Hz 实时控制环，含横/纵向控制、AEB、心跳容错、下发安全裁决等模块 |
| `lx/MCUcode/ADAS_Test/` | ESP32 MCU 固件（FreeRTOS，4 个实时任务：通信看门狗 / UART 解析 / 控制步 / 下发），双路仲裁 + AEB 硬件地板 |
| `HIL/` | 硬件在环（HIL）相关代码：`hil_platform/` 为 FastAPI + React 实时监控与回放平台；`carla_bridge/` 为 CARLA↔Nano 闭环桥（Windows TCP 桥方案 + Ubuntu 原生 ROS2 桥方案） |
| `CALRA/` | CARLA 0.9.16 预编译二进制分发（本地使用，不随仓库分发） |

> 本仓库仅托管核心源码；竞赛文档、实验报告、演示视频等资料未纳入版本管理。

## 快速开始

```bash
# SOC 控制节点（Jetson Nano / 龙芯上运行）
cd lx/SOCCode
python3 -m pip install -r requirements.txt
python3 ADAS.py --role primary   # 主控
python3 ADAS.py --role backup    # 备控

# 离线回归测试（无需 ROS2/硬件）
python3 -m pytest lx/SOCCode/tests

# ESP32 固件（需 ESP-IDF v5.5.3 环境）
cd lx/MCUcode/ADAS_Test
idf.py build flash monitor

# HIL 监控平台
python -m uvicorn server.api_server:app --port 8000   # HIL/hil_platform/
npm --prefix HIL/hil_platform/web run dev             # 前端 http://127.0.0.1:5173
```

## 关键约束

- **Python 3.6 兼容写法**（SOCCode）：目标硬件含旧版 JetPack/嵌入式环境，禁用 `X | None`、`list[int]`、`:=`、`match` 等新语法
- **硬实时**：100Hz / 10ms 预算，所有阻塞 I/O（串口、日志、遥测）在守护线程运行
- **配置集中化**：所有可调参数在 `config.py`，不在算法文件中散布字面量
