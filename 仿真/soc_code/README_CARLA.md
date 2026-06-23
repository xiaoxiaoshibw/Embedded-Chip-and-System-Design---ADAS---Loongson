# CARLA 车辆系统接入说明

本目录新增 `carla_bridge.py`，用于把现有 SOC ADAS 控制内核接入 CARLA。

它不改 CARLA C++/UE4 二进制，也不改 MCU 固件；运行方式是一个 Windows Python 客户端：

```text
CARLA 车辆/道路状态
  -> VehicleSignals
  -> pipeline.run_pure_pipeline()
  -> carla.VehicleControl
  -> CARLA ego 车辆
```

## 环境要求

CARLA 0.9.16 这个发行包的 Python wheel 是 Windows x64 / Python 3.12：

```powershell
py -3.12 --version
py -3.12 -m pip install "D:\Code\自动辅助驾驶仿真平台\CALRA\PythonAPI\carla\dist\carla-0.9.16-cp312-cp312-win_amd64.whl"
py -3.12 -m pip install -r "C:\Users\30680\Desktop\lx\SOCCode\requirements.txt"
```

如果 `py -3.12 --version` 提示找不到运行时，需要先安装 Python 3.12 x64。不能用 Python 3.13 直接加载这个 CARLA wheel。

## 启动 CARLA

先启动 CARLA 服务端：

```powershell
cd "D:\Code\自动辅助驾驶仿真平台\CALRA"
.\CarlaUE4.exe -windowed -ResX=1280 -ResY=720 -quality-level=Low
```

CARLA 默认监听 `127.0.0.1:2000`。

## 运行 ADAS-CARLA 闭环

另开一个 PowerShell：

```powershell
cd "C:\Users\30680\Desktop\lx\SOCCode"
py -3.12 .\carla_bridge.py --map Town03 --duration 120 --loop-hz 20
```

脚本会自动：

- 切到 CARLA 同步模式；
- 生成 ego 车辆和一辆前车；
- 从 CARLA 地图 waypoint 读取道路航向、车道偏移；
- 从 CARLA actor 状态读取前车距离/速度；
- 调用现有 `pipeline.run_pure_pipeline()`；
- 把纵向命令映射为油门/刹车，把横向命令映射为方向盘。

## 常用参数

```powershell
# 不生成前车，只测试巡航 + LKA
py -3.12 .\carla_bridge.py --map Town03 --no-lead --duration 60

# 前车更慢，观察 ACC/AEB
py -3.12 .\carla_bridge.py --map Town03 --lead-gap 25 --lead-speed-diff 45

# 如果方向盘方向相反
py -3.12 .\carla_bridge.py --map Town03 --steer-sign -1

# 如果车道偏移符号相反
py -3.12 .\carla_bridge.py --map Town03 --lane-offset-sign -1

# 跑满原控制器 100Hz，机器性能不够时会卡
py -3.12 .\carla_bridge.py --map Town03 --loop-hz 100 --fixed-delta 0.01
```

## 当前边界

当前 bridge 使用 CARLA 的“真值”状态和地图 waypoint 生成感知量，适合先验证 LKA/ACC/AEB 控制闭环。它还没有接相机、LiDAR、语义分割或 ROS2 topic。后续如果要做感知闭环，可以在这个脚本上增加 CARLA sensors，再把检测结果写入同一个 `VehicleSignals`。
