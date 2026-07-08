# HIL 平台在 Ubuntu 上运行（网站 + HIL 在 Ubuntu，主控在 Nano）

本文档说明把 **HIL 监控/回放平台（网站）** 跑在 Ubuntu 上的方式。拓扑：

```text
Ubuntu 主机
  ├── CARLA（CarlaUE4.sh，真值世界）
  ├── HIL 后端  server.api_server（FastAPI + WebSocket，端口 8000）
  └── HIL 前端  web/（React + Vite，端口 5173 → /live、/replay）
        │  NanoLink → TCP 42110
        ▼
双 Jetson Nano（Foxy，ROS_DOMAIN_ID=43）
  ├── hil_ros_gateway.py（PC↔Nano 网关，TCP 42110 ↔ ROS2 话题）
  └── ADAS.py --role primary / backup（**主控就在 Nano 上**）
        ▼
   ESP32（主备仲裁 + AEB 硬件地板）
```

> **要点**：HIL 平台后端全程 `import carla`、**无 `rclpy`**，所以 **Ubuntu/PC 侧不需要 source ROS2**——只需把 CARLA 的 Python whl 装进当前 Python。ROS2 只在 Nano 上（网关 + ADAS）。
>
> 这条是**网关路径**（与 2026-06-24 实机验证过的 HIL 闭环一致）。它与 `HIL/carla_bridge/pc/carla_ros_node.py`（无网关、直连 ROS2、**不带网站**）是两条独立的桥——网站走的是本文这条。

## 1. Ubuntu 依赖

```bash
# Python 后端依赖（在 hil_platform/ 下）
python3 -m pip install -r requirements.txt        # fastapi / uvicorn / paramiko / pyyaml ...

# CARLA 客户端（仅真实模式需要；mock 模式不需要 CARLA）
pip install /path/to/CARLA/PythonAPI/carla/dist/carla-0.9.16-cp3xx-linux_x86_64.whl
python3 -c "import carla; print('carla ok')"

# 前端（Node 18+）
cd web && npm install && cd ..
```

## 2. mock 模式（无任何硬件，先验证网站本身）

```bash
HIL_MOCK=1 python3 -m server.api_server          # 后端
npm --prefix web run dev                          # 前端（另一个终端）→ http://127.0.0.1:5173/live
```

`seq` 卡死 → NanoB 接管约 150ms、时间轴联动等，全部在 mock 下可演示。

## 3. 真实模式（网站 + HIL 在 Ubuntu，主控在 Nano）

**前置**：先单独起 CARLA：

```bash
/path/to/CARLA/CarlaUE4.sh -quality-level=Low
```

**一键启动**（后端后台 + 前端前台，Ctrl-C 一并收尾）：

```bash
chmod +x start_web_hil_ubuntu.sh start_real_backend_ubuntu.sh

# 默认 ZeroTier 地址（与 Windows 版 start_web_hil.bat 一致）
./start_web_hil_ubuntu.sh                          # 8000 / 5173

# LAN 直连示例（按实际网段覆盖）
GATEWAY_HOST=192.168.3.125 BACKUP_HOST=192.168.3.124 PC_HOST=192.168.3.8 \
  ./start_web_hil_ubuntu.sh 8000 5173
```

打开 `http://127.0.0.1:5173/live`，用**硬件面板**：一键准备 HIL（部署+起网关+起两台 Nano 的 ADAS）→ 加载场景 → 开始仿真。

> 网站会经 `paramiko` SSH 到两台 Nano：部署/启动 `hil_ros_gateway.py` 与 `ADAS.py`、注入故障、回切。账号口令由 `NANO_USER` / `NANO_PW_PRIMARY` / `NANO_PW_BACKUP` 环境变量提供（默认 `jetson` / `yahboom` / `jetson`）。

**只起后端**（前端自己另开）：

```bash
./start_real_backend_ubuntu.sh 8000
# 另一个终端：
cd web && HIL_API=http://127.0.0.1:8000 npm run dev -- --host 127.0.0.1 --port 5173
```

## 4. 关键环境变量

| 变量 | 默认 | 含义 |
|---|---|---|
| `HIL_MOCK` | `1` | `0` = 接真实 CARLA + Nano |
| `HIL_CONTROL` | `internal` | 启动脚本里设为 `nano` = 真实双 Nano + ESP32 |
| `GATEWAY_HOST` / `BACKUP_HOST` | ZeroTier | 主 / 备 Nano 地址 |
| `PC_HOST` | ZeroTier | 本 Ubuntu 机对 Nano 可达的地址（传给网关 `--pc-host`） |
| `GATEWAY_PORT` | `42110` | Nano 网关 TCP 端口 |
| `CARLA_HOST` / `CARLA_PORT` / `CARLA_TOWN` | `127.0.0.1` / `2000` / `Town04` | CARLA 连接 |
| `HIL_PORT` | `8000` | 后端端口 |
| `HIL_CAMERA` | `0` | 是否拉相机帧 |

## 5. 排错

- **后端起不来 / `import carla` 失败**：确认 CARLA whl 已装进**当前** `python3`（真实模式必需；mock 不需要）。`start_real_backend_ubuntu.sh` 启动时会预检并提示。
- **`无法连接 Nano 网关 …:42110`**：先在 `/live` 硬件面板点「一键准备 HIL」让网站把网关部署+拉起；或确认两台 Nano 在线、`ROS_DOMAIN_ID=43`、PC↔Nano 网络互通。
- **前端连不上后端**：检查 `HIL_API` 指向后端实际端口；后端绑 `0.0.0.0`，跨机访问用 Ubuntu 主机 IP。
