#!/usr/bin/env bash
# 启动 HIL 平台真实后端（接 CARLA + 双 Nano，控制源 = nano 网关路径）—— Ubuntu 版。
#
# 与 Windows 版 start_real_backend.ps1 逐项对齐；区别只有：跑在 Ubuntu 的 bash 里。
# 重要：本进程是 HIL 平台「网站后端」，走 NanoLink → TCP 42110 → Nano 上的
# hil_ros_gateway.py → ROS2 → ADAS.py 这条网关路径。**PC 侧不需要 source ROS2**
# （全程 import carla，无 rclpy）；只需把 CARLA 的 Python whl 装进当前 Python。
#
# 用法：
#   ./start_real_backend_ubuntu.sh [BACKEND_PORT]
# 主机/口令/CPU 映射等沿用环境变量覆盖（默认与 .ps1 一致，ZeroTier 地址）：
#   GATEWAY_HOST BACKUP_HOST PC_HOST NANO_USER NANO_PW_PRIMARY NANO_PW_BACKUP
#   CARLA_HOST CARLA_PORT CARLA_TOWN ...
set -euo pipefail

PORT="${1:-${HIL_PORT:-8000}}"

cd "$(cd "$(dirname "$0")" && pwd)"

# —— 接真实 CARLA + 双 Nano（与 start_real_backend.ps1 同一组开关）——
export HIL_MOCK="0"
export HIL_CONTROL="nano"
export GATEWAY_HOST="${GATEWAY_HOST:-10.218.44.10}"
export BACKUP_HOST="${BACKUP_HOST:-10.218.44.155}"
export PC_HOST="${PC_HOST:-10.218.44.190}"
export NANO_USER="${NANO_USER:-jetson}"
export NANO_PW_PRIMARY="${NANO_PW_PRIMARY:-yahboom}"
export NANO_PW_BACKUP="${NANO_PW_BACKUP:-jetson}"
export NANO_FAULT_RESTORE_S="${NANO_FAULT_RESTORE_S:-8}"
export PRIMARY_ADAS_CPUS="${PRIMARY_ADAS_CPUS:-0,1,2}"
export PRIMARY_GATEWAY_CPUS="${PRIMARY_GATEWAY_CPUS:-1}"
export BACKUP_ADAS_CPUS="${BACKUP_ADAS_CPUS:-0,1,2}"
export BACKUP_EDGE_CPUS="${BACKUP_EDGE_CPUS:-3}"
export CARLA_HOST="${CARLA_HOST:-127.0.0.1}"
export CARLA_PORT="${CARLA_PORT:-2000}"
export CARLA_TOWN="${CARLA_TOWN:-Town04}"
export CARLA_TM_PORT="${CARLA_TM_PORT:-8010}"
export HIL_CAMERA="${HIL_CAMERA:-0}"
export HIL_PORT="$PORT"

PYBIN="${PYTHON:-python3}"

# 启动前自检：CARLA whl 是否装进了当前 Python（HIL 平台靠 import carla，缺它会在
# load 场景时才报错，提前在这里点明更省事）。
if ! "$PYBIN" -c "import carla" >/dev/null 2>&1; then
    echo "[warn] 当前 Python（$PYBIN）import carla 失败。" >&2
    echo "       把 CARLA 的 whl 装进来：pip install <CARLA>/PythonAPI/carla/dist/carla-0.9.16-*-linux_x86_64.whl" >&2
    echo "       （仅 HIL_CONTROL=nano 且 HIL_MOCK=0 时需要；mock 模式不需要 CARLA）" >&2
fi

echo "[hil] backend on http://0.0.0.0:${HIL_PORT}  control=nano  gateway=${GATEWAY_HOST}:42110"
exec "$PYBIN" -m server.api_server
