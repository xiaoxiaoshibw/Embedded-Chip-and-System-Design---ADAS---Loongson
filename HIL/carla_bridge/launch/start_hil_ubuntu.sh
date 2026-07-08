#!/usr/bin/env bash
# ============================================================
#  Ubuntu 24.04 + ROS2 Jazzy 上启动 HIL CARLA ROS2 桥节点
#  取代 Windows 的 .ps1 编排：本机跑 CARLA + carla_ros_node.py，
#  跨网 ROS2 DDS 直连两台 Jetson Nano（Foxy）上的 ADAS.py。
#  用法： ./start_hil_ubuntu.sh [scenario] [jetson|esp32]
#  例如： ./start_hil_ubuntu.sh acc jetson
# ============================================================
set -euo pipefail

SCENARIO="${1:-acc}"
SOURCE="${2:-jetson}"
if [ "$#" -gt 0 ]; then
  shift
fi
if [ "$#" -gt 0 ]; then
  shift
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PC_DIR="$(cd "$HERE/../pc" && pwd)"

# ── ROS2 环境 ──
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
# shellcheck disable=SC1090
source "$ROS_SETUP"

# 与两台 Nano 上 ADAS.py 一致：把本 HIL 闭环与旧 /perception_sim(domain 42) 隔离
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
# 跨网段 DDS 必需（沿用旧约定；Jazzy 也兼容此变量）
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# ── DDS 发现 ──
# 同一 LAN 子网（PC 与 Nano 都在 192.168.3.x）：多播发现开箱即用，无需下面任何一行。
# 跨子网 / ZeroTier（如 PC↔Nano 走 10.218.44.x）：多播常不通，二选一启用单播发现：
#   (a) Jazzy 原生静态对等体（最简单，仅 PC 端设即可让 PC 发现 Nano）：
# export ROS_STATIC_PEERS="10.218.44.10;10.218.44.155"
#   (b) Fast DDS profile（PC 与 Nano 两端都 export，最稳）：
# export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/fastdds_peers.xml"
#
# Jazzy(Fast DDS 2.14) ↔ Nano Foxy(Fast DDS 2.0) 跨发行版互通若不稳，
# 两端统一改用 CycloneDDS（见 README_Ubuntu.md「DDS 互通」）：
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "[start_hil_ubuntu] ROS_DOMAIN_ID=$ROS_DOMAIN_ID scenario=$SCENARIO source=$SOURCE"
echo "[start_hil_ubuntu] 先确认 CARLA 已在 :2000 跑起来，且两台 Nano 已在 domain $ROS_DOMAIN_ID 跑 ADAS.py"

cd "$PC_DIR"
exec python3 carla_ros_node.py --scenario "$SCENARIO" --actuation-source "$SOURCE" "$@"
