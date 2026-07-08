#!/usr/bin/env bash
# ADAS HIL 网站一键启动 —— Ubuntu 版（等价于 Windows 的 start_web_hil.bat）。
#
# 拓扑：Ubuntu 跑【网站后端 + 前端 + CARLA】，双 Jetson Nano 跑【主控 ADAS.py + 网关】。
# 后端走网关路径（NanoLink → TCP 42110 → hil_ros_gateway.py → ROS2 → ADAS.py），
# 故 PC 侧无需 ROS2，只需 CARLA whl 装进 Python。
#
# 用法：
#   ./start_web_hil_ubuntu.sh [BACKEND_PORT] [WEB_PORT]
# 默认 8000 / 5173。主机地址用环境变量覆盖（见 start_real_backend_ubuntu.sh）。
#   GATEWAY_HOST=192.168.3.125 BACKUP_HOST=192.168.3.124 PC_HOST=192.168.3.8 \
#     ./start_web_hil_ubuntu.sh        # LAN 直连示例
set -euo pipefail

cd "$(cd "$(dirname "$0")" && pwd)"
HERE="$(pwd)"

BACKEND_PORT="${1:-8000}"
WEB_PORT="${2:-5173}"

GATEWAY_HOST="${GATEWAY_HOST:-10.218.44.10}"
BACKUP_HOST="${BACKUP_HOST:-10.218.44.155}"
PC_HOST="${PC_HOST:-10.218.44.190}"

echo "============================================================"
echo " ADAS HIL WebUI launcher (Ubuntu)"
echo " Backend: http://127.0.0.1:${BACKEND_PORT}"
echo " WebUI:   http://127.0.0.1:${WEB_PORT}/live"
echo " Primary Nano: ${GATEWAY_HOST}   Backup Nano: ${BACKUP_HOST}   PC: ${PC_HOST}"
echo "============================================================"

# 1) 后端（后台）。沿用 start_real_backend_ubuntu.sh 的全部环境开关。
BACKEND_LOG="${HIL_BACKEND_LOG:-/tmp/hil_backend.log}"
GATEWAY_HOST="$GATEWAY_HOST" BACKUP_HOST="$BACKUP_HOST" PC_HOST="$PC_HOST" \
    bash "$HERE/start_real_backend_ubuntu.sh" "$BACKEND_PORT" >"$BACKEND_LOG" 2>&1 &
BACKEND_PID=$!
echo "[hil] backend pid=$BACKEND_PID  log=$BACKEND_LOG"

cleanup() {
    echo
    echo "[hil] 停止后端 pid=$BACKEND_PID ..."
    kill "$BACKEND_PID" 2>/dev/null || true
    wait "$BACKEND_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# 2) 等后端端口起来（最多 ~20s）。失败则把日志尾巴打出来便于排查。
echo "[hil] 等待后端 :$BACKEND_PORT ..."
for _ in $(seq 1 40); do
    if bash -c "exec 3<>/dev/tcp/127.0.0.1/${BACKEND_PORT}" 2>/dev/null; then
        echo "[hil] 后端就绪。"
        break
    fi
    if ! kill -0 "$BACKEND_PID" 2>/dev/null; then
        echo "[hil] 后端进程已退出，日志尾部：" >&2
        tail -n 30 "$BACKEND_LOG" >&2 || true
        exit 1
    fi
    sleep 0.5
done

# 3) 前端（前台，Ctrl-C 收尾会经 trap 一并停掉后端）。
if ! command -v npm >/dev/null 2>&1; then
    echo "[hil] 未找到 npm；后端已在跑。前端请另开终端：cd web && npm install && HIL_API=http://127.0.0.1:${BACKEND_PORT} npm run dev -- --host 127.0.0.1 --port ${WEB_PORT}" >&2
    echo "[hil] 当前前台保持后端运行，Ctrl-C 退出。"
    wait "$BACKEND_PID"
    exit 0
fi

if [ ! -d "$HERE/web/node_modules" ]; then
    echo "[hil] 首次运行，安装前端依赖（npm install）..."
    ( cd "$HERE/web" && npm install )
fi

echo "[hil] 打开 http://127.0.0.1:${WEB_PORT}/live ，用硬件面板：一键准备 HIL → 加载场景 → 开始仿真。"
cd "$HERE/web"
HIL_API="http://127.0.0.1:${BACKEND_PORT}" exec npm run dev -- --host 127.0.0.1 --port "$WEB_PORT"
