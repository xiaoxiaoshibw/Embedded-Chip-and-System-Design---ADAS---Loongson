# HIL 控制台演示视频

## 主备失控接管_HIL控制台.mp4

在 **HIL 控制台（mock 模式，虚拟模拟备机 Nano，无需真实硬件）** 上录制的**主备失控接管**演示（约 30s）：

- 架构：**龙芯 2K1000 作主控**，**Jetson Nano 作热备**，ESP32 作安全仲裁
- 场景 `takeover`（主控故障接管），跟车行驶
- 操作：控制台上选场景 → 加载场景 → 开始
- 第 12s **龙芯主控** 发生 `seq_stuck`（假活：进程还在发心跳但控制进度 seq 停滞）
- ESP32 仲裁检测到 `seq_not_increasing` 后切到备机 **Nano B**：
  - 顶栏 `生效控制器` 龙芯 主控 → **Nano B 接管**，`是否接管` 正常 → **已接管**，`门控裁决` normal → **backup_takeover**
  - 底部 龙芯（主控）**生效中 → 热备**、Nano B（备控）**热备 → 生效中**、ESP32 `takeover_count` 0 → **1**
- 全程车辆无碰撞、指标稳定

**关键帧**：`接管前_龙芯主控生效.png`（龙芯生效中）、`接管后_NanoB接管.png`（Nano B 接管、takeover_count=1、reason=seq_not_increasing）。

**画幅**：1600×1240 整页视口——从顶部状态栏到**底部全部实时曲线**（EGO 速度 / TTC / 前车距离 / 横向误差 / 刹车 / **生效控制器**（12s 处 A→B 阶跃）/ AEB DRAC）一条视频全录入，无需滚动或拆分。

## 本次为演示做的前端改动（已并入源码）

1. **主控标注为「龙芯」**：`lib/format.ts` 的 `nano_a` 标签、`LivePage` 控制器面板名、`FaultPanel` 故障标签、`App.tsx` 顶栏文案（龙芯主控 + Nano 热备），与项目真实架构（龙芯主控 + Nano 热备）一致。
2. **mock 模式下隐藏两组面板**：`LivePage` 把「硬件在环控制」面板与「CARLA 实时画面 / 世界辅助」面板 gate 在 `!mock`——这两块在 mock 模式本就非功能态（硬件 SSH 探测恒 ERROR、CARLA 摄像头恒未就绪），隐藏后演示画面更干净。真实模式（`HIL_MOCK=0`）不受影响，照常显示。

## 复现 / 重录

```bash
# 1) 起后端（mock 模式，单进程托管 /live）
cd HIL/hil_platform
HIL_MOCK=1 HIL_PORT=8000 python -m server.api_server
# 浏览器可直接看： http://127.0.0.1:8000/live

# 2) 自动录制（Playwright + ffmpeg）
python HIL/hil_platform/演示视频/record_hil_failover.py
# 产物：演示视频/hil_video/*.webm，再用 ffmpeg 转 mp4
```

依赖：`pip install playwright && python -m playwright install chromium`，系统装 `ffmpeg`。
`record_hil_failover.py` 用真实 UI 点击（选场景/加载/开始），并用后端 API 兜底确保失控被录到。
