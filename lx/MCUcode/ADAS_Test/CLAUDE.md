# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ADAS AEB (Automatic Emergency Braking) control system — a safety-critical embedded system for autonomous vehicle control. ESP32 microcontroller acts as the real-time safety controller, communicating with dual Jetson Nano computers over UART. The Jetsons run ROS2-based perception and high-level control; the ESP32 executes braking and steering commands with hardware-level safety guarantees.

## Build Commands

This is an ESP-IDF v5.5.3 project targeting ESP32. Requires the ESP-IDF environment to be sourced first.

```bash
# Build firmware
idf.py build

# Flash to ESP32 (COM3, configured in .vscode/settings.json)
idf.py flash

# Monitor serial output
idf.py monitor

# All-in-one
idf.py build flash monitor
```

> **注意：** 原 `jetson_adas_dual_nano (10).py` 遗留快照已删除。当前 SOC 代码在 `SOCCode/`（见其 `CLAUDE.md`）。

## Architecture

### ESP32 Firmware (FreeRTOS)

本目录仅含 ESP32 固件。SOC 控制节点代码在 `lx/SOCCode/`。

### ESP32 Firmware (FreeRTOS)

Four concurrent tasks with strict priority ordering:

| Task | Priority | Period | Role |
|------|----------|--------|------|
| `comm_watchdog_task` | 10 (highest) | 50ms | Emergency brake if both Jetsons timeout (200ms) |
| `uart_rx_task` | 9 | 5ms | Parse UART frames from both Jetsons |
| `control_task` | 8 | 10ms | Run LKA/ACC/AEB control step (registered with TWDT) |
| `tx_task` | 7 | 10ms | Send control outputs (P/D/B) back to both Jetsons |

**Shared state** is protected by a single FreeRTOS mutex (`mtx`). All reads/writes to `JetsonState`, `g_use_secondary`, and control outputs (`psi_cmd`, `delta_cmd`, `a_brake`) must hold the lock. `volatile` on `g_use_secondary` is only for compiler optimization prevention — cross-core visibility comes from the mutex's memory barriers.

**Safety layers** (innermost to outermost):
1. **Jetson AEB** — Jetson computes TTC-based braking, sends `ACC` field
2. **ESP32 AEB (hardware floor)** — `update_aeb()` triggers full brake when `dist <= hard_floor` (40% of safe_dist, min 2.5m). This is the last-resort defense; it does NOT re-compute TTC
3. **Communication watchdog** — highest-priority task bypasses control entirely, sends emergency brake frames directly over UART if both Jetsons are silent for 200ms
4. **TWDT (Task Watchdog Timer)** — hardware reset if `control_task` or `comm_watchdog_task` hangs for 3 seconds

### UART Frame Protocol

**Jetson → ESP32** (tagged format, ~120 bytes):
```
TTC:8.00 DIST:15.50 PSI:0.1234 DELTA:0.0500 SPEED:16.70 ACC:-2.50 OFFSET:0.100 LEADV:14.00 DSAFE:10.00 WMRN:1.98 WHRD:3.06 CURV:0.01 CRC:AB
```

**ESP32 → Jetson** (fixed format):
```
P:+0.1234
D:+0.05
B:-2.50
SRC:0          # 0=primary, 1=secondary, 9=watchdog emergency
```

CRC-8/MAXIM (Dallas polynomial 0x31, initial value 0x00) covers all bytes before ` CRC:`. Frames without CRC are accepted for backward compatibility.

### Dual-Redundancy Arbitration

Two Jetson Nanos connect independently to ESP32 via UART1 (GPIO 16/17) and UART2 (GPIO 18/19). The `arbitrate()` function selects the active source:
- Primary fresh → use primary (auto-recovery when primary returns)
- Primary stale, secondary fresh → switch to secondary (logged as `SWITCH:pri_timeout_*`)
- Both stale → returns whichever was last used; watchdog handles the emergency

The two Jetsons also communicate via UDP heartbeat on LAN for failover coordination. The secondary waits 8 seconds after startup (cold-start grace) before considering takeover.

### Jetson Nano ROS2 Node (legacy snapshot — current code in `SOCCode/`)

Same code runs on both boards, differentiated by `NANO_ROLE` environment variable.

**Subscribed topics:** `/car1_xy`, `/car2xy`, `/car1_psi`, `/car1_v`, `/car2_v`, `/road_psi`, `/heng_error`

**Published topics:** `/jetson/psi`, `/jetson/delta`, `/jetson/brake`, `/esp32/psi`, `/esp32/delta`, `/esp32/brake`, `/jetson/active_role`, `/jetson/lane_offset`

Control loop runs at 50Hz. Algorithms:
- **LKA**: PID heading control + curvature feedforward (`atan(L×κ)`) + CTE PD correction from `/heng_error`
- **ACC**: Time-gap policy with cruise speed target (60 km/h default), cornering speed limiting
- **AEB**: TTC-based and distance-based braking triggers

## Key Configuration

Tunable parameters live in two places:
- **ESP32**: Constants at top of `main/main.c` (e.g., `MCU_AEB_MAX_BRAKE_DECEL`, `JETSON_TIMEOUT_MS`, `WATCHDOG_TIMEOUT_MS`)
- **Jetson**: Module-level constants in the Python file (e.g., `ACC_TIME_GAP`, `K_PSI_P`, `LANE_WARN_MARGIN`, `CRUISE_TARGET_SPEED`)
- **ESP-IDF Kconfig**: `sdkconfig.defaults` overrides UART pins, baudrate, and AEB thresholds

Dynamic boundaries (`WMRN`/`WHRD` fields) are sent by the Jetson each frame and override the ESP32's default margins after first frame arrival.
