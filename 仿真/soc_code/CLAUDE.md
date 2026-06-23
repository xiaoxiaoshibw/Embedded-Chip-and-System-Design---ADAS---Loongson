# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS2 ADAS control node for a pair of Jetson Nano boards (LKA + ACC + AEB), talking to an ESP32 actuator over serial. One Jetson runs `primary`, the other `backup`; they fail over via UDP heartbeat. The control loop runs at `LOOP_HZ` (default 100Hz / 10ms budget).

## Commands

```bash
# Install Python deps (rclpy/std_msgs/geometry_msgs come from ROS2, NOT pip)
python3 -m pip install -r requirements.txt

# Run (role precedence: --role > $NANO_ROLE > primary)
source /opt/ros/<distro>/setup.bash
python3 ADAS.py --role primary      # on the primary Jetson
python3 ADAS.py --role backup       # on the backup Jetson

# Offline replay / scenario sim — same control kernel as the node, no ROS:
python3 replay.py <telemetry.csv>     # 不带路径自动取 /tmp 下最新的 adas_*_telemetry_*.csv
python3 run_scenario.py scenarios/<name>.yaml
# 现成场景（回归基准，修 bug 前优先把现象复现成新 yaml 再动代码）：
#   straight_cruise / curve_follow / cut_in / lead_hard_brake / lead_lost_reacquire

# 遥测离线分析（控制线程通过后台线程落盘，不阻塞 100Hz 控制环）：
#   节点端：telemetry.py，TELEMETRY=0 完全关闭；路径 /tmp/adas_<role>_telemetry_<启动时间>.csv
#   画图：  python3 plot_telemetry.py [CSV路径]   # 不带路径自动取 /tmp 下最新的

# There is NO test suite, build step, or linter configured.
# Use this as the syntax/compat smoke check before deploying:
python3 -m compileall -q .

# Compileall on Py3.10+ will NOT catch 3.6-incompatible syntax. Always also grep:
grep -rnE "\| None\b|: list\[|: dict\[|: tuple\[|: set\[" --include="*.py" .
grep -rn   "from runtime import"                          --include="*.py" .
# Both should be empty.
```

Runtime logs go to `/tmp/adas_<role>.log` (rotating) plus stdout.

Environment overrides (see `config.py` / README): `NANO_ROLE`, `PRIMARY_IP`, `SECONDARY_IP`, `HB_PORT`, `HB_GRACE`, `SERIAL_ESP32` (default `/dev/ttyTHS1`, the 40-pin UART), `SERIAL_BAUDRATE`, `LOOP_HZ`.

## Hard constraint: Python 3.6 target

JetPack 4.x ships Python 3.6. Code must stay 3.6-compatible:
- No PEP 604 unions (`X | None`) — use `typing.Optional[X]`.
- No PEP 585 builtin generics (`list[int]`) — use `typing.List` etc.
- No walrus `:=`, no `match`.
- `dataclasses` is not in the 3.6 stdlib; it is provided via the backport pinned in `requirements.txt`. Keep using `from dataclasses import ...` — do not hand-roll replacements.

`compileall` on a newer dev Python will NOT catch PEP 604/585 at runtime (they parse fine but raise `TypeError`/`NameError` on import under 3.6). Grep for `| None` and lowercase-generic annotations when reviewing.

## Architecture

**Single node, single timer.** `ADAS.py` `AdasNode` is the only ROS2 node. `_control_loop_impl()` is the entire control pipeline, called once per tick by one timer. Everything is sequential within a tick — there is no per-stage threading. The only background threads are I/O offload (serial, heartbeat, logging).

**Real-time discipline is the central design pressure.** The control loop must never block. Consequences that pervade the codebase:
- `serial_link.Esp32Serial`: all pyserial calls (open/read/write) live in `tx`/`rx` daemon threads. The loop only does non-blocking `send()` (drop-oldest queue) and atomic readback. `drain_rx()` is a deliberate no-op kept for call-site compatibility.
- `common.setup_logging`: logging is async via `QueueHandler` → `QueueListener` thread, so `logging.*` in the loop never hits disk.
- `heartbeat.PeerHeartbeat`: UDP send/recv on daemon threads.
- Loop body wrapped in try/except; `CTRL_CONSECUTIVE_ERROR_LIMIT` consecutive failures → `_send_emergency_stop()` (max brake). Sensor dropout → `_send_sensor_timeout_brake()`.

**Per-tick data flow** (`_control_loop_impl`): refresh heartbeat/role → `evaluate_control_health` gate (return early if not active or sensors not ready) → `_handle_takeover_edge` (seed controllers from primary's last frame on backup takeover) → lane width → `compute_lateral_command` → `evaluate_lead_context` → `update_curve_hold` → `update_aeb_alert` → `compute_longitudinal_policy` → smoothing + takeover rate-limit + range clamp → publish ROS topics → send ESP32 frame → broadcast heartbeat.

**Data model split** (`control/context.py`, `control/state.py`):
- `VehicleSignals` — mutable, written only by ROS subscription callbacks (`_*_callback` in ADAS.py), read by the loop. Callbacks validate via `_safe_float` (finite + range).
- `ControlMemory` — mutable cross-tick state (integrators, filters, counters). `dt` is the loop period; everything time-based derives from it.
- `LateralContext` / `LeadContext` / `LongitudinalContext` — frozen/plain result objects passed down the pipeline within one tick.
- `ControlManagers` — bundle of stateful algorithm objects (`LaneWidthEstimator`, `LeadTracker`, `AebAlertManager`, `CurveHoldManager`, `LongitudinalController`, `LonSmoothing`, `OvertakeManager`) handed to the `control/` policy functions. `LateralSmoothing` is owned by `AdasNode` (not `ControlManagers`) because it runs at the very last leg of the send chain, after `_apply_takeover_guard`.

**`control/` package** holds the pure-ish pipeline stages (`lateral_controller`, `lead_tracking`, `longitudinal_policy`, `curve_hold`, `aeb_alert`, `health`) and the ESP32 frame builder (`serial_protocol`). ADAS.py has thin `_*` wrappers around these — put algorithm logic in `control/`，不要写到 node 里。

**感知层与可选/特性开关模块**（启用前先理解其设计前提）：
- `control/perception.py` — **始终启用**的感知层。把 car2 + 可选的 car3..carN（含 `class=3` 行人）当成一组毫米波雷达目标持续监听，每周期产出 `PerceptionFrame`：所有 fresh 目标的相对位置 / 接近速度 / cut-in 预判 + 主前车选举结果，**共享给下游所有需要"实时相对位置"的模块**（OvertakeManager、未来避障 / class-aware AEB 监控、遥测）。模式由 `config.MULTI_TARGET_COUNT` 决定：
  - `==1`（默认）：只跟 car2；感知层**不写回** `signals.lead_*`（仍由 callback 直接写），与改造前字节级一致 → 这是回滚路径。`PerceptionFrame` 仅作下游/遥测视图，控制行为不变。
  - `>1`：订阅 car3..carN；感知层在 `_select_primary_lead` 中承担主前车选举与 `signals.lead_*` 回写（替换原 `MultiTargetTracker.select` 路径）。lead-swap 时仍触发 [[lead-swap-reset]]（`LeadTracker.reset_on_lead_swap()` + 清 `ControlMemory.filtered_lead_*`）。
  - LeadTracker 对 `signals.lead_*` 独立做 `LEAD_REL_FILTER_ALPHA` / `LEAD_V_PROJ_FILTER_ALPHA` 低通；感知层对各 TrackRel 做同 alpha 的"平行滤波"，两套状态互不污染（便于后续独立演进）。**结果是 primary 目标在两处各有一份滤波量——这是有意为之的小冗余，换取 LeadTracker 行为零变化的回滚兜底**。
  - **行人零代码接入**：Simulink 端把行人填进 car3..carN 槽位并发 `actors_class=3` 即可；Nano 端 AEB class-aware 查表自动生效。
- `control/mpc_longitudinal.py` — `LongitudinalController` 的 drop-in 替代。**仅当 `config.LON_CONTROLLER != 'pid'` 才被实例化**，默认 PID 时根本不导入路径。实现是 LQR + 安全距离硬投影（不是在线 QP），构造时一次性 Riccati 迭代，在线 O(1)。异常或超预算自动回退到 PID。接口与 PID 版本严格一致（含 `lon_cmd 正=减速、负=加速`），换控制器不需要改 longitudinal_policy。
- `control/overtake.py` — 静止前车超车状态机（IDLE→WAIT→ACTIVE→PASSING→RETURN）。**强耦合赛道几何**：假设双车道、自车默认行驶在右车道、`heng_error` 符号为"左正右负"（与 Simulink 端 `chart_94` 一致）。只对 `ControlMemory.target_lane_offset` 和 `suppress_lead_for_overtake` 做副作用，主管线其它阶段不感知。改赛道布局或符号约定时必须同步审 overtake。

**Primary/backup failover** (`heartbeat.py`):
- Primary broadcasts `HB:1 SEQ:n PSI:.. DELTA:.. ACC:.. AEB:0/1`. The `AEB` field is **mandatory**; receivers without it (old primaries) have their entire seed rejected — `peer_last_rx` still updates for liveness, but `consume_takeover_seed()` returns `None` and the backup falls back to zero-init. This is by design — see `_parse_primary_hb_fields` in `heartbeat.py` for the sanity range checks (psi/delta/acc are also range-checked, not just inf/nan).
- Backup takes over on socket silence OR stalled SEQ (loop hung but UDP still resending). On the False→True active edge it seeds `lon_smooth` / `lat_smooth` / `last_delta` from the primary's last control frame and applies a tighter rate limit during `TAKEOVER_GUARD_DURATION_S`. **AEB-seed special case**: if the seed's AEB flag is 1 (primary died mid-emergency-brake), the takeover uses `TAKEOVER_LON_RATE_AEB_RELEASE` (looser, ~12 m/s³) instead of `TAKEOVER_LON_RATE` (~6), so the backup can decay out of full brake based on its own ACC/AEB evaluation rather than inheriting the value for 200 ms.
- Flapping within `TAKEOVER_COOLDOWN_S` only extends the guard window (no re-seed). The cooldown branch reads `lon_smooth.prev` (the rate-limit anchor), **not** `.value` (the post-LP-filter readout) — the two differ during steady-state filtering and using `.value` here would cause a one-step inconsistency at guard entry.
- **Primary-side backup watchdog**: primary now tracks `BACKUP:*` heartbeats. After `HB_BACKUP_TIMEOUT_S` of silence (default 2 s) it logs critical and publishes `False` on `/jetson/failover_available`. Use that topic to detect "no plan B" — the system still drives, but you should know.

## Conventions

- **Config is centralized and star-imported.** Modules do `from config import *`. Add every tunable to `config.py`; do not scatter literals into algorithm files. `dt` is always derived from `LOOP_HZ` — don't hardcode periods.
- **`runtime` is mutated at startup.** `ADAS.main()` writes `os.environ['NANO_ROLE']` then calls `runtime.configure_runtime()` *before* constructing `AdasNode`. Other modules must `import runtime` and read `runtime.NANO_ROLE` / `runtime.IS_PRIMARY` etc. at use time — never `from runtime import IS_PRIMARY` (it would capture a stale pre-configure value).
- **Hot-path logging is rate-limited.** Use `LOG_EVERY_N_CYCLES`, `RateLimitedCritical`, and the existing per-event throttle timestamps. Don't add unthrottled logging inside the loop.
- **Lateral + longitudinal symmetry.** Final ESP32-bound `delta_tx` must pass through `self.lat_smooth.update(...)` (mirrors `lon_smooth.update`). The takeover guard applies via `max_rate_override=TAKEOVER_DELTA_RATE`, *not* a separate post-hoc clamp. Do not bypass `lat_smooth` — any new lateral output path must go through it.
- **Cross-tick lead state is bound to a target ID.** When `_select_primary_lead` swaps the primary lead (multi-target, see `control/perception.py`), call `LeadTracker.reset_on_lead_swap()` and zero `ControlMemory.filtered_lead_*` so old/new targets don't bleed into the same low-pass filters. 触发该路径需 `config.MULTI_TARGET_COUNT > 1`；保持 `1` 时感知层只读不写，零行为变化（回滚路径）。
- **AEB-flag in heartbeat is mandatory.** Adding new HB fields → keep the field strictly required and have `_parse_primary_hb_fields` return `None` on missing/out-of-range. Rolling upgrades require restarting primary and backup together.
- **CLS field in heartbeat is OPTIONAL** (deliberate asymmetry with AEB). It's an optimization hint for the takeover guard's vulnerable-target rate selection; missing/invalid CLS falls back to `ACTOR_CLASS_UNKNOWN` and the takeover uses the regular `TAKEOVER_LON_RATE`. Safety is owned by the AEB flag; CLS only refines the *post-takeover* decay envelope. Do not promote CLS to mandatory — it would break rolling upgrade compatibility for a non-safety hint.
- **class-aware AEB tables live in `config.py` only.** `AEB_CLASS_TTC_MULT` / `AEB_CLASS_ENGAGE_DIST` / `AEB_CLASS_BYPASS_MIN_LEAD_V` / `AEB_CLASS_LAT_GATE_MULT` / `AEB_CLASS_FULL_CONFIRM_CYCLES` are all dicts keyed by `ACTOR_CLASS_*`. Every lookup uses `.get(cls, <vehicle-default>)` so an unknown class falls back to "treat as vehicle" — never crash on a new class.
- **Comments and docstrings are in Chinese**; match the surrounding style when editing.
