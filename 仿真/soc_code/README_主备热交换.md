# 主备 Jetson Nano 热交换操作手册

本文档说明如何在**车辆运行中**对 ADAS 双 Jetson Nano 集群做以下操作而**不中断控制**：

1. 滚动升级（升级代码 / 改配置 / 重启进程）
2. 角色互换（长期换主）
3. 物理热插拔（更换硬件）

适用对象：现场运维 / 部署工程师。

---

## 0. 操作前必做的检查

任何"热"操作前，务必先确认冗余可用。**没有 plan B 就不要停任何一台。**

```bash
# 1) 看谁在实际控制
ros2 topic echo --once /jetson/active_role
# 期望输出: primary  或  secondary_standby（说明备机健康但未接管）

# 2) 看冗余是否可用
ros2 topic echo --once /jetson/failover_available
# 期望输出: data: true

# 3) 看最近有没有接管/失援事件
tail -n 200 /tmp/adas_primary.log /tmp/adas_backup.log | \
    grep -E "TAKEOVER|HB|EMERGENCY|SANITY"
# 期望: 无 critical 级事件（除启动时的初始化日志）
```

任一项不通过，**停止操作，先排查**。

另开一个终端做实时监控，整个操作过程都保留：

```bash
# 控制权变化
ros2 topic echo /jetson/active_role &
# 冗余状态
ros2 topic echo /jetson/failover_available &
# 关键日志
tail -F /tmp/adas_primary.log /tmp/adas_backup.log | \
    grep --line-buffered -E "TAKEOVER|HB|EMERGENCY|MULTI_TARGET" &
```

---

## 1. 滚动升级（最常用）

目标：换代码 / 改配置 / 重启进程，**车不停、控制不中断**。

### 1.1 协议兼容性硬约束

本仓库强制主备心跳协议同版本：心跳报文必须含 `AEB:0/1` 字段。
缺字段时备机会拒绝种子（`peer_last_rx` 仍更新，但接管时回落到 zero-init）。

→ **必须先升级备机，后升级主机**。反过来会让中间窗口里"备机已新、主机仍旧"，
   主机不发 AEB 字段，备机收到也会拒绝；虽然不会失控，但接管时无法"无感降级"。

### 1.2 步骤

```bash
# ── 在备机（secondary）上 ──

# (1) 拷贝新代码，不重启
rsync -avh new_code/ ~/adas/

# (2) 重启备机进程
sudo systemctl restart adas-backup
# 或: pkill -f 'ADAS.py.*backup' && nohup python3 ADAS.py --role backup &

# 此时主机日志预期:
#   [HB] backup heartbeat lost (2000ms), failover unavailable
# /jetson/failover_available → False，持续 ≈ 备机进程启动时间 + HB_GRACE_S(3s)

# (3) 等备机回来
# 主机日志预期:
#   [HB] backup heartbeat resumed, failover available
# /jetson/failover_available → True
# 必须看到这条日志再继续，否则停止操作

# ── 在主机（primary）上 ──

# (4) 拷贝新代码
rsync -avh new_code/ ~/adas/

# (5) 重启主机进程 —— 真正的切换时刻
sudo systemctl restart adas-primary

# 备机日志预期（几乎瞬间）:
#   [HB] primary silence 500ms, backup takeover
#   [TAKEOVER] seed psi=.. delta=.. lon=+.. aeb=0 guard=200ms
# 此时备机变成 secondary_active，向 ESP32 输出控制

# (6) 主机重新就绪后:
# 备机日志预期:
#   [HB] primary heartbeat detected, holding active for 300ms
#   [HB] primary heartbeat stable, backup standby
# 控制权回到主机，/jetson/active_role 变回 primary
```

### 1.3 风险窗口与缓解

- **步骤 5 接管 + 步骤 6 回切**，每次经历一次 takeover guard，
  纵向输出被 `TAKEOVER_LON_RATE = 6 m/s³` 限速 200 ms。
  方向盘经 `lat_smooth` + `TAKEOVER_DELTA_RATE` 平滑，不会跳变。
- 步骤 5 → 6 之间，备机以自身评估输出，约 = 主机进程启动时间 + `HB_GRACE_S`（默认 3 s）。
  这段时间车正常控制，仅控制源换人。
- **要不要缩短 HB_GRACE_S？** 改 0.5 s 可，但 Jetson Nano 上 ROS init 通常超过 0.5 s，
  会导致主机始终被备机判定为"刚启动还没就绪"。**不建议改**。

### 1.4 回滚

如果新版本上线后发现问题，立即回滚步骤同上（先备机后主机），代码替换回旧版本即可。
**回滚前先确认旧版本心跳协议同样含 AEB 字段**，否则属于"降级回不带 AEB 的旧协议"，
当前仓库的备机会拒绝旧主机种子。

---

## 2. 角色互换（让原备机长期当主）

适用场景：主机硬件需要长时间维护、想轮换硬件。

⚠ **此操作必须停车进行，不属于热交换**。

```bash
# (1) 在原主机上
sudo systemctl stop adas-primary

# (2) 在原备机上：停备机服务、改 NANO_ROLE、起动为主机
sudo systemctl stop adas-backup
# 修改 systemd 单元里的 --role 参数和 NANO_ROLE 环境变量
sudo systemctl edit adas-primary --full   # 在新机器上启用 primary 单元
sudo systemctl start adas-primary

# (3) 在原主机（现要当备机）上
sudo systemctl edit adas-backup --full
sudo systemctl start adas-backup
```

### 注意事项

1. **IP 不动**：`PRIMARY_IP` / `SECONDARY_IP` 是物理设备地址，不是角色地址。
   互换角色不要改 IP 配置，否则双方找不到对方。
2. 中间会有"两台都没在控"的时间窗口（约 1~3 s）。**操作前让车停稳**。
3. 互换完成后再用 0 节里的检查清单确认。

---

## 3. 物理热插拔（更换 Jetson 硬件）

### 3.1 替换备机（零风险路径）

适用：备机硬件故障 / 升级硬件版本。

```bash
# (1) 直接断开原备机：拔电源、拔网线、拔 UART（如果备机这台也接了 ESP32）
#     主机日志预期: [HB] backup heartbeat lost (2000ms), failover unavailable
#     /jetson/failover_available → False
#     主机仍正常控制车辆

# (2) 接入新 Jetson:
#     - 配置同样的 IP (SECONDARY_IP)
#     - 部署同版本代码
#     - 设置 NANO_ROLE=backup
#     - 启动 adas-backup 服务

# (3) 主机日志预期: [HB] backup heartbeat resumed, failover available
#     /jetson/failover_available → True
```

整个过程主机始终在控，无切换、无窗口。

### 3.2 替换主机（必须让备机接管再操作）

⚠ 风险高于替换备机。建议在**低速直道**进行。

```bash
# (1) 操作前必做检查通过

# (2) 直接拔主机电源（不要用 shutdown -h）
#     备机日志预期（≤ 500ms 内）:
#       [HB] primary silence 500ms, backup takeover
#       [TAKEOVER] seed psi=.. delta=.. lon=+.. aeb=0 guard=200ms
#     /jetson/active_role 变为 secondary_active
#     此时备机是实际控制者

# (3) 接入新主机:
#     - 配置 PRIMARY_IP
#     - 部署同版本代码
#     - 设置 NANO_ROLE=primary
#     - 启动 adas-primary 服务

# (4) 新主机就绪后:
#     备机日志预期:
#       [HB] primary heartbeat detected, holding active for 300ms
#       [HB] primary heartbeat stable, backup standby
#     控制权回到新主机
```

### 3.3 为什么不用 `shutdown -h`？

`shutdown -h` 会让主机进程走正常退出路径，最后一帧可能正在做某种状态变换（比如
ACC 释放、巡航重启），种子语义不确定。直接断电反而让主机最后一帧是稳定运行帧，
备机种子最可信。

### 3.4 风险窗口

- 步骤 2 → 3 之间：
  - 500 ms 主机静默期（备机检测到主机死所需时间）
  - 200 ms 接管保护（`TAKEOVER_GUARD_DURATION_S`）
  - **合计 ~700 ms** 控制"半响应"窗口。
- 这 700 ms 内备机基于自身评估输出。因为 `lat_smooth` + `lon_smooth` 速率限幅在工作，
  方向盘 / 制动不会阶跃，但有经验的司机能感觉到响应变缓。
- **绝对不要**在高速、弯道、紧急路况下做主机热插拔。

---

## 4. AEB 期间发生主机故障的特殊情形

如果主机正在 AEB 全制动时进程崩溃 / 被强杀：

1. 心跳里 `AEB:1` 标志会被备机识别。
2. 备机接管时使用 `TAKEOVER_LON_RATE_AEB_RELEASE = 12 m/s³`（而不是常规的 6），
   允许备机在 200 ms 保护窗内从 10 m/s² 全制动衰减到自己评估出的合理值。
3. 这避免了"主机临死前一帧是全制动 → 备机继承 200 ms 错误全制动"的问题。

运维不需要做任何特殊操作，备机自动处理。日志会显示 `aeb=1`：

```
[TAKEOVER] seed psi=.. delta=.. lon=+10.00 aeb=1 guard=200ms
```

如果你看到 `aeb=1` 接管，事后要检查主机为什么在 AEB 时崩了 —— 那是真正要修的 bug。

注：日志格式自 2026-05-24 起新增 `cls=<actor_class>` 字段（见下一节）。新格式为：

```
[TAKEOVER] seed psi=.. delta=.. lon=+10.00 aeb=1 cls=3 guard=200ms
```

---

## 4.1 行人/障碍场景接管的衰减

如果接管发生时主前车是行人（`cls=3`）或静止障碍（`cls=2`），但主机本帧**并不**处于 AEB：

1. 心跳里 `CLS:N` 字段（**可选**，缺失即 `cls=0=UNKNOWN`）被备机识别。
2. 备机接管使用 `TAKEOVER_LON_RATE_VULNERABLE = 4 m/s³`（更严，数值小=变化慢），
   防止备机在接管瞬间对行人/障碍快速放车——给备机自身评估留出时间。
3. 三分支优先级：`AEB:1 > CLS∈{2,3} > 其他` —— AEB 全制动衰减优先于"行人保护"。

预期日志（行人场景，主机非 AEB）：

```
[TAKEOVER] seed psi=.. delta=.. lon=-0.50 aeb=0 cls=3 guard=200ms
```

CLS 字段在心跳协议里**故意设为可选**，与强制要求的 AEB 字段对照：CLS 只是接管期
速率选择的优化提示，缺失自动退化到 `TAKEOVER_LON_RATE = 6 m/s³`，行为与改造前一致。
这样旧主机（无 CLS 字段）和新备机也能正常滚动升级，不会因为协议不匹配丢种子。

---

## 5. 监控话题与日志对照表

| 话题 / 日志 | 含义 | 期望值 |
|---|---|---|
| `/jetson/active_role` | 当前控制者 | `primary` 或 `secondary_standby` |
| `/jetson/failover_available` | 冗余是否可用 | `True` |
| `[HB] backup heartbeat lost` | 主机看不到备机 | 仅升级备机时短暂出现 |
| `[HB] backup heartbeat resumed` | 备机恢复在线 | 升级备机完成后必出现 |
| `[HB] primary silence ...ms, backup takeover` | 备机接管 | 仅升级主机/换主机时出现 |
| `[TAKEOVER] seed ... aeb=0 guard=200ms` | 备机种子初始化 | 接管时出现一次 |
| `[TAKEOVER] seed ... aeb=1 ...` | 备机从 AEB 主机接管 | 不应在升级时出现，出现就要排查 |
| `[TAKEOVER] flapping detected` | 1 s 内多次接管 | 升级时不应出现，出现说明双机网络异常 |
| `[HB] primary heartbeat stable, backup standby` | 控制权回主机 | 升级主机完成后必出现 |
| `[HB] N primary HB frames rejected by sanity` | 主机心跳脏值 / 缺字段 | 不应出现，出现说明版本不一致或主机算错 |

---

## 6. 绝对禁止

1. **同时重启两台**。一旦两端同时进入 `HB_GRACE_S` 宽限期（默认 3 s），ESP32 会在
   这 3 s 内收不到任何控制帧；车按 ESP32 端的"最后一帧保持"行为继续行驶 —— 这通常意味着
   保持当前转角 + 当前制动指令。后果不可预测。
2. **新主机 + 旧备机 / 旧主机 + 新备机**长期混跑。短暂的（升级过程中的）混跑是允许的，
   但完成升级前不要让车长时间在混跑状态。
3. **修改 PRIMARY_IP / SECONDARY_IP 不重启对端**。心跳会失联，备机会接管，
   且回切时新 IP 双方不互认。
4. **在高速 / 弯道 / 紧急路况下做主机热插拔**。请等到直道低速。
5. **删除 `--no-verify` 风格地跳过 sanity 检查**。心跳里 `_parse_primary_hb_fields`
   的范围检查是无感降级的基石，绝不允许放宽。

---

## 7. 进一步降低切换感知（可选改造）

当前 `HEARTBEAT_TIMEOUT_S = 500ms` 是硬天花板。若要把滚动升级时的切换做到"完全无感"，
可以做以下改造（不在本仓库默认实现内）：

1. 把 `HEARTBEAT_TIMEOUT_S` 降到 50~80 ms。
2. 把 `HB_SEND_INTERVAL_S` 降到 5~10 ms（最好绑到 `LOOP_HZ`）。
3. 给主机/备机心跳线程 `chrt -f 80` 实时优先级（需要 PREEMPT_RT 内核）。
4. 在主机进程加 `atexit` 钩子：systemctl 发 SIGTERM → 主机在退出前多发 3 帧
   带 `HANDOFF:1` 字段的"主动让位"心跳，备机看到就跳过 silence 判定直接接管。

第 4 项约 30 行改动。如有需要可单独发起一次实施。

---

## 附：服务单元参考

`/etc/systemd/system/adas-primary.service` 模板：

```ini
[Unit]
Description=ADAS Primary Node
After=network-online.target

[Service]
Type=simple
User=nvidia
Environment=NANO_ROLE=primary
Environment=PRIMARY_IP=192.168.31.131
Environment=SECONDARY_IP=192.168.31.161
Environment=SERIAL_ESP32=/dev/ttyTHS1
Environment=LOOP_HZ=100
WorkingDirectory=/home/nvidia/adas
ExecStartPre=/bin/bash -c 'source /opt/ros/foxy/setup.bash'
ExecStart=/usr/bin/python3 ADAS.py --role primary
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

备机单元仅 `NANO_ROLE` / `--role` / `Description` 不同。
