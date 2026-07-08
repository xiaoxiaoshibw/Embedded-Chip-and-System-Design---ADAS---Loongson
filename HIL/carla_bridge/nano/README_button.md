# ADAS 物理按钮（pin40 一键刷新+重启）

每台 Jetson Nano 接一个 3 脚按钮模块（VCC / GND / OUT），按一下 →
**刷新自身状态快照 + `systemctl restart adas-hil-<role>.service`**（只重启 ADAS 控制栈，
连带刷新 ml_inferd / lockstepd，不重启整机）。

## 接线（40-pin 物理脚号 / BOARD 编号）

| 模块脚 | 接到 Nano | 物理脚号 |
|---|---|---|
| VCC | **3.3V** | pin 1（或 pin 17） |
| GND | GND | pin 39（紧邻 40，最顺手） |
| OUT | 信号 | **pin 40** |

> ⚠️ VCC 必须接 **3.3V，禁止接 5V**。Nano GPIO 仅 3.3V 容限，5V 进 pin40 会烧脚。
> 本载板的 Jetson.GPIO **忽略内部上下拉**，故必须用“有源”3 脚模块（自带上下拉、会驱动 OUT），
> 不能用纯两脚被动按钮悬空接。

## LED 反馈（板载绿灯 pwr）

按下时让板载电源绿灯 `/sys/class/leds/pwr` 闪烁确认：
- **守护启动**：闪 1 下（已就绪 / 已采基线）。
- **按一下**：快闪 3 下，随后执行重启。
- 闪烁前临时把 trigger 设为 `none`，闪完**恢复原 trigger**（出厂 `system-throttle`），不改变绿灯常态。
- 关掉反馈：`BUTTON_LED=0`；换灯：`BUTTON_LED_DIR=/sys/class/leds/<名>`。
- 本板可用灯：`pwr`（绿色电源灯，推荐）、`mmc0::`（SD 活动灯，默认随读写闪，不建议占用）。

## 蜂鸣器反馈（有源蜂鸣器模块 VCC/GND/IO）

按一下 → **短鸣一声**（守护启动时也短鸣一声确认接好）。

接线（IO 用空闲输出脚 pin 33）：

| 模块脚 | 接到 Nano | 物理脚号 |
|---|---|---|
| IO | 信号 | **pin 33**（= gpio13） |
| GND | GND | pin 34（紧邻 33） |
| VCC | 5V（响）或 3.3V（轻） | pin 2 / pin 4（5V），或 pin 17（3.3V） |

- 必须用**有源**蜂鸣器（通电即响，IO 控制开关）；无源蜂鸣器需 PWM，本脚本不驱动。
- 默认 IO 高电平=响（active-high）。若按下不响、或一直响，改极性：`BUZZER_ACTIVE=low`。
- 关蜂鸣：`BUTTON_BUZZER=0`；换脚：`BUZZER_PIN=<BOARD脚号>`；单声时长：`BUZZER_BEEP`（默认 0.15s）。

## 工作原理

- 启动时自动采样空闲电平作基线，把“相反的稳定电平”识别为按下
  （模块按下拉高或拉低都能用）。也可用 `BUTTON_ACTIVE=high|low` 强制。
- 去抖 0.06s + 释放后才重新触发 + 冷却 8s，避免抖动/长按反复重启。
- 触发动作 = `adas_button.do_action()`：先把角色/服务态/最新遥测写 `/tmp/adas_button.log`，
  再重启 `adas-hil-<role>.service`（角色读 `/etc/adas/adas.env` 的 `NANO_ROLE`，
  或自动探测 active 的 `adas-hil-*` 单元；可用 `ADAS_UNIT` 覆盖）。

## 安装（两台都装，已部署）

```bash
# 1) 拷脚本到 Nano
scp adas_button.py jetson@<nano>:/home/jetson/adas/adas_button.py
scp adas-button.service jetson@<nano>:/tmp/

# 2) 装 systemd 单元并开机自启
sudo cp /tmp/adas-button.service /etc/systemd/system/adas-button.service
sudo systemctl daemon-reload
sudo systemctl enable --now adas-button.service
systemctl is-active adas-button.service        # → active
tail -f /tmp/adas_button.log                    # 看启动基线 / 按下事件
```

## 自检 / 排错

```bash
# 看守护检测到的空闲/按下电平
tail -5 /tmp/adas_button.log
# 不按按钮、直接验证“按一下”的完整动作（会真的重启 ADAS 服务）
sudo python3 -c "import sys; sys.path.insert(0,'/home/jetson/adas'); import adas_button as b; b.do_action()"
# 确认要重启的单元解析正确
python3 -c "import sys; sys.path.insert(0,'/home/jetson/adas'); import adas_button as b; print(b.resolve_unit())"
```

环境变量（写进 `adas-button.service` 的 `Environment=` 可覆盖）：
`BUTTON_PIN`（默认 40）、`BUTTON_ACTIVE`（auto/high/low）、
`BUTTON_DEBOUNCE`（0.06s）、`BUTTON_COOLDOWN`（8s）、`ADAS_UNIT`（覆盖单元名）。
