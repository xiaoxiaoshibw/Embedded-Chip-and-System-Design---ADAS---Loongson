#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""龙芯派 2K1000 物理按钮守护：J1 排针接 3 脚按钮模块(VCC/GND/OUT)，按一下触发 on_press()。

接线（按官方 J1 60 针原理图，管脚号=物理排针号，非 GPIO 编号）：
  VCC → J1 pin 1  (P3V3，3.3V)   ← 必须接 3.3V，禁止接 P5V(pin2/4)，GPIO 只有 3.3V 容限
  GND → J1 pin 6  (GND)
  OUT → J1 pin 7  (LS2K_GPIO07 → sysfs /sys/class/gpio/gpio7)

依据：官方 J1 60pin 原理图，pin1=P3V3、pin6=GND、pin7=LS2K_GPIO07。
GPIO 编号与信号名数字一致（LS2K_GPIOxx ↔ sysfs gpioxx），gpiochip0 base=0 ngpio=64 已核实。

本板 /sys/class/gpio/export 仅 root 可写（loongson 用户不在任何 gpio 组），
本脚本须以 root 运行（sudo python3 loongson_button.py）。

行为：
  - 无法确定该板 sysfs-gpio 是否有内部上下拉（未见官方说明），按 Nano 同款经验，
    启动时自动采样空闲电平作基线，把“相对稳定的相反电平”识别为“按下”，
    模块无论是按下拉高还是拉低都能用（也可用 BUTTON_ACTIVE 环境变量强制 high/low）。
  - 去抖 + 释放后才重新触发 + 冷却，避免抖动/长按反复触发。
  - 触发动作在 on_press() 中——当前只做状态快照记录；板上还没有常驻 systemd
    单元（SOCCode 目前只是脚本化跑 run_scenario.py/bench_loop.py），
    等部署成常驻服务后把重启逻辑接到 on_press() 里（参照
    HIL/carla_bridge/nano/adas_button.py 的 resolve_unit()+systemctl restart 模式）。

环境变量：
  BUTTON_PIN      默认 7（GPIO 编号，非 J1 物理脚号）
  BUTTON_ACTIVE   auto(默认)/high/low —— 按下时的电平
  BUTTON_DEBOUNCE 默认 0.06s
  BUTTON_COOLDOWN 默认 4s
"""

import os
import sys
import time

PIN = int(os.environ.get('BUTTON_PIN', '7'))
ACTIVE = os.environ.get('BUTTON_ACTIVE', 'auto').lower()
DEBOUNCE_S = float(os.environ.get('BUTTON_DEBOUNCE', '0.06'))
COOLDOWN_S = float(os.environ.get('BUTTON_COOLDOWN', '4'))
POLL_S = 0.02
STATUS_LOG = '/tmp/loongson_button.log'

GPIO_DIR = '/sys/class/gpio'
PIN_DIR = os.path.join(GPIO_DIR, 'gpio%d' % PIN)


def log(msg):
    line = '%s | %s' % (time.strftime('%Y-%m-%d %H:%M:%S'), msg)
    print(line, flush=True)
    try:
        with open(STATUS_LOG, 'a') as f:
            f.write(line + '\n')
    except Exception:
        pass


def gpio_export():
    if os.path.isdir(PIN_DIR):
        return
    with open(os.path.join(GPIO_DIR, 'export'), 'w') as f:
        f.write(str(PIN))
    # 内核建 sysfs 节点是异步的，等一下再用
    for _ in range(50):
        if os.path.isdir(PIN_DIR):
            return
        time.sleep(0.02)
    raise RuntimeError('导出 gpio%d 超时，检查是否已被占用' % PIN)


def gpio_unexport():
    try:
        with open(os.path.join(GPIO_DIR, 'unexport'), 'w') as f:
            f.write(str(PIN))
    except Exception:
        pass


def gpio_set_direction_in():
    with open(os.path.join(PIN_DIR, 'direction'), 'w') as f:
        f.write('in')


def gpio_read():
    with open(os.path.join(PIN_DIR, 'value')) as f:
        return int(f.read().strip())


def on_press():
    """按下时触发的动作——先只做状态快照，部署成 systemd 服务后在此接 restart 逻辑。"""
    log('=== 按钮触发 ===')
    try:
        import glob
        files = sorted(glob.glob('/tmp/*telemetry*.csv') + glob.glob('/home/loongson/adas/SOCCode/*telemetry*.csv'),
                       key=os.path.getmtime, reverse=True)
        if files:
            f = files[0]
            log('最新遥测=%s  mtime=%s' %
                (os.path.basename(f), time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(f)))))
        else:
            log('未发现遥测文件（SOCCode 当前未以常驻服务运行）')
    except Exception as e:
        log('快照异常: %r' % e)


def detect_idle_level():
    samples = []
    t0 = time.time()
    while time.time() - t0 < 0.4:
        samples.append(gpio_read())
        time.sleep(0.01)
    return 1 if sum(samples) * 2 > len(samples) else 0


def main():
    if os.geteuid() != 0:
        print('须以 root 运行：sudo python3 loongson_button.py', file=sys.stderr)
        return 1

    gpio_export()
    gpio_set_direction_in()

    idle = detect_idle_level()
    if ACTIVE == 'high':
        pressed_level = 1
    elif ACTIVE == 'low':
        pressed_level = 0
    else:
        pressed_level = 1 - idle
    log('按钮守护启动 gpio%d 空闲电平=%d 按下电平=%d (ACTIVE=%s)' % (PIN, idle, pressed_level, ACTIVE))

    last_action = 0.0
    armed = (idle != pressed_level)
    stable_level = idle
    stable_since = time.time()
    last_logged = idle

    try:
        while True:
            lvl = gpio_read()
            now = time.time()
            if lvl != stable_level:
                stable_level = lvl
                stable_since = now
            stable = (now - stable_since) >= DEBOUNCE_S

            if stable and stable_level != last_logged:
                log('gpio%d 电平变化 -> %d%s' %
                    (PIN, stable_level, '（=按下）' if stable_level == pressed_level else '（=空闲）'))
                last_logged = stable_level

            if stable and stable_level == pressed_level and armed and (now - last_action) >= COOLDOWN_S:
                last_action = now
                armed = False
                try:
                    on_press()
                except Exception as e:
                    log('动作异常: %r' % e)
                stable_level = gpio_read()
                stable_since = time.time()
                last_logged = stable_level

            if stable and stable_level != pressed_level:
                armed = True
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        pass
    finally:
        gpio_unexport()
    return 0


if __name__ == '__main__':
    sys.exit(main())
