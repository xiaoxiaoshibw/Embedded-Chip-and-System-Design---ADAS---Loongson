#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ADAS 物理按钮守护：pin40 接 3 脚按钮模块(VCC/GND/OUT)，按一下→刷新自身状态 + 重启 ADAS 服务。

接线（Jetson Nano 40-pin 物理脚号，BOARD 编号）：
  VCC → pin 1  (3.3V)        ← 必须接 3.3V，禁止接 5V（GPIO 仅 3.3V 容限，5V 会烧脚）
  GND → pin 39 (GND，紧邻 40)
  OUT → pin 40 (信号)

行为：
  - 这块载板的 Jetson.GPIO 忽略内部上下拉，故空闲电平由模块自身驱动。
  - 启动时自动采样空闲电平作基线，把“相反的稳定电平”识别为“按下”，
    因此模块无论是按下拉高还是拉低都能用（也可用 BUTTON_ACTIVE 环境变量强制）。
  - 去抖 + 释放后才重新触发 + 冷却，避免抖动/长按反复重启。
  - 触发时先把当前状态快照写入 /tmp/adas_button.log（“刷新自身状态”），
    再 systemctl restart adas-hil-<role>.service（重启控制栈，连带刷新 ml_inferd/lockstepd）。

环境变量：
  BUTTON_PIN      默认 40（BOARD 物理脚号）
  BUTTON_ACTIVE   auto(默认)/high/low —— 按下时的电平
  BUTTON_DEBOUNCE 默认 0.06s（稳定时间）
  BUTTON_COOLDOWN 默认 8s（两次触发最小间隔）
  ADAS_UNIT       覆盖要重启的 systemd 单元名（默认按角色自动推断）
"""

import os
import subprocess
import sys
import time

import Jetson.GPIO as GPIO

PIN = int(os.environ.get('BUTTON_PIN', '40'))
ACTIVE = os.environ.get('BUTTON_ACTIVE', 'auto').lower()
DEBOUNCE_S = float(os.environ.get('BUTTON_DEBOUNCE', '0.06'))
COOLDOWN_S = float(os.environ.get('BUTTON_COOLDOWN', '4'))
POLL_S = 0.02
ENV_FILE = '/etc/adas/adas.env'
STATUS_LOG = '/tmp/adas_button.log'

# 板载绿灯反馈（/sys/class/leds/pwr）：按下闪烁确认，闪完恢复原 trigger。
LED_FEEDBACK = os.environ.get('BUTTON_LED', '1') != '0'
LED_DIR = os.environ.get('BUTTON_LED_DIR', '/sys/class/leds/pwr')
_led_orig_trigger = None

# 有源蜂鸣器模块（VCC/GND/IO）：IO 接 BUZZER_PIN，按下短鸣一声。
BUZZER_ENABLED = os.environ.get('BUTTON_BUZZER', '1') != '0'
BUZZER_PIN = int(os.environ.get('BUZZER_PIN', '33'))   # BOARD 物理脚号
# 鸣叫时的有效电平：多数有源蜂鸣器模块 IO 高=响（active-high）；个别 active-low。
BUZZER_ACTIVE_HIGH = os.environ.get('BUZZER_ACTIVE', 'high').lower() != 'low'
BUZZER_BEEP_S = float(os.environ.get('BUZZER_BEEP', '0.15'))
_buzzer_ready = False


def setup_buzzer():
    """把 BUZZER_PIN 设为输出并置静音电平。"""
    global _buzzer_ready
    if not BUZZER_ENABLED:
        return
    try:
        off = GPIO.LOW if BUZZER_ACTIVE_HIGH else GPIO.HIGH
        GPIO.setup(BUZZER_PIN, GPIO.OUT, initial=off)
        _buzzer_ready = True
    except Exception as e:
        log('蜂鸣器初始化失败 pin=%d: %r' % (BUZZER_PIN, e))


def beep(times=1, on_s=None, off_s=0.08):
    """蜂鸣 times 声（默认每声 BUZZER_BEEP_S）。"""
    if not _buzzer_ready:
        return
    on_s = BUZZER_BEEP_S if on_s is None else on_s
    on = GPIO.HIGH if BUZZER_ACTIVE_HIGH else GPIO.LOW
    off = GPIO.LOW if BUZZER_ACTIVE_HIGH else GPIO.HIGH
    try:
        for i in range(times):
            GPIO.output(BUZZER_PIN, on)
            time.sleep(on_s)
            GPIO.output(BUZZER_PIN, off)
            if i != times - 1:
                time.sleep(off_s)
    except Exception as e:
        log('蜂鸣异常: %r' % e)
        try:
            GPIO.output(BUZZER_PIN, off)
        except Exception:
            pass


def _led_write(node, val):
    try:
        with open(os.path.join(LED_DIR, node), 'w') as f:
            f.write(str(val))
        return True
    except Exception:
        return False


def _led_save_trigger():
    """记下当前 trigger（中括号里的那个），供闪烁后恢复。"""
    global _led_orig_trigger
    try:
        with open(os.path.join(LED_DIR, 'trigger')) as f:
            for tok in f.read().split():
                if tok.startswith('[') and tok.endswith(']'):
                    _led_orig_trigger = tok[1:-1]
                    return
    except Exception:
        _led_orig_trigger = None


def led_blink(times=1, on_s=0.12, off_s=0.12):
    """绿灯闪 times 下；接管前先关 trigger，闪完恢复原 trigger。"""
    if not LED_FEEDBACK:
        return
    if _led_orig_trigger is None:
        _led_save_trigger()   # 惰性保存：任何入口（含独立 do_action）都能正确恢复
    if not _led_write('trigger', 'none'):
        return  # 无 LED 或无权限 → 静默跳过
    try:
        for _ in range(times):
            _led_write('brightness', 0)
            time.sleep(off_s)
            _led_write('brightness', 255)
            time.sleep(on_s)
    finally:
        # 恢复原状：写回原 trigger（写不回就保持常亮）
        if _led_orig_trigger:
            _led_write('trigger', _led_orig_trigger)
        else:
            _led_write('brightness', 255)


def log(msg):
    line = '%s | %s' % (time.strftime('%Y-%m-%d %H:%M:%S'), msg)
    print(line, flush=True)
    try:
        with open(STATUS_LOG, 'a') as f:
            f.write(line + '\n')
    except Exception:
        pass


def _read_env(key, default=''):
    try:
        with open(ENV_FILE) as f:
            for ln in f:
                ln = ln.strip()
                if ln.startswith('#') or '=' not in ln:
                    continue
                if ln.startswith('export '):
                    ln = ln[len('export '):]
                k, _, v = ln.partition('=')
                if k.strip() == key:
                    return v.strip().strip('"').strip("'")
    except Exception:
        pass
    return default


def resolve_unit():
    """要重启的 ADAS systemd 单元：优先 ADAS_UNIT，其次按角色，最后探测 active 单元。"""
    u = os.environ.get('ADAS_UNIT', '').strip()
    if u:
        return u
    role = _read_env('NANO_ROLE', '').lower()
    if role in ('primary', 'backup'):
        cand = 'adas-hil-%s.service' % role
        if _unit_exists(cand):
            return cand
    # 回退：找当前 active 的 adas-hil-* 单元
    try:
        out = subprocess.check_output(
            ['systemctl', 'list-units', '--type=service', '--state=active',
             '--no-legend', 'adas-hil-*.service'],
            stderr=subprocess.DEVNULL).decode('utf-8', 'replace')
        for ln in out.splitlines():
            name = ln.split()[0] if ln.split() else ''
            if name.startswith('adas-hil-'):
                return name
    except Exception:
        pass
    return 'adas-hil-%s.service' % (role or 'primary')


def _unit_exists(name):
    try:
        rc = subprocess.call(['systemctl', 'cat', name],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return rc == 0
    except Exception:
        return False


def snapshot_status(unit):
    """刷新自身状态：把当前角色/服务态/最新遥测快照写日志。"""
    role = _read_env('NANO_ROLE', '?')
    log('=== 按钮触发：刷新自身状态 ===')
    log('角色=%s  单元=%s' % (role, unit))
    try:
        st = subprocess.check_output(['systemctl', 'is-active', unit],
                                     stderr=subprocess.DEVNULL).decode().strip()
    except subprocess.CalledProcessError as e:
        st = e.output.decode().strip() if e.output else 'unknown'
    log('服务当前状态=%s' % st)
    # 最新遥测文件 + mtime
    try:
        import glob
        files = sorted(glob.glob('/tmp/adas_*_telemetry_*.csv'),
                       key=os.path.getmtime, reverse=True)
        if files:
            f = files[0]
            log('最新遥测=%s  mtime=%s' %
                (os.path.basename(f),
                 time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(f)))))
    except Exception:
        pass


def do_action():
    unit = resolve_unit()
    # 先发重启：--no-block 让 systemctl 立即返回（实测 ~40ms），不阻塞轮询循环。
    # 旧实现用阻塞 restart 会冻结循环数秒，期间漏掉“松开”沿 → 之后永远 disarmed
    # （表现为“按两次就不行 + 有延迟”）。
    log('>>> systemctl restart --no-block %s' % unit)
    rc = subprocess.call(['systemctl', 'restart', '--no-block', unit])
    log('<<< restart 返回码=%d' % rc)
    # 再给反馈（短）
    beep(times=1)
    led_blink(times=2, on_s=0.08, off_s=0.08)
    snapshot_status(unit)


def detect_idle_level():
    """采样前 0.4s 的多数电平作为空闲基线。"""
    samples = []
    t0 = time.time()
    while time.time() - t0 < 0.4:
        samples.append(GPIO.input(PIN))
        time.sleep(0.01)
    return 1 if sum(samples) * 2 > len(samples) else 0


def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PIN, GPIO.IN)
    setup_buzzer()

    _led_save_trigger()

    idle = detect_idle_level()
    if ACTIVE == 'high':
        pressed_level = 1
    elif ACTIVE == 'low':
        pressed_level = 0
    else:  # auto
        pressed_level = 1 - idle
    log('按钮守护启动 pin=%d 空闲电平=%d 按下电平=%d (ACTIVE=%s) LED=%s 蜂鸣器=%s(pin%d)'
        % (PIN, idle, pressed_level, ACTIVE, '开' if LED_FEEDBACK else '关',
           '开' if _buzzer_ready else '关', BUZZER_PIN))
    led_blink(times=1, on_s=0.15, off_s=0.15)   # 就绪指示：闪 1 下
    beep(times=1, on_s=0.05)                     # 就绪指示：短鸣一声（确认蜂鸣器接好）

    last_action = 0.0
    # 启动时若线已处于“按下”电平（例如未接模块时 pin 悬空读到 pressed_level），
    # 不要立即误触发——要求线先回到空闲电平才武装，避免开机/未接线时自发重启。
    armed = (idle != pressed_level)
    stable_level = idle
    stable_since = time.time()
    last_logged = idle      # 诊断：稳定电平变化时记一行

    try:
        while True:
            lvl = GPIO.input(PIN)
            now = time.time()
            if lvl != stable_level:
                stable_level = lvl
                stable_since = now
            stable = (now - stable_since) >= DEBOUNCE_S

            if stable and stable_level != last_logged:
                log('pin%d 电平变化 -> %d%s' %
                    (PIN, stable_level,
                     '（=按下）' if stable_level == pressed_level else '（=空闲）'))
                last_logged = stable_level

            if stable and stable_level == pressed_level and armed \
                    and (now - last_action) >= COOLDOWN_S:
                last_action = now
                armed = False
                try:
                    do_action()
                except Exception as e:
                    log('动作异常: %r' % e)
                # do_action 期间循环短暂暂停，重新同步电平跟踪，避免漏掉释放沿
                stable_level = GPIO.input(PIN)
                stable_since = time.time()
                last_logged = stable_level
            # 稳定回到空闲电平 → 重新武装
            if stable and stable_level != pressed_level:
                armed = True
            time.sleep(POLL_S)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    sys.exit(main())
