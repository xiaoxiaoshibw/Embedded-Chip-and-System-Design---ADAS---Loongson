#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""SOC 控制节点 worker（主/备各跑一个进程）。

把 SOCCode 的真实控制栈跑在联合仿真里，链路与 Jetson 实机逐段对应：
  - 感知输入：UDP JSON（替代 ROS 话题回调），写 VehicleSignals + 时间戳；
  - 控制解算：100Hz 调 pipeline.run_pure_pipeline()，发送链尾部的
    clamp → lat_smooth → takeover guard 与 ADAS._control_loop_impl 逐行对齐；
  - 输出：control.serial_protocol.build_esp32_payload() 的真实帧
    经"虚拟 UART"（UDP，前缀 "P "/"B "）发给虚拟 ESP32；
  - 主备心跳：与 heartbeat.py 相同的线格式
    （HB:1 SEQ.. PSI.. DELTA.. ACC.. AEB.. CLS.. / BACKUP:1 ACTIVE:x），
    解析复用 heartbeat._parse_primary_hb_fields；备机静默/SEQ 停滞超时接管，
    接管沿（False→True）从主机最后一帧种子初始化平滑器（无感降级核心）；
  - 故障注入：UDP 控制口接收 KILL（进程退出，模拟宕机）/ HANG（循环卡死，
    心跳静默但进程仍在）。

单机双进程跑同一份代码，差异仅 --role 与端口，与"同一份代码靠 NANO_ROLE
区分主备"的实机部署方式一致。
"""

import argparse
import json
import math
import os
import socket
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import paths  # noqa: E402 — adds HIL/carla_bridge/pc/ to sys.path for shared bridge modules

from bridge_config import (  # noqa: E402
    ESP32_UART_PORT,
    FAULT_PORT_BACKUP,
    FAULT_PORT_PRIMARY,
    HB_PORT_BACKUP,
    HB_PORT_PRIMARY,
    SENSOR_PORT_BACKUP,
    SENSOR_PORT_PRIMARY,
    STATUS_PORT,
)

sys.path.insert(0, os.path.join(_HERE, 'soc_code'))


def _load_soc(role):
    """在设置 NANO_ROLE 之后导入 SOCCode 模块（config 在 import 时读环境变量）。"""
    os.environ['NANO_ROLE'] = role
    os.environ.setdefault('TELEMETRY', '0')
    import config
    import runtime
    runtime.configure_runtime(role)
    # 实验用：接管时延极限扫描时按环境变量覆盖心跳超时（仅当显式设置才生效，
    # 不影响正常运行）。读取处（HeartbeatLink）均在运行时取 cfg.HEARTBEAT_TIMEOUT_S，
    # 故 import 后改模块属性即可生效。
    _hb_override = os.environ.get('SWEEP_HB_TIMEOUT_S')
    if _hb_override:
        config.HEARTBEAT_TIMEOUT_S = float(_hb_override)
    from heartbeat import _parse_primary_hb_fields
    from lateral import LateralSmoothing
    from pipeline import run_pure_pipeline
    from replay import build_stack
    from control.serial_protocol import Esp32ControlFrame, build_esp32_payload
    return {
        'config': config,
        'parse_hb': _parse_primary_hb_fields,
        'LateralSmoothing': LateralSmoothing,
        'run_pure_pipeline': run_pure_pipeline,
        'build_stack': build_stack,
        'Esp32ControlFrame': Esp32ControlFrame,
        'build_esp32_payload': build_esp32_payload,
    }


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class HeartbeatLink:
    """主备心跳（与 SOCCode/heartbeat.py 同线格式、同判定语义）。

    与原 PeerHeartbeat 的差异仅在传输层：原版主备两机各绑同一端口号，
    单机双进程会端口冲突，这里改为本机两个端口（主 9201 / 备 9202），
    报文格式与超时/接管/回切判定逐条保持一致。
    """

    def __init__(self, role, soc):
        self.role = role
        self.is_primary = (role == 'primary')
        self.cfg = soc['config']
        self._parse = soc['parse_hb']
        self._lock = threading.Lock()
        self._running = True

        local_port = HB_PORT_PRIMARY if self.is_primary else HB_PORT_BACKUP
        peer_port = HB_PORT_BACKUP if self.is_primary else HB_PORT_PRIMARY
        self._peer = ('127.0.0.1', peer_port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('127.0.0.1', local_port))
        # 备机静默检测延迟 ≈ 此 recv 超时 + HEARTBEAT_TIMEOUT_S。原 0.05s 把备机就绪
        # 下限钉在 ~60ms，使 JETSON_TIMEOUT_MS 压到 45ms 时备机赶不上→ESP32 先全力制动。
        # 收紧到 5ms 后备机就绪 ~30ms，可支撑 45ms 干净接管。须与 heartbeat.py 同步。
        self._sock.settimeout(0.005)

        grace = float(os.environ.get('HB_GRACE', '2.0'))
        now = time.monotonic()
        self.peer_last_rx = now + grace
        self._last_seq_change_t = now + grace
        self._last_rx_seq = -1
        self._takeover = False
        self._prev_active = False
        self._takeover_edge_pending = False
        self._primary_restored_t = 0.0
        self._tx_seq = 0
        # 主机最后一帧种子
        self._seed = None
        self._seed_t = 0.0
        # 主机侧：备机存活
        self.backup_last_rx = -1e9
        self.events = []

        threading.Thread(target=self._rx_loop, daemon=True).start()
        if not self.is_primary:
            threading.Thread(target=self._backup_tx_loop, daemon=True).start()

    # ── 发送 ──
    def send_hb(self, psi, delta, acc, aeb_active, lead_cls=0):
        """主机每控制周期广播控制量心跳（同 PeerHeartbeat.send_hb）。"""
        try:
            self._tx_seq = (self._tx_seq + 1) & 0x7FFFFFFF
            msg = ('HB:1 SEQ:%d PSI:%.4f DELTA:%.4f ACC:%+.2f AEB:%d CLS:%d\n'
                   % (self._tx_seq, psi, delta, acc,
                      1 if aeb_active else 0, int(lead_cls)))
            self._sock.sendto(msg.encode('ascii'), self._peer)
        except Exception:
            pass

    def _backup_tx_loop(self):
        while self._running:
            try:
                with self._lock:
                    active = self._takeover
                msg = 'BACKUP:1 ACTIVE:%d\n' % (1 if active else 0)
                self._sock.sendto(msg.encode('ascii'), self._peer)
            except Exception:
                pass
            time.sleep(self.cfg.HB_SEND_INTERVAL_S)

    # ── 接收 + 接管判定 ──
    def _rx_loop(self):
        while self._running:
            try:
                try:
                    data, _ = self._sock.recvfrom(256)
                except socket.timeout:
                    data = None
                except OSError:
                    if not self._running:
                        return
                    data = None
                if data:
                    self._handle(data.decode('ascii', errors='ignore').strip())
            finally:
                if not self.is_primary:
                    self._check_takeover()

    def _handle(self, msg):
        now = time.monotonic()
        if self.is_primary:
            if 'BACKUP:1' in msg:
                with self._lock:
                    self.backup_last_rx = now
            return
        if not msg.startswith('HB:'):
            return
        seed = self._parse(msg)
        with self._lock:
            self.peer_last_rx = now
            if seed is not None:
                psi, delta, acc, aeb_flag, cls_val, seq = seed
                self._seed = (psi, delta, acc, aeb_flag, cls_val)
                self._seed_t = now
                if seq is not None and seq != self._last_rx_seq:
                    self._last_rx_seq = seq
                    self._last_seq_change_t = now
            if self._takeover:
                # 主机恢复 → 延迟 HB_STANDBY_HANDOFF_S 再回 standby
                if self._primary_restored_t <= 0.0:
                    self._primary_restored_t = now
                    self.events.append((now, 'primary heartbeat detected, holding active'))
                elif (now - self._primary_restored_t) >= self.cfg.HB_STANDBY_HANDOFF_S:
                    self._takeover = False
                    self._primary_restored_t = 0.0
                    self.events.append((now, 'primary stable, backup standby'))

    def _check_takeover(self):
        with self._lock:
            if self._takeover:
                return
            now = time.monotonic()
            silence = now - self.peer_last_rx
            seq_stale = now - self._last_seq_change_t
            if silence > self.cfg.HEARTBEAT_TIMEOUT_S:
                self._takeover = True
                self._primary_restored_t = 0.0
                self.events.append(
                    (now, 'primary silence %.0fms, backup TAKEOVER' % (silence * 1e3)))
            elif (seq_stale > self.cfg.HEARTBEAT_TIMEOUT_S
                  and self._last_rx_seq >= 0):
                self._takeover = True
                self._primary_restored_t = 0.0
                self.events.append(
                    (now, 'primary SEQ stalled %.0fms, backup TAKEOVER' % (seq_stale * 1e3)))

    def is_active(self):
        if self.is_primary:
            return True
        with self._lock:
            active = self._takeover
            if active and not self._prev_active:
                self._takeover_edge_pending = True
            self._prev_active = active
            return active

    def consume_takeover_seed(self):
        """同 PeerHeartbeat.consume_takeover_seed（含时效检查）。"""
        with self._lock:
            if not self._takeover_edge_pending:
                return None
            self._takeover_edge_pending = False
            if self._seed is None or self._seed_t <= 0.0:
                return None
            if (time.monotonic() - self._seed_t) > self.cfg.HEARTBEAT_TIMEOUT_S * 2.0:
                return None
            return self._seed

    def drain_events(self):
        with self._lock:
            ev, self.events = self.events, []
        return ev

    def close(self):
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass


class SocWorker:
    def __init__(self, role):
        self.role = role
        self.is_primary = (role == 'primary')
        self.soc = _load_soc(role)
        cfg = self.soc['config']
        self.cfg = cfg
        self.dt = 1.0 / float(cfg.LOOP_HZ)

        self.signals, self.memory, self.managers = self.soc['build_stack']()
        self.lat_smooth = self.soc['LateralSmoothing'](dt=self.dt)
        self.hb = HeartbeatLink(role, self.soc)

        # 接管守护状态（对齐 AdasNode）
        self._takeover_guard_until = -1e9
        self._takeover_prev_lon = 0.0
        self._takeover_aeb_seed = False
        self._takeover_seed_cls = 0
        self._last_takeover_init_t = -1e9

        self._hang = False
        self._running = True
        self._last_status_t = 0.0
        self._last_aeb = False

        # 感知接收
        sport = SENSOR_PORT_PRIMARY if self.is_primary else SENSOR_PORT_BACKUP
        self._sensor_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sensor_sock.bind(('127.0.0.1', sport))
        self._sensor_sock.settimeout(0.05)
        threading.Thread(target=self._sensor_loop, daemon=True).start()

        # 故障注入
        fport = FAULT_PORT_PRIMARY if self.is_primary else FAULT_PORT_BACKUP
        self._fault_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._fault_sock.bind(('127.0.0.1', fport))
        self._fault_sock.settimeout(0.2)
        threading.Thread(target=self._fault_loop, daemon=True).start()

        # 输出（虚拟 UART + 状态）
        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._uart_prefix = b'P ' if self.is_primary else b'B '
        self._esp32_addr = ('127.0.0.1', ESP32_UART_PORT)
        self._status_addr = ('127.0.0.1', STATUS_PORT)

    # ── 感知回调（对应 ROS 订阅回调线程）──
    def _sensor_loop(self):
        while self._running:
            try:
                data, _ = self._sensor_sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                if not self._running:
                    return
                continue
            try:
                frame = json.loads(data.decode('utf-8'))
            except Exception:
                continue
            now = time.monotonic()
            s = self.signals
            with s._lock:
                s.ego_x = float(frame['ego_x'])
                s.ego_y = float(frame['ego_y'])
                s.ego_yaw = float(frame['ego_yaw'])
                s.ego_v = float(frame['ego_v'])
                s.ego_received = True
                s.ego_psi_received = True
                s.ego_last_rx = now
                s.road_psi = float(frame['road_psi'])
                s.road_received = True
                s.road_last_rx = now
                s.lane_offset = float(frame['lane_offset'])
                s.lane_offset_received = True
                s.lane_offset_last_rx = now
                if frame.get('lead_present'):
                    s.lead_x = float(frame['lead_x'])
                    s.lead_y = float(frame['lead_y'])
                    s.lead_yaw = float(frame['lead_yaw'])
                    s.lead_v = float(frame['lead_v'])
                    s.lead_cls = int(frame.get('lead_cls', 1))
                    s.lead_received = True
                    s.lead_last_rx_time = now
                    s.lead_v_last_rx_time = now
                    s.lead_cls_last_rx_time = now

    def _fault_loop(self):
        while self._running:
            try:
                data, _ = self._fault_sock.recvfrom(64)
            except socket.timeout:
                continue
            except OSError:
                return
            cmd = data.decode('ascii', errors='ignore').strip().upper()
            if cmd == 'KILL':
                print('[%s] FAULT INJECT: KILL — simulating crash' % self.role,
                      flush=True)
                os._exit(1)
            elif cmd == 'HANG':
                print('[%s] FAULT INJECT: HANG — control loop frozen' % self.role,
                      flush=True)
                self._hang = True

    # ── 接管沿种子（对齐 AdasNode._handle_takeover_edge）──
    def _handle_takeover_edge(self, now):
        seed = self.hb.consume_takeover_seed()
        if seed is None:
            return
        cfg = self.cfg
        psi_seed, delta_seed, lon_seed, aeb_seed, cls_seed = seed
        if not math.isfinite(delta_seed):
            delta_seed = self.memory.last_delta
        if not math.isfinite(lon_seed):
            lon_seed = self._takeover_prev_lon
        delta_seed = _clamp(delta_seed, -cfg.MAX_DELTA, cfg.MAX_DELTA)
        lon_seed = _clamp(lon_seed, -cfg.LON_CMD_MAX_DRIVE_ACCEL,
                          cfg.LON_CMD_MAX_BRAKE_DECEL)

        in_cooldown = (now - self._last_takeover_init_t) < cfg.TAKEOVER_COOLDOWN_S
        self._takeover_guard_until = now + cfg.TAKEOVER_GUARD_DURATION_S
        self._takeover_aeb_seed = bool(aeb_seed)
        self._takeover_seed_cls = int(cls_seed)
        if in_cooldown:
            self._takeover_prev_lon = self.managers.lon_smooth.prev
            return
        self.managers.lon_smooth.reset(lon_seed)
        self.lat_smooth.reset(delta_seed)
        self.memory.last_delta = delta_seed
        self.memory.psi_i_term = 0.0
        self.memory.psi_prev_err = 0.0
        self._takeover_prev_lon = lon_seed
        self._last_takeover_init_t = now
        print('[%s] TAKEOVER seed psi=%.4f delta=%.4f lon=%+.2f aeb=%d'
              % (self.role, psi_seed, delta_seed, lon_seed, int(aeb_seed)),
              flush=True)

    def _takeover_lon_rate(self):
        cfg = self.cfg
        if self._takeover_aeb_seed:
            return cfg.TAKEOVER_LON_RATE_AEB_RELEASE
        if self._takeover_seed_cls in (cfg.ACTOR_CLASS_PEDESTRIAN,
                                       cfg.ACTOR_CLASS_OBSTACLE):
            return cfg.TAKEOVER_LON_RATE_VULNERABLE
        return cfg.TAKEOVER_LON_RATE

    # ── 单个控制周期（对齐 _control_loop_impl 的发送链）──
    def _tick(self, now):
        cfg = self.cfg
        # 热待机优化：备机 standby 时也解算并向 ESP32 持续发控制帧。MCU 仲裁层在
        # 主控帧新鲜时只用主、忽略备帧（见 virtual_esp32._arbitrate）；主控一旦超时，
        # 备帧已在 MCU 侧就绪 → 瞬间干净切换，消除“主超时但备机还没开始发帧”竞态
        # 窗口里偶发的 ~10 m/s² 全力制动冲击。备机活着时备帧不被采用，故严格更安全。
        # 接管沿种子仍仅在 standby→active 翻转拍消费（对齐 ADAS._control_loop_impl）。
        # 由 config.BACKUP_HOT_STANDBY 控制（默认 True=热待机；False=冷待机旧行为，
        # standby 不解算不发帧，用于冷/热对照实验），与真机 ADAS.py 同一开关。
        active = self.hb.is_active()
        if not active and not getattr(cfg, 'BACKUP_HOT_STANDBY', True):
            return
        if active:
            self._handle_takeover_edge(now)

        signals_snap = self.signals.snapshot()
        if not (signals_snap.ego_received and signals_snap.road_received):
            return  # 感知尚未就绪

        takeover_rate = (self._takeover_lon_rate()
                         if now < self._takeover_guard_until else None)
        res = self.soc['run_pure_pipeline'](
            now, signals_snap, self.memory, self.managers, takeover_rate)

        lateral_ctx = res.lateral_ctx
        lon_ctx = res.lon_ctx
        lon_cmd = res.lon_cmd

        # NaN 防护：任一关键量非有限则本帧不发（ESP32 侧看门狗兜底）
        if not all(math.isfinite(v) for v in (
                lateral_ctx.delta, lateral_ctx.upd_psi, lon_cmd,
                lateral_ctx.cur_off)):
            self.memory.cycle_count += 1
            return

        ttc_tx = lon_ctx.ttc if math.isfinite(lon_ctx.ttc) else 999.99
        dist_tx = _clamp(lon_ctx.dist, 0.0, 999.99)
        psi_tx = _clamp(lateral_ctx.upd_psi, -9.9999, 9.9999)
        delta_tx = _clamp(lateral_ctx.delta, -9.9999, 9.9999)
        speed_tx = _clamp(signals_snap.ego_v, -99.99, 99.99)
        lon_tx = _clamp(lon_cmd, -cfg.LON_CMD_MAX_DRIVE_ACCEL,
                        cfg.LON_CMD_MAX_BRAKE_DECEL)

        takeover_lat_rate = (cfg.TAKEOVER_DELTA_RATE
                             if now < self._takeover_guard_until else None)
        delta_tx = self.lat_smooth.update(delta_tx,
                                          max_rate_override=takeover_lat_rate)
        self.memory.last_delta = delta_tx

        # 接管保护窗内 lon 变化率保险网（对齐 _apply_takeover_guard）
        if now < self._takeover_guard_until:
            max_step = self._takeover_lon_rate() * self.memory.dt
            lon_tx = _clamp(lon_tx, self._takeover_prev_lon - max_step,
                            self._takeover_prev_lon + max_step)
            self._takeover_prev_lon = lon_tx

        payload = self.soc['build_esp32_payload'](self.soc['Esp32ControlFrame'](
            ttc=ttc_tx, dist=dist_tx, psi=psi_tx, delta=delta_tx,
            speed=speed_tx, lon=lon_tx, offset=lateral_ctx.cur_off,
            lead_v_proj=lon_ctx.lead_v_proj,
            min_safe_dist=lon_ctx.min_safe_dist,
            lane_warn_margin=self.memory.lane_warn_margin,
            lane_hard_margin=self.memory.lane_hard_margin,
            filtered_curv=self.memory.filtered_curv,
        ))
        try:
            self._tx_sock.sendto(self._uart_prefix + payload, self._esp32_addr)
        except Exception:
            pass

        if self.is_primary:
            self.hb.send_hb(psi_tx, delta_tx, lon_tx,
                            aeb_active=bool(lon_ctx.aeb_active),
                            lead_cls=int(signals_snap.lead_cls))
        self._last_aeb = bool(lon_ctx.aeb_active)
        self.memory.last_acc_has_lead = res.lead_ctx.acc_has_lead
        self.memory.cycle_count += 1

    def _send_status(self, now):
        msg = {
            'role': self.role,
            'active': bool(self.hb.is_active()),
            'aeb': self._last_aeb,
            'cycle': self.memory.cycle_count,
            'guard': now < self._takeover_guard_until,
            'events': [e for _, e in self.hb.drain_events()],
        }
        try:
            self._tx_sock.sendto(json.dumps(msg).encode('utf-8'),
                                 self._status_addr)
        except Exception:
            pass

    def run(self):
        print('[%s] SOC worker up: LOOP_HZ=%d pipeline=SOCCode'
              % (self.role, self.cfg.LOOP_HZ), flush=True)
        next_t = time.monotonic()
        while self._running:
            while self._hang:
                time.sleep(0.5)   # 模拟控制循环卡死：心跳/控制帧全部停发
            now = time.monotonic()
            try:
                self._tick(now)
            except Exception as e:
                print('[%s] tick error: %r' % (self.role, e), flush=True)
            if now - self._last_status_t >= 0.1:
                self._send_status(now)
                self._last_status_t = now
            next_t += self.dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()   # 落后则重新对齐，不补帧


def main():
    parser = argparse.ArgumentParser(description='SOC 控制节点 worker')
    parser.add_argument('--role', choices=['primary', 'backup'], required=True)
    args = parser.parse_args()
    SocWorker(args.role).run()


if __name__ == '__main__':
    main()
