#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""主备心跳与接管逻辑。

通过 UDP 实现主备 Nano 之间的心跳检测：
  - 主机周期发送 HB:1（含控制量 + AEB flag），接收备机存活标志。
  - 备机监听主机心跳；连续超时后自动接管为 active 状态。
  - 主机恢复后备机自动退回 standby。
  - 主机侧 watchdog：备机心跳超时则视为 failover 不可用，外部话题可见。

心跳格式（ASCII，换行结束）：
  主机 -> 备机: "HB:1 SEQ:N STATE:ACTIVE_CONTROL PSI:xx DELTA:xx ACC:xx AEB:0/1\\n"
  备机 -> 主机: "BACKUP:1 ACTIVE:0/1\\n"

AEB 字段强制要求（同版本部署）：缺失则备机拒绝整帧种子（peer_last_rx
仍照常更新，仅种子被丢弃），回落到零初始化语义，避免旧主机把过期/
脏种子喂给新备机。

SEQ 单调递增；备机据此检测"主机仍在发但序号停滞"（消息源被卡在缓冲区）
这类故障，并把序号停滞和真正的 socket 静默区分开。
"""

import logging
import math
import socket
import threading
import time

from config import (
    HB_BACKUP_TIMEOUT_S,
    HB_SEND_INTERVAL_S,
    HB_STANDBY_HANDOFF_S,
    HEARTBEAT_TIMEOUT_S,
    LON_CMD_MAX_BRAKE_DECEL,
    LON_CMD_MAX_DRIVE_ACCEL,
    MAX_DELTA,
)
import runtime


# 心跳种子物理范围 sanity（B5）：超过这些就视为脏数据，回落零初始化。
_HB_SANITY_PSI = 2.0 * math.pi               # |psi| 上界
_HB_SANITY_DELTA = 1.2 * MAX_DELTA           # 略放宽到 1.2 倍执行器上限
_HB_SANITY_ACC_LOW = -(LON_CMD_MAX_DRIVE_ACCEL + 1.0)
_HB_SANITY_ACC_HIGH = LON_CMD_MAX_BRAKE_DECEL + 1.0

HB_STATE_BOOTING = 'BOOTING'
HB_STATE_NO_INPUT_IDLE = 'NO_INPUT_IDLE'
HB_STATE_READY_STANDBY = 'READY_STANDBY'
HB_STATE_ACTIVE_CONTROL = 'ACTIVE_CONTROL'
HB_STATE_FAULT_TAKEOVER = 'FAULT_TAKEOVER'

_HB_VALID_STATES = frozenset({
    HB_STATE_BOOTING,
    HB_STATE_NO_INPUT_IDLE,
    HB_STATE_READY_STANDBY,
    HB_STATE_ACTIVE_CONTROL,
    HB_STATE_FAULT_TAKEOVER,
})
_HB_TAKEOVER_ARMING_STATES = frozenset({
    HB_STATE_READY_STANDBY,
    HB_STATE_ACTIVE_CONTROL,
})


def _parse_primary_hb_fields(msg: str):
    """从主机心跳报文中解析关键字段。

    返回 (psi, delta, acc, aeb_flag, cls, seq, state) 元组；psi/delta/acc/aeb_flag
    任一缺失/非有限/越界则返回 None。CLS 和 STATE 字段**可选**；缺失 STATE
    按 ACTIVE_CONTROL 处理，以兼容旧主机，但新版本会显式发送 STATE。

    AEB 字段强制要求：缺失视为协议版本不匹配，拒绝整帧种子。这避免旧版主机
    把不含"是否处于全制动"语义的种子喂给新备机，造成 200ms 错误继承全制动。
    """
    psi = delta = acc = None
    aeb_flag = None
    cls_val = 0  # CLS 缺省 = UNKNOWN，可选字段
    seq = None
    state = HB_STATE_ACTIVE_CONTROL
    for token in msg.split():
        if ':' not in token:
            continue
        tag, _, val = token.partition(':')
        if tag == 'SEQ':
            try:
                seq = int(val)
            except ValueError:
                pass
            continue
        if tag == 'AEB':
            try:
                aeb_flag = 1 if int(val) != 0 else 0
            except ValueError:
                pass
            continue
        if tag == 'CLS':
            try:
                c = int(val)
                if 0 <= c <= 255:
                    cls_val = c
            except ValueError:
                pass
            continue
        if tag == 'STATE':
            if val in _HB_VALID_STATES:
                state = val
            continue
        try:
            v = float(val)
        except ValueError:
            continue
        if v != v or v in (float('inf'), float('-inf')):
            continue
        if tag == 'PSI':
            psi = v
        elif tag == 'DELTA':
            delta = v
        elif tag == 'ACC':
            acc = v
    if psi is None or delta is None or acc is None or aeb_flag is None:
        return None
    # 全零 seed 拒绝（B7）：PSI=0, DELTA=0, ACC=0 全在合法范围内，
    # 不会被物理范围 sanity 拒绝。但全零 seed 通常是损坏的心跳
    # （如主机进程卡死后 UDP 缓冲区残留的零填充报文），备机以
    # "直行 + 无加减速" 初始化可能在高速时偏离车道。
    # 例外：低速（ACC 接近零）时直行是合理的，不拒绝。
    # 这里用 aeb_flag 区分：AEB=1 时 ACC 通常是大负值，不会全零；
    # AEB=0 且全零 → 大概率是损坏帧。
    if (abs(psi) < 1e-6 and abs(delta) < 1e-6 and abs(acc) < 1e-6
            and aeb_flag == 0):
        return None
    # 物理范围 sanity：clamp 在主机侧若失效，超物理范围的脏值不应进入备机种子
    if abs(psi) > _HB_SANITY_PSI:
        return None
    if abs(delta) > _HB_SANITY_DELTA:
        return None
    if acc < _HB_SANITY_ACC_LOW or acc > _HB_SANITY_ACC_HIGH:
        return None
    return psi, delta, acc, aeb_flag, cls_val, seq, state


class PeerHeartbeat:
    """主备心跳管理器，通过 UDP 实现跨机存活检测与角色切换。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._takeover = False             # 备机是否已接管
        self._advertise_active = False      # 备机是否对外广播 active 状态
        self._running = True
        self._last_logged_backup = False
        # 备机检测到主机恢复后，记录第一次收到主机心跳的时刻；
        # 经过 HB_STANDBY_HANDOFF_S 才真正切回 standby（_takeover=False）。
        self._primary_restored_t: float = 0.0

        # 主机最后一帧广播的控制量，作为备机接管时的种子。
        self._last_primary_psi: float = 0.0
        self._last_primary_delta: float = 0.0
        self._last_primary_lon: float = 0.0
        self._last_primary_aeb: int = 0       # 主机最后一帧是否处于 AEB
        self._last_primary_cls: int = 0       # 主机最后一帧前车 class（0=UNKNOWN，可选字段）
        self._last_primary_state: str = HB_STATE_BOOTING
        self._takeover_armed: bool = False
        self._last_primary_frame_t: float = 0.0
        # B5 sanity 失败计数（限频日志用）
        self._hb_sanity_reject_count: int = 0
        self._hb_sanity_last_log_t: float = 0.0
        # 上一次 is_active() 返回值，用于检测 False→True 边沿。
        self._prev_active = False
        # 仅在边沿翻转时为 True，被 consume_takeover_seed() 消费后置 False。
        self._takeover_edge_pending = False

        # 主机发送序号；备机记录上次见到的序号，用于检测主机卡死
        self._tx_seq: int = 0
        self._last_rx_seq: int = -1
        self._last_seq_change_t: float = time.monotonic() + runtime.HB_GRACE_S

        # 启动宽限期：这段时间内不判定主机超时
        self._startup_grace = runtime.HB_GRACE_S
        self.peer_last_rx = time.monotonic() + self._startup_grace

        # 主机侧 watchdog（B4）：备机存活心跳的最近接收时刻。
        # 备机端不使用此字段。同样使用宽限期：启动后给备机一段时间上线。
        # _backup_alive_seen 用于"曾经在线过"判断，避免开机就报"无冗余"。
        self._backup_last_rx: float = time.monotonic() + self._startup_grace
        self._backup_alive_seen: bool = False
        self._backup_lost_logged: bool = False

        if not runtime.IS_PRIMARY:
            logging.info('[HB] startup grace %.0fs', self._startup_grace)

        # 主机发送到备机 IP，备机发送到主机 IP
        peer_ip = runtime.SECONDARY_IP if runtime.IS_PRIMARY else runtime.PRIMARY_IP
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('0.0.0.0', runtime.HB_PORT))
        # 5ms 轮询用于支撑 35ms 心跳超时（HEARTBEAT_TIMEOUT_S）；旧 50ms 会把备机接管
        # 检测下限钉在约 60ms，使 JETSON_TIMEOUT_MS 压到 58ms 时备机赶不上→ESP32 先全力制动。
        self._sock.settimeout(0.005)
        self._peer_addr = (peer_ip, runtime.HB_PORT)

        logging.info(
            '[HB] UDP bound 0.0.0.0:%d peer=%s:%d role=%s',
            runtime.HB_PORT,
            peer_ip,
            runtime.HB_PORT,
            runtime.NANO_ROLE,
        )

        # 所有共享状态初始化完成后再启动守护线程，避免线程先于字段就绪运行。
        threading.Thread(target=self._tx_loop, daemon=True).start()
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def _tx_loop(self):
        """心跳发送循环：备机发送存活标志。主机心跳由控制循环发送。"""
        while self._running:
            try:
                if not runtime.IS_PRIMARY:
                    with self._lock:
                        active = self._advertise_active
                    # sendto 在锁外执行，避免阻塞 rx 线程
                    msg = f'BACKUP:1 ACTIVE:{int(active)}\n'.encode('ascii')
                    self._sock.sendto(msg, self._peer_addr)
            except Exception as e:
                logging.debug('[HB] send error: %s', e)
            time.sleep(HB_SEND_INTERVAL_S)

    def _rx_loop(self):
        """心跳接收循环：监听对方心跳消息，更新超时状态与接管标志。

        无论本轮是否成功收到数据，都会在 finally 中执行 _check_takeover，
        避免偶发 socket 异常导致接管判定被跳过。
        """
        while self._running:
            try:
                try:
                    data, _ = self._sock.recvfrom(256)
                except socket.timeout:
                    pass
                except OSError as e:
                    # socket 被 close 时会抛 OSError，正常退出循环
                    if not self._running:
                        return
                    logging.debug('[HB] recv error: %s', e)
                except Exception as e:
                    logging.debug('[HB] recv error: %s', e)
                else:
                    self._handle_msg(data)
            finally:
                if not runtime.IS_PRIMARY:
                    self._check_takeover()
                else:
                    self._check_backup_watchdog()

    def _handle_msg(self, data: bytes):
        """处理一帧 UDP 心跳报文。"""
        msg = data.decode('ascii', errors='ignore').strip()
        if not runtime.IS_PRIMARY:
            # 备机收到主机心跳，更新时间戳并取消接管
            if msg.startswith('HB:'):
                seed = _parse_primary_hb_fields(msg)
                now = time.monotonic()
                # 仅收到 HB 字符串但解析失败：保活有效，但种子被拒。
                # 限频上报，避免主机持续发脏帧时刷屏。
                sanity_rejected = (seed is None)
                with self._lock:
                    self.peer_last_rx = now
                    if seed is not None:
                        psi, delta, acc, aeb_flag, cls_val, seq, state = seed
                        self._last_primary_psi = psi
                        self._last_primary_delta = delta
                        self._last_primary_lon = acc
                        self._last_primary_aeb = aeb_flag
                        self._last_primary_cls = cls_val
                        self._last_primary_state = state
                        if state in _HB_TAKEOVER_ARMING_STATES:
                            self._takeover_armed = True
                        self._last_primary_frame_t = now
                        # 序号变化才视为主机控制循环仍在推进
                        if seq is not None and seq != self._last_rx_seq:
                            self._last_rx_seq = seq
                            self._last_seq_change_t = now
                    else:
                        self._hb_sanity_reject_count += 1
                    if self._takeover:
                        # 延迟退出：第一次收到主机心跳时仅记录时间，继续维持 active；
                        # 主机心跳连续维持 HB_STANDBY_HANDOFF_S 后才真正退回 standby。
                        if self._primary_restored_t <= 0.0:
                            self._primary_restored_t = now
                            logging.warning(
                                '[HB] primary heartbeat detected, holding active for %dms',
                                int(HB_STANDBY_HANDOFF_S * 1000),
                            )
                        elif (now - self._primary_restored_t) >= HB_STANDBY_HANDOFF_S:
                            self._takeover = False
                            self._advertise_active = False
                            self._primary_restored_t = 0.0
                            logging.warning('[HB] primary heartbeat stable, backup standby')
                # sanity 拒收限频日志（持锁之外）
                if sanity_rejected:
                    now2 = time.monotonic()
                    if (now2 - self._hb_sanity_last_log_t) >= 1.0:
                        with self._lock:
                            cnt = self._hb_sanity_reject_count
                            self._hb_sanity_reject_count = 0
                        logging.warning(
                            '[HB] %d primary HB frames rejected by sanity in last 1s '
                            '(missing AEB field or out-of-range values)', cnt,
                        )
                        self._hb_sanity_last_log_t = now2
        else:
            # 主机收到备机心跳，记录备机是否已接管 + 更新 watchdog 时间戳。
            if 'BACKUP:1' in msg:
                now = time.monotonic()
                with self._lock:
                    self._backup_last_rx = now
                    self._backup_alive_seen = True
                    if self._backup_lost_logged:
                        # 备机重新上线，复位告警标志（_check_takeover 会重新输出 INFO）
                        self._backup_lost_logged = False
                        logging.warning('[HB] backup heartbeat resumed, failover available')
                if 'ACTIVE:1' in msg:
                    if not self._last_logged_backup:
                        logging.warning('[HB] backup takeover active')
                        self._last_logged_backup = True
                elif 'ACTIVE:0' in msg:
                    self._last_logged_backup = False

    def _check_takeover(self):
        """备机超时判定：每轮 rx_loop 末尾无条件执行一次。

        两个独立触发条件：
          1. socket 静默 > HEARTBEAT_TIMEOUT_S：主机进程崩溃或网络断开。
          2. SEQ 停滞 > HEARTBEAT_TIMEOUT_S：UDP 还在重发同一帧（缓冲区残留），
             但主机控制循环已经卡死，这种情况下接管同样必要。
        """
        with self._lock:
            if self._takeover:
                return
            now = time.monotonic()
            silence = now - self.peer_last_rx
            seq_stale = now - self._last_seq_change_t
            if not self._takeover_armed:
                return
            if self._last_primary_state not in _HB_TAKEOVER_ARMING_STATES:
                return
            if silence > HEARTBEAT_TIMEOUT_S:
                self._takeover = True
                self._advertise_active = False
                logging.critical(
                    '[HB] primary silence %.0fms state=%s, backup takeover',
                    silence * 1000, self._last_primary_state,
                )
            elif seq_stale > HEARTBEAT_TIMEOUT_S and self._last_rx_seq >= 0:
                self._takeover = True
                self._advertise_active = False
                logging.critical(
                    '[HB] primary SEQ stalled %.0fms (seq=%d state=%s), backup takeover',
                    seq_stale * 1000, self._last_rx_seq, self._last_primary_state,
                )

    def _check_backup_watchdog(self):
        """主机侧 watchdog：备机心跳超时则告警，并标记 failover 不可用。

        is_failover_available() 由控制循环每周期读取并发布到 ROS 话题，
        这里只负责日志限频（重复告警降为 WARNING）。
        """
        with self._lock:
            now = time.monotonic()
            seen = self._backup_alive_seen
            since = now - self._backup_last_rx
            already = self._backup_lost_logged
            timed_out = since > HB_BACKUP_TIMEOUT_S
            if timed_out and seen and not already:
                self._backup_lost_logged = True
                should_log = True
            else:
                should_log = False
        if should_log:
            logging.critical(
                '[HB] backup heartbeat lost (%.0fms), failover unavailable',
                since * 1000,
            )

    def is_failover_available(self) -> bool:
        """主备冗余当前是否可用。

        - 主机：备机心跳在 HB_BACKUP_TIMEOUT_S 内有过更新即视为可用。
          备机从未上线时返回 False（开机后等到第一帧 BACKUP 才算"可用过"）。
        - 备机：始终返回 True —— 备机自己就是 plan B；至于"主机是否还在"
          已由 _takeover 状态机表达，无需在此重复。
        """
        if not runtime.IS_PRIMARY:
            return True
        with self._lock:
            if not self._backup_alive_seen:
                return False
            return (time.monotonic() - self._backup_last_rx) <= HB_BACKUP_TIMEOUT_S

    def send_hb(self, psi, delta, acc, aeb_active: bool = False, lead_cls: int = 0,
                state: str = HB_STATE_ACTIVE_CONTROL):
        """主机发送包含控制量的心跳帧（在控制循环内调用）。

        socket.sendto 本身线程安全，无需持锁；不让控制线程与 rx 线程争锁。
        SEQ 字段单调递增，用于备机检测主机控制循环是否卡死。
        aeb_active 反映本帧 lon_tx 是否由 AEB 路径产生；备机据此在接管
        瞬间决定是否使用更宽松的衰减速率（避免继承 200ms 全制动）。
        lead_cls：主前车 actor class（0=UNKNOWN/无前车）。备机据此在
        非 AEB 接管时，对行人/障碍场景选用更严的衰减速率（B2 路径）。
        CLS 是协议**可选**字段——旧备机不解析也不报错。
        """
        try:
            # 序号回绕到 2^31，足够 248 天 @ 100Hz
            self._tx_seq = (self._tx_seq + 1) & 0x7FFFFFFF
            aeb_flag = 1 if aeb_active else 0
            cls_i = int(lead_cls) if 0 <= int(lead_cls) <= 255 else 0
            state_s = state if state in _HB_VALID_STATES else HB_STATE_ACTIVE_CONTROL
            msg = (f'HB:1 SEQ:{self._tx_seq} '
                   f'STATE:{state_s} '
                   f'PSI:{psi:.4f} DELTA:{delta:.4f} ACC:{acc:+.2f} '
                   f'AEB:{aeb_flag} CLS:{cls_i}\n')
            self._sock.sendto(msg.encode('ascii'), self._peer_addr)
        except Exception:
            pass

    def send_backup_alive(self, active_flag):
        """备机更新并广播自己的存活状态（接管时 active=True）。"""
        if runtime.IS_PRIMARY:
            return
        with self._lock:
            self._advertise_active = bool(self._takeover and active_flag)

    def is_active(self):
        """判断当前节点是否处于可输出控制的状态。

        主机始终为 True；备机仅在接管后为 True。
        同时检测 False→True 边沿，置位 _takeover_edge_pending，
        供主循环 consume_takeover_seed() 触发种子初始化。
        """
        if runtime.IS_PRIMARY:
            return True
        with self._lock:
            active = self._takeover
            if active and not self._prev_active:
                self._takeover_edge_pending = True
            self._prev_active = active
            return active

    def consume_takeover_seed(self):
        """主循环检测到接管后调用一次，返回主机最后一帧控制量作为种子。

        返回 (psi, delta, lon, aeb_flag, lead_cls) 5 元组；若当前不是边沿
        触发或没有有效种子，返回 None，由调用方按零初始化语义继续运行。

        aeb_flag=1 表示主机最后一帧由 AEB 路径产生（lon 通常接近最大制动），
        调用方应据此使用更宽松的衰减速率，避免备机原样继承 200ms 全制动。
        lead_cls 是主前车 class（0/1/2/3），用于非 AEB 接管时选 vulnerable
        衰减速率（行人/障碍）。心跳缺 CLS 字段时此处为 0。

        时效检查：种子帧时间戳距今超过 HEARTBEAT_TIMEOUT_S×2 时视为过期，
        返回 None 而非使用可能已经失效的旧控制量（例如主机崩溃前的 AEB 全制动值）。
        """
        with self._lock:
            if not self._takeover_edge_pending:
                return None
            self._takeover_edge_pending = False
            if self._last_primary_frame_t <= 0.0:
                return None
            # 时效检查：种子过期则回落到零初始化，避免用主机崩溃前的极端值
            seed_age = time.monotonic() - self._last_primary_frame_t
            if seed_age > HEARTBEAT_TIMEOUT_S * 2.0:
                logging.warning(
                    '[HB] takeover seed expired (age=%.2fs > %.2fs), using zero init',
                    seed_age, HEARTBEAT_TIMEOUT_S * 2.0,
                )
                return None
            return (
                self._last_primary_psi,
                self._last_primary_delta,
                self._last_primary_lon,
                self._last_primary_aeb,
                self._last_primary_cls,
            )

    def close(self):
        """关闭心跳套接字，停止收发线程。"""
        self._running = False
        try:
            self._sock.close()
        except Exception:
            pass
        logging.info('[HB] UDP socket closed')
