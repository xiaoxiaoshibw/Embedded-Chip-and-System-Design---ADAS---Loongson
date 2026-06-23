#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""mqtt_lite —— 纯标准库 MQTT 3.1.1（QoS0）客户端 + 极简 broker。零第三方依赖。

为什么自研：本平台坚持「纯标准库、零 pip、零 CDN」。MQTT 不在标准库里，
paho-mqtt 需 pip 安装。这里用 socket 直接实现 MQTT 3.1.1 控制报文中演示够用的
子集（CONNECT / CONNACK / PUBLISH / SUBSCRIBE / SUBACK / PINGREQ / PINGRESP /
DISCONNECT，仅 QoS0），发布端与订阅端共用 `Client`；网站还可用内置 `Broker`
把消息中转直接起在本机（无需安装 mosquitto）。

字节级兼容标准 broker：把 host/port 指向 mosquitto 等任何标准 broker 即可互通。

独立启动 broker：
    python mqtt_lite.py --broker --host 0.0.0.0 --port 1883

发布端：
    c = Client('127.0.0.1', 1883, client_id='pub')
    c.connect(); c.loop_start()
    c.publish('adas/state', '{"v":1}')

订阅端：
    c = Client('127.0.0.1', 1883, client_id='sub')
    c.on_message = lambda topic, payload: print(topic, payload)
    c.connect(); c.loop_start(); c.subscribe('adas/#')
"""

import select
import socket
import struct
import sys
import threading
import time

# ── MQTT 控制报文类型（高 4 位）──
_CONNECT = 0x10
_CONNACK = 0x20
_PUBLISH = 0x30
_SUBSCRIBE = 0x80
_SUBACK = 0x90
_PINGREQ = 0xC0
_PINGRESP = 0xD0
_DISCONNECT = 0xE0


# ══════════════════════════════════════════════════════════
# 报文编解码工具
# ══════════════════════════════════════════════════════════

def _encode_remaining_length(n):
    """剩余长度按 MQTT 变长字节整数编码。"""
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        if n > 0:
            b |= 0x80
        out.append(b)
        if n <= 0:
            break
    return bytes(out)


def _encode_string(s):
    """UTF-8 字符串：2 字节大端长度 + 内容。"""
    raw = s.encode('utf-8')
    return struct.pack('!H', len(raw)) + raw


def _recv_exact(sock, n):
    """从 socket 精确读取 n 字节（处理 TCP 分段）。"""
    buf = b''
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('对端关闭连接')
        buf += chunk
    return buf


def _read_remaining_length(sock):
    """从 socket 读取并解码剩余长度。"""
    multiplier = 1
    value = 0
    while True:
        byte = _recv_exact(sock, 1)[0]
        value += (byte & 0x7F) * multiplier
        if (byte & 0x80) == 0:
            break
        multiplier *= 128
        if multiplier > 128 ** 3:
            raise ValueError('剩余长度字段非法')
    return value


def _read_packet(sock):
    """读取一个完整 MQTT 报文，返回 (报文类型, 标志位, body 字节)。"""
    first = _recv_exact(sock, 1)[0]
    ptype = first & 0xF0
    flags = first & 0x0F
    length = _read_remaining_length(sock)
    body = _recv_exact(sock, length) if length else b''
    return ptype, flags, body


def topic_matches(filter_str, topic):
    """MQTT 主题过滤匹配（支持单层通配 + 与多层通配 #）。"""
    if filter_str == topic:
        return True
    fparts = filter_str.split('/')
    tparts = topic.split('/')
    i = 0
    for i, fp in enumerate(fparts):
        if fp == '#':
            return True  # # 必须在末尾，匹配剩余所有层级
        if i >= len(tparts):
            return False
        if fp == '+':
            continue
        if fp != tparts[i]:
            return False
    return len(fparts) == len(tparts)


# ══════════════════════════════════════════════════════════
# 客户端
# ══════════════════════════════════════════════════════════

class Client(object):
    """极简 MQTT 客户端（QoS0）。线程安全发布，后台读线程派发订阅消息，断线自动重连。"""

    def __init__(self, host='127.0.0.1', port=1883, client_id=None,
                 keepalive=30, reconnect_delay=2.0):
        self.host = host
        self.port = int(port)
        self.client_id = client_id or ('mqttlite-%d' % (int(time.time() * 1000) & 0xFFFFFF))
        self.keepalive = int(keepalive)
        self.reconnect_delay = float(reconnect_delay)
        self.on_message = None          # 回调 (topic:str, payload:bytes)
        self.on_connect = None          # 回调 ()，每次（重）连成功后调用
        self._sock = None
        self._lock = threading.Lock()   # 保护 _sock 及写操作
        self._subs = []                 # 已订阅过滤器（用于重连后重订）
        self._stop = threading.Event()
        self._thread = None
        self._sub_pid = 0

    # ── 连接 / 断开 ──
    def connect(self):
        """建立连接并完成 CONNECT 握手；失败抛异常（让调用方感知 broker 不可用）。"""
        sock = socket.create_connection((self.host, self.port), timeout=5.0)
        sock.settimeout(None)
        flags = 0x02  # clean session
        vh = _encode_string('MQTT') + bytes([0x04, flags]) + struct.pack('!H', self.keepalive)
        payload = _encode_string(self.client_id)
        body = vh + payload
        sock.sendall(bytes([_CONNECT]) + _encode_remaining_length(len(body)) + body)
        ptype, _flags, cbody = _read_packet(sock)
        if ptype != _CONNACK or len(cbody) < 2 or cbody[1] != 0x00:
            sock.close()
            raise ConnectionError('CONNECT 被拒绝 (CONNACK rc=%s)' %
                                  (cbody[1] if len(cbody) >= 2 else '?'))
        with self._lock:
            self._sock = sock
        return True

    def _resubscribe(self):
        for topic in list(self._subs):
            self._send_subscribe(topic)

    def loop_start(self):
        """启动后台读线程（派发订阅消息 + 心跳 + 自动重连）。"""
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name='mqtt-%s' % self.client_id)
        self._thread.start()

    def _loop(self):
        last_ping = time.monotonic()
        while not self._stop.is_set():
            sock = self._sock
            if sock is None:
                # 断线重连
                try:
                    self.connect()
                    self._resubscribe()
                    last_ping = time.monotonic()
                    if self.on_connect:
                        try:
                            self.on_connect()
                        except Exception:
                            pass
                except Exception:
                    self._stop.wait(self.reconnect_delay)
                continue
            try:
                r, _, _ = select.select([sock], [], [], 1.0)
                now = time.monotonic()
                if now - last_ping >= self.keepalive * 0.5:
                    self._send(bytes([_PINGREQ, 0x00]))
                    last_ping = now
                if not r:
                    continue
                ptype, _flags, body = _read_packet(sock)
                if ptype == _PUBLISH:
                    self._dispatch_publish(body)
                # PINGRESP / SUBACK 无需处理
            except Exception:
                self._drop()
                self._stop.wait(self.reconnect_delay)

    def _dispatch_publish(self, body):
        if len(body) < 2:
            return
        tlen = struct.unpack('!H', body[:2])[0]
        topic = body[2:2 + tlen].decode('utf-8', 'replace')
        payload = body[2 + tlen:]
        if self.on_message:
            try:
                self.on_message(topic, payload)
            except Exception:
                pass

    def _drop(self):
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None

    # ── 发布 / 订阅 ──
    def _send(self, data):
        with self._lock:
            if self._sock is None:
                raise ConnectionError('未连接')
            self._sock.sendall(data)

    def publish(self, topic, payload, retain=False):
        """发布一条消息（QoS0）。payload 可为 str 或 bytes。未连接时静默丢弃（QoS0 语义）。"""
        if isinstance(payload, str):
            payload = payload.encode('utf-8')
        header = _PUBLISH | (0x01 if retain else 0x00)
        body = _encode_string(topic) + payload
        try:
            self._send(bytes([header]) + _encode_remaining_length(len(body)) + body)
            return True
        except Exception:
            self._drop()
            return False

    def _send_subscribe(self, topic):
        self._sub_pid = (self._sub_pid % 0xFFFF) + 1
        body = struct.pack('!H', self._sub_pid) + _encode_string(topic) + bytes([0x00])
        self._send(bytes([_SUBSCRIBE | 0x02]) + _encode_remaining_length(len(body)) + body)

    def subscribe(self, topic):
        """订阅主题（支持 + / # 通配）。记录以便重连后自动重订。"""
        if topic not in self._subs:
            self._subs.append(topic)
        try:
            self._send_subscribe(topic)
        except Exception:
            self._drop()

    def close(self):
        self._stop.set()
        try:
            self._send(bytes([_DISCONNECT, 0x00]))
        except Exception:
            pass
        self._drop()


# ══════════════════════════════════════════════════════════
# 极简 broker（消息中转，零安装）
# ══════════════════════════════════════════════════════════

class _Conn(object):
    """broker 侧的一个客户端连接。"""

    def __init__(self, sock, addr):
        self.sock = sock
        self.addr = addr
        self.lock = threading.Lock()

    def send(self, data):
        with self.lock:
            self.sock.sendall(data)


class Broker(object):
    """纯标准库 MQTT broker（QoS0）。支持多订阅者 + 通配主题 + 心跳。"""

    def __init__(self, host='0.0.0.0', port=1883):
        self.host = host
        self.port = int(port)
        self._srv = None
        self._subs = []        # [(filter_str, _Conn)]
        self._subs_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        """后台线程启动 broker；端口占用返回 False。"""
        try:
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(64)
        except OSError as e:
            print('[BROKER] 监听 %s:%d 失败: %s' % (self.host, self.port, e), flush=True)
            return False
        self._srv = srv
        self._thread = threading.Thread(target=self._accept_loop, daemon=True, name='mqtt-broker')
        self._thread.start()
        print('[BROKER] MQTT broker 已启动: %s:%d' % (self.host, self.port), flush=True)
        return True

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                sock, addr = self._srv.accept()
            except OSError:
                break
            sock.settimeout(None)
            threading.Thread(target=self._handle, args=(_Conn(sock, addr),),
                             daemon=True).start()

    def _handle(self, conn):
        try:
            # 必须先收 CONNECT
            ptype, _flags, _body = _read_packet(conn.sock)
            if ptype != _CONNECT:
                return
            conn.send(bytes([_CONNACK, 0x02, 0x00, 0x00]))  # 接受连接
            while not self._stop.is_set():
                ptype, flags, body = _read_packet(conn.sock)
                if ptype == _PUBLISH:
                    self._on_publish(flags, body)
                elif ptype == _SUBSCRIBE:
                    self._on_subscribe(conn, body)
                elif ptype == _PINGREQ:
                    conn.send(bytes([_PINGRESP, 0x00]))
                elif ptype == _DISCONNECT:
                    break
        except Exception:
            pass
        finally:
            self._remove_conn(conn)
            try:
                conn.sock.close()
            except Exception:
                pass

    def _on_subscribe(self, conn, body):
        pid = struct.unpack('!H', body[:2])[0]
        i = 2
        codes = bytearray()
        while i < len(body):
            tlen = struct.unpack('!H', body[i:i + 2])[0]
            i += 2
            topic = body[i:i + tlen].decode('utf-8', 'replace')
            i += tlen
            i += 1  # 跳过请求 QoS
            with self._subs_lock:
                self._subs.append((topic, conn))
            codes.append(0x00)  # 授予 QoS0
        conn.send(bytes([_SUBACK]) + _encode_remaining_length(2 + len(codes)) +
                  struct.pack('!H', pid) + bytes(codes))

    def _on_publish(self, flags, body):
        qos = (flags >> 1) & 0x03
        tlen = struct.unpack('!H', body[:2])[0]
        topic = body[2:2 + tlen].decode('utf-8', 'replace')
        idx = 2 + tlen
        if qos > 0:
            idx += 2  # QoS>0 含报文标识符（broker 不回 PUBACK，仅跳过）
        payload = body[idx:]
        out = (bytes([_PUBLISH]) +
               _encode_remaining_length(len(_encode_string(topic)) + len(payload)) +
               _encode_string(topic) + payload)
        with self._subs_lock:
            targets = [c for (f, c) in self._subs if topic_matches(f, topic)]
        for c in targets:
            try:
                c.send(out)
            except Exception:
                self._remove_conn(c)

    def _remove_conn(self, conn):
        with self._subs_lock:
            self._subs = [(f, c) for (f, c) in self._subs if c is not conn]

    def stop(self):
        self._stop.set()
        if self._srv is not None:
            try:
                self._srv.close()
            except Exception:
                pass


def _main(argv=None):
    import argparse
    p = argparse.ArgumentParser(description='mqtt_lite —— 纯标准库 MQTT broker / 自检')
    p.add_argument('--broker', action='store_true', help='启动内置 broker')
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=1883)
    p.add_argument('--selftest', action='store_true', help='本机回环自检（broker+收发）')
    args = p.parse_args(argv)

    if args.selftest:
        b = Broker('127.0.0.1', args.port)
        if not b.start():
            sys.exit(1)
        got = []
        sub = Client('127.0.0.1', args.port, client_id='selftest-sub')
        sub.on_message = lambda t, pl: got.append((t, pl.decode('utf-8')))
        sub.connect(); sub.loop_start(); sub.subscribe('adas/#')
        time.sleep(0.3)
        pub = Client('127.0.0.1', args.port, client_id='selftest-pub')
        pub.connect(); pub.loop_start()
        for i in range(3):
            pub.publish('adas/state', '{"i":%d}' % i)
            time.sleep(0.1)
        time.sleep(0.3)
        pub.close(); sub.close(); b.stop()
        print('[SELFTEST] 收到 %d 条:' % len(got), got)
        sys.exit(0 if len(got) == 3 else 2)

    if args.broker:
        b = Broker(args.host, args.port)
        if not b.start():
            sys.exit(1)
        print('[BROKER] Ctrl+C 退出')
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print('\n[BROKER] 退出')
            b.stop()
        return

    p.print_help()


if __name__ == '__main__':
    _main()
