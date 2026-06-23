#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统一可爱演示网站 —— 一站式服务（纯标准库，零第三方依赖、零 CDN）。

把三块东西整合进【同一个网站】，统一可爱风格：
  🏠 总览        —— 项目报告亮点（来自 unpacked_report2 → report_highlights.json）
  🚗 实时驾驶舱   —— ADAS 实时运行（LKA/ACC/AEB/超车/行人/主备接管）
  🧠 AI 智能监控  —— 边缘 KPI 喂给 AI（规则引擎 / 可选 Ollama qwen2.5:3b）给评估与调参建议
  📊 边缘计算     —— 5s 滑窗 KPI + 事件流 + 上云
  📖 项目报告     —— 功能/非功能需求 + 三大创新点卡片

数据来源（三种后端都能接）：
  • 默认：进程内驱动驾驶仿真脚本（sim_feed）+ 边缘计算（edge）+ AI 分析（ai_analyzer），
    无需 CARLA / 无需 Ollama 也能完整演示。
  • --adas-url http://127.0.0.1:8088 ：改为代理真实 ADAS 后端的实时数据（HTTP 轮询）。
  • --mqtt-host <broker> ：订阅局域网 MQTT broker 上发布端推来的状态（推荐用于真实
    上车 / 跨机演示）。配 --mqtt-broker 可顺带在本机起一个纯标准库 broker（零安装）。

接口：
  GET /                → 单页可爱前端（web/）
  GET /api/state       → 合并状态快照（驾驶 + 边缘 + AI）
  GET /api/stream      → SSE 实时推送（~12Hz）
  GET /api/report      → 报告亮点 JSON
  GET /healthz
"""

import argparse
import json
import os
import sys
import threading
import time
import urllib.request
import webbrowser

# Windows 控制台中文输出（防 GBK 崩，不依赖 PYTHONUTF8 环境变量）
try:
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')
except Exception:
    pass

try:
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
except ImportError:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn

    class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, _ROOT)

import adas_core as sim_feed          # 本地自包含（原「主控」包已不在仓库）
from edge_engine import EdgeEngine    # 本地自包含
from ai_analyzer import AiAnalyzer

_WEB_DIR = os.path.join(_HERE, 'web')
_CONTENT_TYPES = {
    '.html': 'text/html; charset=utf-8', '.css': 'text/css; charset=utf-8',
    '.js': 'application/javascript; charset=utf-8', '.json': 'application/json; charset=utf-8',
    '.svg': 'image/svg+xml', '.png': 'image/png', '.ico': 'image/x-icon',
}


class DataHub(object):
    """聚合驾驶仿真 + 边缘计算 + AI 分析，维护最新合并状态（线程安全）。"""

    def __init__(self, edge, fps=20.0, speed=1.0, adas_url=None,
                 mqtt_host=None, mqtt_port=1883, mqtt_topic='adas/state',
                 scenarios_dir=None, repo_dir=None):
        self.edge = edge
        self.fps = float(fps)
        self.speed = float(speed)
        self.adas_url = adas_url
        self.mqtt_host = mqtt_host
        self.mqtt_port = int(mqtt_port)
        self.mqtt_topic = mqtt_topic
        # 数据来自外部后端（HTTP 代理或 MQTT 订阅）：AI 改读外部带来的 edge 快照
        self._external = bool(adas_url) or bool(mqtt_host)
        self._proxy_edge = {}
        self._state = {'connected': False}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._fps_est = float(fps)
        self.ai = None  # 由 server 注入
        # 录制场景 CSV 回溯（可自由选择回放哪一个场景）
        self._scn_list = []
        self._scn_rows = []
        self._scn_idx = 0
        self._scn_key = None
        self._scn_name = ''
        self._scn_lock = threading.Lock()
        self._csv = None
        self._rep_v_ema = None        # 回放速度 EMA（用于平滑加速度→jerk，抑制二次差分噪声）
        self._rep_v_prev = None
        self._repo = None             # 数据仓库索引（归类 + KPI 摘要 + 风险），有则优先
        if not self._external:
            try:
                import csv_replay
                self._csv = csv_replay
                idx = None
                if repo_dir:
                    try:
                        import repo_index
                        idx = repo_index.load_index(repo_dir)
                    except Exception:
                        idx = None
                if idx and idx.get('records'):
                    # 数据仓库模式：场景来自索引（含归类/摘要/风险/日期）
                    self._repo = idx
                    self._scn_list = [
                        {'key': r['id'], 'name': r['name'],
                         'path': os.path.join(repo_dir, r['id'])}
                        for r in idx['records']]
                elif scenarios_dir:
                    # 回退：扁平的录制场景目录
                    self._scn_list = csv_replay.list_scenarios(scenarios_dir)
                if self._scn_list:
                    self.select_scenario(self._scn_list[0]['key'])
            except Exception:
                self._scn_list = []

    def start(self):
        threading.Thread(target=self._loop, daemon=True, name='datahub').start()

    def set_ai(self, ai):
        self.ai = ai

    def _loop(self):
        if self.mqtt_host:
            self._mqtt_loop()
        elif self.adas_url:
            self._proxy_loop()
        elif self._scn_list:
            self._csv_loop()
        else:
            self._sim_loop()

    # 录制场景 CSV 回溯：按 fps 回放当前选中场景的行，到末尾循环；可随时切换场景
    def _csv_loop(self):
        dt = 1.0 / self.fps
        wall0 = time.monotonic()
        n = 0
        while not self._stop.is_set():
            with self._scn_lock:
                rows, idx = self._scn_rows, self._scn_idx
                name, key = self._scn_name, self._scn_key
                if rows:
                    self._scn_idx = (idx + 1) % len(rows)
            if not rows:
                self._set({'connected': False,
                           'error': '未找到场景 CSV，请先导出到「录制场景CSV」目录'})
                self._stop.wait(0.5)
                continue
            row = rows[idx % len(rows)]
            try:
                t = float(row.get('t') or (idx * dt))
            except (TypeError, ValueError):
                t = idx * dt
            fr = self._csv.row_to_frame(row, name, key)
            lead = fr['lead']
            # 平滑加速度：CSV 无 accel 列，若让 edge 对 20Hz 速度做二次差分算 jerk，
            # 量化(0.01)+控制抖动会被放大几百倍 → 满屏假「顿挫(大 jerk)」淹没真实事件。
            # 这里先对速度做 EMA 再差分得平滑 accel 交给 edge（据此算 jerk），
            # 让边缘事件流反映各场景真实事件（接近/AEB/行人/超车/车道偏移…）。
            ev = fr['ego_v']
            if idx == 0:                 # 新一圈开头：重置边缘滑窗，避免跨圈指令跳变制造假 jerk
                try:
                    self.edge.reset()
                except Exception:
                    pass
            # 加速度取控制器平滑纵向指令 lon_cmd(正=减速→accel 负=减速)，远比对 20Hz
            # 速度二次差分稳定；急刹判定(accel<=-3.5)与控制器急刹阈值天然一致。但车已停稳
            # (v<0.3)时实际加速度≈0——此刻 lon_cmd 是"保持制动"指令而非真实减速，置 0
            # 以免停车保持段把"急刹次数/jerk"灌水。
            accel_in = 0.0 if ev < 0.3 else -fr['lon_cmd']
            # edge 必须用单调递增的回放时钟（n*dt）：CSV 的 t 列每场景 0→40，切换/循环时
            # 会倒退，导致 edge 的 5s 滑窗按时间 prune 失效、旧场景数据残留与新场景混淆。
            self.edge.feed(
                n * dt, ev, lead['gap'] if lead['detected'] else None,
                lead['lead_v'] if lead['detected'] else None, lead['detected'],
                lane_offset=fr['lane_offset'], accel=accel_in, aeb_active=fr['aeb'],
                ped_warn=fr['ped_warn'], ped_ttc=fr['ped_ttc'],
                boundary_brake=(fr['lon_cmd'] if fr['lon_src'] == 'boundary' else 0.0),
                overtake_active=(fr['overtake_state'] != 'idle'),
                failover_src=fr['failover_src'])
            now = time.monotonic()
            inst = 1.0 / max(1e-3, now - getattr(self, '_last_wall', now))
            self._last_wall = now
            self._fps_est += 0.1 * (inst - self._fps_est)
            st = self._compose(t, fr)
            st['replay'] = True
            self._set(st)
            n += 1
            target = wall0 + (n * dt) / max(0.1, self.speed)
            sl = target - time.monotonic()
            if sl > 0:
                self._stop.wait(sl)

    def select_scenario(self, key):
        """切换回放的场景（加载其 CSV、指针归零）。线程安全。"""
        for s in self._scn_list:
            if s['key'] == key:
                rows = self._csv.load_rows(s['path'])
                with self._scn_lock:
                    self._scn_rows = rows
                    self._scn_idx = 0
                    self._scn_key = key
                    self._scn_name = s['name']
                    self._rep_v_ema = None
                    self._rep_v_prev = None
                try:
                    self.edge.reset()   # 清边缘滑窗，立即反映新场景（不留旧场景残影）
                except Exception:
                    pass
                return True
        return False

    def scenarios_info(self):
        if self._repo:
            return {
                'repo': True,
                'records': self._repo.get('records', []),
                'category_order': self._repo.get('category_order', []),
                'category_cn': self._repo.get('category_cn', {}),
                'generated': self._repo.get('generated'),
                'current': self._scn_key,
            }
        return {'repo': False,
                'scenarios': [{'key': s['key'], 'name': s['name']}
                              for s in self._scn_list],
                'current': self._scn_key}

    # 订阅局域网 MQTT broker：发布端（run_adas.py / web_demo.py）每帧推来的状态 JSON
    def _mqtt_loop(self):
        from mqtt_lite import Client
        last_rx = [0.0]

        def on_msg(_topic, payload):
            try:
                raw = payload.decode('utf-8') if isinstance(payload, (bytes, bytearray)) else payload
                st = json.loads(raw)
            except Exception:
                return
            # 与 _proxy_loop 一致：注入本地 AI 快照 + 暴露发布端 edge 给 AI 分析
            st['ai'] = self.ai.snapshot() if self.ai else None
            self._proxy_edge = st.get('edge') or {}
            self._set(st)
            last_rx[0] = time.monotonic()

        c = Client(self.mqtt_host, self.mqtt_port, client_id='moedrive-web')
        c.on_message = on_msg
        # 首连（失败也不退出，交给后台读线程持续重连）
        try:
            c.connect()
            c.subscribe(self.mqtt_topic)
        except Exception:
            self._set({'connected': False,
                       'error': 'MQTT broker 未连接: %s:%d' % (self.mqtt_host, self.mqtt_port)})
        c.on_connect = lambda: c.subscribe(self.mqtt_topic)
        c.loop_start()
        # 数据陈旧检测：超过 3s 没收到发布端消息 → 提示等待
        while not self._stop.is_set():
            if last_rx[0] and (time.monotonic() - last_rx[0] > 3.0):
                self._set({'connected': False,
                           'error': '等待发布端 MQTT 数据…（topic=%s）' % self.mqtt_topic})
            self._stop.wait(1.0)
        c.close()

    # 自包含仿真驱动
    def _sim_loop(self):
        dt = 1.0 / self.fps
        t = 0.0
        wall0 = time.monotonic()
        last_wall = wall0
        while not self._stop.is_set():
            loop_t = t % sim_feed.TOTAL_S
            fr = sim_feed.frame(loop_t)
            lead = fr['lead']
            self.edge.feed(
                t, fr['ego_v'], lead['gap'] if lead['detected'] else None,
                lead['lead_v'] if lead['detected'] else None, lead['detected'],
                lane_offset=fr['lane_offset'], aeb_active=fr['aeb'],
                ped_warn=fr['ped_warn'], ped_ttc=fr['ped_ttc'],
                boundary_brake=(fr['lon_cmd'] if fr['lon_src'] == 'boundary' else 0.0),
                overtake_active=(fr['overtake_state'] != 'idle'),
                failover_src=fr['failover_src'])
            now = time.monotonic()
            inst = 1.0 / max(1e-3, now - last_wall)
            last_wall = now
            self._fps_est += 0.1 * (inst - self._fps_est)
            self._set(self._compose(t, fr))
            t += dt
            target = wall0 + (t / max(0.1, self.speed))
            sl = target - time.monotonic()
            if sl > 0:
                self._stop.wait(sl)

    # 代理真实 ADAS 后端
    def _proxy_loop(self):
        url = self.adas_url.rstrip('/') + '/api/state'
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(url, timeout=2.0) as resp:
                    st = json.loads(resp.read().decode('utf-8'))
                # ADAS 后端自带 edge；把它的 edge 喂进本地 AI（通过 hub 暴露）
                st['ai'] = self.ai.snapshot() if self.ai else None
                # 用 ADAS 的 edge 快照覆盖本地（AI 也读它）
                self._proxy_edge = st.get('edge') or {}
                self._set(st)
            except Exception:
                self._set({'connected': False, 'error': 'ADAS 后端未连接: %s' % self.adas_url})
            self._stop.wait(1.0 / 12.0)

    def edge_snapshot_for_ai(self):
        """供 AI 分析读取的边缘快照（外部后端模式用发布端的，本地模式用自身 edge）。"""
        if self._external:
            return getattr(self, '_proxy_edge', {}) or {}
        return self.edge.snapshot()

    def _compose(self, t, fr):
        lead = fr['lead']
        ai_snap = self.ai.snapshot() if self.ai else None
        return {
            'connected': True,
            'sim_t': round(t, 2),
            'scenario': fr['seg'],
            'scenario_desc': fr['desc'],
            'fps': round(self._fps_est, 1),
            'ego': {'v_ms': round(fr['ego_v'], 2), 'v_kmh': round(fr['ego_v'] * 3.6, 1),
                    'lane_offset': round(fr['lane_offset'], 3),
                    'lane_width': round(fr['lane_width'], 2),
                    'curvature': round(fr['curvature'], 4)},
            'lead': lead,
            'ped': {'warn': fr['ped_warn'],
                    'ttc': round(fr['ped_ttc'], 2) if fr['ped_ttc'] is not None else None},
            'control': {'steer': round(fr['steer'], 3), 'throttle': round(fr['throttle'], 2),
                        'brake': round(fr['brake'], 2), 'delta': round(fr['steer'] * 0.4, 4),
                        'lon_cmd': round(fr['lon_cmd'], 2), 'lon_src': fr['lon_src']},
            'mode': fr['mode'], 'features': fr['features'], 'aeb': fr['aeb'],
            'overtake_state': fr['overtake_state'], 'failover_src': fr['failover_src'],
            'edge': self.edge.snapshot(),
            'ai': ai_snap,
        }

    def _set(self, state):
        with self._lock:
            self._state = state

    def get(self):
        with self._lock:
            return dict(self._state)

    def stop(self):
        self._stop.set()


def _load_report():
    try:
        with open(os.path.join(_HERE, 'report_highlights.json'), encoding='utf-8') as fh:
            return fh.read()
    except Exception:
        return '{}'


REPORT_JSON = _load_report()


def make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            return

        def _bytes(self, body, ctype, code=200):
            try:
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
            except Exception:
                pass

        def _json(self, obj, code=200):
            self._bytes(json.dumps(obj, ensure_ascii=False).encode('utf-8'),
                        'application/json; charset=utf-8', code)

        def _static(self, rel):
            if not rel or rel == '/':
                rel = 'index.html'
            rel = rel.lstrip('/')
            safe = os.path.normpath(os.path.join(_WEB_DIR, rel))
            if not safe.startswith(os.path.normpath(_WEB_DIR)):
                return self._json({'error': 'forbidden'}, 403)
            if not os.path.isfile(safe):
                return self._json({'error': 'not found', 'path': rel}, 404)
            ext = os.path.splitext(safe)[1].lower()
            try:
                with open(safe, 'rb') as fh:
                    self._bytes(fh.read(), _CONTENT_TYPES.get(ext, 'application/octet-stream'))
            except Exception:
                self._json({'error': 'read failed'}, 500)

        def _stream(self):
            try:
                self.send_response(200)
                self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Connection', 'keep-alive')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                while True:
                    payload = json.dumps(hub.get(), ensure_ascii=False)
                    self.wfile.write(('data: %s\n\n' % payload).encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(1.0 / 12.0)
            except Exception:
                return

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/state':
                self._json(hub.get())
            elif path == '/api/stream':
                self._stream()
            elif path == '/api/scenarios':
                self._json(hub.scenarios_info())
            elif path == '/api/select':
                from urllib.parse import urlparse, parse_qs
                qs = parse_qs(urlparse(self.path).query)
                key = (qs.get('scenario') or [''])[0]
                ok = hub.select_scenario(key)
                self._json({'ok': ok, 'current': hub._scn_key})
            elif path == '/api/report':
                self._bytes(REPORT_JSON.encode('utf-8'), 'application/json; charset=utf-8')
            elif path == '/healthz':
                self._json({'ok': True})
            else:
                self._static(path)
    return Handler


def main(argv=None):
    p = argparse.ArgumentParser(description='ADAS 统一可爱演示网站')
    p.add_argument('--port', type=int, default=8099)
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--speed', type=float, default=1.0, help='仿真倍速')
    p.add_argument('--adas-url', default=None, help='代理真实 ADAS 后端（如 http://127.0.0.1:8088）')
    p.add_argument('--mqtt-host', default=None,
                   help='订阅 MQTT broker 上的发布端状态（如 192.168.x.x 或 127.0.0.1）')
    p.add_argument('--mqtt-port', type=int, default=1883, help='MQTT broker 端口（默认 1883）')
    p.add_argument('--mqtt-topic', default='adas/state', help='订阅主题（默认 adas/state）')
    p.add_argument('--mqtt-broker', action='store_true',
                   help='在本机内置一个纯标准库 MQTT broker（零安装，发布端连本机即可）')
    p.add_argument('--ai-interval', type=float, default=12.0, help='AI 分析间隔秒')
    p.add_argument('--repo-dir', default=None,
                   help='历史数据仓库目录（默认 ./数据仓库，有 index.json 则优先用于归类回溯）')
    p.add_argument('--scenarios-dir', default=None,
                   help='录制场景 CSV 目录（默认 ./录制场景CSV）。无仓库索引时回退用它')
    p.add_argument('--no-scenarios', action='store_true',
                   help='禁用历史回溯，改用内置脚本仿真')
    p.add_argument('--no-browser', action='store_true')
    args = p.parse_args(argv)

    # 可选：本机内置 MQTT broker（消息中转），发布端与本网站都连它
    broker = None
    if args.mqtt_broker:
        from mqtt_lite import Broker
        broker = Broker(host='0.0.0.0', port=args.mqtt_port)
        if not broker.start():
            print('[WEB] 内置 broker 启动失败，请检查端口 %d' % args.mqtt_port)
            sys.exit(1)
        # 启用内置 broker 时默认订阅本机
        if not args.mqtt_host:
            args.mqtt_host = '127.0.0.1'

    outbox = os.path.join(_HERE, 'output', 'cloud_outbox')
    edge = EdgeEngine(window_s=5.0, emit_interval_s=1.0, outbox_dir=outbox)
    scenarios_dir = None
    repo_dir = None
    if not args.no_scenarios:
        repo_dir = args.repo_dir or os.path.join(_HERE, '数据仓库')
        scenarios_dir = args.scenarios_dir or os.path.join(_HERE, '录制场景CSV')
    hub = DataHub(edge, fps=20.0, speed=args.speed, adas_url=args.adas_url,
                  mqtt_host=args.mqtt_host, mqtt_port=args.mqtt_port,
                  mqtt_topic=args.mqtt_topic, scenarios_dir=scenarios_dir,
                  repo_dir=repo_dir)
    ai = AiAnalyzer(hub.edge_snapshot_for_ai, interval_s=args.ai_interval)
    hub.set_ai(ai)
    hub.start()
    ai.start()

    handler = make_handler(hub)
    try:
        httpd = ThreadingHTTPServer((args.host, args.port), handler)
        httpd.daemon_threads = True
    except OSError as e:
        print('[WEB] 端口 %d 启动失败: %s' % (args.port, e))
        sys.exit(1)

    url = 'http://%s:%d/' % ('127.0.0.1' if args.host == '0.0.0.0' else args.host, args.port)
    print('=' * 56)
    print(' ADAS 统一可爱演示网站已启动')
    print(' 浏览器打开:  %s' % url)
    if args.mqtt_host:
        src_desc = 'MQTT 订阅 %s:%d topic=%s%s' % (
            args.mqtt_host, args.mqtt_port, args.mqtt_topic,
            '（含本机内置 broker）' if broker else '')
    elif args.adas_url:
        src_desc = '代理 ADAS 后端 ' + args.adas_url
    elif hub._repo:
        src_desc = '历史数据仓库回溯（%d 条记录，按类型/日期归类，可在驾驶舱选择回放）' % len(hub._scn_list)
    elif hub._scn_list:
        src_desc = '录制场景回溯（%d 个场景，可在驾驶舱切换）' % len(hub._scn_list)
    else:
        src_desc = '内置仿真(无需CARLA)'
    print(' 数据源:      %s' % src_desc)
    print(' AI 后端:     启动后自动探测 Ollama，未就绪则用内置规则引擎')
    print(' Ctrl+C 退出')
    print('=' * 56)
    if not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print('\n[EXIT] 退出')
    finally:
        hub.stop()
        ai.stop()
        edge.close()
        if broker is not None:
            try:
                broker.stop()
            except Exception:
                pass
        try:
            httpd.server_close()
        except Exception:
            pass


if __name__ == '__main__':
    main()
