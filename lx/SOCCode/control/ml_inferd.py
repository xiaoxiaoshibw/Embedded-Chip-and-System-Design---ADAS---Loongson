#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ML 推理守护进程。

独立进程，绑定到指定 CPU 核（默认 core 3），通过 TCP localhost 接收
特征向量并返回推理结果。控制环（ADAS.py）通过 ml_bridge 与此进程通信。

设计目标：
- 独立 GIL：推理不阻塞 100Hz 控制环
- 崩溃隔离：ONNX Runtime segfault 不拖垮 ADAS
- 确定性调度：绑核避免缓存污染 core 0

协议：长度前缀 JSON（先 4 字节大端 uint32，后 JSON payload）。
端口：ML_INFERD_PORT 环境变量，默认 19999。

启动（由 ADAS.py main() 自动执行）：
  taskset -c 3 python3 control/ml_inferd.py

降级：ml_inferd 不可用时 ml_bridge 自动降级为 no-op，控制不受影响。
"""

from __future__ import absolute_import, division, print_function

import json
import logging
import os
import signal
import socket
import struct
import sys
import time
try:
    import ctypes
    _LIBC = ctypes.CDLL(None)
    _PR_SET_PDEATHSIG = 1
    _LIBC.prctl(_PR_SET_PDEATHSIG, signal.SIGTERM, 0, 0, 0)
except Exception:
    pass

# 将 SOCCode 根目录加入 sys.path
_SELF_DIR = os.path.dirname(os.path.abspath(__file__))
_SOCCode_DIR = os.path.dirname(_SELF_DIR)
if _SOCCode_DIR not in sys.path:
    sys.path.insert(0, _SOCCode_DIR)

from config import ML_BACKEND, ML_NUM_THREADS

PORT = int(os.environ.get('ML_INFERD_PORT', '19999'))
_BACKLOG = 1


def _load_predictors(backend, num_threads):
    """加载 (acc, aeb) 推理器。与 ml_bridge._load_predictors 同逻辑。"""
    ml_pkg = os.path.join(_SOCCode_DIR, '..', 'ml', 'ml')
    ml_pkg = os.path.normpath(ml_pkg)
    if ml_pkg not in sys.path:
        sys.path.insert(0, ml_pkg)

    order = []
    if backend == 'onnx':
        order = ['onnx']
    elif backend == 'torch':
        order = ['torch']
    else:
        order = ['onnx', 'torch']

    last_err = None
    for be in order:
        try:
            if be == 'onnx':
                from inference_onnx import (
                    create_acc_predictor_onnx, create_aeb_predictor_onnx,
                )
                acc = create_acc_predictor_onnx(num_threads=num_threads)
                aeb = create_aeb_predictor_onnx(num_threads=num_threads)
                return acc, aeb, 'onnx'
            else:
                import torch
                if num_threads and num_threads > 0:
                    try:
                        torch.set_num_threads(int(num_threads))
                    except Exception:
                        pass
                from inference import create_acc_predictor, create_aeb_predictor
                acc = create_acc_predictor()
                aeb = create_aeb_predictor()
                return acc, aeb, 'torch'
        except Exception as e:
            last_err = e
    raise RuntimeError('no ML backend available: %s' % (last_err,))


# 半帧兜底：进入半帧后连续空闲超过此值（秒）才判坏连接放弃；
# 帧间空闲（尚未收到任何字节）不计入，可无限等待。
_HALF_FRAME_TIMEOUT_S = 5.0


def _recv_exact(conn, n):
    """从 socket 接收精确 n 字节。返回 bytes 或 None（连接断开）。

    帧间空闲（buf 为空）容忍 conn.settimeout(1.0) 触发的 socket.timeout，
    持续等待下一帧——控制环在静默待机（无感知）时会长时间不发请求，
    旧实现因 1s recv 超时直接 close，导致 ml_bridge 永久降级（CLOSE-WAIT）。
    一旦进入半帧（已收到部分字节）则用 _HALF_FRAME_TIMEOUT_S 兜底，
    防止对端发半包后挂死造成永久阻塞。
    """
    buf = b''
    half_frame_deadline = None
    while len(buf) < n:
        try:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
            half_frame_deadline = None  # 收到数据，重置半帧计时
        except socket.timeout:
            if not buf:
                continue  # 帧间空闲，继续等待下一帧
            now = time.time()
            if half_frame_deadline is None:
                half_frame_deadline = now + _HALF_FRAME_TIMEOUT_S
            elif now >= half_frame_deadline:
                return None
    return buf


def _run_inference(acc, aeb, acc_features, aeb_features):
    """执行推理，返回结果 dict。"""
    result = {}
    if acc_features is not None and acc is not None:
        try:
            acc.update(*acc_features)
            result['acc_pred'] = float(acc.predict())
        except Exception:
            result['acc_pred'] = 0.0
    if aeb_features is not None and aeb is not None:
        try:
            aeb.update(*aeb_features)
            cls_id, probs = aeb.predict()
            result['aeb_class'] = int(cls_id)
            result['aeb_probs'] = [float(p) for p in probs]
            p1, p2 = float(probs[1]), float(probs[2])
            result['should_brake'] = (p1 + p2) > 0.5
            result['brake_intensity'] = p1 * 0.5 + p2 * 1.0
        except Exception:
            result['aeb_class'] = 0
            result['should_brake'] = False
            result['brake_intensity'] = 0.0
    return result


def main():
    logging.basicConfig(
        format='[ML-INFERD] %(levelname)s %(message)s',
        level=logging.INFO,
    )

    # 加载模型（一次性，之后常驻）
    try:
        acc, aeb, backend = _load_predictors(ML_BACKEND, ML_NUM_THREADS)
        if backend == 'torch':
            logging.critical('BACKEND=torch — PyTorch 内存开销巨大（~150MB+）！'
                             '应设 ADAS_ML_BACKEND=onnx')
    except Exception as e:
        logging.error('failed to load models: %s', e)
        sys.exit(1)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', PORT))
    server.listen(_BACKLOG)
    logging.info('ready on port %d backend=%s threads=%s',
                 PORT, backend, ML_NUM_THREADS)

    while True:
        conn, addr = server.accept()
        conn.settimeout(1.0)  # recv 超时 1s，防止半帧永久阻塞
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        logging.debug('connection from %s:%d', addr[0], addr[1])
        try:
            while True:
                raw_len = _recv_exact(conn, 4)
                if raw_len is None:
                    break
                msglen = struct.unpack('!I', raw_len)[0]
                data = _recv_exact(conn, msglen)
                if data is None:
                    break

                msg = json.loads(data.decode('utf-8'))
                if msg.get('reset'):
                    if acc is not None:
                        acc.reset()
                    if aeb is not None:
                        aeb.reset()
                    result = {}
                elif msg.get('ping'):
                    result = {'pong': True}
                else:
                    acc_feat = tuple(msg.get('acc_features') or []) or None
                    aeb_feat = tuple(msg.get('aeb_features') or []) or None
                    result = _run_inference(acc, aeb, acc_feat, aeb_feat)

                resp = json.dumps(result).encode('utf-8')
                conn.sendall(struct.pack('!I', len(resp)) + resp)
        except (ConnectionResetError, BrokenPipeError, OSError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass


if __name__ == '__main__':
    main()
