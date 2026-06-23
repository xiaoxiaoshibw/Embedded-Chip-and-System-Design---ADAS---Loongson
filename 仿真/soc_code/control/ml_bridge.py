# -*- coding: utf-8 -*-
"""ML 推理桥接模块。

将 SOC 控制环的 VehicleSignals 适配为 ML 推理 API 所需的特征格式，
并处理 100Hz → 10Hz 降采样。ML 推理结果作为规则控制的辅助信号，
不替代任何安全关键路径。

推理后端可选（config.ML_BACKEND）：
- 'onnx'（推荐）：onnxruntime 跑 checkpoints/*.onnx，轻、快、尾部延迟紧，Nano 首选；
- 'torch'：PyTorch 跑 *.pt（开发机回归/对拍）；
- 'auto'：优先 onnx，失败回退 torch。
后端依赖缺失时模块自动降级为 no-op，控制环行为不变。

实时安全（config.ML_ASYNC=True，默认）：推理在守护线程异步执行，
控制环只做非阻塞入队 + 原子读最新结果——把推理耗时与 100Hz/10ms 预算彻底解耦，
即便单次推理偶发变慢也绝不吃 tick。ML_ASYNC=False 时退回同步内联（离线确定性回归用）。

用法（在纵向策略中，受 config.ML_ENABLED 开关控制）：
    ml_result = ml_bridge.update(now, signals, lead_ctx)
    if ml_result and ml_result.aeb_class >= 1:
        # ML 认为有碰撞风险，可作为辅助信号
        ...
"""

import logging
import os
import sys
import threading
import time

try:
    import queue
except ImportError:  # pragma: no cover - Py2 兜底，实际目标是 Py3.6
    import Queue as queue  # type: ignore

from typing import Optional

from common import is_finite
from config import LOOP_HZ, ML_BACKEND, ML_NUM_THREADS, ML_ASYNC

# ML 包路径（相对于 SOCCode/）
_ML_PKG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            '..', 'ml', 'ml')
_ML_PKG_DIR = os.path.normpath(_ML_PKG_DIR)

# 降采样：ML 模型以 10Hz 训练，控制环以 100Hz 运行
_ML_SAMPLE_INTERVAL = max(1, LOOP_HZ // 10)

# worker 线程的 RESET 指令哨兵
_RESET = object()

# 推理错误日志限频（避免热路径刷屏）
_last_infer_err_t = [0.0]


def _log_infer_error(e):
    t = time.time()
    if t - _last_infer_err_t[0] > 5.0:
        logging.warning('[ML] inference error: %s', e)
        _last_infer_err_t[0] = t


def _load_predictors(backend, num_threads):
    """按 backend 加载 (acc, aeb) 推理器，返回 (acc, aeb, used_backend)。

    backend: 'onnx' | 'torch' | 'auto'。失败抛异常，由调用方降级。
    """
    if _ML_PKG_DIR not in sys.path:
        sys.path.insert(0, _ML_PKG_DIR)

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
            logging.warning('[ML] backend %s load failed: %s', be, e)
    if last_err is not None:
        raise last_err
    raise RuntimeError('no ML backend available')


class MlPrediction:
    """ML 推理结果（单周期）。"""
    __slots__ = ('acc_pred', 'aeb_class', 'aeb_probs', 'should_brake', 'brake_intensity')

    def __init__(self):
        self.acc_pred = 0.0          # ACC 预测加速度 (m/s²)
        self.aeb_class = 0           # AEB 分类: 0=safe, 1=warning, 2=emergency
        self.aeb_probs = None        # AEB 概率分布 (3,)
        self.should_brake = False    # ML 建议制动
        self.brake_intensity = 0.0   # 制动强度 0~1


class MlBridge:
    """ML 推理桥接器，在控制环中按降采样频率调用。"""

    def __init__(self):
        self._cycle_count = 0
        self._last_result = MlPrediction()   # 原子引用，控制环读、worker 写
        self._acc = None
        self._aeb = None
        self._backend = None
        self._enabled = False
        self._async = bool(ML_ASYNC)
        self._queue = None
        self._worker = None

        if os.path.isdir(_ML_PKG_DIR):
            try:
                self._acc, self._aeb, self._backend = _load_predictors(
                    ML_BACKEND, ML_NUM_THREADS)
                self._enabled = True
            except Exception as e:
                logging.warning('[ML] failed to load models, ML disabled: %s', e)
                self._enabled = False
        else:
            logging.warning('[ML] package dir not found, ML disabled: %s', _ML_PKG_DIR)

        if self._enabled and self._async:
            self._start_worker()

        if self._enabled:
            logging.info(
                '[ML] bridge enabled, backend=%s threads=%s mode=%s interval=%d cycles',
                self._backend, ML_NUM_THREADS,
                'async' if self._async else 'sync', _ML_SAMPLE_INTERVAL)
        else:
            logging.info('[ML] bridge disabled (models not available)')

    @property
    def enabled(self):
        # type: () -> bool
        return self._enabled

    @property
    def backend(self):
        # type: () -> Optional[str]
        return self._backend

    # ── 守护线程 ──

    def _start_worker(self):
        # 容量 8：稳态下采样帧 100ms 一个、推理仅 ~ms，队列几乎不积压；
        # 留余量是为了容忍控制环偶发突发（追帧），避免丢掉滑窗连续帧。
        self._queue = queue.Queue(maxsize=8)
        self._worker = threading.Thread(
            target=self._worker_loop, name='ml-infer', daemon=True)
        self._worker.start()

    def _worker_loop(self):
        while True:
            item = self._queue.get()
            try:
                if item is _RESET:
                    if self._acc is not None:
                        self._acc.reset()
                    if self._aeb is not None:
                        self._aeb.reset()
                    self._last_result = MlPrediction()
                    continue
                acc_features, aeb_features = item
                self._last_result = self._run_inference(acc_features, aeb_features)
            except Exception as e:
                _log_infer_error(e)

    def _enqueue(self, item):
        """非阻塞入队，满则丢最旧（保留最新帧，符合实时取向）。"""
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                pass

    def _enqueue_reset(self):
        try:
            while True:
                self._queue.get_nowait()
        except queue.Empty:
            pass
        try:
            self._queue.put_nowait(_RESET)
        except queue.Full:
            pass

    # ── 推理执行（worker 线程或同步路径调用）──

    def _run_inference(self, acc_features, aeb_features):
        result = MlPrediction()

        if acc_features is not None and self._acc is not None:
            self._acc.update(*acc_features)
            result.acc_pred = self._acc.predict()

        if aeb_features is not None and self._aeb is not None:
            self._aeb.update(*aeb_features)
            class_id, probs = self._aeb.predict()
            result.aeb_class = class_id
            result.aeb_probs = probs
            # 直接由概率推导，避免重复 predict()
            p1 = float(probs[1])
            p2 = float(probs[2])
            result.should_brake = (p1 + p2) > 0.5
            result.brake_intensity = p1 * 0.5 + p2 * 1.0

        return result

    def reset(self):
        """重置 ML 缓冲区（场景切换时调用）。"""
        self._cycle_count = 0
        self._last_result = MlPrediction()
        if self._async and self._queue is not None:
            self._enqueue_reset()
        else:
            if self._acc is not None:
                self._acc.reset()
            if self._aeb is not None:
                self._aeb.reset()

    def update(self, now, signals, lead_ctx):
        # type: (float, object, object) -> Optional[MlPrediction]
        """每控制周期调用一次，按降采样频率执行推理。

        返回 MlPrediction（始终返回最新可用结果，非采样/预热周期返回上一次结果）。
        ML 不可用时返回 None。

        异步模式：仅做特征构建 + 非阻塞入队 + 原子读最新结果，不在本调用内推理。
        """
        if not self._enabled:
            return None

        self._cycle_count += 1
        if self._cycle_count % _ML_SAMPLE_INTERVAL != 0:
            return self._last_result

        # 构建特征（轻量 numpy，留在控制环线程；无共享状态）
        acc_features = self._build_acc_features(signals, lead_ctx)
        aeb_features = self._build_aeb_features(signals, lead_ctx)

        if acc_features is None and aeb_features is None:
            return self._last_result

        if self._async and self._queue is not None:
            # 推理交给守护线程，本调用立即返回最近一次结果
            self._enqueue((acc_features, aeb_features))
            return self._last_result

        # 同步路径（ML_ASYNC=False，离线确定性回归）
        self._last_result = self._run_inference(acc_features, aeb_features)
        return self._last_result

    @staticmethod
    def _build_acc_features(signals, lead_ctx):
        """从 VehicleSignals 构建 ACC 特征 (7维)。

        特征顺序与 ml/ml/ 推理器 update() 一致：
        gap_distance, v_ego, v_lead, relative_speed, acc_ego, acc_lead, time_headway
        """
        if not signals.lead_received or not signals.ego_received:
            return None

        v_ego = signals.ego_v
        if not is_finite(v_ego) or v_ego < 0:
            return None

        # 距离：使用 lead_ctx 的 x_rel（投影距离），更准确
        gap = float(lead_ctx.x_rel) if is_finite(lead_ctx.x_rel) else 0.0
        if gap <= 0:
            return None

        v_lead = float(lead_ctx.predicted_lead_v_proj) if is_finite(lead_ctx.predicted_lead_v_proj) else 0.0
        rel_speed = v_lead - v_ego  # 负值 = 接近（ML 训练时的约定）

        # 加速度：SOC 没有直接的 ego_acc 信号，用 0 近似
        # （ML 训练数据中 acc_ego 来自差分，短期近似为 0 可接受）
        acc_ego = 0.0
        # 前车加速度：从 lead_ctx 获取（如有）
        acc_lead = float(getattr(lead_ctx, 'lead_accel', 0.0) or 0.0)
        if not is_finite(acc_lead):
            acc_lead = 0.0

        thw = gap / max(v_ego, 0.1)  # 时间间距

        return (gap, v_ego, v_lead, rel_speed, acc_ego, acc_lead, thw)

    @staticmethod
    def _build_aeb_features(signals, lead_ctx):
        """从 VehicleSignals 构建 AEB 特征 (10维)。

        特征顺序与 ml/ml/ 推理器 update() 一致：
        gap_distance, v_ego, v_lead, relative_speed, ttc, inverse_ttc,
        drac, thw, closing_speed, acc_lead
        """
        if not signals.lead_received or not signals.ego_received:
            return None

        v_ego = signals.ego_v
        if not is_finite(v_ego) or v_ego < 0:
            return None

        gap = float(lead_ctx.x_rel) if is_finite(lead_ctx.x_rel) else 0.0
        if gap <= 0:
            return None

        v_lead = float(lead_ctx.predicted_lead_v_proj) if is_finite(lead_ctx.predicted_lead_v_proj) else 0.0
        rel_speed = v_lead - v_ego

        closing_speed = max(v_ego - v_lead, 0.0)
        if closing_speed > 0.1:
            ttc = gap / closing_speed
            ttc = min(ttc, 100.0)  # 训练时上限 100s
            inverse_ttc = 1.0 / ttc
            drac = (closing_speed ** 2) / (2.0 * gap)
            drac = min(drac, 20.0)  # 训练时上限 20 m/s²
        else:
            ttc = 100.0
            inverse_ttc = 0.0
            drac = 0.0

        thw = gap / max(v_ego, 0.1)

        acc_lead = float(getattr(lead_ctx, 'lead_accel', 0.0) or 0.0)
        if not is_finite(acc_lead):
            acc_lead = 0.0

        return (gap, v_ego, v_lead, rel_speed, ttc, inverse_ttc, drac, thw, closing_speed, acc_lead)
