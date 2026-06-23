# -*- coding: utf-8 -*-
"""真实 CARLA 世界端封装。

设计原则（与需求一致）：**不重写**已有 CARLA 场景代码，复用 `carla_bridge/pc/carla_link.py`
的 `CarlaLink`（连接 / 同步模式 / 生成 ego+前车 / 真值感知 / 执行器映射 / 旁观者
跟车），本类只在其之上做两件新事：
  1. 把 hil_platform 的 params（ego_speed/front_distance/...）适配成 CarlaLink 的
     场景字典（一处 adapter，不动 carla_link）；
  2. 「自由操控世界」的新能力（天气 / NPC 交通流 / 前车接管 / 手动驾驶 / 摄像头），
     全部作用在 CarlaLink 已持有的 world/client/actor 上，纯增量。

被 SimulationCore 经 RealHilBridge 独占持有：全局只有一处 tick、一处 actor 管理。
本模块对 carla 做惰性 import —— 没装 carla / mock 模式下导入本文件不会报错。
"""

from __future__ import annotations

import math
import os
import sys
import tempfile
import threading
import time
from typing import Any, Dict, List, Optional

# 复用 HIL/carla_bridge/pc/ 的 CarlaLink 与场景库（把其目录加入 sys.path 以解析其同级 import）
_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
_INTEG_DIR = os.path.join(_REPO_ROOT, "carla_bridge", "pc")

# 摄像头编码：优先 numpy+PIL 在内存里转 JPEG（最稳，绕开 CARLA save_to_disk 对
# 非 ASCII 路径静默失败的问题——本仓库路径含中文）。缺库时回退到 ASCII 临时目录存 PNG。
try:
    import numpy as _np
    from PIL import Image as _PILImage
    _CAM_ENCODER = "pil"
except Exception:
    _np = None
    _PILImage = None
    _CAM_ENCODER = "save_to_disk"

CARLA_HOST = os.environ.get("CARLA_HOST", "127.0.0.1")
CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))
TOWN = os.environ.get("CARLA_TOWN", "Town04")
# TrafficManager 专用端口：默认 8000 会与 HIL 后端(8000)冲突，固定到 8010
TM_PORT = int(os.environ.get("CARLA_TM_PORT", "8010"))

# 场景名 → 自车出生点（沿用 carla_bridge/pc/scenarios 的取值）
_SPAWN_INDEX = {
    "acc_follow": 30, "aeb_brake": 30, "lka_curve": 30,
    "cut_in": 30, "takeover": 30,
}


def _import_carla():
    import carla  # noqa
    return carla


def _import_carla_link():
    if _INTEG_DIR not in sys.path:
        sys.path.insert(0, _INTEG_DIR)
    import carla_link  # type: ignore
    return carla_link


def params_to_scenario(name: str, params: Dict[str, Any]) -> Dict[str, Any]:
    """hil_platform params → CarlaLink 场景字典（carla_bridge/pc/scenarios 同构）。"""
    front_d = float(params.get("front_distance", 40.0))
    front_v = float(params.get("front_speed", 35.0)) / 3.6           # km/h→m/s
    ftime = float(params.get("fault_trigger_time", 0.0) or 0.0)
    spawn_index = _SPAWN_INDEX.get(name, 30)

    if name == "lka_curve":
        lead = None
    elif name == "aeb_brake":
        brake_t = ftime or 8.0
        lead = {"gap0": front_d,
                "profile": [(0.0, front_v), (brake_t, 0.0)],
                "hard_brake": (brake_t, brake_t + 7.0)}
    elif name == "cut_in":
        # 注：carla_link 无横向切入机动，这里近似为「近距前车 + 切入速度」。
        # 真正的横向 cut-in 机动留作后续（用 set_lead_speed + 变道脚本扩展）。
        cut_d = float(params.get("cut_in_trigger_distance", 25.0))
        cut_v = float(params.get("cut_in_speed", 40.0)) / 3.6
        lead = {"gap0": cut_d, "profile": [(0.0, cut_v)], "hard_brake": None}
    else:  # acc_follow / takeover / 其它
        lead = {"gap0": front_d, "profile": [(0.0, front_v)], "hard_brake": None}

    return {"name": name, "duration": 0.0, "spawn_index": spawn_index,
            "lead": lead, "timeline": [], "notes": []}


class CarlaWorld:
    """真实 CARLA 世界：建图 / 感知 / 执行 + 自由操控 + 摄像头显示。"""

    def __init__(self, host: str = CARLA_HOST, port: int = CARLA_PORT, town: str = TOWN,
                 enable_camera: bool = True):
        self.host = host
        self.port = port
        self.town = town
        self.enable_camera = enable_camera
        self.carla = None
        self.link = None                      # carla_bridge.pc.carla_link.CarlaLink 实例
        self._npcs: List[Any] = []
        self._tm = None
        self._camera = None
        self._collision = None
        self._collided = False
        # ASCII 临时路径仅用于无 PIL 的回退（CARLA save_to_disk 不支持中文路径）
        self._cam_path = os.path.join(tempfile.gettempdir(), "hil_live_cam.png")
        self._cam_jpeg: Optional[bytes] = None
        self._cam_lock = threading.Lock()
        self._cam_last_save = 0.0
        self._lead_speed_override: Optional[float] = None   # m/s，None=按场景脚本
        self._manual = False
        self._manual_cmd = {"throttle": 0.0, "brake": 0.0, "steer": 0.0}
        self.params: Dict[str, Any] = {}

    # ── 建图 ──
    def load(self, scenario_name: str, params: Dict[str, Any]) -> None:
        self.carla = _import_carla()
        carla_link = _import_carla_link()
        self.params = dict(params)
        self.reset()   # 清理上一轮 actor

        scenario = params_to_scenario(scenario_name, params)
        # 复用 CarlaLink 完成连接 / 同步模式 / 生成 ego+前车 / 参考线
        try:
            self.link = carla_link.CarlaLink(
                self.carla, self.host, self.port, scenario, town=self.town)
        except RuntimeError as exc:
            raise RuntimeError(
                "连接/初始化 CARLA 失败（确认 CarlaUE4.exe 已运行、端口 %d 就绪）：%s"
                % (self.port, exc))
        self.set_weather(str(params.get("weather", "clear")))
        self._attach_collision()
        if self.enable_camera:
            self._attach_camera()

    # ── 仿真推进（委托 CarlaLink，单处 tick）──
    def tick(self) -> float:
        return float(self.link.tick())

    def sense(self) -> Dict[str, Any]:
        """返回控制器/StateFrame 所需的感知量（在 carla_link.sense 之上整理）。
        含 'raw'（carla_link 原始帧）供 NanoLink 构造网关感知载荷——一次 sense 调用兼顾两路。"""
        frame, gap = self.link.sense(self.link.world.get_snapshot().timestamp.elapsed_seconds)
        ego_yaw = float(frame.get("ego_yaw", 0.0))
        road_psi = float(frame.get("road_psi", 0.0))
        he = math.atan2(math.sin(ego_yaw - road_psi), math.cos(ego_yaw - road_psi))
        return {
            "ego_v": float(frame.get("ego_v", 0.0)),
            "lead_present": bool(frame.get("lead_present", False)),
            "lead_v": float(frame.get("lead_v", 0.0)),
            "gap": gap,
            "lane_offset": float(frame.get("lane_offset", 0.0)),
            "heading_error": he,
            "raw": frame,
        }

    def apply(self, delta: float, a_brake: float):
        """把 delta(rad)/a_brake(+减速) 经 carla_link 映射并下发 ego。返回 (steer,thr,brk)。"""
        if self._manual:
            c = self._manual_cmd
            self.link.ego.apply_control(self.carla.VehicleControl(
                throttle=float(c["throttle"]), brake=float(c["brake"]), steer=float(c["steer"])))
            return c["steer"], c["throttle"], c["brake"]
        return self.link.apply_ego({"delta": delta, "a_brake": a_brake})

    def drive_lead(self, sim_t: float) -> None:
        if self.link.lead is None:
            return
        if self._lead_speed_override is not None:
            self._drive_lead_to(self._lead_speed_override)
        else:
            self.link.drive_lead(sim_t)

    def update_spectator(self) -> None:
        self.link.update_spectator()

    # ── 自由操控：天气 ──
    def set_weather(self, name: str) -> None:
        if self.link is None:
            return
        carla = self.carla
        presets = {
            "clear": carla.WeatherParameters.ClearNoon,
            "rain": carla.WeatherParameters.HardRainNoon,
            "fog": getattr(carla.WeatherParameters, "MidRainyNoon", carla.WeatherParameters.CloudyNoon),
            "night": getattr(carla.WeatherParameters, "ClearNight", carla.WeatherParameters.ClearSunset),
        }
        w = presets.get(name, carla.WeatherParameters.ClearNoon)
        if name == "fog":
            w = carla.WeatherParameters(cloudiness=80, fog_density=80, fog_distance=15)
        self.link.world.set_weather(w)

    # ── 自由操控：NPC 交通流 ──
    def spawn_npc(self, count: int) -> int:
        if self.link is None:
            return 0
        carla = self.carla
        world = self.link.world
        if self._tm is None:
            self._tm = self.link.client.get_trafficmanager(TM_PORT)
            self._tm.set_synchronous_mode(True)
        bp_lib = world.get_blueprint_library()
        vehicle_bps = [b for b in bp_lib.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4]
        spawn_points = world.get_map().get_spawn_points()
        import random
        random.shuffle(spawn_points)
        spawned = 0
        for sp in spawn_points:
            if spawned >= count:
                break
            bp = random.choice(vehicle_bps)
            actor = world.try_spawn_actor(bp, sp)
            if actor is not None:
                actor.set_autopilot(True, self._tm.get_port())
                self._npcs.append(actor)
                spawned += 1
        return spawned

    def clear_npc(self) -> int:
        n = len(self._npcs)
        if self._npcs and self.link is not None and self.carla is not None:
            try:
                self.link.client.apply_batch(
                    [self.carla.command.DestroyActor(a) for a in self._npcs])
            except Exception:
                for a in self._npcs:
                    try:
                        a.destroy()
                    except Exception:
                        pass
        self._npcs = []
        return n

    # ── 自由操控：前车接管 / 手动驾驶 ──
    def set_lead_speed(self, kmh: Optional[float]) -> None:
        self._lead_speed_override = None if kmh is None else float(kmh) / 3.6

    def _drive_lead_to(self, v_target: float) -> None:
        lead = self.link.lead
        vel = lead.get_velocity()
        v = math.sqrt(vel.x ** 2 + vel.y ** 2)
        err = v_target - v
        throttle = max(0.0, min(0.8, 0.3 + 0.2 * err))
        brake = max(0.0, min(1.0, -0.2 * err)) if err < -0.3 else 0.0
        # 简易车道保持沿用 carla_link 的内部方法
        steer = self.link._lead_lane_keep()
        lead.apply_control(self.carla.VehicleControl(
            throttle=float(throttle), brake=float(brake), steer=float(steer)))

    def set_manual(self, on: bool) -> None:
        self._manual = bool(on)

    def manual_cmd(self, throttle: float = 0.0, brake: float = 0.0, steer: float = 0.0) -> None:
        self._manual_cmd = {"throttle": float(throttle), "brake": float(brake), "steer": float(steer)}

    @property
    def manual(self) -> bool:
        return self._manual

    # ── 显示：RGB 摄像头 ──
    def _attach_camera(self) -> None:
        carla = self.carla
        world = self.link.world
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", "640")
        bp.set_attribute("image_size_y", "360")
        bp.set_attribute("fov", "90")
        tf = carla.Transform(carla.Location(x=-6.0, z=3.5), carla.Rotation(pitch=-12.0))
        self._camera = world.spawn_actor(bp, tf, attach_to=self.link.ego)
        self._camera.listen(self._on_camera)

    def _on_camera(self, image) -> None:
        # 限频（~10Hz）。优先 numpy+PIL 在内存编码 JPEG（绕开中文路径问题）。
        now = time.monotonic()
        if now - self._cam_last_save < 0.1:
            return
        self._cam_last_save = now
        try:
            if _CAM_ENCODER == "pil":
                arr = _np.frombuffer(image.raw_data, dtype=_np.uint8)
                arr = arr.reshape((image.height, image.width, 4))
                rgb = arr[:, :, :3][:, :, ::-1]   # BGRA → RGB
                import io
                buf = io.BytesIO()
                _PILImage.fromarray(rgb).save(buf, format="JPEG", quality=70)
                with self._cam_lock:
                    self._cam_jpeg = buf.getvalue()
            else:
                image.save_to_disk(self._cam_path)
        except Exception:
            pass

    def camera_jpeg(self) -> Optional[bytes]:
        with self._cam_lock:
            return self._cam_jpeg

    def camera_path(self) -> Optional[str]:
        return self._cam_path if os.path.isfile(self._cam_path) else None

    # ── 碰撞检测（CARLA 碰撞传感器，准确替代 gap 阈值估计）──
    def _attach_collision(self) -> None:
        world = self.link.world
        bp = world.get_blueprint_library().find("sensor.other.collision")
        self._collision = world.spawn_actor(bp, self.carla.Transform(), attach_to=self.link.ego)
        self._collided = False
        self._collision.listen(self._on_collision)

    def _on_collision(self, _event) -> None:
        self._collided = True

    def collision_occurred(self) -> bool:
        return self._collided

    # ── 清理 ──
    def reset(self) -> None:
        """有序拆除，避免「同步模式 + actor 销毁」的 CARLA 客户端崩溃（C++ 段错误无法 try 捕获，
        只能靠顺序规避）：① TM 退出同步 → ② 世界切回 async → ③ 停摄像头 → ④ 销毁 NPC → ⑤ 关 link。"""
        if self._tm is not None:
            try:
                self._tm.set_synchronous_mode(False)
            except Exception:
                pass
            self._tm = None
        if self.link is not None:
            try:
                s = self.link.world.get_settings()
                s.synchronous_mode = False
                s.fixed_delta_seconds = None
                self.link.world.apply_settings(s)
            except Exception:
                pass
        for sensor_attr in ("_camera", "_collision"):
            sensor = getattr(self, sensor_attr)
            if sensor is not None:
                try:
                    sensor.stop()
                except Exception:
                    pass
                try:
                    sensor.destroy()
                except Exception:
                    pass
                setattr(self, sensor_attr, None)
        self._collided = False
        self.clear_npc()
        if self.link is not None:
            try:
                self.link.close()
            except Exception:
                pass
            self.link = None
        self._cam_jpeg = None
        self._lead_speed_override = None
        self._manual = False

    def close(self) -> None:
        self.reset()
