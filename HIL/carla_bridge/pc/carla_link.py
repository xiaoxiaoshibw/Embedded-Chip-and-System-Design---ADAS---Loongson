#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CARLA 世界端：连接/生成车辆/感知提取/执行器映射/前车脚本。

坐标约定见 bridge_config.py：控制器运动学在 CARLA 左手系下自洽，
yaw(rad) 直接作 psi，lane_offset 右正，steer 与 delta 同号。

两个桥接层处理：
1. 角度连续性：CARLA yaw 在 ±180° 回绕，控制器对 road_psi 做低通+差分
   （曲率），对跳变敏感 → 维护展开（unwrap）后的连续角；
2. 参考线跟踪：map.get_waypoint 会吸附"最近车道"，超车变道跨线瞬间
   lane_offset 突跳 ±一个车道宽 → 这里沿自车初始车道维护一条连续参考
   中心线（按纵向投影推进，不随横向位置吸附），road_psi / lane_offset
   均相对参考线计算，超车横向跟踪因此连续。
"""

import math

from bridge_config import (
    ACCEL_DEADBAND,
    BRAKE_DECEL_GAIN,
    CARLA_TIMEOUT_S,
    DRAG_COMP,
    EGO_BLUEPRINT,
    FIXED_DT,
    LEAD_BLUEPRINT,
    STEER_MODE,
    STEER_SIGN,
    LANE_OFFSET_SIGN,
    SPAWN_WAYPOINT_MAX_DIST_M,
    START_THROTTLE_MIN,
    START_THROTTLE_SPEED_MPS,
    THROTTLE_ACCEL_GAIN,
)
from scenarios import lead_in_hard_brake, lead_target_speed


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _speed(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def _xy_distance(a, b):
    return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))


class _AngleUnwrapper:
    """把回绕角序列展开为连续角。"""

    def __init__(self):
        self._acc = None

    def update(self, raw):
        if self._acc is None:
            self._acc = raw
            return raw
        d = raw - self._acc
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        self._acc += d
        return self._acc


class CarlaLink:
    def __init__(self, carla, host, port, scenario, town=None,
                 no_rendering=False, spawn_index=None):
        self.carla = carla
        self.scenario = scenario
        self.client = carla.Client(host, port)
        self.client.set_timeout(CARLA_TIMEOUT_S)
        cur = self.client.get_world()
        if town and town not in cur.get_map().name:
            print('loading map %s ...' % town, flush=True)
            self.world = self.client.load_world(town)
        else:
            self.world = cur
        self.map = self.world.get_map()
        self.original_settings = self.world.get_settings()

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DT
        settings.no_rendering_mode = bool(no_rendering)
        self.world.apply_settings(settings)

        if spawn_index is None:
            spawn_index = int(scenario.get('spawn_index', 0))

        self.spawned = []
        self._cleanup_previous_hil_actors()
        self.ego = self._spawn(EGO_BLUEPRINT, 'hero', spawn_index)
        self.world.tick()

        self.lead = None
        lead_cfg = scenario.get('lead')
        if lead_cfg:
            ego_wp = self.map.get_waypoint(self.ego.get_transform().location)
            gap0 = float(lead_cfg.get('gap0', 40.0))
            lead_wps = ego_wp.next(gap0)
            if not lead_wps:
                raise RuntimeError('无法在前方 %.0fm 找到前车生成点' % gap0)
            self.lead = self._spawn_at(LEAD_BLUEPRINT, 'lead',
                                       lead_wps[0].transform)

        phys = self.ego.get_physics_control()
        self.max_steer_rad = math.radians(
            max(w.max_steer_angle for w in phys.wheels) or 70.0)

        self._ego_yaw_unwrap = _AngleUnwrapper()
        self._road_psi_unwrap = _AngleUnwrapper()
        self._lead_yaw_unwrap = _AngleUnwrapper()
        self._lead_v_int = 0.0
        self.sim_t0 = None

        # 参考中心线：锚定自车初始车道，按纵向投影推进
        self._ref_wp = self._driving_waypoint(
            self.ego.get_transform().location, project_to_road=False)
        if self._ref_wp is None:
            self._ref_wp = self._driving_waypoint(
                self.ego.get_transform().location, project_to_road=True)
        if self._ref_wp is None:
            raise RuntimeError('自车不在可驾驶车道附近，无法初始化参考线')

        for _ in range(5):
            self.world.tick()

    def _driving_waypoint(self, location, project_to_road):
        try:
            return self.map.get_waypoint(
                location, project_to_road=project_to_road,
                lane_type=self.carla.LaneType.Driving)
        except RuntimeError:
            return None

    def _aligned_spawn_transform(self, transform):
        wp = self._driving_waypoint(transform.location, project_to_road=False)
        if wp is None:
            wp = self._driving_waypoint(transform.location, project_to_road=True)
        if wp is None:
            return None
        if _xy_distance(transform.location, wp.transform.location) > SPAWN_WAYPOINT_MAX_DIST_M:
            return None

        loc = self.carla.Location(
            x=float(wp.transform.location.x),
            y=float(wp.transform.location.y),
            z=float(transform.location.z))
        rot = self.carla.Rotation(
            pitch=float(transform.rotation.pitch),
            yaw=float(wp.transform.rotation.yaw),
            roll=float(transform.rotation.roll))
        return self.carla.Transform(loc, rot)

    def _spawn(self, bp_name, role, spawn_index):
        bp = self.world.get_blueprint_library().find(bp_name)
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', role)
        points = self.map.get_spawn_points()
        if not points:
            raise RuntimeError('地图没有生成点')
        n = len(points)
        for off in range(n):
            idx = (spawn_index + off) % n
            transform = self._aligned_spawn_transform(points[idx])
            if transform is None:
                continue
            actor = self.world.try_spawn_actor(bp, transform)
            if actor is not None:
                if off != 0:
                    print('spawn_index %d 不可用，已顺延到 %d'
                          % (spawn_index, idx), flush=True)
                self.spawned.append(actor)
                return actor
        raise RuntimeError('无法生成 %s' % bp_name)

    def _cleanup_previous_hil_actors(self):
        stale = []
        for actor in self.world.get_actors().filter('vehicle.*'):
            role = actor.attributes.get('role_name', '')
            if role in ('hero', 'lead'):
                stale.append(actor)
        if stale:
            print('cleaning %d stale HIL vehicle(s)' % len(stale), flush=True)
            for actor in stale:
                try:
                    actor.destroy()
                except RuntimeError:
                    pass
            # Let CARLA settle the actor registry before trying to respawn.
            for _ in range(2):
                self.world.tick()

    def _spawn_at(self, bp_name, role, transform):
        bp = self.world.get_blueprint_library().find(bp_name)
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', role)
        transform.location.z += 0.5
        actor = self.world.try_spawn_actor(bp, transform)
        if actor is None:
            raise RuntimeError('无法生成前车 %s' % bp_name)
        self.spawned.append(actor)
        return actor

    # ── 仿真推进 ──
    def tick(self):
        self.world.tick()
        t = float(self.world.get_snapshot().timestamp.elapsed_seconds)
        if self.sim_t0 is None:
            self.sim_t0 = t
        return t - self.sim_t0

    # ── 参考线推进：沿初始车道中心线按纵向投影前移 ──
    def _advance_ref(self, ego_loc):
        wp = self._ref_wp
        for _ in range(6):
            f = wp.transform.get_forward_vector()
            dx = ego_loc.x - wp.transform.location.x
            dy = ego_loc.y - wp.transform.location.y
            s = f.x * dx + f.y * dy
            if s <= 0.3:
                break
            nxts = wp.next(min(max(s, 0.5), 5.0))
            if not nxts:
                break
            # 分叉时取离自车最近的分支
            wp = min(nxts, key=lambda w:
                     (w.transform.location.x - ego_loc.x) ** 2
                     + (w.transform.location.y - ego_loc.y) ** 2)
        if _xy_distance(ego_loc, wp.transform.location) > 20.0:
            nearest = self._driving_waypoint(ego_loc, project_to_road=True)
            if nearest is not None:
                wp = nearest
        self._ref_wp = wp
        return wp

    # ── 感知提取（真值 → 感知帧 dict）──
    def sense(self, sim_t):
        ego_tf = self.ego.get_transform()
        ref_wp = self._advance_ref(ego_tf.location)

        ego_yaw = self._ego_yaw_unwrap.update(
            math.radians(float(ego_tf.rotation.yaw)))
        ego_v = _speed(self.ego)

        road_yaw_raw = math.radians(float(ref_wp.transform.rotation.yaw))
        road_psi = self._road_psi_unwrap.update(road_yaw_raw)
        # lane_offset：自车相对参考线，右正（CARLA 系）
        dx = ego_tf.location.x - ref_wp.transform.location.x
        dy = ego_tf.location.y - ref_wp.transform.location.y
        c, s = math.cos(road_yaw_raw), math.sin(road_yaw_raw)
        lane_offset = LANE_OFFSET_SIGN * (-s * dx + c * dy)

        frame = {
            't': sim_t,
            'ego_x': float(ego_tf.location.x),
            'ego_y': float(ego_tf.location.y),
            'ego_yaw': ego_yaw,
            'ego_v': ego_v,
            'road_psi': road_psi,
            'lane_offset': lane_offset,
            'lead_present': False,
        }
        forward = float('inf')
        if self.lead is not None:
            lead_tf = self.lead.get_transform()
            ldx = lead_tf.location.x - ego_tf.location.x
            ldy = lead_tf.location.y - ego_tf.location.y
            forward = math.cos(ego_yaw) * ldx + math.sin(ego_yaw) * ldy
            if 0.5 < forward < 120.0:
                frame.update({
                    'lead_present': True,
                    'lead_x': float(lead_tf.location.x),
                    'lead_y': float(lead_tf.location.y),
                    'lead_yaw': self._lead_yaw_unwrap.update(
                        math.radians(float(lead_tf.rotation.yaw))),
                    'lead_v': _speed(self.lead),
                    'lead_cls': 1,
                })
        return frame, forward

    # ── 执行（虚拟 ESP32 输出 → 自车）──
    def apply_ego(self, esp32_out):
        delta = float(esp32_out['delta'])
        a_brake = float(esp32_out['a_brake'])   # 正=减速
        a_des = -a_brake
        v = _speed(self.ego)

        if STEER_MODE == 'physical':
            steer = STEER_SIGN * delta / max(self.max_steer_rad, 1e-6)
        else:
            steer = STEER_SIGN * delta / 0.4363   # MAX_DELTA=25°
        steer = _clamp(steer, -1.0, 1.0)

        throttle = 0.0
        brake = 0.0
        if a_des > ACCEL_DEADBAND:
            throttle = _clamp((a_des + DRAG_COMP * v) / THROTTLE_ACCEL_GAIN,
                              0.0, 1.0)
            if v < START_THROTTLE_SPEED_MPS:
                throttle = max(throttle, START_THROTTLE_MIN)
        elif a_des < -ACCEL_DEADBAND:
            brake = _clamp(-a_des / BRAKE_DECEL_GAIN, 0.0, 1.0)
        else:
            throttle = _clamp(DRAG_COMP * v / THROTTLE_ACCEL_GAIN, 0.0, 0.3)

        self.ego.apply_control(self.carla.VehicleControl(
            throttle=float(throttle), brake=float(brake), steer=float(steer)))
        return steer, throttle, brake

    # ── 前车脚本控制 ──
    def drive_lead(self, sim_t):
        if self.lead is None:
            return
        lead_cfg = self.scenario.get('lead')
        if lead_in_hard_brake(lead_cfg, sim_t):
            self.lead.apply_control(self.carla.VehicleControl(
                throttle=0.0, brake=1.0, steer=self._lead_lane_keep()))
            return
        v_tgt = lead_target_speed(lead_cfg, sim_t)
        v = _speed(self.lead)
        err = v_tgt - v
        self._lead_v_int = _clamp(self._lead_v_int + err * FIXED_DT, -5.0, 5.0)
        a_cmd = 0.8 * err + 0.1 * self._lead_v_int
        throttle = _clamp(a_cmd / 3.0 + DRAG_COMP * v / 3.0, 0.0, 0.8)
        brake = _clamp(-a_cmd / 6.0, 0.0, 1.0) if a_cmd < -0.2 else 0.0
        self.lead.apply_control(self.carla.VehicleControl(
            throttle=float(throttle), brake=float(brake),
            steer=self._lead_lane_keep()))

    def _lead_lane_keep(self):
        """前车简易车道保持（P 控制航向+横向偏差）。"""
        tf = self.lead.get_transform()
        wp = self.map.get_waypoint(tf.location, project_to_road=True,
                                   lane_type=self.carla.LaneType.Driving)
        nxt = wp.next(6.0)
        if not nxt:
            return 0.0
        tgt = nxt[0].transform
        yaw = math.radians(float(tf.rotation.yaw))
        tgt_yaw = math.radians(float(tgt.rotation.yaw))
        he = math.atan2(math.sin(tgt_yaw - yaw), math.cos(tgt_yaw - yaw))
        dx = tf.location.x - wp.transform.location.x
        dy = tf.location.y - wp.transform.location.y
        ryaw = math.radians(float(wp.transform.rotation.yaw))
        lat = -math.sin(ryaw) * dx + math.cos(ryaw) * dy
        return _clamp(1.2 * he - 0.15 * lat, -0.5, 0.5)

    # ── 旁观者跟车视角 ──
    def update_spectator(self):
        tf = self.ego.get_transform()
        yaw = math.radians(float(tf.rotation.yaw))
        loc = self.carla.Location(
            x=tf.location.x - math.cos(yaw) * 9.0,
            y=tf.location.y - math.sin(yaw) * 9.0,
            z=tf.location.z + 4.5)
        self.world.get_spectator().set_transform(self.carla.Transform(
            loc, self.carla.Rotation(pitch=-16.0, yaw=tf.rotation.yaw)))

    def close(self):
        try:
            self.ego.apply_control(self.carla.VehicleControl(brake=1.0))
        except Exception:
            pass
        for actor in reversed(self.spawned):
            try:
                actor.destroy()
            except Exception:
                pass
        try:
            self.world.apply_settings(self.original_settings)
        except Exception:
            pass
