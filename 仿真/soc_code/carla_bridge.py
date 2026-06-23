#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CARLA bridge for the SOC ADAS control kernel.

This script runs the existing pure ADAS pipeline against a live CARLA server:

    CARLA world state -> VehicleSignals -> run_pure_pipeline()
    -> carla.VehicleControl -> ego vehicle

It intentionally stays outside the ROS/serial/ESP32 runtime.  Use it on the
Windows CARLA machine with the CARLA Python 3.12 wheel installed.
"""

from __future__ import print_function

import argparse
import math
import os
import random
import sys
import time


def _import_carla():
    try:
        import carla
        return carla
    except ImportError:
        raise SystemExit(
            '无法导入 carla 模块。请先安装本机 CARLA wheel，例如：\n'
            '  py -3.12 -m pip install '
            '"D:\\Code\\自动辅助驾驶仿真平台\\CALRA\\PythonAPI\\carla\\dist\\'
            'carla-0.9.16-cp312-cp312-win_amd64.whl"\n'
            '然后用同一个 Python 3.12 运行本脚本。'
        )


def _load_adas_modules(loop_hz):
    # config.py 在 import 时读取 LOOP_HZ，因此必须先写环境变量。
    os.environ['LOOP_HZ'] = str(int(loop_hz))

    from config import (  # noqa: WPS433 - loaded after LOOP_HZ is set
        ACTOR_CLASS_VEHICLE,
        LANE_DEFAULT_WIDTH,
        LON_CMD_MAX_BRAKE_DECEL,
        LON_CMD_MAX_DRIVE_ACCEL,
        MAX_DELTA,
    )
    from pipeline import run_pure_pipeline  # noqa: WPS433
    from replay import build_stack  # noqa: WPS433

    return {
        'ACTOR_CLASS_VEHICLE': ACTOR_CLASS_VEHICLE,
        'LANE_DEFAULT_WIDTH': LANE_DEFAULT_WIDTH,
        'LON_CMD_MAX_BRAKE_DECEL': LON_CMD_MAX_BRAKE_DECEL,
        'LON_CMD_MAX_DRIVE_ACCEL': LON_CMD_MAX_DRIVE_ACCEL,
        'MAX_DELTA': MAX_DELTA,
        'run_pure_pipeline': run_pure_pipeline,
        'build_stack': build_stack,
    }


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _yaw_rad(rotation):
    return math.radians(float(rotation.yaw))


def _speed_mps(actor):
    v = actor.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def _relative_xy(ref_transform, target_transform):
    """Return target position in ref frame as (forward_m, right_m)."""
    dx = target_transform.location.x - ref_transform.location.x
    dy = target_transform.location.y - ref_transform.location.y
    yaw = _yaw_rad(ref_transform.rotation)
    forward = math.cos(yaw) * dx + math.sin(yaw) * dy
    right = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return forward, right


def _forward_location(carla, transform, behind_m=8.0, z_m=4.0):
    yaw = _yaw_rad(transform.rotation)
    return carla.Location(
        x=transform.location.x - math.cos(yaw) * behind_m,
        y=transform.location.y - math.sin(yaw) * behind_m,
        z=transform.location.z + z_m,
    )


class CarlaAdasVehicleSystem(object):
    """CARLA vehicle system driven by the existing ADAS pipeline."""

    def __init__(self, carla, args, adas):
        self.carla = carla
        self.args = args
        self.adas = adas

        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(args.timeout)

        if args.map:
            self.world = self.client.load_world(args.map)
        else:
            self.world = self.client.get_world()

        self.map = self.world.get_map()
        self.traffic_manager = self.client.get_trafficmanager(args.tm_port)
        self.original_settings = self.world.get_settings()

        self.ego = None
        self.lead = None
        self.spawned = []
        self.last_lead_forward = None
        self.last_log_t = 0.0

        self.signals, self.memory, self.managers = adas['build_stack']()

    def setup(self):
        if self.args.seed is not None:
            random.seed(self.args.seed)
            try:
                self.traffic_manager.set_random_device_seed(self.args.seed)
            except RuntimeError:
                pass

        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.args.fixed_delta
        settings.no_rendering_mode = bool(self.args.no_rendering)
        self.world.apply_settings(settings)
        self.traffic_manager.set_synchronous_mode(True)

        self.ego = self._spawn_ego()
        if not self.args.no_lead:
            self.lead = self._spawn_lead(self.ego.get_transform())

        # Let physics and Traffic Manager settle for a few frames.
        for _ in range(5):
            self.world.tick()

    def run(self):
        start_wall = time.time()
        start_sim = None
        frame = 0
        print('CARLA ADAS bridge started. Press Ctrl+C to stop.')

        while True:
            self.world.tick()
            snapshot = self.world.get_snapshot()
            now = float(snapshot.timestamp.elapsed_seconds)
            if start_sim is None:
                start_sim = now
            if self.args.duration > 0.0 and (now - start_sim) >= self.args.duration:
                break

            self._update_signals(now)
            result = self.adas['run_pure_pipeline'](
                now, self.signals, self.memory, self.managers, None,
            )
            control = self._to_vehicle_control(result)
            self.ego.apply_control(control)
            self.memory.cycle_count += 1

            if self.args.follow_spectator:
                self._update_spectator()
            if self.args.log_every > 0 and frame % self.args.log_every == 0:
                self._log_status(now, result, control, start_wall)
            frame += 1

    def close(self):
        try:
            if self.ego is not None:
                self.ego.apply_control(self.carla.VehicleControl(
                    throttle=0.0,
                    brake=1.0,
                    steer=0.0,
                ))
        except RuntimeError:
            pass

        if not self.args.keep_actors:
            for actor in reversed(self.spawned):
                try:
                    actor.destroy()
                except RuntimeError:
                    pass

        try:
            self.traffic_manager.set_synchronous_mode(False)
        except RuntimeError:
            pass
        try:
            self.world.apply_settings(self.original_settings)
        except RuntimeError:
            pass

    def _pick_blueprint(self, filter_pattern, role_name):
        library = self.world.get_blueprint_library()
        blueprints = list(library.filter(filter_pattern))
        if not blueprints and filter_pattern != 'vehicle.*':
            blueprints = list(library.filter('vehicle.*'))
        if not blueprints:
            raise RuntimeError('No vehicle blueprints found for %s' % filter_pattern)

        blueprints = sorted(blueprints, key=lambda bp: bp.id)
        bp = random.choice(blueprints) if self.args.random_blueprint else blueprints[0]
        if bp.has_attribute('role_name'):
            bp.set_attribute('role_name', role_name)
        if bp.has_attribute('color'):
            values = bp.get_attribute('color').recommended_values
            if values:
                bp.set_attribute('color', random.choice(values))
        return bp

    def _spawn_ego(self):
        spawn_points = self.map.get_spawn_points()
        if not spawn_points:
            raise RuntimeError('Current CARLA map has no spawn points')

        bp = self._pick_blueprint(self.args.ego_filter, 'hero')
        count = len(spawn_points)
        for offset in range(count):
            transform = spawn_points[(self.args.spawn_index + offset) % count]
            actor = self.world.try_spawn_actor(bp, transform)
            if actor is not None:
                actor.set_autopilot(False)
                self.spawned.append(actor)
                print('spawned ego: id=%s type=%s' % (actor.id, actor.type_id))
                return actor
        raise RuntimeError('Could not spawn ego vehicle')

    def _spawn_lead(self, ego_transform):
        bp = self._pick_blueprint(self.args.lead_filter, 'lead')
        transform = self._lead_spawn_transform(ego_transform)
        actor = self.world.try_spawn_actor(bp, transform)

        if actor is None:
            spawn_points = self.map.get_spawn_points()
            for transform in spawn_points:
                actor = self.world.try_spawn_actor(bp, transform)
                if actor is not None:
                    break
        if actor is None:
            raise RuntimeError('Could not spawn lead vehicle')

        self._set_autopilot(actor, True)
        try:
            self.traffic_manager.auto_lane_change(actor, False)
            self.traffic_manager.distance_to_leading_vehicle(actor, 8.0)
            self.traffic_manager.vehicle_percentage_speed_difference(
                actor, self.args.lead_speed_diff,
            )
        except RuntimeError:
            pass
        self.spawned.append(actor)
        print('spawned lead: id=%s type=%s' % (actor.id, actor.type_id))
        return actor

    def _lead_spawn_transform(self, ego_transform):
        waypoint = self.map.get_waypoint(
            ego_transform.location,
            project_to_road=True,
            lane_type=self.carla.LaneType.Driving,
        )
        next_points = waypoint.next(float(self.args.lead_gap)) if waypoint else []
        if next_points:
            transform = next_points[0].transform
        else:
            transform = ego_transform
            transform.location.x += self.args.lead_gap
        transform.location.z += 0.6
        return transform

    def _set_autopilot(self, actor, enabled):
        try:
            actor.set_autopilot(enabled, self.args.tm_port)
        except TypeError:
            actor.set_autopilot(enabled)

    def _update_signals(self, now):
        ego_tf = self.ego.get_transform()
        ego_wp = self.map.get_waypoint(
            ego_tf.location,
            project_to_road=True,
            lane_type=self.carla.LaneType.Driving,
        )

        ego_yaw = _yaw_rad(ego_tf.rotation)
        ego_v = _speed_mps(self.ego)
        road_yaw = ego_yaw
        lane_offset = 0.0

        if ego_wp is not None:
            road_tf = ego_wp.transform
            road_yaw = _yaw_rad(road_tf.rotation)
            _, lane_right = _relative_xy(road_tf, ego_tf)
            lane_offset = lane_right * self.args.lane_offset_sign

        self.signals.ego_x = float(ego_tf.location.x)
        self.signals.ego_y = float(ego_tf.location.y)
        self.signals.ego_yaw = ego_yaw
        self.signals.ego_v = ego_v
        self.signals.ego_received = True
        self.signals.ego_psi_received = True
        self.signals.ego_last_rx = now

        self.signals.road_psi = road_yaw
        self.signals.road_received = True
        self.signals.road_last_rx = now

        self.signals.lane_offset = lane_offset
        self.signals.lane_offset_received = True
        self.signals.lane_offset_last_rx = now

        lead = self._find_front_vehicle()
        if lead is None:
            self.signals.lead_received = False
            self.last_lead_forward = None
            return

        actor, forward_m, right_m = lead
        lead_tf = actor.get_transform()
        self.signals.lead_x = float(lead_tf.location.x)
        self.signals.lead_y = float(lead_tf.location.y)
        self.signals.lead_yaw = _yaw_rad(lead_tf.rotation)
        self.signals.lead_v = _speed_mps(actor)
        self.signals.lead_cls = self.adas['ACTOR_CLASS_VEHICLE']
        self.signals.lead_received = True
        self.signals.lead_last_rx_time = now
        self.signals.lead_v_last_rx_time = now
        self.signals.lead_cls_last_rx_time = now
        self.last_lead_forward = forward_m

    def _find_front_vehicle(self):
        ego_tf = self.ego.get_transform()
        actors = self.world.get_actors().filter('vehicle.*')
        best = None
        best_forward = float('inf')
        lateral_window = self.args.detect_lateral_window
        if lateral_window <= 0.0:
            lateral_window = self.adas['LANE_DEFAULT_WIDTH'] * 0.75

        for actor in actors:
            if actor.id == self.ego.id:
                continue
            forward_m, right_m = _relative_xy(ego_tf, actor.get_transform())
            if forward_m <= 0.5 or forward_m > self.args.detect_range:
                continue
            if abs(right_m) > lateral_window:
                continue
            if forward_m < best_forward:
                best = (actor, forward_m, right_m)
                best_forward = forward_m
        return best

    def _to_vehicle_control(self, result):
        lon_cmd = _clamp(
            float(result.lon_cmd),
            -self.adas['LON_CMD_MAX_DRIVE_ACCEL'],
            self.adas['LON_CMD_MAX_BRAKE_DECEL'],
        )
        delta = _clamp(
            float(result.lateral_ctx.delta),
            -self.adas['MAX_DELTA'],
            self.adas['MAX_DELTA'],
        )

        if lon_cmd >= 0.0:
            throttle = 0.0
            brake = _clamp(
                (lon_cmd / max(self.adas['LON_CMD_MAX_BRAKE_DECEL'], 0.1))
                * self.args.brake_gain,
                0.0,
                1.0,
            )
        else:
            throttle = _clamp(
                (-lon_cmd / max(self.adas['LON_CMD_MAX_DRIVE_ACCEL'], 0.1))
                * self.args.throttle_gain,
                0.0,
                1.0,
            )
            brake = 0.0

        steer = _clamp(
            self.args.steer_sign * delta / max(self.adas['MAX_DELTA'], 1e-6),
            -1.0,
            1.0,
        )
        return self.carla.VehicleControl(
            throttle=float(throttle),
            brake=float(brake),
            steer=float(steer),
            hand_brake=False,
            reverse=False,
        )

    def _update_spectator(self):
        ego_tf = self.ego.get_transform()
        spectator = self.world.get_spectator()
        spectator.set_transform(self.carla.Transform(
            _forward_location(self.carla, ego_tf),
            self.carla.Rotation(pitch=-18.0, yaw=ego_tf.rotation.yaw, roll=0.0),
        ))

    def _log_status(self, now, result, control, start_wall):
        lead_gap = 'none'
        if self.signals.lead_received and self.last_lead_forward is not None:
            lead_gap = '%.1fm' % self.last_lead_forward
        print(
            '[%.2fs sim / %.1fs wall] v=%.2fm/s lead=%s off=%.2fm '
            'steer=%.2f throttle=%.2f brake=%.2f aeb=%s'
            % (
                now,
                time.time() - start_wall,
                self.signals.ego_v,
                lead_gap,
                self.signals.lane_offset,
                control.steer,
                control.throttle,
                control.brake,
                bool(result.lon_ctx.aeb_active),
            )
        )


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description='Run lx/SOCCode ADAS control kernel in CARLA.',
    )
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('-p', '--port', default=2000, type=int)
    parser.add_argument('--timeout', default=10.0, type=float)
    parser.add_argument('--tm-port', default=8000, type=int)
    parser.add_argument('--map', default=None, help='CARLA map name, e.g. Town03')

    parser.add_argument(
        '--loop-hz',
        default=int(float(os.environ.get('LOOP_HZ', '20'))),
        type=int,
        help='ADAS/CARLA synchronous tick rate. Default: env LOOP_HZ or 20.',
    )
    parser.add_argument(
        '--fixed-delta',
        default=None,
        type=float,
        help='CARLA fixed delta seconds. Default: 1 / --loop-hz.',
    )
    parser.add_argument('--duration', default=60.0, type=float,
                        help='Simulation seconds. <=0 runs until Ctrl+C.')
    parser.add_argument('--no-rendering', action='store_true')

    parser.add_argument('--spawn-index', default=0, type=int)
    parser.add_argument('--ego-filter', default='vehicle.tesla.model3')
    parser.add_argument('--lead-filter', default='vehicle.audi.tt')
    parser.add_argument('--random-blueprint', action='store_true')
    parser.add_argument('--no-lead', action='store_true')
    parser.add_argument('--lead-gap', default=35.0, type=float)
    parser.add_argument(
        '--lead-speed-diff',
        default=25.0,
        type=float,
        help='Traffic Manager speed difference percentage; positive is slower.',
    )
    parser.add_argument('--detect-range', default=90.0, type=float)
    parser.add_argument(
        '--detect-lateral-window',
        default=4.5,
        type=float,
        help='Front-vehicle lateral search window in meters.',
    )

    parser.add_argument('--steer-sign', default=1.0, type=float)
    parser.add_argument('--lane-offset-sign', default=1.0, type=float)
    parser.add_argument('--throttle-gain', default=1.0, type=float)
    parser.add_argument('--brake-gain', default=1.0, type=float)

    parser.add_argument('--follow-spectator', action='store_true', default=True)
    parser.add_argument('--no-follow-spectator', dest='follow_spectator',
                        action='store_false')
    parser.add_argument('--keep-actors', action='store_true')
    parser.add_argument('--log-every', default=20, type=int)
    parser.add_argument('--seed', default=42, type=int)
    return parser


def main(argv=None):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    if args.loop_hz < 1:
        raise SystemExit('--loop-hz must be positive')
    if args.fixed_delta is None:
        args.fixed_delta = 1.0 / float(args.loop_hz)

    carla = _import_carla()
    adas = _load_adas_modules(args.loop_hz)

    system = CarlaAdasVehicleSystem(carla, args, adas)
    try:
        system.setup()
        system.run()
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        system.close()


if __name__ == '__main__':
    main(sys.argv[1:])
