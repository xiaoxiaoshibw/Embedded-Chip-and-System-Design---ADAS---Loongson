#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""采集对比绘图所需的真实逐帧数据（headless CARLA）。

产出 CSV 到 成果/数据/：
  overtake_baseline.csv  无创新（无接近调速器、无ML） → 复现 ACC↔AEB"加速—急刹"打架
  overtake_ours.csv      我方系统（接近调速器+仲裁+ML） → 平滑驶近、停稳、超车
  cutin_ours.csv         加塞避让（含 ML 风险概率逐帧）
  pedestrian_ours.csv    行人横穿制动

用法（CARLA 需先 -RenderOffScreen 启动）：
  python capture_plot_data.py
注：无显卡机 CARLA 每次 load_world 后易崩，本脚本一次 load_world 跑完 4 段。
"""
import os, sys, csv, math

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
_ADAS = os.path.join(_ROOT, 'lx', '_4070_clean', 'ADAS_Central', 'adas_central')
sys.path.insert(0, _ADAS)
_OUT = os.path.join(_ROOT, '成果', '数据')

import carla
import config as cfg
import longitudinal_control as lonmod
from perception import PerceptionLayer
from lateral_control import LateralController
from longitudinal_control import LongitudinalController
from overtake import OvertakeManager, OvertakeState
from scenarios import ScenarioManager
from arbitration import arbitrate_longitudinal
from ml_assist import MlAssist


def _apply(ego, lon_cmd, delta, v):
    c = carla.VehicleControl()
    if lon_cmd > cfg.ACCEL_DEADBAND:
        c.brake = min(1.0, lon_cmd / cfg.BRAKE_DECEL_GAIN)
    elif lon_cmd < -cfg.ACCEL_DEADBAND:
        c.throttle = min(1.0, (abs(lon_cmd) + cfg.DRAG_COMP * v) / cfg.THROTTLE_ACCEL_GAIN)
    else:
        c.throttle = cfg.DRAG_COMP * v
    c.steer = max(-1.0, min(1.0, (delta / cfg.MAX_DELTA) * cfg.STEER_SIGN))
    ego.apply_control(c)
    return c


def run(world, sp, scen, csv_name, dur, governor_on=True, ml_on=True, gap=None,
        overtake_enabled=True):
    # 创新开关（同时切换两项防打架机制）：
    #   接近调速器 ACC_APPROACH_DECEL：极大=关闭车距封顶
    #   受控接近门控 AEB_REQUIRED_DECEL_TRIGGER：0=关闭(老 TTC-AEB 直接打架)
    bak = (lonmod.ACC_APPROACH_DECEL, lonmod.AEB_REQUIRED_DECEL_TRIGGER)
    lonmod.ACC_APPROACH_DECEL = 1.2 if governor_on else 1e9
    lonmod.AEB_REQUIRED_DECEL_TRIGGER = 3.5 if governor_on else 0.0
    if gap is not None:
        import scenarios as scn
        scn.OVT_LEAD_GAP_M = gap

    print('[run %s] governor_on=%s APPROACH_DECEL=%.3g AEB_TRIGGER=%.3g overtake=%s' % (
        csv_name, governor_on, lonmod.ACC_APPROACH_DECEL,
        lonmod.AEB_REQUIRED_DECEL_TRIGGER, overtake_enabled), flush=True)
    bp = world.get_blueprint_library().find('vehicle.tesla.model3')
    bp.set_attribute('role_name', 'ego')
    ego = world.spawn_actor(bp, sp)
    for _ in range(5):
        world.tick()

    scenario = ScenarioManager(carla, world, ego, scen)
    perception = PerceptionLayer(world, ego, scenario.lead_actor)
    lateral = LateralController(); longitudinal = LongitudinalController()
    overtake = OvertakeManager(); ml = MlAssist(enabled=ml_on)

    rows = []
    n = int(dur / cfg.FIXED_DT)
    for i in range(n):
        world.tick(); t = (i + 1) * cfg.FIXED_DT
        frame = perception.sense(t)
        scenario.update(t, frame.ego_v)
        if overtake_enabled:
            ovt = overtake.update(t, frame.ego_v, frame.lead,
                                  frame.ego_x, frame.ego_y, frame.ego_yaw)
        else:
            ovt = {'active': False, 'target_lane_offset': 0.0, 'cruise_v': 0.0}
        ped_warn = any(p.ttc < 4.0 and p.crossing for p in frame.pedestrians)
        eff_off = frame.lane_offset - (ovt['target_lane_offset'] if ovt['active'] else 0)
        delta, bnd = lateral.compute(frame.ego_v, frame.ego_yaw, frame.road_psi,
                                     frame.road_curvature, eff_off, frame.lane_width)
        lon, rule_aeb, mode = longitudinal.compute(
            t, frame.ego_v, frame.lead, frame.road_curvature)

        ped_brake = 0.0
        if ped_warn:
            for p in frame.pedestrians:
                if p.ttc < 2.5: ped_brake = max(ped_brake, 5.0)
                elif p.ttc < 4.0: ped_brake = max(ped_brake, 2.0)

        ml_res = ml.update_and_predict(frame.ego_v, frame.lead)
        ml_risk = ml_res['aeb_probs'][1] + ml_res['aeb_probs'][2]
        ml_brake = 0.0
        if ml.available and frame.lead.detected and frame.lead.relative_speed > 0:
            inten = ml_res['brake_intensity']
            if inten > cfg.ML_AEB_INTENSITY_DEADBAND:
                ml_brake = (inten - cfg.ML_AEB_INTENSITY_DEADBAND) / \
                    (1 - cfg.ML_AEB_INTENSITY_DEADBAND) * cfg.ML_MAX_DECEL
            acc_ff = -ml_res['acc_pred'] - cfg.ML_ACC_FF_DEADBAND
            if acc_ff > 0: ml_brake = max(ml_brake, acc_ff)
            ml_brake = max(0.0, min(ml_brake, cfg.ML_MAX_DECEL))

        ovt_shift = ovt['active'] and overtake.state in (
            OvertakeState.SHIFTING_LEFT, OvertakeState.PASSING, OvertakeState.RETURNING)
        ovt_cruise = longitudinal._compute_cruise(frame.ego_v, ovt['cruise_v']) if ovt_shift else lon
        lon, final_aeb, src = arbitrate_longitudinal(
            lon, rule_aeb, overtake_active=ovt_shift, overtake_cruise=ovt_cruise,
            ped_brake=ped_brake, boundary_brake=bnd, ml_brake=ml_brake)
        c = _apply(ego, lon, delta, frame.ego_v)

        gp = frame.lead.distance if frame.lead.detected else -1
        ttc = min(frame.lead.ttc, 99) if frame.lead.detected else 99
        rows.append([round(t, 2), round(frame.ego_v, 3), round(gp, 2), round(ttc, 2),
                     round(eff_off, 3), round(lon, 3), round(c.throttle, 3),
                     round(c.brake, 3), int(bool(rule_aeb)), int(bool(final_aeb)),
                     overtake.state, int(ped_warn), round(ml_risk, 3), round(ml_brake, 3)])

    os.makedirs(_OUT, exist_ok=True)
    path = os.path.join(_OUT, csv_name)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t', 'v', 'gap', 'ttc', 'eff_off', 'lon_cmd', 'throttle',
                    'brake', 'rule_aeb', 'final_aeb', 'ovt', 'ped', 'ml_risk', 'ml_brake'])
        w.writerows(rows)
    print('saved %s (%d rows)' % (path, len(rows)))

    scenario.close(); ego.destroy()
    for _ in range(3): world.tick()
    lonmod.ACC_APPROACH_DECEL, lonmod.AEB_REQUIRED_DECEL_TRIGGER = bak


def main():
    client = carla.Client('127.0.0.1', 2000); client.set_timeout(60.0)
    world = client.load_world('Town04')
    s = world.get_settings(); s.synchronous_mode = True
    s.fixed_delta_seconds = cfg.FIXED_DT; s.no_rendering_mode = True
    world.apply_settings(s)
    sp = world.get_map().get_spawn_points()[min(cfg.EGO_SPAWN_INDEX, 50)]

    # ACC 接近静止前车（关超车以隔离 ACC/AEB 行为）：基线 vs 我方
    run(world, sp, 'overtake', 'acc_approach_baseline.csv', 24,
        governor_on=False, ml_on=False, gap=26.0, overtake_enabled=False)
    run(world, sp, 'overtake', 'acc_approach_ours.csv', 24,
        governor_on=True, ml_on=True, gap=26.0, overtake_enabled=False)
    # 超车机动（开超车）
    run(world, sp, 'overtake', 'overtake_ours.csv', 28,
        governor_on=True, ml_on=True, gap=20.0, overtake_enabled=True)
    run(world, sp, 'cutin', 'cutin_ours.csv', 24, governor_on=True, ml_on=True)
    run(world, sp, 'pedestrian', 'pedestrian_ours.csv', 22, governor_on=True, ml_on=True)

    s.synchronous_mode = False; world.apply_settings(s)


if __name__ == '__main__':
    main()
