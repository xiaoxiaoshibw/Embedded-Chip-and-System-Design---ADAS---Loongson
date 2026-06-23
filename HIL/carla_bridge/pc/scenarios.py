#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""演示场景库。

每个场景 = 前车行为脚本 + 故障注入时间线 + 时长/出生点 + 讲解要点。
被 cli.py / run_cosim.py 消费；新增场景只需在 SCENARIOS 里加一项。

字段说明：
  duration     仿真时长 (s)
  spawn_index  自车出生点序号（Town04；超车场景需右侧有车道）
  lead         None = 不生成前车（LKA 纯车道保持）；否则:
    gap0         初始车距 (m)
    profile      [(t, 目标速度 m/s)] 分段阶梯
    hard_brake   (t0, t1) 区间内前车直接 brake=1.0（None=无）
  timeline     [(t, action)]，action ∈ kill_primary / restore_primary /
               kill_backup / restore_backup / kill_both / restore_both /
               hang_primary
  notes        展示讲解要点（CLI 运行前打印）
"""

SCENARIOS = {
    'lka': {
        'name': 'LKA 车道保持',
        'duration': 60.0,
        'spawn_index': 30,
        'lead': None,
        'timeline': [],
        'notes': [
            '无前车，自车以系统巡航速度沿高速车道行驶',
            '观察点：弯道中的曲率前馈 + CTE 修正，车辆始终居中',
            '控制链路：CARLA→主控 LKA→虚拟 ESP32 仲裁→转向执行',
        ],
    },
    'acc': {
        'name': 'ACC 自适应巡航',
        'duration': 60.0,
        'spawn_index': 30,
        'lead': {
            'gap0': 24.0,
            # HIL 实机链路下 ESP32 回读后的 CARLA 自车速度约 1~2 m/s；
            # 前车速度按同一尺度设置，避免一开始就跑出 ADAS 的 60m 跟踪范围。
            'profile': [(0.0, 1.2), (15.0, 0.5), (30.0, 1.6), (45.0, 1.0)],
            'hard_brake': None,
        },
        'timeline': [],
        'notes': [
            '前车 7→3.5→8→5 m/s 变速，自车保持安全时距跟随',
            '观察点：跟车距离随速度自适应；前车加速时自车平滑跟进',
            '控制台每秒打印 gap / TTC，可对照安全距离',
        ],
    },
    'aeb': {
        'name': 'AEB 自动紧急制动',
        'duration': 45.0,
        'spawn_index': 30,
        'lead': {
            'gap0': 40.0,
            'profile': [(0.0, 7.0), (15.0, 0.0), (22.0, 6.0)],
            'hard_brake': (15.0, 22.0),
        },
        'timeline': [],
        'notes': [
            't=15s 前车全力急刹，SOC 端 TTC 判定触发 AEB',
            '双保险：若 SOC 帧异常，虚拟 ESP32 的硬件 AEB 地板',
            '  (dist<=hard_floor 全力制动) 仍会兜底——与实车 main.c 一致',
            '前车停驻 7s 后恢复（<8s，不触发超车状态机）',
        ],
    },
    'overtake': {
        'name': '静止前车自动超车',
        'duration': 75.0,
        'spawn_index': 125,
        'lead': {
            'gap0': 32.0,
            'profile': [(0.0, 0.0)],
            'hard_brake': (0.0, 9999.0),   # 前车全程驻车
        },
        'timeline': [],
        'notes': [
            '前方 32m 静止车辆：自车 ACC 减速停在安全距离',
            '前车持续静止 8s（OVT_LEAD_LONG_STILL_S）判定"长期阻塞"',
            '超车状态机 IDLE→WAIT→ACTIVE→PASSING→RETURN：借道-超越-回道',
            '注意：当前符号约定下向【右】借道，出生点需右侧有车道；',
            '  若停在路肩请换 --spawn-index',
        ],
    },
    'failover': {
        'name': '安全无感降级（主备冗余）',
        'duration': 75.0,
        'spawn_index': 30,
        'lead': {
            'gap0': 45.0,
            'profile': [(0.0, 7.0), (10.0, 3.5), (18.0, 7.0),
                        (40.0, 0.0), (47.0, 6.0)],
            'hard_brake': (40.0, 47.0),
        },
        'timeline': [
            (20.0, 'kill_primary'),
            (32.0, 'restore_primary'),
            (55.0, 'kill_both'),
            (62.0, 'restore_both'),
        ],
        'notes': [
            't=20s 杀主控：备机 0.5s 心跳静默判定接管，',
            '  用主机最后一帧种子初始化平滑器 → 控制量连续（无感）',
            '  虚拟 ESP32 仲裁 SRC: PRIMARY→BACKUP（SWITCH:pri_timeout）',
            't=32s 主控重启：仲裁自动切回 PRIMARY，备机延迟回 standby',
            't=40s 前车急刹：备好的 AEB 在降级链路上同样有效',
            't=55s 双控宕机：200ms 通信看门狗紧急制动（SRC:9，安全兜底）',
            't=62s 双控恢复：链路自动恢复行驶',
        ],
    },
    'free': {
        'name': '自由交互（手动故障注入）',
        'duration': 0.0,            # 0 = 一直运行，q 退出
        'spawn_index': 30,
        'lead': {
            'gap0': 45.0,
            'profile': [(0.0, 7.0), (20.0, 4.0), (35.0, 7.5)],
            'hard_brake': None,
        },
        'timeline': [],
        'notes': [
            '不预设故障，由你现场注入（输入字符后回车）：',
            '  p=杀主控  P=重启主控  b=杀备控  B=重启备控',
            '  h=主控卡死(HANG，演示 SEQ 停滞接管)  q=退出',
        ],
    },
}

# 菜单展示顺序
ORDER = ['lka', 'acc', 'aeb', 'overtake', 'failover', 'free']


def lead_target_speed(lead_cfg, t):
    """按 profile 分段阶梯取前车目标速度。"""
    if not lead_cfg:
        return 0.0
    v = lead_cfg['profile'][0][1]
    for t_seg, v_seg in lead_cfg['profile']:
        if t >= t_seg:
            v = v_seg
    return v


def lead_in_hard_brake(lead_cfg, t):
    hb = lead_cfg.get('hard_brake') if lead_cfg else None
    return bool(hb) and hb[0] <= t < hb[1]
