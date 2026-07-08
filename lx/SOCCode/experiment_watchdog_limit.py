#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""双路静默看门狗（WATCHDOG_TIMEOUT_MS）压缩下限 SIL 扫描实验。

背景：`config.py`/`main.c` 里的 WATCHDOG_TIMEOUT_MS=200（两路 Jetson 都静默
超过此值 → ESP32 独立触发全力制动）此前只有理论推导，没有像
JETSON_TIMEOUT_MS（主备接管阈值，110 次真实 SIL 试验支撑，见
`lx/图片/16_接管时延极限/`）那样的重复试验数据。原本承载这类实验的 `仿真/`
目录当前不在 checkout 里，这里按最小范围重建：只移植
`control/virtual_esp32_watchdog.py` 里的仲裁+看门狗判定逻辑（不含完整虚拟
ESP32/CARLA 联合仿真），够用来回答"这个阈值能压到多少"这一个问题。

三类试验，对应三个不同的失效/风险维度：
  1. **双杀试验**（double-kill）：两路同时静默，测真实检测到的紧急制动延迟
     （min/median/max，N=110 次，phase 随机化——延迟的分布来源于"停帧时刻
     相对于发帧周期/看门狗轮询周期的相位"，与真实 110 次接管试验里
     min15/中位47/最坏63ms 的分布同一个成因）。
  2. **单杀试验**（single-kill，耦合校验）：只停一路，验证 arbitrate() 会
     正常切到另一路、且看门狗全程不应误触发——这是"WATCHDOG_TIMEOUT_MS 须
     > JETSON_TIMEOUT_MS"这条推导的直接验证，不是假设。
  3. **最坏抖动重合试验**（worst-jitter coincidence）：两路"同时"各发生一次
     2026-07 实机 LOOP TIMING 真实抓到的最坏单拍延迟（49.44ms，未做实时性
     硬化前的真实最大值），纯属抖动、并非真故障，检验此时看门狗会不会被
     误触发——这是压缩下限最关键的安全约束（虚警在这里的代价是"两路都好、
     纯粹因为一次巧合抖动就被判双死、零计算机输入全力刹车"，比它防的故障
     更危险，所以判据比双杀试验的"延迟多快"更重要）。

用法：
    cd lx/SOCCode
    python experiment_watchdog_limit.py [--repeats 110] [--out data_watchdog_limit.json]

输出：控制台汇总表 + 可选 JSON（逐配置聚合结果，供后续画图）。

Python 3.6 兼容；注释中文。
"""

import argparse
import json
import random

from control.virtual_esp32_watchdog import VirtualEsp32Watchdog

# 与 lx/MCUcode/ADAS_Test/main/main.c 当前一致（main.c 是唯一真源，这里只读）
JETSON_TIMEOUT_MS_DEPLOYED = 58
WATCHDOG_TIMEOUT_MS_DEPLOYED = 200
WATCHDOG_CHECK_MS_DEPLOYED = 50

# 2026-07-01/02 实机 [LOOP TIMING] 日志抓到的真实最坏单拍耗时（100Hz 控制环，
# 实时性硬化前）。用作"两路巧合抖动"的最坏输入，而非虚构值。
REAL_WORST_TICK_MS = 49.44

NOMINAL_FRAME_PERIOD_MS = 10.0   # Jetson 100Hz 控制环，正常发帧周期


def run_double_kill_trial(rng, watchdog_timeout_ms, watchdog_check_ms,
                          jetson_timeout_ms):
    """两路同时静默，返回看门狗真正触发紧急态的检测延迟 (ms)。"""
    wd = VirtualEsp32Watchdog(jetson_timeout_ms, watchdog_timeout_ms,
                             watchdog_check_ms)
    frame_phase = rng.uniform(0, NOMINAL_FRAME_PERIOD_MS)
    poll_phase = rng.uniform(0, watchdog_check_ms)
    kill_t = 1000.0

    t = frame_phase
    while t < kill_t:
        wd.on_frame('pri', t)
        wd.on_frame('sec', t)
        t += NOMINAL_FRAME_PERIOD_MS

    horizon = kill_t + watchdog_timeout_ms + 3 * watchdog_check_ms + 50
    tpoll = kill_t + poll_phase
    while tpoll < horizon:
        if wd.watchdog_check(tpoll):
            return tpoll - kill_t
        tpoll += watchdog_check_ms
    return None  # 不应发生：horizon 留了充分余量


def run_single_kill_trial(rng, watchdog_timeout_ms, watchdog_check_ms,
                          jetson_timeout_ms, backup_cold_start_delay_ms=0.0):
    """只停主控，验证 (a) 备控在合理时间内被仲裁选中 (b) 看门狗全程不误触发。

    backup_cold_start_delay_ms：备控从主控死亡到发出第一帧的延迟。
      - 0.0（默认）＝当前部署的热待机模式（`config.BACKUP_HOT_STANDBY=True`，
        备控 standby 期已持续发新鲜帧，见 ADAS.py `_hot_standby_tx`）——这种
        模式下备控从不真正"陈旧"，本函数测不出双静默耦合风险，只能确认
        "单路故障不会误伤看门狗"这个平凡结论。
      - >0 ＝冷启动备控（`BACKUP_HOT_STANDBY=False` 或备控刚重启/尚未预热），
        备控在这段延迟内也是陈旧的——这才是"WATCHDOG_TIMEOUT_MS 须大于
        JETSON_TIMEOUT_MS"这条耦合约束真正要防的场景：主控刚死、备控还没
        来得及证明自己活着，若两路同时被看门狗判定"陈旧"，会在备控接管
        指令生效前抢先触发全力制动。

    返回 (switch_latency_ms 或 None, watchdog_false_triggered: bool)。
    """
    wd = VirtualEsp32Watchdog(jetson_timeout_ms, watchdog_timeout_ms,
                             watchdog_check_ms)
    frame_phase = rng.uniform(0, NOMINAL_FRAME_PERIOD_MS)
    kill_t = 1000.0
    run_span = 400.0

    # 关键：帧到达事件与 arbitrate/watchdog_check 查询必须按时间顺序交替处理。
    # 如果先把"未来"的帧（备控冷启动延迟之后恢复发帧）整批灌进状态、再回头
    # 查询较早时刻的 arbitrate/watchdog_check，会让 last_rx 被尚未发生的帧
    # 污染，查询到的"陈旧度"变成负数——查询永远看到"新鲜"，冷启动耦合风险
    # 测试形同虚设（这是本实验第一版的真实 bug，此处已修正）。
    events = []   # (t, channel)
    t = frame_phase
    while t < kill_t:
        events.append((t, 'pri'))
        events.append((t, 'sec'))
        t += NOMINAL_FRAME_PERIOD_MS
    # 主控死亡后不再发帧；备控延迟 backup_cold_start_delay_ms 后恢复发帧
    # （热待机=0 ≈ 立即；冷启动=真实延迟）。
    t2 = kill_t + backup_cold_start_delay_ms
    while t2 < kill_t + run_span:
        events.append((t2, 'sec'))
        t2 += NOMINAL_FRAME_PERIOD_MS
    events.sort(key=lambda e: e[0])

    ei = [0]

    def apply_events_up_to(now):
        while ei[0] < len(events) and events[ei[0]][0] <= now:
            wd.on_frame(events[ei[0]][1], events[ei[0]][0])
            ei[0] += 1

    switch_latency = None
    watchdog_false = False
    tarb = kill_t
    twd = kill_t
    end_t = kill_t + run_span
    while tarb < end_t or twd < end_t:
        if tarb <= twd and tarb < end_t:
            apply_events_up_to(tarb)
            if wd.arbitrate(tarb) == 'sec' and switch_latency is None:
                switch_latency = tarb - kill_t
            tarb += NOMINAL_FRAME_PERIOD_MS
        elif twd < end_t:
            apply_events_up_to(twd)
            if wd.watchdog_check(twd):
                watchdog_false = True
            twd += watchdog_check_ms
        else:
            break

    return switch_latency, watchdog_false


def run_worst_jitter_coincidence(watchdog_timeout_ms, watchdog_check_ms,
                                 jetson_timeout_ms,
                                 spike_ms=REAL_WORST_TICK_MS,
                                 n_phase_samples=200):
    """两路"同时"各发生一次真实最坏单拍延迟（纯抖动，非故障），检验看门狗
    是否会被误触发。

    保守判据：真实抖动发生的时刻相对于看门狗轮询相位是不可控的——只要
    spike_ms 超过阈值，就存在一个"陈旧窗口" (watchdog_timeout_ms, spike_ms)，
    某次巧合的轮询相位迟早会落进这个窗口触发误报。这里密集扫描
    [0, watchdog_check_ms) 内的轮询相位，只要任一相位会误触发就判定为
    存在风险，不依赖某一次巧合相位是否"运气好躲过去"。
    """
    base = 1000.0
    spike_arrival = base + spike_ms
    for i in range(n_phase_samples):
        poll_phase = (i / float(n_phase_samples)) * watchdog_check_ms
        wd = VirtualEsp32Watchdog(jetson_timeout_ms, watchdog_timeout_ms,
                                 watchdog_check_ms)
        t = 0.0
        while t < base:
            wd.on_frame('pri', t)
            wd.on_frame('sec', t)
            t += NOMINAL_FRAME_PERIOD_MS
        # 显式补一帧钉在 base：代表"抖动发生前最后一帧刚好此刻到达"，
        # spike_ms 之后的空窗才是纯粹的抖动本身，不掺入 while 循环 strict <
        # 带来的额外陈旧度。
        wd.on_frame('pri', base)
        wd.on_frame('sec', base)

        twd = base + poll_phase
        while twd < spike_arrival:
            if wd.watchdog_check(twd):
                return True
            twd += watchdog_check_ms
    return False


def evaluate_config(watchdog_timeout_ms, watchdog_check_ms, jetson_timeout_ms,
                    n_repeats, seed):
    rng = random.Random(seed)

    double_kill_lat = []
    for _ in range(n_repeats):
        lat = run_double_kill_trial(rng, watchdog_timeout_ms,
                                    watchdog_check_ms, jetson_timeout_ms)
        if lat is not None:
            double_kill_lat.append(lat)

    single_kill_false = 0
    single_kill_switch_lat = []
    n_single = max(10, n_repeats // 5)
    for _ in range(n_single):
        lat, false_trig = run_single_kill_trial(
            rng, watchdog_timeout_ms, watchdog_check_ms, jetson_timeout_ms,
            backup_cold_start_delay_ms=0.0)   # 当前部署：热待机
        if false_trig:
            single_kill_false += 1
        if lat is not None:
            single_kill_switch_lat.append(lat)

    # 冷启动备控场景（假设性：BACKUP_HOT_STANDBY=False 或备控刚重启）——
    # 单独统计，不计入 reliable（当前实际部署是热待机），只用来验证/量化
    # "WATCHDOG_TIMEOUT_MS 须 > JETSON_TIMEOUT_MS" 这条耦合约束。
    cold_start_false = 0
    n_cold = max(10, n_repeats // 5)
    for _ in range(n_cold):
        _, false_trig = run_single_kill_trial(
            rng, watchdog_timeout_ms, watchdog_check_ms, jetson_timeout_ms,
            backup_cold_start_delay_ms=float(jetson_timeout_ms))
        if false_trig:
            cold_start_false += 1

    worst_jitter_false = run_worst_jitter_coincidence(
        watchdog_timeout_ms, watchdog_check_ms, jetson_timeout_ms)

    double_kill_lat.sort()
    n = len(double_kill_lat)
    reliable = (single_kill_false == 0) and (not worst_jitter_false) and (n > 0)

    return {
        'watchdog_timeout_ms': watchdog_timeout_ms,
        'watchdog_check_ms': watchdog_check_ms,
        'jetson_timeout_ms': jetson_timeout_ms,
        'n_double_kill': n,
        'detect_min_ms': double_kill_lat[0] if n else None,
        'detect_median_ms': double_kill_lat[n // 2] if n else None,
        'detect_max_ms': double_kill_lat[-1] if n else None,
        'single_kill_false_positive_count': single_kill_false,
        'single_kill_n': n_single,
        'single_kill_switch_lat_max_ms': (
            max(single_kill_switch_lat) if single_kill_switch_lat else None),
        'cold_start_false_positive_count': cold_start_false,
        'cold_start_n': n_cold,
        'worst_jitter_false_positive': worst_jitter_false,
        'reliable': reliable,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--repeats', type=int, default=110,
                    help='每个配置的双杀试验重复次数（默认 110，对齐接管实验规模）')
    ap.add_argument('--out', default=None, help='结果 JSON 输出路径（可选）')
    ap.add_argument('--seed', type=int, default=20260703)
    args = ap.parse_args()

    # 扫描网格：WATCHDOG_TIMEOUT_MS 从部署值 200 一路下探；WATCHDOG_CHECK_MS
    # 按当前部署比例（200/50=4）同步压缩，不单独压 timeout 不压轮询周期
    # （否则瓶颈会先撞在轮询粒度上，而不是我们想测的判据本身）。
    watchdog_timeouts = [200, 180, 160, 140, 120, 110, 100, 90, 80, 75,
                        70, 65, 60, 58, 56, 54, 52, 50, 48, 46, 44, 42, 40,
                        35, 30]

    results = []
    for i, wt in enumerate(watchdog_timeouts):
        wc = max(10, round(wt / 4.0))
        cfg = evaluate_config(
            watchdog_timeout_ms=wt, watchdog_check_ms=wc,
            jetson_timeout_ms=JETSON_TIMEOUT_MS_DEPLOYED,
            n_repeats=args.repeats, seed=args.seed + i)
        results.append(cfg)

    reliable_wt = [r['watchdog_timeout_ms'] for r in results if r['reliable']]
    floor = min(reliable_wt) if reliable_wt else None

    print('=' * 112)
    print('%-6s %-6s %-9s %-9s %-9s %-14s %-16s %-9s %-9s' % (
        'WD_MS', 'CHK_MS', 'det_min', 'det_med', 'det_max',
        'single_false', 'cold_start_false*', 'jit_false', 'reliable'))
    print('-' * 112)
    for r in results:
        print('%-6d %-6d %-9s %-9s %-9s %-14s %-16s %-9s %-9s' % (
            r['watchdog_timeout_ms'], r['watchdog_check_ms'],
            '%.1f' % r['detect_min_ms'] if r['detect_min_ms'] is not None else 'NA',
            '%.1f' % r['detect_median_ms'] if r['detect_median_ms'] is not None else 'NA',
            '%.1f' % r['detect_max_ms'] if r['detect_max_ms'] is not None else 'NA',
            '%d/%d' % (r['single_kill_false_positive_count'], r['single_kill_n']),
            '%d/%d' % (r['cold_start_false_positive_count'], r['cold_start_n']),
            'YES' if r['worst_jitter_false_positive'] else 'no',
            'YES' if r['reliable'] else 'NO'))
    print('=' * 112)
    print('* cold_start_false 是假设性场景（BACKUP_HOT_STANDBY=False），不计入 reliable 判据，'
         '仅用于量化 WATCHDOG_TIMEOUT_MS 须 > JETSON_TIMEOUT_MS(%d) 这条耦合约束'
         % JETSON_TIMEOUT_MS_DEPLOYED)
    if floor is not None:
        floor_row = next(r for r in results if r['watchdog_timeout_ms'] == floor)
        print('SIL 实测稳健下限：WATCHDOG_TIMEOUT_MS=%d (CHECK_MS=%d) -> '
             '检测延迟 中位=%.1fms 最坏=%.1fms，单杀 0 误触发，最坏抖动重合不误触发' % (
                 floor, floor_row['watchdog_check_ms'],
                 floor_row['detect_median_ms'], floor_row['detect_max_ms']))
    else:
        print('扫描范围内没有找到可靠配置——说明下限比 %d 更高，或抖动模型/网格需调整' %
             min(watchdog_timeouts))
    print('部署值 WATCHDOG_TIMEOUT_MS=%d 对照：' % WATCHDOG_TIMEOUT_MS_DEPLOYED)
    deployed_row = next((r for r in results
                        if r['watchdog_timeout_ms'] == WATCHDOG_TIMEOUT_MS_DEPLOYED),
                       None)
    if deployed_row:
        print('  中位=%.1fms 最坏=%.1fms reliable=%s' % (
            deployed_row['detect_median_ms'], deployed_row['detect_max_ms'],
            deployed_row['reliable']))

    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            json.dump({
                'results': results,
                'floor_watchdog_timeout_ms': floor,
                'real_worst_tick_ms_used': REAL_WORST_TICK_MS,
                'jetson_timeout_ms_deployed': JETSON_TIMEOUT_MS_DEPLOYED,
                'repeats_per_config': args.repeats,
            }, f, ensure_ascii=False, indent=2)
        print('saved', args.out)


if __name__ == '__main__':
    main()
