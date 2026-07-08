#pragma once

// ADAS 节点配置与运行时参数——C++ 移植自 lx/SOCCode/config.py。
//
// 与 Python 版本的对应关系：
//   - Python `frozenset(_SAFETY_CRITICAL_PARAMS)`（12 个执行器/安全阈值）在这里
//     直接声明为 `constexpr double`——**编译期常量，物理上不可能被 params.yaml
//     覆盖**，比 Python 版本"运行时白名单拦截"多一层保证。
//   - 其余标量（tunable）声明为 `inline <T>`（可变，C++17 内联变量语义），
//     用 X-Macro 列表登记进 applyParamOverrides() 的覆盖表，行为对齐 Python
//     `_load_param_overrides()`：只覆盖已存在的全大写标量名，类型强制转换，
//     安全关键名单硬拒绝。
//   - Python 的 AEB_CLASS_*/AEB_PATH_OBS_HALF_WIDTH 字典（非标量，_load_param_overrides
//     显式跳过 "non-scalar param"）在此对应为 constexpr std::array，同样不可覆盖。
//   - 依赖环境变量解析的项（LOOP_HZ、VEHICLE_ADAPTER、ML_*、RT_*、LOCKSTEP_*、
//     SERIAL_ESP32 等）此处只放 Python 侧"环境变量缺省值"这一档默认值；
//     真正的环境变量读取留给 Phase 3（runtime 移植）在启动时覆盖这些 inline 变量，
//     覆盖顺序与 Python 一致：先环境变量解析，再应用 params.yaml。

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace soc::config {

// ── 角度换算辅助（对应 Python `math.radians(...)`）──
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDegToRad = kPi / 180.0;

// ── 角色常量 ──
inline std::string ROLE_PRIMARY = "primary";
inline std::string ROLE_BACKUP = "backup";

// 运行时配置（数据形状对应 Python RuntimeConfig dataclass）。
// 环境变量解析函数 resolve_runtime_config() 属于 Phase 3（runtime 移植），
// 这里只声明纯数据结构。
struct RuntimeConfig {
    std::string nano_role;
    bool is_primary = true;
    std::string primary_ip;
    std::string secondary_ip;
    int hb_port = 9877;
    double hb_grace_s = 3.0;
    std::string log_file;
};

// ── actor 分类常量（非 tunable，标识符语义，不进覆盖表）──
inline constexpr int ACTOR_CLASS_UNKNOWN = 0;
inline constexpr int ACTOR_CLASS_VEHICLE = 1;
inline constexpr int ACTOR_CLASS_OBSTACLE = 2;
inline constexpr int ACTOR_CLASS_PEDESTRIAN = 3;
inline constexpr int kActorClassCount = 4;

// ── 安全关键参数：constexpr——不进覆盖表，编译期即锁定，任何 params.yaml 都动不了 ──
inline constexpr double MAX_DELTA = 25.0 * kDegToRad;              // 最大方向盘转角 (rad)
inline constexpr double MAX_DELTA_RATE = 50.0 * kDegToRad;         // 方向盘转角变化率限幅 (rad/s)
inline constexpr double LON_CMD_MAX_BRAKE_DECEL = 6.0;             // 最大制动减速度指令 (m/s²)
inline constexpr double LON_CMD_MAX_DRIVE_ACCEL = 6.0;             // 最大驱动加速度指令 (m/s²)
inline constexpr double CTRL_DT_MIN = 0.002;                       // 控制周期下界 (s)
inline constexpr double CTRL_DT_MAX = 0.050;                       // 控制周期上界 (s)
inline constexpr double AEB_EMERGENCY_DIST = 5.0;                  // 紧急制动距离 (m)
inline constexpr double TTC_BRAKE_FULL = 5.0;                      // TTC 全制动阈值 (s)
inline constexpr double SAFE_EGO_MAX_DECEL = 6.0;                  // 自车最大减速度 (m/s²)
inline constexpr double SAFE_LEAD_MAX_DECEL = 8.0;                 // 前车最大减速度 (m/s²)
inline constexpr double WHEEL_BASE = 3.0;                          // 轴距 (m)
inline constexpr double STEER_SIGN = 1.0;                          // 转向方向符号

// ============================================================
// 可覆盖标量参数：X-Macro 列表 —— X(NAME, 默认值)
// 同一份列表同时驱动“变量声明”和“覆盖注册表”，避免两处维护不同步。
// ============================================================

// ---- 字符串（ROS 话题名 / 模式选择 / 设备路径）----
#define SOC_CONFIG_STRINGS(X) \
    X(TOPIC_JETSON_PSI, "/jetson/psi") \
    X(TOPIC_JETSON_DELTA, "/jetson/delta") \
    X(TOPIC_JETSON_BRAKE, "/jetson/brake") \
    X(TOPIC_JETSON_LANE_OFFSET, "/jetson/lane_offset") \
    X(TOPIC_ESP32_PSI, "/esp32/psi") \
    X(TOPIC_ESP32_DELTA, "/esp32/delta") \
    X(TOPIC_ESP32_BRAKE, "/esp32/brake") \
    X(TOPIC_JETSON_ACTIVE_ROLE, "/jetson/active_role") \
    X(TOPIC_JETSON_LANE_WIDTH_EST, "/jetson/lane_width_est") \
    X(TOPIC_JETSON_LEAD_CLS, "/jetson/lead_cls") \
    X(TOPIC_JETSON_FAILOVER_AVAILABLE, "/jetson/failover_available") \
    X(TOPIC_CAR1_XY, "/car1_xy") \
    X(TOPIC_CAR1_PSI, "/car1_psi") \
    X(TOPIC_CAR2_XY, "/car2xy") \
    X(TOPIC_CAR1_V, "/car1_v") \
    X(TOPIC_CAR2_V, "/car2_v") \
    X(TOPIC_ROAD_PSI, "/road_psi") \
    X(TOPIC_HENG_ERROR, "/heng_error") \
    X(TOPIC_SET_PARAM, "/adas/set_param") \
    X(TOPIC_CAR2_CLASS, "/car2_class") \
    X(MULTI_TARGET_TOPIC_XY_FMT, "/car{}_xy") \
    X(MULTI_TARGET_TOPIC_V_FMT, "/car{}_v") \
    X(MULTI_TARGET_TOPIC_CLASS_FMT, "/car{}_class") \
    X(LON_CONTROLLER, "pid") \
    X(VEHICLE_ADAPTER, "esp32") \
    X(LAT_CONTROLLER, "pid") \
    X(COMFORT_LAYER, "legacy") \
    X(ML_BACKEND, "onnx") \
    X(ML_INFERD_HOST, "127.0.0.1") \
    X(LOCKSTEPD_HOST, "127.0.0.1") \
    X(LEAD_ESTIMATOR, "legacy") \
    X(SERIAL_ESP32, "/dev/ttyTHS1")

// ---- 布尔 ----
#define SOC_CONFIG_BOOLS(X) \
    X(ML_ENABLED, false) \
    X(ML_ASYNC, true) \
    X(RT_THREAD_PIN, true) \
    X(LOCKSTEP_ENABLED, false) \
    X(LOCKSTEP_INJECT, false) \
    X(SAFETY_SUPERVISOR_ENABLED, true) \
    X(GATE_FILTER_EXT_ENABLED, false) \
    X(IDLE_STANDBY_ENABLED, true) \
    X(BACKUP_HOT_STANDBY, true) \
    X(AEB_PATH_CHECK_ENABLED, false)

// ---- 整数 ----
#define SOC_CONFIG_INTS(X) \
    X(MULTI_TARGET_COUNT, 1) \
    X(ML_NUM_THREADS, 1) \
    X(ML_INFERD_PORT, 19999) \
    X(ML_INFERD_CORE, 3) \
    X(LOCKSTEPD_PORT, 19998) \
    X(LOCKSTEPD_CORE, 2) \
    X(MPC_RICCATI_ITERS, 200) \
    X(BAUDRATE, 115200) \
    X(LOG_EVERY_N_CYCLES, 100) \
    X(LOOP_HZ, 100) \
    X(RT_CONTROL_CORE, 0) \
    X(RT_FIFO_PRIO, 0) \
    X(LOCKSTEP_CHECKER_CORE, 2) \
    X(LOCKSTEP_DEBOUNCE_N, 2) \
    X(LANE_EST_MIN_SAMPLES, 60) \
    X(LANE_EST_PERCENTILE, 95) \
    X(LEAD_CONFIRM_CYCLES, 5) \
    X(AEB_ALERT_ARM_CONFIRM_CYCLES, 3) \
    X(CTRL_CONSECUTIVE_ERROR_LIMIT, 5) \
    X(AEB_FULL_CONFIRM_CYCLES, 5) \
    X(ACC_LEAD_RELEASE_LANE_OUT_CYCLES, 4) \
    X(ACC_LEAD_RELEASE_OPENING_CYCLES, 4) \
    X(AEB_PATH_CONFIRM_CYCLES, 3)

// ---- 浮点（绝大多数控制增益/阈值）----
#define SOC_CONFIG_DOUBLES(X) \
    X(MULTI_TARGET_FWD_MAX, 60.0) \
    X(MULTI_TARGET_FWD_MIN, 0.5) \
    X(CUTIN_HORIZON_S, 1.2) \
    X(CUTIN_CORRIDOR_RATIO, 1.7) \
    X(CUTIN_MIN_LAT_RATE, 0.08) \
    X(CUTIN_LAT_RATE_ALPHA, 0.15) \
    X(STANLEY_K_CTE, 0.85) \
    X(STANLEY_SOFTENING_V, 2.0) \
    X(STANLEY_HEADING_GAIN, 1.0) \
    X(STANLEY_CTE_MAX, 1.2) \
    X(COMFORT_JERK_ACCEL, 0.8) \
    X(COMFORT_JERK_DECEL, 1.4) \
    X(COMFORT_JERK_RELEASE, 1.0) \
    X(MPC_TS, 0.10) \
    X(MPC_Q_E, 1.0) \
    X(MPC_Q_V, 2.5) \
    X(MPC_R, 12.0) \
    X(MPC_LEAD_FF_GAIN, 1.0) \
    X(LEAD_KF_JERK_PSD, 6.0) \
    X(LEAD_KF_MEAS_VAR, 0.20) \
    X(LEAD_KF_INIT_V_VAR, 1.0) \
    X(LEAD_KF_INIT_A_VAR, 4.0) \
    X(LEAD_KF_GATE_SIGMA, 4.0) \
    X(RT_PIN_RESWEEP_S, 3.0) \
    X(LOCKSTEP_DELTA_EPS, 1e-9) \
    X(LOCKSTEP_LON_EPS, 1e-9) \
    X(LOCKSTEP_INJECT_DELTA, 0.05) \
    X(LOCKSTEP_SAFE_BRAKE_CMD, 2.5) \
    X(HEARTBEAT_TIMEOUT_S, 0.035) \
    X(HB_SEND_INTERVAL_S, 0.01) \
    X(HB_STANDBY_HANDOFF_S, 0.3) \
    X(HB_BACKUP_TIMEOUT_S, 2.0) \
    X(LANE_DEFAULT_WIDTH, 3.8) \
    X(LANE_WIDTH_MIN, 3.5) \
    X(LANE_WIDTH_MAX, 14.0) \
    X(LANE_EST_TIMEOUT_S, 2.0) \
    X(LANE_WIDTH_FILTER_ALPHA, 0.008) \
    X(LANE_WIDTH_MAX_RATE, 0.15) \
    X(LANE_EST_WINDOW_STRAIGHT, 20.0) \
    X(LANE_EST_WINDOW_CURVE, 6.0) \
    X(LANE_EST_CURV_THRESH, 0.008) \
    X(K_LAT_COMP, 0.18) \
    X(K_PSI_P, 0.9) \
    X(K_PSI_I, 0.06) \
    X(K_PSI_D, 0.03) \
    X(MAX_PSI_ERR, 60.0 * kDegToRad) \
    X(MAX_PSI_I, 8.0 * kDegToRad) \
    X(MAX_PSI_D, 180.0 * kDegToRad) \
    X(K_PREVIEW_GAIN, 0.55) \
    X(K_DELTA, 1.4) \
    X(CURVE_PREVIEW_ATTEN_MAX, 0.45) \
    X(CURVE_PREVIEW_ATTEN_SCALE, 0.020) \
    X(PREVIEW_TIME_MIN, 0.8) \
    X(PREVIEW_TIME_MAX, 2.0) \
    X(PREVIEW_SPEED_REF, 16.7) \
    X(K_FF_CURV, 0.4) \
    X(MAX_FF_DELTA, 20.0 * kDegToRad) \
    X(CURVE_FF_ATTEN_MAX, 0.35) \
    X(CURVE_FF_ATTEN_SCALE, 0.050) \
    X(K_CTE, 0.06) \
    X(K_CTE_D, 0.02) \
    X(MAX_CTE_CORR, 12.0 * kDegToRad) \
    X(CTE_EFFECTIVE_LIMIT, 1.2) \
    X(MAX_CTE_DOT, 5.0) \
    X(CTE_FILTER_ALPHA, 0.1111) \
    X(CURVE_CTE_BOOST_MAX, 0.60) \
    X(CURVE_CTE_BOOST_SCALE, 0.020) \
    X(ROAD_PSI_FILTER_ALPHA, 0.25) \
    X(CORNERING_RRATE_THRESH, 0.08) \
    X(I_DECAY_IN_CORNER, 0.97) \
    X(PSI_I_LOW_SPEED_GATE, 0.5) \
    X(PSI_I_LOW_SPEED_DECAY, 0.95) \
    X(BOUNDARY_DELTA_RATE_MULT, 2.0) \
    X(VEHICLE_HALF_WIDTH, 0.90) \
    X(MIN_LANE_SAFE_MARGIN, 0.5) \
    X(LANE_WARN_RATIO, 0.55) \
    X(LANE_HARD_RATIO, 0.92) \
    X(K_LATERAL_SOFT, 0.45) \
    X(K_LATERAL_HARD, 1.10) \
    X(BOUNDARY_BRAKE_EXTRA, 1.4) \
    X(ACC_D0, 2.5) \
    X(ACC_TIME_GAP, 2.0) \
    X(ACC_KD, 0.4) \
    X(ACC_KI, 0.02) \
    X(ACC_KV, 0.8) \
    X(ACC_KA, 1.0) \
    X(ACC_FF_MAX, 0.6) \
    X(ACC_VDIFF_ALPHA_CLOSING, 0.40) \
    X(ACC_VDIFF_ALPHA_OPENING, 0.10) \
    X(ACC_I_MAX, 1.5) \
    X(ACC_I_PAUSE_VDIFF, 1.5) \
    X(ACC_GAP_ERR_DRIVE_CAP, 8.0) \
    X(ACC_GAP_ERR_BRAKE_CAP, 6.0) \
    X(ACC_DRIVE_MAX_BASE, 0.6) \
    X(ACC_DRIVE_MAX_GAIN_V, 0.30) \
    X(ACC_DRIVE_MAX_GAIN_D, 0.05) \
    X(ACC_DRIVE_MAX_LIMIT, 1.6) \
    X(ACC_BRAKE_MAX_BASE, 1.2) \
    X(ACC_BRAKE_MAX_GAIN_V, 0.35) \
    X(ACC_BRAKE_MAX_GAIN_D, 0.08) \
    X(ACC_BRAKE_MAX_LIMIT, 2.5) \
    X(ACC_STEADY_GAP_BAND, 0.8) \
    X(ACC_STEADY_VREL_BAND, 0.30) \
    X(ACC_I_DECAY_SAT, 0.98) \
    X(ACC_I_DECAY_STEADY, 0.92) \
    X(LEAD_TIMEOUT_S, 0.5) \
    X(CRUISE_KP, 0.25) \
    X(DRIVER_SET_SPEED, 8.0) \
    X(SYSTEM_MAX_CRUISE, 10.0) \
    X(ROAD_LIMIT_SPEED, 8.0) \
    X(TTC_BRAKE_START, 15.0) \
    X(AEB_SAFE_DIST_BUFFER, 8.0) \
    X(SAFE_REACTION_TIME, 0.35) \
    X(SAFE_DIST_STANDSTILL, 6.0) \
    X(SAFE_DIST_MAX, 120.0) \
    X(SAFE_DIST_LOW_SPEED_REF, 8.0) \
    X(CORNERING_MAX_LAT_ACCEL, 2.2) \
    X(CORNERING_SPEED_MIN, 3.0) \
    X(CURV_FILTER_ALPHA, 0.12) \
    X(VTGT_FILTER_ALPHA, 0.0204) \
    X(LON_FILTER_ALPHA, 0.1429) \
    X(LEAD_LAT_STRAIGHT_RATIO, 0.33) \
    X(LEAD_LAT_CURVE_RATIO, 0.18) \
    X(LEAD_LAT_MAX_STRAIGHT_MIN, 1.8) \
    X(LEAD_LAT_MAX_STRAIGHT_MAX, 3.5) \
    X(LEAD_LAT_MAX_CURVE_MIN, 1.2) \
    X(LEAD_LAT_MAX_CURVE_MAX, 2.4) \
    X(CURV_LEAD_THRESH, 0.01) \
    X(AEB_CURV_SUPPRESS_MAX, 0.50) \
    X(AEB_CURV_SCALE, 0.03) \
    X(TTC_AEB_MAX_DIST, 20.0) \
    X(TTC_AEB_MAX_LAT_RATIO, 0.60) \
    X(AEB_MAX_ENGAGE_DIST, 25.0) \
    X(LEAD_MAX_TRACK_DIST, 60.0) \
    X(LEAD_ACCEL_TAU_FAST, 0.05) \
    X(LEAD_ACCEL_TAU_SLOW, 0.20) \
    X(LEAD_ACCEL_TAU_DIFF, 0.15) \
    X(LEAD_ACCEL_MAX, 4.0) \
    X(LON_RATE_ACCEL_CRUISE, 1.20) \
    X(LON_RATE_DECEL_CRUISE, 2.50) \
    X(LON_RATE_ACCEL_ACC, 1.80) \
    X(LON_RATE_DECEL_ACC, 3.00) \
    X(LON_RATE_BRAKE_RELEASE, 4.0) \
    X(LON_RATE_AEB, 60.0) \
    X(LON_RATE_BOUNDARY, 8.0) \
    X(LON_OUTPUT_ALPHA, 0.25) \
    X(SAFETY_REACTION_TIME, 1.0) \
    X(SAFETY_DIST_BUFFER, 1.2) \
    X(SAFETY_PREBRAKE_MARGIN, 4.0) \
    X(SAFETY_PREBRAKE_MAX, 4.5) \
    X(SAFETY_FULL_BRAKE_MARGIN, 0.5) \
    X(SAFETY_CUTIN_TTC, 3.0) \
    X(SAFETY_CUTIN_BRAKE, 3.0) \
    X(SAFETY_CURVE_LAT_ACCEL, 1.2) \
    X(SAFETY_CURVE_SPEED_KP, 1.5) \
    X(SAFETY_CTE_WARN, 0.8) \
    X(SAFETY_CTE_HARD, 1.4) \
    X(LAT_RATE_NORMAL, 80.0 * kDegToRad) \
    X(LAT_OUTPUT_ALPHA, 0.50) \
    X(TAKEOVER_GUARD_DURATION_S, 0.2) \
    X(TAKEOVER_LON_RATE, 6.0) \
    X(TAKEOVER_LON_RATE_AEB_RELEASE, 12.0) \
    X(TAKEOVER_LON_RATE_VULNERABLE, 4.0) \
    X(TAKEOVER_DELTA_RATE, 25.0 * kDegToRad) \
    X(GATE_STEER_DIFF_MAX, 12.0 * kDegToRad) \
    X(GATE_STEER_DIFF_MAX_TRANSITION, 6.0 * kDegToRad) \
    X(TAKEOVER_COOLDOWN_S, 1.0) \
    X(SENSOR_TIMEOUT_BRAKE_S, 0.5) \
    X(SENSOR_TIMEOUT_BRAKE_CMD, 2.0) \
    X(LEAD_CLASS_STALE_TIMEOUT_S, 2.0) \
    X(SENSOR_STALE_TIMEOUT_S, 0.3) \
    X(EMERGENCY_STOP_MIN_INTERVAL_S, 0.1) \
    X(STANDBY_KEEPALIVE_INTERVAL_S, 0.1) \
    X(STANDBY_HOLD_BRAKE_CMD, 1.0) \
    X(STANDBY_ENTER_S, 3.0) \
    X(STANDBY_LOG_INTERVAL_S, 10.0) \
    X(CONTROL_LOOP_BUDGET_S, 0.008) \
    X(CONTROL_LOOP_SLOW_LOG_INTERVAL_S, 1.0) \
    X(CURVE_HOLD_CURV_THRESH, 0.006) \
    X(CURVE_HOLD_SPEED_KP, 0.35) \
    X(CURVE_HOLD_SPEED_KI, 0.02) \
    X(CURVE_HOLD_I_MAX, 2.0) \
    X(CURVE_HOLD_EXIT_CURV, 0.006) \
    X(CURVE_HOLD_TIMEOUT_S, 8.0) \
    X(CURVE_HOLD_ACTIVATE_LOSS_S, 0.25) \
    X(CURVE_HOLD_REACQ_STABLE_S, 0.20) \
    X(LEAD_KEEPALIVE_S, 0.35) \
    X(AEB_ALERT_TIMEOUT_S, 3.0) \
    X(AEB_ALERT_HOLD_TIME_S, 5.0) \
    X(AEB_ALERT_ARM_MIN_LEAD_V, 1.0) \
    X(AEB_ALERT_INVALID_LEAD_V, 0.5) \
    X(OVT_TRIGGER_DIST_M, 30.0) \
    X(OVT_TRIGGER_MIN_DIST_M, 3.0) \
    X(OVT_LEAD_STILL_V, 0.5) \
    X(OVT_EGO_STILL_V, 0.5) \
    X(OVT_CONFIRM_TIME_S, 2.0) \
    X(OVT_LEAD_LONG_STILL_S, 8.0) \
    X(OVT_RESUME_LEAD_V, 1.0) \
    X(OVT_LANE_OFFSET_M, 3.5) \
    X(OVT_SHIFT_DONE_M, 2.8) \
    X(OVT_LEAD_PASSED_FWD_M, 12.0) \
    X(OVT_RETURN_DONE_M, 0.4) \
    X(OVT_LANE_SHIFT_RATE_M_S, 0.7) \
    X(OVT_CRUISE_TARGET_V, 5.0) \
    X(OVT_CRUISE_DRIVE_ACCEL, 1.2) \
    X(CURV_NO_ACCEL_THRESH, 0.008) \
    X(CURV_GUARD_LEAD_MARGIN, 0.010) \
    X(CURV_GUARD_RELEASE_RATE, 0.060) \
    X(CURV_IN_CURVE_EXIT_RATIO, 0.60) \
    X(LEAD_MEMORY_S, 1.5) \
    X(LEAD_LOSS_COAST_S, 1.0) \
    X(LEAD_CURVE_LOSS_HOLD_S, 1.0) \
    X(LEAD_CURVE_HOLD_LAT_RATIO, 0.34) \
    X(LEAD_CURVE_HOLD_LAT_MAX, 3.8) \
    X(LEAD_REL_FILTER_ALPHA, 0.12) \
    X(LEAD_V_PROJ_FILTER_ALPHA, 0.18) \
    X(LEAD_ACCEL_FILTER_ALPHA, 0.20) \
    X(LEAD_REACQ_PROTECT_S, 1.2) \
    X(LEAD_REACQ_MIN_PROJ_RATIO, 0.65) \
    X(LEAD_REACQ_MIN_LAST_RATIO, 0.80) \
    X(LEAD_REACQ_MAX_DECEL, 0.18) \
    X(LEAD_DROP_GLITCH_DIST, 20.0) \
    X(LEAD_DROP_GLITCH_RATIO, 0.55) \
    X(LEAD_DROP_GLITCH_BLEND, 0.92) \
    X(ACC_COMFORT_ACCEL, 0.30) \
    X(ACC_COMFORT_DECEL, 0.45) \
    X(CURVE_EXIT_GRACE_S, 1.2) \
    X(ACC_REACQ_DRIVE_MAX, 0.80) \
    X(ACC_REACQ_BRAKE_MAX, 1.50) \
    X(ACC_REACQ_FF_MAX, 0.60) \
    X(ACC_NORMAL_BRAKE_MAX, 3.00) \
    X(AEB_STOP_HOLD_S, 1.2) \
    X(ACC_MATCH_TAU_S, 1.2) \
    X(ACC_MATCH_BRAKE_MARGIN, 0.6) \
    X(ACC_NO_BRAKE_DIST_MARGIN, 3.0) \
    X(ACC_MIN_VALID_LEAD_V, 2.0) \
    X(ACC_CLOSE_SLOW_LEAD_DIST, 25.0) \
    X(ACC_LEAD_KEEPALIVE_S, 0.12) \
    X(ACC_MAX_VALID_TTC_S, 20.0) \
    X(ACC_LEAD_OPENING_MIN_DIST, 30.0) \
    X(ACC_LEAD_OPENING_DELTA_M, 0.02) \
    X(ACC_LEAD_COLLAPSE_RATIO, 0.70) \
    X(ACC_LEAD_COLLAPSE_EGO_GAP, 1.0) \
    X(ACC_LEAD_ACQUIRE_GRACE_S, 0.8) \
    X(ACC_LEAD_ACQUIRE_MAX_BRAKE, -0.7) \
    X(AEB_PATH_HORIZON_S, 2.5) \
    X(AEB_PATH_STEP_S, 0.1) \
    X(AEB_PATH_LAT_MARGIN, 0.3) \
    X(AEB_PATH_LON_MARGIN, 2.5) \
    X(AEB_PATH_PREFILTER_LAT, 3.5) \
    X(AEB_PATH_TCOL_FULL_S, 1.2) \
    X(AEB_PATH_TCOL_START_S, 2.5) \
    X(AEB_PATH_YAWRATE_SIGN, 1.0) \
    X(AEB_PATH_MIN_EGO_V, 1.0) \
    X(AEB_RSS_REACT_S, 0.5) \
    X(AEB_RSS_EGO_DECEL, 6.0) \
    X(AEB_RSS_OBS_DECEL, 8.0) \
    X(AEB_RSS_MIN_DIST, 2.0)

// ---- 生成变量声明 ----
#define SOC_CONFIG_DECLARE_STRING(NAME, VAL) inline std::string NAME = VAL;
#define SOC_CONFIG_DECLARE_BOOL(NAME, VAL) inline bool NAME = VAL;
#define SOC_CONFIG_DECLARE_INT(NAME, VAL) inline int NAME = VAL;
#define SOC_CONFIG_DECLARE_DOUBLE(NAME, VAL) inline double NAME = VAL;

SOC_CONFIG_STRINGS(SOC_CONFIG_DECLARE_STRING)
SOC_CONFIG_BOOLS(SOC_CONFIG_DECLARE_BOOL)
SOC_CONFIG_INTS(SOC_CONFIG_DECLARE_INT)
SOC_CONFIG_DOUBLES(SOC_CONFIG_DECLARE_DOUBLE)

#undef SOC_CONFIG_DECLARE_STRING
#undef SOC_CONFIG_DECLARE_BOOL
#undef SOC_CONFIG_DECLARE_INT
#undef SOC_CONFIG_DECLARE_DOUBLE

// ============================================================
// class 差异化查表（对应 Python AEB_CLASS_*/AEB_PATH_OBS_HALF_WIDTH 字典）。
// 非标量（Python `_load_param_overrides` 显式跳过字典），故为 constexpr，不可覆盖。
// 索引 = ACTOR_CLASS_*；表值来自 config.py 中字典字面量在“覆盖生效前”的原始值，
// 即便 AEB_MAX_ENGAGE_DIST/AEB_FULL_CONFIRM_CYCLES 本身可被覆盖，这些表也不随之变化
// ——与 Python 行为一致（字典在模块加载时早于 _load_param_overrides() 求值一次性构建）。
// ============================================================
inline constexpr std::array<double, kActorClassCount> AEB_CLASS_TTC_MULT = {
    1.0, 1.0, 1.4, 1.6};
inline constexpr std::array<double, kActorClassCount> AEB_CLASS_ENGAGE_DIST = {
    25.0, 25.0, 32.0, 40.0};
inline constexpr std::array<bool, kActorClassCount> AEB_CLASS_BYPASS_MIN_LEAD_V = {
    false, false, true, true};
inline constexpr std::array<double, kActorClassCount> AEB_CLASS_LAT_GATE_MULT = {
    1.0, 1.0, 1.15, 2.50};
inline constexpr std::array<int, kActorClassCount> AEB_CLASS_FULL_CONFIRM_CYCLES = {
    5, 5, 3, 2};
inline constexpr std::array<double, kActorClassCount> AEB_PATH_OBS_HALF_WIDTH = {
    0.9, 0.9, 0.6, 0.4};

// 按 class 查表；越界（未来新增 class）回退到 VEHICLE 档，语义对应 Python
// `.get(cls, <vehicle 默认>)`——未知 class 一律当车辆处理，绝不越界/崩溃。
template <typename T, std::size_t N>
constexpr T getClassParam(const std::array<T, N>& table, int actor_class) {
    if (actor_class >= 0 && static_cast<std::size_t>(actor_class) < N) {
        return table[static_cast<std::size_t>(actor_class)];
    }
    return table[static_cast<std::size_t>(ACTOR_CLASS_VEHICLE)];
}

// ============================================================
// params.yaml 覆盖机制（对应 Python `_load_param_overrides()`）
// ============================================================

// 单条覆盖记录：名称 + 覆盖前/后的字符串表示，供日志/诊断使用
// （对应 Python `APPLIED_PARAM_OVERRIDES` 字典）。
struct ParamOverride {
    std::string name;
    std::string old_value;
    std::string new_value;
};

// 读取 params_yaml_path 指向的文件（若不存在则直接返回空，不视为错误），
// 用“极简扁平解析”（`KEY: value` / `KEY = value`，`#` 起注释，仅标量，
// 对应 Python `_parse_params_text`）解析后逐项应用：
//   - 名称必须已在覆盖表中登记（即上面 4 个 X-Macro 列表之一）；
//   - 安全关键参数（本文件中的 12 个 constexpr）从不登记，天然被拒绝；
//   - 类型按原变量类型强制转换（bool 接受 1/true/yes/on 等，失败该项跳过）。
// 返回值：实际生效的覆盖列表。
std::vector<ParamOverride> applyParamOverrides(const std::string& params_yaml_path);

}  // namespace soc::config
