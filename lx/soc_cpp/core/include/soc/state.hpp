#pragma once

// 轻量级控制数据模型——C++ 移植自 lx/SOCCode/control/state.py。
// 全部是纯数据结构（无 I/O、无跨线程共享），值语义，直接对应 Python dataclass 字段。

#include <limits>
#include <optional>
#include <string>

namespace soc {

// 前车跟踪与 ACC 门控结果。
struct LeadContext {
    double x_rel = 0.0;                         // 前车相对于自车的纵向距离 (m)
    double y_rel = 0.0;                         // 前车相对于自车的横向距离 (m)
    bool lead_fresh = false;                     // 前车数据是否在超时内
    double lead_lat_max = 0.0;                   // 当前横向检测窗口 (m)
    double lead_lat_straight = 0.0;              // 直道横向窗口 (m)
    double lead_lat_curve = 0.0;                 // 弯道横向窗口 (m)
    double lead_lat_gate = 0.0;                  // AEB 用的更窄横向门限 (m)
    bool raw_has_lead = false;                    // 原始（未确认）前车存在标志
    bool has_lead = false;                        // 经过确认/记忆机制的前车标志
    bool lead_detected = false;                   // 同 has_lead，语义别名
    bool lead_valid_for_alert = false;            // 是否满足 AEB 告警触发条件
    bool lead_speed_invalid_for_alert = false;    // 前车速度太低不适合告警
    bool acc_has_lead = false;                    // ACC 是否允许使用前车
    bool acc_lead_valid = false;                  // ACC 前车最终有效性
    bool acc_lost_this_cycle = false;             // 本周期 ACC 刚丢失前车
    std::string acc_reject_reason;                // ACC 拒绝前车的理由
    bool lead_in_lane_for_acc = false;            // 前车是否在 ACC 车道门限内
    double predicted_lead_v_proj = 0.0;           // 滤波后的前车投影速度 (m/s)
    bool lane_out_release = false;                // 因前车偏出车道释放 ACC
    bool dist_opening_release = false;            // 因前车远离释放 ACC
    double acc_ff_before = 0.0;                   // 前馈夹紧前的 FF 值
    double acc_ff_after = 0.0;                    // 前馈夹紧后的 FF 值
    double acc_eval_dist = 0.0;                   // ACC 评估用的纵向距离 (m)
    double acc_ttc = std::numeric_limits<double>::infinity();  // ACC 评估用的 TTC (s)
    double raw_lead_v_proj = 0.0;                 // 原始前车投影速度 (m/s)
    bool recent_curve_exit = false;               // 最近退出弯道（还在保护期）
    bool recent_reacq = false;                    // 最近重新获取前车（在保护期内）
    bool lead_acquired = false;                   // 本周期新获取到前车
    int lead_cls = 0;                             // 主前车 actor class（透传给 AEB 选阈值）
};

// 纵向控制单周期计算结果。
struct LongitudinalContext {
    double lon_cmd = 0.0;                          // 最终纵向加速度指令 (m/s², 正=减速)
    bool aeb_active = false;                       // 是否 AEB 激活
    double dist = 999.99;                          // 前车距离 (m)
    double ttc = std::numeric_limits<double>::infinity();  // 碰撞时间 (s)
    double lead_v_proj = 0.0;                      // 前车投影速度 (m/s)
    double min_safe_dist = 0.0;                    // 最小安全距离 (m)
    bool lead_acquire_grace_active = false;        // 前车获取保护期是否活跃
    double acc_ff_before = 0.0;                    // 前车加速度 FF 限幅前
    double acc_ff_after = 0.0;                     // 前车加速度 FF 限幅后
    double closing_speed = 0.0;                    // 接近速度 (m/s)
    // ── AEB 预测路径碰撞检查诊断（control/aeb_path_check.py；仅启用时回填）──
    bool aeb_path_risk = false;                     // 本拍路径/RSS 命中（未消抖）
    double aeb_path_tcol = std::numeric_limits<double>::infinity();  // 最早预测碰撞时间 (s)
    double aeb_rss_dist = 0.0;                      // 触发目标 RSS 最小安全距离 (m)
};

// 前车跟踪模块的输入数据。
struct LeadTrackingInputs {
    double ego_x = 0.0;
    double ego_y = 0.0;
    double ego_yaw = 0.0;
    double ego_v = 0.0;
    double lead_x = 0.0;
    double lead_y = 0.0;
    double lead_yaw = 0.0;
    double lead_v = 0.0;
    int lead_cls = 0;
    bool lead_received = false;
    double lead_last_rx_time = 0.0;
    double filtered_curv = 0.0;
    double cur_lane_width = 0.0;
    bool lane_locked = false;
    bool last_acc_has_lead = false;
    double filtered_lead_v_proj = 0.0;
    double last_lead_v_proj = 0.0;
    double last_lead_reacq_t = -1e9;
    double last_curve_t = -1e9;  // 最后一次检测到弯道的时间（来自 ControlMemory）
};

// 前车跟踪模块需跨周期保持的状态。
struct LeadTrackerState {
    double filtered_x_rel = 0.0;                  // 滤波后的纵向相对距离
    double filtered_y_rel = 0.0;                  // 滤波后的横向相对距离
    double filtered_v_proj = 0.0;                 // 滤波后的前车投影速度
    bool rel_filter_primed = false;               // 相对位置/速度滤波器是否已用首帧测量值初始化
    double prev_abs_y_rel = -1.0;                 // 上一拍 |y_rel|（切入横向逼近速率，-1=未初始化）
    double prev_y_rel_t = -1e9;                   // 上一拍 y_rel 时间戳
    double cutin_lat_rate = 0.0;                  // 低通后的横向逼近速率 (m/s)
    double last_confirmed_lead_t = -1e9;          // 上次确认前车时间
    double last_lead_x_rel = 0.0;                 // 上次确认的纵向距离
    double last_lead_y_rel = 0.0;                 // 上次确认的横向距离
    int lead_confirm_count = 0;                   // 连续确认计数
    std::optional<double> prev_acc_eval_dist;     // 上一次 ACC 评估距离
    double prev_acc_eval_lead_v_proj = 0.0;       // 上一次前车投影速度
    double last_acc_lead_valid_t = -1e9;          // 上次 ACC 有效时间
    std::string last_acc_reject_reason;           // 上次 ACC 拒绝理由
    std::string last_acc_release_reason;          // 上次 ACC 释放理由
    int acc_lane_out_release_count = 0;           // 前车偏出车道连续计数
    int acc_dist_opening_release_count = 0;       // 前车远离连续计数
};

// AEB 告警状态机。
struct AebAlertState {
    bool active = false;             // 告警是否激活
    double start_t = 0.0;            // 激活起始时间
    bool has_lead = false;           // 是否有前车（用于告警退出判断）
    bool armed = false;              // 是否已就绪（见过有效前车）
    double last_lead_time = 0.0;     // 上次见到有效前车的时间
    double hold_speed = 0.0;         // 告警时自车速度（用于保持）
    double cooldown_until = 0.0;     // 冷却截止时间
    double stop_hold_until = 0.0;    // 停车保持截止时间
};

// 弯道保持状态机。
struct CurveHoldState {
    bool active = false;             // 是否处于弯道保持模式
    double v_target = 0.0;           // 保持的目标速度 (m/s)
    double start_t = 0.0;            // 激活起始时间
    double v_i = 0.0;                // 速度积分项
    bool prev_has_lead = false;      // 上周期是否有前车
    bool prev_raw_has_lead = false;  // 上周期原始前车标志
    double loss_since = -1e9;        // 前车丢失起始时间
    double reacq_since = -1e9;       // 前车重新获取起始时间
};

}  // namespace soc
