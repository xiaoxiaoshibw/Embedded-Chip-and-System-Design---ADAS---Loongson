#pragma once

// 控制子包数据模型——C++ 移植自 lx/SOCCode/control/context.py。

#include <memory>
#include <mutex>
#include <optional>

#include "soc/config.hpp"

namespace soc {

// 来自 ROS 话题的车辆感知信号，每个回调更新对应字段。
//
// Python 版本把 `threading.Lock` 内嵌进 dataclass 本身，`with signals:` 直接锁字段写入，
// `snapshot()` 在锁保护下做 `copy.copy()`。C++ 版本把锁固定在类内部（不对外暴露），
// 只导出两个入口：
//   - `withLock(fn)`：在锁保护下执行一组字段写入（对应 Python `with self.signals:`）。
//   - `snapshot()`：锁保护下的值拷贝（对应 Python `signals.snapshot()`），返回值类型
//     `VehicleSignals::Data`，保证快照内所有字段来自同一帧或相邻帧，不会撕裂。
class VehicleSignals {
public:
    struct Data {
        double ego_x = 0.0;               // 自车 X 坐标
        double ego_y = 0.0;               // 自车 Y 坐标
        double ego_yaw = 0.0;             // 自车航向角 (rad)
        double ego_v = 0.0;               // 自车速度 (m/s)
        double lead_x = 0.0;              // 前车 X 坐标
        double lead_y = 0.0;              // 前车 Y 坐标
        double lead_yaw = 0.0;            // 前车航向角 (rad)
        double lead_v = 0.0;              // 前车速度 (m/s)
        // 主前车的 actor class：0=UNKNOWN（缺省），1=VEHICLE，2=OBSTACLE，3=PEDESTRIAN；
        // AEB 按 class 取差异化 TTC 阈值。多目标未启用/class 话题未发布时维持 0。
        int lead_cls = 0;
        double road_psi = 0.0;            // 道路航向角 (rad)
        double lane_offset = 0.0;         // 车道横向偏移 (m)
        bool ego_received = false;         // 是否已收到自车位姿
        bool ego_psi_received = false;     // 是否已收到自车航向话题
        bool lead_received = false;        // 是否已收到前车位姿
        bool road_received = false;        // 是否已收到道路航向
        bool lane_offset_received = false; // 是否已收到车道偏移
        double lead_last_rx_time = 0.0;    // 前车位姿最近接收时刻
        double lead_v_last_rx_time = 0.0;  // 前车速度最近接收时刻
        double lane_offset_last_rx = 0.0;  // 车道偏移最近接收时刻
        double ego_last_rx = 0.0;          // 自车位姿最近接收时刻（卡帧检测）
        double road_last_rx = 0.0;         // 道路航向最近接收时刻（卡帧检测）
        // /car{N}_class 话题最近接收时刻：超过 LEAD_CLASS_STALE_TIMEOUT_S 视为 class
        // 信息陈旧（仅遥测/限频告警，不主动降级 cls 值——见 health 模块注释）。
        double lead_cls_last_rx_time = -1e9;
    };

    // 在锁保护下执行一组字段写入（ROS 回调线程调用）。
    // 用法：signals.withLock([](Data& d) { d.ego_x = x; d.ego_y = y; });
    template <typename Fn>
    void withLock(Fn&& fn) {
        std::lock_guard<std::mutex> guard(mutex_);
        fn(data_);
    }

    // 返回当前状态的值拷贝快照，供控制循环单周期使用（控制循环线程调用）。
    // 锁保护下执行拷贝，确保拷贝期间 ROS 回调不会修改任何字段。
    Data snapshot() const {
        std::lock_guard<std::mutex> guard(mutex_);
        return data_;
    }

private:
    mutable std::mutex mutex_;
    Data data_;
};

// 运行时可调控制增益。
struct ControlGains {
    double lat_kp = config::K_PSI_P;
    double lat_ki = config::K_PSI_I;
    double lat_kd = config::K_PSI_D;
    double acc_kd = config::ACC_KD;
    double acc_ki = config::ACC_KI;
    double acc_kv = config::ACC_KV;
    double acc_ka = config::ACC_KA;
};

// 横向控制单周期计算结果，传递给纵向模块和主循环。
// Python 版本是 frozen dataclass（构造后不可变）；C++ 版本用普通 struct（值语义，
// 通过“构造后不修改”的使用约定维持同等不可变语义，而非语言层强制）。
struct LateralContext {
    double dyn_prev = 0.0;         // 自适应预览时间 (s)
    double rrate = 0.0;            // 道路航向变化率 (rad/s)
    double prev_psi = 0.0;         // 预览航向角 (rad)
    double raw_curv = 0.0;         // 原始曲率 (1/m)
    double curv_guard = 0.0;       // 曲率保护值（= max(|滤波|, |原始|)）
    bool in_curve = false;         // 是否处于弯道
    double delta = 0.0;            // 最终方向盘转角 (rad)
    double delta_ff = 0.0;         // 曲率前馈方向盘转角分量 (rad)
    double delta_cte = 0.0;        // CTE 修正方向盘转角分量 (rad)
    double boundary_delta = 0.0;   // 边界修正转角 (rad)
    double boundary_brake = 0.0;   // 边界制动力 (m/s²)
    bool boundary_warn = false;    // 是否触发边界预警
    double raw_cte = 0.0;          // 原始横向偏移 (m)
    double cur_off = 0.0;          // 当前车道偏移 (m)
    double upd_psi = 0.0;          // 预测下一拍航向角 (rad)
};

// 控制环需要跨周期保持的内部状态。每个 100Hz 周期读写这些字段，
// 实现积分器、滤波器等有状态算法。
struct ControlMemory {
    explicit ControlMemory(double dt_) : dt(dt_) {}

    double dt;                                  // 控制周期 (s)
    int cycle_count = 0;                        // 周期计数器
    double psi_i_term = 0.0;                     // 航向 PID 的 I 项累积
    double psi_prev_err = 0.0;                   // 航向 PID 的上一拍误差
    double filtered_road_psi = 0.0;              // 低通滤波后的道路航向
    double prev_road_psi = 0.0;                  // 上一拍道路航向（用于计算转向率）
    double last_delta = 0.0;                     // 上一拍方向盘转角（用于变化率限制）
    double filtered_cte = 0.0;                   // 低通滤波后的横向偏移
    double cte_prev = 0.0;                       // 上一拍 CTE（用于微分）
    double filtered_curv = 0.0;                  // 低通滤波后的曲率
    double curv_guard_hold = 0.0;                // 曲率保护峰值保持（抗尖刺 + 慢释放跨拍状态）
    bool in_curve_latch = false;                 // in_curve 滞回锁存（防边界 0/1 翻转）
    double filtered_v_tgt = 0.0;                 // 低通滤波后的目标速度
    double filtered_lead_v_proj = 0.0;           // 低通滤波后的前车投影速度
    double last_lead_v_proj = 0.0;               // 上一拍前车投影速度
    double last_lead_v_time = 0.0;               // 上一拍前车速度时间戳
    double filtered_lead_accel = 0.0;            // 低通滤波后的前车加速度
    double last_curve_t = -1e9;                  // 上次检测到弯道的时间
    bool last_acc_has_lead = false;              // 上周期 ACC 是否有前车
    double last_lead_reacq_t = -1e9;             // 上次重新获取前车的时间
    double filtered_lon = 0.0;                   // 低通滤波后的纵向指令
    bool acc_acquire_ff_clamp_logged = false;    // 前车重获制动夹紧日志标记
    int aeb_full_confirm_count = 0;              // AEB 全制动确认计数
    double lane_safe_margin = 0.0;               // 车道安全余量 (m)
    double lane_warn_margin = 0.0;               // 车道预警余量 (m)
    double lane_hard_margin = 0.0;               // 车道硬边界余量 (m)
    ControlGains gains;                          // 运行时控制增益
    double driver_set_speed = config::DRIVER_SET_SPEED;
    double system_max_cruise = config::SYSTEM_MAX_CRUISE;
    double road_limit_speed = config::ROAD_LIMIT_SPEED;
    int runtime_command_seq = 0;
    // 横向控制帧门控：仿真端以 20Hz 发布感知，本循环以 100Hz 运行，
    // 仅在道路航向有新帧时推进横向有状态计算，其余拍沿用上一帧结果。
    double lat_last_road_rx = -1.0;              // 上次已处理的道路航向帧接收时刻
    double lat_last_update_t = -1.0;             // 上次横向更新时刻（用于真实 dt）
    // 帧间沿用的上一帧 LateralContext。Python 版本用 `object = None`（类型擦除）；
    // C++ 版本用具体类型的 std::optional，天然替代"None=未初始化"语义。
    std::optional<LateralContext> lat_cached_ctx;
    // 边界修正与 road_psi 帧门控解耦：边界修正只依赖 lane_offset，
    // 在 road_psi 无新帧、但 lane_offset 有新帧的拍上单独刷新边界。
    std::optional<LateralContext> lat_base_ctx;  // 上一道路帧的"未叠加边界"LateralContext
    double lat_base_delta = 0.0;                 // 上一道路帧"叠加边界前"的方向盘转角 (rad)
    double lat_frame_dt = 0.0;                   // 上一道路帧的真实 dt（边界限幅/upd_psi 复用）
    double lat_last_lane_rx = -1.0;              // 上次已处理的车道偏移帧接收时刻
    // ── 超车（双车道）目标偏移 ──
    // 超车状态机写入的目标横向偏移：0 表示沿路径（右车道）行驶，
    // +OVT_LANE_OFFSET_M 表示切到左车道。横向控制器用 (cte - target_lane_offset)
    // 作为追踪误差，边界判定也使用偏移后的相对值。
    double target_lane_offset = 0.0;
    // 超车 ACTIVE 阶段抑制前车：把 ACC 退化为巡航以便加速绕过停止前车。
    bool suppress_lead_for_overtake = false;
};

// ── 各算法管理器（Phase 2 移植）的前向声明 ──
// Phase 1 阶段这些类型尚未实现；ControlManagers 只持有 unique_ptr<IncompleteType>，
// 析构函数延后到 Phase 2（届时类型完整）再定义——C++ 中 unique_ptr<T> 的默认构造/
// 移动不要求 T 完整，只有析构才需要，因此这里先声明、后定义是合法且常见的 pImpl 手法。
class LaneWidthEstimator;
class LeadTracker;
class AebAlertManager;
class CurveHoldManager;
class LongitudinalController;
class LonSmoothing;
class OvertakeManager;
class MlBridge;
class LateralModelController;
class ComfortLayer;
class LeadEstimator;
class AebPathChecker;

// 各算法管理器的集合，传入控制策略计算函数。nullptr 表示对应特性关闭，
// 对应 Python `ControlManagers` 里 `xxx: object = None`——"None=原行为回退路径"的契约
// 在 C++ 中同样成立：调用侧对每个可选管理器做空检查，行为与未启用时逐字节一致。
struct ControlManagers {
    std::unique_ptr<LaneWidthEstimator> lane_est;
    std::unique_ptr<LeadTracker> lead_tracker;
    std::unique_ptr<AebAlertManager> aeb_alert;
    std::unique_ptr<CurveHoldManager> curve_hold;
    std::unique_ptr<LongitudinalController> lon_ctrl;
    std::unique_ptr<LonSmoothing> lon_smooth;
    std::unique_ptr<OvertakeManager> overtake;              // nullptr = 原行为回退
    std::unique_ptr<MlBridge> ml_bridge;                    // nullptr = ML 关闭
    std::unique_ptr<LateralModelController> lateral_model;  // nullptr = PID 路径
    std::unique_ptr<ComfortLayer> comfort_layer;            // nullptr = legacy 平滑路径
    std::unique_ptr<LeadEstimator> lead_estimator;          // nullptr = legacy 差分路径
    std::unique_ptr<AebPathChecker> aeb_path;               // nullptr = 关闭

    ControlManagers();
    ~ControlManagers();  // 定义延后到 Phase 2（届时上述类型完整）

    ControlManagers(ControlManagers&&) noexcept;
    ControlManagers& operator=(ControlManagers&&) noexcept;
    ControlManagers(const ControlManagers&) = delete;
    ControlManagers& operator=(const ControlManagers&) = delete;
};

}  // namespace soc
