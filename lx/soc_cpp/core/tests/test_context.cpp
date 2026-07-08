#include "soc/context.hpp"

#include <atomic>
#include <thread>

#include <gtest/gtest.h>

TEST(VehicleSignalsTest, WithLockWritesAreVisibleInSnapshot) {
    soc::VehicleSignals signals;
    signals.withLock([](soc::VehicleSignals::Data& d) {
        d.ego_x = 1.5;
        d.ego_y = -2.5;
        d.ego_received = true;
        d.lead_cls = 3;
    });

    auto snap = signals.snapshot();
    EXPECT_DOUBLE_EQ(snap.ego_x, 1.5);
    EXPECT_DOUBLE_EQ(snap.ego_y, -2.5);
    EXPECT_TRUE(snap.ego_received);
    EXPECT_EQ(snap.lead_cls, 3);
}

TEST(VehicleSignalsTest, SnapshotIsIndependentCopy) {
    soc::VehicleSignals signals;
    signals.withLock([](soc::VehicleSignals::Data& d) { d.ego_x = 1.0; });

    auto snap = signals.snapshot();
    signals.withLock([](soc::VehicleSignals::Data& d) { d.ego_x = 2.0; });

    // 快照是值拷贝，不应随后续写入变化——这是控制循环单周期使用快照的前提。
    EXPECT_DOUBLE_EQ(snap.ego_x, 1.0);
    EXPECT_DOUBLE_EQ(signals.snapshot().ego_x, 2.0);
}

// 并发原子性测试：写线程把 ego_x/ego_y 锁保护下成对更新为相同值，
// 读线程反复 snapshot() 并断言 ego_x == ego_y 恒成立。
// 若 snapshot() 没有正确持锁，读线程可能拿到"半帧"（ego_x 是新值、ego_y 还是旧值），
// 断言会失败——这正是 Python 版本 docstring 里强调的"撕裂窗口"问题。
TEST(VehicleSignalsTest, SnapshotUnderConcurrentWritesNeverTears) {
    soc::VehicleSignals signals;
    std::atomic<bool> stop{false};
    std::atomic<int> mismatches{0};

    std::thread writer([&] {
        for (int i = 0; i < 20000 && !stop; ++i) {
            double v = static_cast<double>(i);
            signals.withLock([v](soc::VehicleSignals::Data& d) {
                d.ego_x = v;
                d.ego_y = v;
            });
        }
    });

    std::thread reader([&] {
        for (int i = 0; i < 20000; ++i) {
            auto snap = signals.snapshot();
            if (snap.ego_x != snap.ego_y) {
                mismatches.fetch_add(1);
            }
        }
        stop = true;
    });

    writer.join();
    reader.join();

    EXPECT_EQ(mismatches.load(), 0);
}

TEST(ControlMemoryTest, ConstructsWithRequiredDtAndDefaults) {
    soc::ControlMemory memory(0.01);

    EXPECT_DOUBLE_EQ(memory.dt, 0.01);
    EXPECT_EQ(memory.cycle_count, 0);
    EXPECT_FALSE(memory.in_curve_latch);
    EXPECT_FALSE(memory.lat_cached_ctx.has_value());
    EXPECT_FALSE(memory.lat_base_ctx.has_value());
    EXPECT_DOUBLE_EQ(memory.gains.lat_kp, soc::config::K_PSI_P);
    EXPECT_DOUBLE_EQ(memory.driver_set_speed, soc::config::DRIVER_SET_SPEED);
}

TEST(ControlMemoryTest, LatCachedCtxHoldsLateralContextValue) {
    soc::ControlMemory memory(0.01);
    soc::LateralContext ctx;
    ctx.delta = 0.1;
    ctx.in_curve = true;

    memory.lat_cached_ctx = ctx;

    ASSERT_TRUE(memory.lat_cached_ctx.has_value());
    EXPECT_DOUBLE_EQ(memory.lat_cached_ctx->delta, 0.1);
    EXPECT_TRUE(memory.lat_cached_ctx->in_curve);
}
