#include "soc/config.hpp"

#include <cstdio>
#include <fstream>

#include <gtest/gtest.h>

namespace {

// 每个用例用独立文件名，避免并行跑测试时互相踩踏。
class TempParamsFile {
public:
    explicit TempParamsFile(const std::string& name) : path_(name) {}
    ~TempParamsFile() { std::remove(path_.c_str()); }

    void write(const std::string& contents) {
        std::ofstream f(path_);
        f << contents;
    }

    const std::string& path() const { return path_; }

private:
    std::string path_;
};

}  // namespace

TEST(ConfigOverride, MissingFileReturnsEmptyWithoutError) {
    auto result = soc::config::applyParamOverrides("this_file_does_not_exist_12345.yaml");
    EXPECT_TRUE(result.empty());
}

TEST(ConfigOverride, OverridesOrdinaryDoubleParam) {
    TempParamsFile file("test_params_double.yaml");
    const double original = soc::config::ACC_KD;
    file.write("ACC_KD: 0.75\n");

    auto result = soc::config::applyParamOverrides(file.path());

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0].name, "ACC_KD");
    EXPECT_DOUBLE_EQ(soc::config::ACC_KD, 0.75);

    soc::config::ACC_KD = original;  // 恢复，避免影响其它测试用例
}

TEST(ConfigOverride, SafetyCriticalParamCannotBeOverridden) {
    TempParamsFile file("test_params_safety.yaml");
    const double before = soc::config::MAX_DELTA;
    file.write("MAX_DELTA: 3.0\n");

    auto result = soc::config::applyParamOverrides(file.path());

    // 安全关键参数从未登记进覆盖表，applyParamOverrides 对它静默跳过——
    // 不在返回列表里，且由于是 constexpr，值物理上不可能改变。
    for (const auto& item : result) {
        EXPECT_NE(item.name, "MAX_DELTA");
    }
    EXPECT_DOUBLE_EQ(soc::config::MAX_DELTA, before);
}

TEST(ConfigOverride, UnknownParamNameIsIgnored) {
    TempParamsFile file("test_params_unknown.yaml");
    file.write("THIS_NAME_DOES_NOT_EXIST: 42\n");

    auto result = soc::config::applyParamOverrides(file.path());
    EXPECT_TRUE(result.empty());
}

TEST(ConfigOverride, BoolCoercionAcceptsCommonForms) {
    TempParamsFile file("test_params_bool.yaml");
    const bool original = soc::config::SAFETY_SUPERVISOR_ENABLED;
    file.write("SAFETY_SUPERVISOR_ENABLED: off\n");

    auto result = soc::config::applyParamOverrides(file.path());

    ASSERT_EQ(result.size(), 1u);
    EXPECT_FALSE(soc::config::SAFETY_SUPERVISOR_ENABLED);

    soc::config::SAFETY_SUPERVISOR_ENABLED = original;
}

TEST(ConfigOverride, BadBoolValueIsSkipped) {
    TempParamsFile file("test_params_bad_bool.yaml");
    const bool original = soc::config::SAFETY_SUPERVISOR_ENABLED;
    file.write("SAFETY_SUPERVISOR_ENABLED: not_a_bool\n");

    auto result = soc::config::applyParamOverrides(file.path());

    EXPECT_TRUE(result.empty());
    EXPECT_EQ(soc::config::SAFETY_SUPERVISOR_ENABLED, original);
}

TEST(ConfigOverride, EqualsSignSyntaxAlsoWorks) {
    TempParamsFile file("test_params_equals.yaml");
    const int original = soc::config::LEAD_CONFIRM_CYCLES;
    file.write("LEAD_CONFIRM_CYCLES = 9\n");

    auto result = soc::config::applyParamOverrides(file.path());

    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(soc::config::LEAD_CONFIRM_CYCLES, 9);

    soc::config::LEAD_CONFIRM_CYCLES = original;
}

TEST(ConfigClassTable, GetClassParamLooksUpByActorClass) {
    using soc::config::AEB_CLASS_TTC_MULT;
    using soc::config::getClassParam;

    EXPECT_DOUBLE_EQ(getClassParam(AEB_CLASS_TTC_MULT, soc::config::ACTOR_CLASS_UNKNOWN), 1.0);
    EXPECT_DOUBLE_EQ(getClassParam(AEB_CLASS_TTC_MULT, soc::config::ACTOR_CLASS_PEDESTRIAN), 1.6);
}

TEST(ConfigClassTable, OutOfRangeClassFallsBackToVehicle) {
    using soc::config::AEB_CLASS_TTC_MULT;
    using soc::config::getClassParam;

    EXPECT_DOUBLE_EQ(getClassParam(AEB_CLASS_TTC_MULT, 99), 1.0);  // VEHICLE 档 = 1.0
    EXPECT_DOUBLE_EQ(getClassParam(AEB_CLASS_TTC_MULT, -1), 1.0);
}
