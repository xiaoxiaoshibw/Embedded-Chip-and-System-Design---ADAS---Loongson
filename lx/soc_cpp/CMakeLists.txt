cmake_minimum_required(VERSION 3.16)
project(soc_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(SOC_CPP_BUILD_TESTS "Build GoogleTest unit tests for soc_core" ON)

# core/ 是纯 C++17 算法库，无 Linux/ROS2 专属调用，可在任意平台（含本机 Windows）
# 独立编译验证——这是本仓库在 Windows 开发机上推进 SOCCode C++ 移植回归的关键前提。
add_subdirectory(core)

# ros_node/ 依赖 rclcpp，只有在 ROS2（ament_cmake）环境下才编译（Nano / 龙芯上，Phase 4）。
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  add_subdirectory(ros_node)
endif()
