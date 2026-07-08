# soc_core: 纯 C++17 算法库，移植自 lx/SOCCode/{config.py,control/*.py,pipeline.py}。
# 无 Linux/ROS2/线程以外的 OS 专属调用，任意平台可编译——本机 Windows 开发/回归用这个库。

add_library(soc_core STATIC
    src/config.cpp
    src/context.cpp
)

target_include_directories(soc_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(soc_core PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(soc_core PRIVATE /W4)
else()
    target_compile_options(soc_core PRIVATE -Wall -Wextra)
endif()

if(SOC_CPP_BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    # Windows: 强制用与本项目一致的运行时库设置，避免 gtest/项目之间 CRT 不匹配链接错误。
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    add_executable(soc_core_tests
        tests/test_config.cpp
        tests/test_context.cpp
    )
    target_link_libraries(soc_core_tests PRIVATE soc_core gtest_main)

    include(GoogleTest)
    gtest_discover_tests(soc_core_tests)
endif()
