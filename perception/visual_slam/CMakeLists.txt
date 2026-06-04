cmake_minimum_required(VERSION 3.10)
project(slam_practice)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 生成 compile_commands.json,让 VSCode + clangd 不再飘红
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)

# ---- ch3 Eigen 练习 ----
# 现代 CMake:直接 link imported target,自动带上头文件路径
add_executable(ex01_basics ch3_eigen/ex01_basics.cpp)
target_link_libraries(ex01_basics Eigen3::Eigen)

add_executable(ex02_rotation ch3_eigen/ex02_rotation.cpp)
target_link_libraries(ex02_rotation Eigen3::Eigen)

add_executable(ex03_gimbal_lock ch3_eigen/ex03_gimbal_lock.cpp)
target_link_libraries(ex03_gimbal_lock Eigen3::Eigen)

add_executable(ex04_exercises ch3_eigen/ex04_exercises.cpp)
target_link_libraries(ex04_exercises Eigen3::Eigen)

# ---- ch4 Sophus 练习(header-only,见 ch4_sophus/README.md)----
# 不用 find_package(Sophus):其 Config 要求 Eigen 3.4 主版本,与本机 Eigen 5 冲突。
# Sophus 头与 Eigen5 已验证兼容,故直接定位头文件 + 链接 Eigen 即可。
# 未装 Sophus 时跳过,不影响 ch3 构建。
find_path(SOPHUS_INCLUDE_DIR sophus/se3.hpp PATHS /opt/homebrew/include /usr/local/include)
if(SOPHUS_INCLUDE_DIR)
    foreach(ex ex01_sophus_basics ex02_trajectory_error ex03_adjoint)
        add_executable(${ex} ch4_sophus/${ex}.cpp)
        target_include_directories(${ex} PRIVATE ${SOPHUS_INCLUDE_DIR})
        target_compile_definitions(${ex} PRIVATE SOPHUS_USE_BASIC_LOGGING)
        target_link_libraries(${ex} Eigen3::Eigen)
    endforeach()
else()
    message(STATUS "Sophus 头文件未找到,跳过 ch4_sophus(安装步骤见 ch4_sophus/README.md)")
endif()
