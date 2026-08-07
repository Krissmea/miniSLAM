cmake_minimum_required(VERSION 3.10)

project(mini_slam)


# C++版本
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 头文件路径
include_directories(
    include
    /usr/include/eigen3
)


# OpenCV
find_package(OpenCV REQUIRED)
# spdlog / fmt
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# 可执行文件
add_executable(
    mini_slam

    src/main.cpp
    src/dataset.cpp
    src/config.cpp
    src/point_cloud.cpp
    src/feature_tracker.cpp
    src/VO.cpp
    src/pose.cpp
)

target_compile_definitions(
    mini_slam
    PRIVATE MINI_SLAM_DEFAULT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/config/miniSLAM.yaml"
)


# 链接库
target_link_libraries(
    mini_slam

    ${OpenCV_LIBS}
    spdlog::spdlog
    fmt::fmt
)
