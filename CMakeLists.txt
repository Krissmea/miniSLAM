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

# =========================
# C++ standard
# =========================

target_compile_features(
    mini_slam
    PRIVATE
        cxx_std_20
)


# =========================
# miniSLAM headers
# =========================

target_include_directories(
    mini_slam
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

# =========================
# config
# =========================

target_compile_definitions(
    mini_slam
    PRIVATE MINI_SLAM_DEFAULT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/config/miniSLAM.yaml"
)

# =========================
# libraries
# =========================
# 链接库

target_link_libraries(
    mini_slam

    PRIVATE
        krisea::common
        krisea::log
        ${OpenCV_LIBS}
)

# =========================
# output
# =========================

set_target_properties(
    mini_slam
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)