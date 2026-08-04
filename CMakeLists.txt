cmake_minimum_required(VERSION 3.10)

project(mini_slam)


# C++版本
set(CMAKE_CXX_STANDARD 17)
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
    src/camera.cpp
    src/config.cpp
    src/point_cloud.cpp
    src/feature_tracker.cpp
    src/VO.cpp
)


# 链接库
target_link_libraries(
    mini_slam

    ${OpenCV_LIBS}
)