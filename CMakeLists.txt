cmake_minimum_required(VERSION 3.16)

project(mini_slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


# =============================================================================
# VIO algorithm library (ROS independent)
# =============================================================================

set(MINI_SLAM_VIO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/algorithms/vio")

add_library(
    mini_slam_vio
    # VO
    ${MINI_SLAM_VIO_DIR}/VO.cpp

    # estimator
    ${MINI_SLAM_VIO_DIR}/src/estimator/config.cpp

    # feature manager

    # feature tracker
    ${MINI_SLAM_VIO_DIR}/src/feature_Tracker/feature_tracker.cpp

    # initial

    # utility
    ${MINI_SLAM_VIO_DIR}/src/utility/point_cloud.cpp
    ${MINI_SLAM_VIO_DIR}/src/utility/pose.cpp
)

add_library(krisea::mini_slam_vio ALIAS mini_slam_vio)

target_include_directories(
    mini_slam_vio
    PUBLIC
        # VO.hpp
        ${MINI_SLAM_VIO_DIR}

        # module headers
        ${MINI_SLAM_VIO_DIR}/include
        ${MINI_SLAM_VIO_DIR}/include/estimator

        ${MINI_SLAM_VIO_DIR}/include/feature_Tracker
        
        ${MINI_SLAM_VIO_DIR}/include/utility

        # OpenCV public headers are used by your headers
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
    mini_slam_vio
    PUBLIC
        krisea::common
        krisea::log
        Eigen3::Eigen
        ${OpenCV_LIBS}
)

target_compile_features(mini_slam_vio PUBLIC cxx_std_20)

# =============================================================================
# Offline dataset application
# =============================================================================

add_executable(
    mini_slam
    apps/mini_slam_main.cpp
    ${MINI_SLAM_VIO_DIR}/src/estimator/dataset.cpp
)

target_compile_definitions(
    mini_slam
    PRIVATE
        MINI_SLAM_DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/tum.yaml"
)

target_link_libraries(mini_slam PRIVATE krisea::mini_slam_vio)

set_target_properties(
    mini_slam
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# =============================================================================
# ROS 2 adapter and application
# =============================================================================

option(MINI_SLAM_BUILD_ROS2 "Build the ROS 2 adapter and node" ON)

if(MINI_SLAM_BUILD_ROS2)
    find_package(ament_cmake QUIET)

    if(ament_cmake_FOUND)
        find_package(rclcpp REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(nav_msgs REQUIRED)
        find_package(message_filters REQUIRED)

        # Communication/conversion only. It intentionally does not link VIO.
        add_library(
            mini_slam_ros_adapter
            src/adapter/ros_adapter.cpp
            src/adapter/ros_converter.cpp
        )

        add_library(krisea::mini_slam_ros_adapter ALIAS mini_slam_ros_adapter)

        target_include_directories(
            mini_slam_ros_adapter
            PUBLIC
                ${CMAKE_CURRENT_SOURCE_DIR}/include
                ${OpenCV_INCLUDE_DIRS}
        )

        target_link_libraries(
            mini_slam_ros_adapter
            krisea::common
            krisea::log
            Eigen3::Eigen
            ${OpenCV_LIBS}
        )

        target_compile_features(mini_slam_ros_adapter PUBLIC cxx_std_20)

        ament_target_dependencies(
            mini_slam_ros_adapter
            rclcpp
            sensor_msgs
            geometry_msgs
            nav_msgs
            message_filters
        )

        # Composition point only: this executable connects adapter and algorithm.
        add_executable(
            mini_slam_ros_node
            apps/mini_slam_ros_main.cpp
        )

        target_compile_definitions(
            mini_slam_ros_node
            PRIVATE
                MINI_SLAM_DEFAULT_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/orbbec_ros.yaml"
        )

        target_link_libraries(
            mini_slam_ros_node
            krisea::mini_slam_vio
            krisea::mini_slam_ros_adapter
        )

        ament_target_dependencies(mini_slam_ros_node rclcpp)

        set_target_properties(
            mini_slam_ros_node
            PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
        )
    else()
        message(STATUS "ROS 2 not found: skipping mini_slam_ros_adapter and mini_slam_ros_node")
    endif()
endif()
