cmake_minimum_required(VERSION 3.10)
project(ekf_cal)

set(EKF_CAL_MAJOR_VERSION 0)
set(EKF_CAL_MINOR_VERSION 2)
set(EKF_CAL_PATCH_VERSION 0)
set(EKF_CAL_VERSION ${EKF_CAL_MAJOR_VERSION}.${EKF_CAL_MINOR_VERSION}.${EKF_CAL_PATCH_VERSION})
configure_file(src/infrastructure/ekf_cal_version.hpp.in infrastructure/ekf_cal_version.hpp)

# Include What You Use
find_program(IWYU_PATH NAMES include-what-you-use iwyu)
set(IWYU_PATH_AND_OPTIONS ${IWYU_PATH} -Xiwyu --no_fwd_decls -Xiwyu --mapping_file=${CMAKE_CURRENT_SOURCE_DIR}/.vscode/iwyu.imp)
if(IWYU_PATH)
    message(STATUS "Using include-what-you-use")
    set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH_AND_OPTIONS})
    set(CMAKE_C_INCLUDE_WHAT_YOU_USE ${IWYU_PATH_AND_OPTIONS})
else (IWYU_PATH)
    message(STATUS "Could not find include-what-you-use")
endif(IWYU_PATH)


# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# @TODO Use this if statement to forgo building ROS components
if(NOT DEFINED ${ROS_DISTRO})
    set(ROS_DISTRO humble)
endif()

list(APPEND CMAKE_PREFIX_PATH /opt/ros/${ROS_DISTRO}/)

# @TODO Should this compile without ROS? Just make EKF library?
set(ROS_PKGS
    ament_cmake
    cv_bridge
    geometry_msgs
    OpenCV
    rclcpp
    sensor_msgs
    std_msgs
    tf2_ros
)
foreach(pkg ${ROS_PKGS})
    find_package(${pkg} REQUIRED)
endforeach()

set(SIM_PKGS
    yaml-cpp
)
foreach(pkg ${SIM_PKGS})
    find_package(${pkg} REQUIRED)
endforeach()

include_directories(PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)


# SIM Infrastructure
set(SIM_INF_SRCS
    src/infrastructure/data_logger.cpp
    src/infrastructure/debug_logger.cpp
    src/infrastructure/sim/sim_debug_logger.cpp
    src/infrastructure/sim/truth_engine_cyclic.cpp
    src/infrastructure/sim/truth_engine_spline.cpp
    src/infrastructure/sim/truth_engine.cpp
)
add_library(SIM_INF ${SIM_INF_SRCS})


# ROS Infrastructure
set(ROS_INF_SRCS
    src/infrastructure/data_logger.cpp
    src/infrastructure/debug_logger.cpp
    src/infrastructure/ros/ros_debug_logger.cpp
)
add_library(ROS_INF ${ROS_INF_SRCS})


# ROS Utilities
set(ROS_UTL_SRCS
    src/utility/ros_helper.cpp
)
add_library(ROS_UTL ${ROS_UTL_SRCS})


# EKF Utilities
set(EKF_UTL_SRCS
    src/utility/math_helper.cpp
    src/utility/string_helper.cpp
    src/utility/type_helper.cpp
)
add_library(EKF_UTL ${EKF_UTL_SRCS})


# EKF sources
set(EKF_SRCS
    src/ekf/ekf.cpp
    src/ekf/types.cpp
    src/ekf/update/imu_updater.cpp
    src/ekf/update/msckf_updater.cpp
    src/ekf/update/updater.cpp
    src/sensors/camera_message.cpp
    src/sensors/camera.cpp
    src/sensors/imu.cpp
    src/sensors/sensor.cpp
    src/trackers/feature_tracker.cpp
)
add_library(EKF_LIB ${EKF_SRCS})
target_link_libraries(EKF_LIB ROS_INF EKF_UTL)


# Simulation Sources
set(SIM_SRCS
    src/sensors/sim/sim_camera.cpp
    src/sensors/sim/sim_imu.cpp
    src/trackers/sim/sim_feature_tracker.cpp
    src/utility/sim/sim_rng.cpp
)
add_library(SIM_LIB ${SIM_SRCS})


# ROS Sources
set(ROS_SRCS
    src/sensors/ros/ros_camera_message.cpp
    src/sensors/ros/ros_camera.cpp
    src/sensors/ros/ros_imu_message.cpp
)
add_library(ROS_LIB ${ROS_SRCS})
target_link_libraries(ROS_LIB EKF_LIB ROS_INF ROS_UTL EKF_UTL)


# EKF CAL Node
add_library(EKF_NODE_LIB src/application/ros/node/ekf_cal_node.cpp)
target_include_directories(EKF_NODE_LIB PUBLIC "${PROJECT_BINARY_DIR}")
add_executable(ekf_cal_node src/application/ros/ekf_cal_main.cpp)
message(STATUS "PROJECT_BINARY_DIR: ${PROJECT_BINARY_DIR}")
target_link_libraries(ekf_cal_node EKF_NODE_LIB EKF_LIB ROS_LIB ROS_INF ROS_UTL EKF_UTL)

# Simulation
add_executable(sim src/application/sim/simulation.cpp)
include_directories(${YAML_CPP_INCLUDE_DIRS})
target_include_directories(sim PUBLIC "${PROJECT_BINARY_DIR}")
target_link_libraries(sim SIM_LIB EKF_LIB SIM_INF EKF_UTL ${YAML_CPP_LIBRARIES})


# Add all ROS dependencies
ament_target_dependencies(ROS_INF rclcpp)
ament_target_dependencies(SIM_LIB OpenCV)
ament_target_dependencies(EKF_LIB OpenCV)
ament_target_dependencies(ROS_UTL ${ROS_PKGS})
ament_target_dependencies(ROS_LIB ${ROS_PKGS})
ament_target_dependencies(EKF_NODE_LIB ${ROS_PKGS})
ament_target_dependencies(ekf_cal_node ${ROS_PKGS})
ament_target_dependencies(sim OpenCV)


# gtest
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    set(test_files
        src/ekf/test/ekf_test.cpp
        src/ekf/test/types_test.cpp
        src/ekf/update/test/imu_updater_test.cpp
        src/ekf/update/test/msckf_updater_test.cpp
        src/infrastructure/sim/test/sim_debug_logger_test.cpp
        src/infrastructure/sim/test/truth_engine_test.cpp
        src/infrastructure/test/data_logger_test.cpp
        src/sensors/ros/test/ros_camera_test.cpp
        src/sensors/ros/test/ros_imu_test.cpp
        src/sensors/test/camera_test.cpp
        src/sensors/test/imu_test.cpp
        src/trackers/test/feature_tracker_test.cpp
        src/utility/test/math_helper_test.cpp
        src/utility/test/ros_helper_test.cpp
        src/utility/test/type_helper_test.cpp
    )

    foreach(f_name IN LISTS test_files)
        get_filename_component(nam ${f_name} NAME_WE)
        ament_add_gtest(${nam} ${f_name})
        ament_target_dependencies(${nam} ${ROS_PKGS})
        target_include_directories(${nam} PUBLIC ${CMAKE_SOURCE_DIR}/src/)
        target_link_libraries(${nam} EKF_LIB ROS_LIB SIM_INF ROS_UTL EKF_UTL)
    endforeach()

    set(ros_infrastructure_tests
        src/infrastructure/ros/test/ros_debug_logger_test.cpp
    )

    foreach(f_name IN LISTS ros_infrastructure_tests)
        get_filename_component(nam ${f_name} NAME_WE)
        ament_add_gtest(${nam} ${f_name})
        ament_target_dependencies(${nam} ${ROS_PKGS})
        target_include_directories(${nam} PUBLIC ${CMAKE_SOURCE_DIR}/src/)
        target_link_libraries(${nam} EKF_LIB ROS_LIB ROS_INF ROS_UTL EKF_UTL)
    endforeach()


    ament_add_gtest(ekf_cal_node_test src/application/ros/node/test/ekf_cal_node_test.cpp)
    ament_target_dependencies(ekf_cal_node_test ${ROS_PKGS})
    target_include_directories(ekf_cal_node_test PUBLIC ${CMAKE_SOURCE_DIR}/src/)
    target_link_libraries(ekf_cal_node_test EKF_NODE_LIB EKF_LIB ROS_LIB ROS_INF ROS_UTL EKF_UTL)
endif()

install(TARGETS
    ekf_cal_node
    DESTINATION lib/${PROJECT_NAME}
)

# @todo(jhartzer): is the config directory necessary to install?
install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

