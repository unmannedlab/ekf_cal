cmake_minimum_required(VERSION 3.5)
project(ekf_cal)

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

list(APPEND CMAKE_PREFIX_PATH /opt/ros/foxy/)

# find dependencies
set(ROS_PKGS
    ament_cmake
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_ros
)
foreach(pkg ${ROS_PKGS})
    find_package(${pkg} REQUIRED)
endforeach(pkg)

# List all sources
set(${PROJECT_NAME}_LIB
    src/ekf/EKF.cpp
    src/ekf/sensors/Camera.cpp
    src/ekf/sensors/Imu.cpp
    src/ekf/sensors/Lidar.cpp
    src/ekf/sensors/Sensor.cpp
)

# Create library with sources
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME})

# EKF CAL Node
add_executable(${PROJECT_NAME}_node src/EkfCalNode.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp rclcpp std_msgs sensor_msgs geometry_msgs tf2_ros)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)
endif()

ament_package()
