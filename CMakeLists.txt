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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(${PROJECT_NAME}_src
    src/EkfCalNode.cpp
    src/ekf/EKF.cpp
    src/ekf/sensors/Camera.cpp
    src/ekf/sensors/Imu.cpp
    src/ekf/sensors/Lidar.cpp
    src/ekf/sensors/Sensor.cpp
)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_src})

ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp std_msgs sensor_msgs geometry_msgs)

target_link_libraries(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    src
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    config launch
    DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
