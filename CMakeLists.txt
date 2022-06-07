##--------------------------------------------------------------------------------------------------------------------##
##                                                                                                                    ##
##                                                      EKF-CAL                                                       ##
##                                                                                                                    ##
##                                       Kalman Filter-Based Sensor Calibration                                       ##
##                                                                                                                    ##
##                                          Copyright (C) 2021 Jacob Hartzer                                          ##
##                                                                                                                    ##
## This program is free software: you can redistribute it and/or modify it under the terms of the                     ##
## GNU General Public License as published by the Free Software Foundation, either version 3 of the License,          ##
## or (at your option) any later version.                                                                             ##
##                                                                                                                    ##
## This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;                          ##
## without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                          ##
## See the GNU General Public License for more details.                                                               ##
##                                                                                                                    ##
## You should have received a copy of the GNU General Public License along with this program.                         ##
## If not, see <https://www.gnu.org/licenses/>.                                                                       ##
##                                                                                                                    ##
##--------------------------------------------------------------------------------------------------------------------##

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

# Nodes
add_subdirectory(src/ekf)

add_executable(${PROJECT_NAME} src/node/EkfCalNode.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp std_msgs sensor_msgs geometry_msgs)
target_link_libraries(${PROJECT_NAME} EKF)
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
