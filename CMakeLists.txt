cmake_minimum_required(VERSION 3.5)
project(ekf_cal)

set(EKF_CAL_MAJOR_VERSION 0)
set(EKF_CAL_MINOR_VERSION 0)
set(EKF_CAL_PATCH_VERSION 0)
set(EKF_CAL_VERSION ${EKF_CAL_MAJOR_VERSION}.${EKF_CAL_MINOR_VERSION}.${EKF_CAL_PATCH_VERSION})

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

if(NOT DEFINED ${ROS_DISTRO})
    set(ROS_DISTRO humble)
endif()

list(APPEND CMAKE_PREFIX_PATH /opt/ros/${ROS_DISTRO}/)

# TODO Should this compile without ROS? Just make EKF library?
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

# EKF sources
set(EKF_SRCS
    src/ekf/EKF.cpp
    src/ekf/Types.hpp
    src/sensors/Camera.cpp
    src/sensors/IMU.cpp
    src/sensors/Sensor.cpp
    src/sensors/Tracker.cpp
)

# Create library with EKF sources
add_library(EKF_LIB ${EKF_SRCS})

ament_target_dependencies(EKF_LIB OpenCV)

target_include_directories(EKF_LIB PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

# Simulation Sources
set(SIM_SRCS
    src/infrastructure/Logger.cpp
    src/infrastructure/sim/SimLog.cpp
)

# Create library with Sim sources
add_library(SIM_LIB ${SIM_SRCS})

# ROS Sources
set(ROS_SRCS
    src/infrastructure/Logger.cpp
    src/infrastructure/ros/RosLog.cpp
    src/sensors/ros/RosCamera.cpp
    src/sensors/ros/RosIMU.cpp
)

# Create library with ROS sources
add_library(ROS_LIB ${ROS_SRCS})

target_link_libraries(ROS_LIB EKF_LIB)

target_include_directories(ROS_LIB PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ROS_LIB ${ROS_PKGS})

install(TARGETS EKF_LIB)

# EKF CAL Node
add_executable(ekf_cal_node src/application/ros/EkfCalNode.cpp)
target_include_directories(ekf_cal_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ekf_cal_node ${ROS_PKGS})
target_link_libraries(ekf_cal_node EKF_LIB ROS_LIB)

# State Publisher Node
add_executable(state_pub_node src/application/ros/StatePublisherNode.cpp)
target_include_directories(state_pub_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(state_pub_node ${ROS_PKGS})
target_link_libraries(state_pub_node EKF_LIB ROS_LIB)

install(TARGETS
    ekf_cal_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    config
    launch
    DESTINATION share/${PROJECT_NAME}/
)

# gtest
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    set(test_files
        src/utility/test/test_EkfCalNode.cpp
        src/utility/test/test_MathHelper.cpp
        src/utility/test/test_TypeHelper.cpp
        src/ekf/test/test_EKF.cpp
        src/sensors/ros/test/test_RosCamera.cpp
        src/sensors/ros/test/test_RosIMU.cpp
        src/sensors/test/test_IMU.cpp
    )

    foreach(f_name IN LISTS test_files)
        get_filename_component(nam ${f_name} NAME_WE)
        ament_add_gtest(${nam} ${f_name})
        ament_target_dependencies(${nam} ${ROS_PKGS})
        target_include_directories(${nam} PUBLIC
            ${CMAKE_SOURCE_DIR}/src/
        )
        target_link_libraries(${nam} EKF_LIB ROS_LIB)
    endforeach()

endif()

ament_package()

# Doxygen
set(DOXYGEN_STUB_FILE ${PROJECT_SOURCE_DIR}/docs/empty.html)

add_custom_target(DOXYGEN
    DEPENDS ${DOXYGEN_STUB_FILE}
)

find_package(Doxygen)
if(NOT DOXYGEN_FOUND)

    message(WARNING "Doxygen is needed to build the documentation.")

else()

    set(DOXYFILE ${PROJECT_SOURCE_DIR}/.doxyfile)

    add_custom_command(
        OUTPUT ${DOXYGEN_STUB_FILE}
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        MAIN_DEPENDENCY ${doxyfile} ${doxyfile_in}
        COMMENT "Generating HTML documentation"
    )
endif()
