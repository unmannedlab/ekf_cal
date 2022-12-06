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
    set(ROS_DISTRO foxy)
endif()

list(APPEND CMAKE_PREFIX_PATH /opt/ros/${ROS_DISTRO}/)

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
endforeach()

# EKF sources
# TODO: Fix separate linking
set(EKF_SRCS
    src/ekf/EKF.cpp
    src/sensors/Sensor.cpp
)

# Create library with EKF sources
add_library(EKF_LIB ${EKF_SRCS})

target_include_directories(EKF_LIB PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

# ROS Infrastructure Sources
set(ROS_INFRASTRUCTURE
    src/infrastructure/Logger.cpp
    src/infrastructure/ros/RosLog.cpp
    src/sensors/ros/RosCamera.cpp
    src/sensors/ros/RosIMU.cpp
    src/sensors/Camera.cpp
    src/sensors/IMU.cpp
    src/ekf/EKF.cpp
    src/sensors/Sensor.cpp
)

add_library(ROS_INF_LIB ${ROS_INFRASTRUCTURE})
target_include_directories(ROS_INF_LIB PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ROS_INF_LIB ${ROS_PKGS})

install(TARGETS EKF_LIB)

# EKF CAL Node
add_executable(ROS_APPLICATION src/application/ros/EkfCalNode.cpp)
target_include_directories(ROS_APPLICATION PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ROS_APPLICATION ${ROS_PKGS})
target_link_libraries(ROS_APPLICATION EKF_LIB ROS_INF_LIB)

install(TARGETS
    ROS_APPLICATION
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
        src/sensors/ros/test/test_Camera.cpp
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
        target_link_libraries(${nam} EKF_LIB ROS_INF_LIB)
    endforeach()

endif()

ament_package()

# Doxygen
set(DOXYGEN_STUB_FILE ${PROJECT_SOURCE_DIR}/docs/empty.html)

add_custom_target(DOXYGEN all
    DEPENDS ${DOXYGEN_STUB_FILE}
)

find_package(Doxygen)
if(NOT DOXYGEN_FOUND)

    message(FATAL_ERROR "Doxygen is needed to build the documentation.")

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
