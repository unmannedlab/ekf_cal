
set(test_files
    test_EkfCalNode.cpp
    test_MathHelper.cpp
    test_TypeHelper.cpp
    ekf/test_EKF.cpp
    ekf/sensors/test_Camera.cpp
    ekf/sensors/test_Imu.cpp
)

foreach(f_name IN LISTS test_files)
    get_filename_component(nam ${f_name} NAME_WE)
    ament_add_gtest(${nam} ${f_name})
    ament_target_dependencies(${nam} ${ROS_PKGS})
    target_include_directories(${nam} PUBLIC
        ${CMAKE_SOURCE_DIR}/src/
        ${CMAKE_SOURCE_DIR}/test/
    )
    target_link_libraries(${nam} ${PROJECT_NAME})
endforeach()
