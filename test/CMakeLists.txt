
set(test_files
    test_EkfCalNode.cpp
    test_EkfHealthNode.cpp
    test_MathHelper.cpp
    test_TypeHelper.cpp
    test_VizNode.cpp
    ekf/test_EKF.cpp
    ekf/sensors/test_Camera.cpp
    ekf/sensors/test_Imu.cpp
    ekf/sensors/test_Lidar.cpp
    ekf/sensors/test_Sensor.cpp
)
foreach(f_name IN LISTS test_files)
    get_filename_component(nam ${f_name} NAME_WE)
    ament_add_gtest(${nam} ${f_name} ${${PROJECT_NAME}_src})
    target_link_libraries(${nam} ${PROJECT_NAME})
endforeach()
