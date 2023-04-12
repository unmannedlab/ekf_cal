# EKF-CAL

Extended Kalman Filter - Calibration and Localization

## Dependencies
The EKF-CAL package has the following dependencies:
- OpenCV
- Doxygen
- Google Test

These can be installed by running `rosdep` in the base directory of this repo
```
rosdep install --from-paths src -y --ignore-src
```

## Build

Building can be done simply with the following command:

```
colcon build --symlink-install --packages-select ekf_cal
```

For full CMake support, the following expected variables are environment:
- PLANTUML_INSTALL_DIR
- ROS_DISTRO

## Documentation
Documentation can be generated using the following command:
```
doxygen .doxyfile
```

## Launch
For an example of a filter node launch file, see `launch/multi-imu-launch.py`

In particular, note the configuration file `config/multi-imu.yaml` 

The configuration file specifies which sensor topics should to use and the initialization values.

## References
1. J. Hartzer and S. Saripalli, "
   Online Multi Camera-IMU Calibration", 
   IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR), 2022

2. F. M. Mirzaei and S. I. Roumeliotis, 
   "1|A Kalman filter-based algorithm for IMU-camera calibration,"
   2007 IEEE/RSJ International Conference on Intelligent Robots and Systems, 2007, pp. 2427-2434, 
   doi: 10.1109/IROS.2007.4399342.
