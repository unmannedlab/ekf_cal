# EKF_CAL

Extended Kalman Filter - Calibration and Localization

## Dependencies
The EKF_CAL package has the following dependencies:
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

## Testing & Static Analysis
Once the package has been built, unit tests and static analysis can be run with the following commands
```
colcon test --packages-select ekf_cal --event-handlers console_direct+
```

A test code coverage report can be generated using the following commands
``` 
colcon test --packages-select ekf_cal --pytest-with-coverage \ 
   --pytest-args --cov-report=term --event-handlers console_direct+
colcon lcov-result --packages-select ekf_cal --filter '*_test.cpp' '*_main.cpp'
```

## Documentation
Documentation can be generated using the following command:
```
doxygen .doxyfile
```

A single pdf can be generated of the documentation using the following command
```
doxygen .doxyfile && cd docs/doxygen/latex && make
```

## Launch
For an example of a filter node launch file, see [imu-cam.launch.py](launch/imu-cam.launch.py)

In particular, note the configuration file [imu-cam.yaml](config/imu-cam.yaml).

The configuration file specifies which sensor topics should to use and the initialization values.

## Simulations
Simulations can be run using a YAML configuration file that extends the base configuration file
with additional parameters. See the example [imu-cam.yaml](config/imu-cam.yaml).

Multiple simulations can be run in parallel using the [run_sim.py](eval/run_sim.py). An example
using a single input is given below

```
python3 eval/run_sim.py config/imu-cam.yaml
```

The results of a run can be plotted using [plot.py](eval/plot.py)
```
python3 eval/plot.py config/imu-cam
```

## References
1. J. Hartzer and S. Saripalli, "
   Online Multi Camera-IMU Calibration", 
   IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR), 2022
