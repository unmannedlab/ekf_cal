# Main Page {#mainpage}

Extended Kalman Filter - Calibration and Localization

## Pages

- @subpage filter
- @subpage software

## Dependencies
The EKF_CAL package has the following hard dependencies that are required for all compilations:
- [OpenCV](https://opencv.org/)
- [Eigen 3](https://eigen.tuxfamily.org/index.php?title=Main_Page)

The following dependencies are for building the ROS node and simulation, respectively
- [ROS2](https://docs.ros.org/en/rolling/index.html)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)

The following soft dependencies useful for development and documentation
- [Doxygen](https://www.doxygen.nl/index.html)
- [Google Test](https://google.github.io/googletest/)

These can be installed by running `rosdep` in the base directory of this repo
```
rosdep install --from-paths src -y --ignore-src
```

## Build
Building can be done simply with the following command:

```
colcon build --symlink-install --packages-select ekf_cal
```

For full CMake support, the following environment variables are expected:
- PLANTUML_INSTALL_DIR
- ROS_DISTRO

## Testing & Static Analysis
Once the package has been built, unit tests and static analysis can be run with the following commands
```
colcon test --packages-select ekf_cal --event-handlers console_direct+
```

A test code coverage report can be generated using the following commands
``` 
colcon build --symlink-install --packages-select ekf_cal \
   --event-handlers console_cohesion+ \
   --cmake-args -DCMAKE_C_FLAGS='--coverage' -DCMAKE_CXX_FLAGS='--coverage'

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
For an example of a filter node launch file, see [example.launch.py](launch/example.launch.py)

In particular, note the configuration file [example.yaml](config/example.yaml).

The configuration file specifies which sensor topics should to use and the initialization values.

## Simulations
Simulations can be run using a YAML configuration file that extends the base configuration file
with additional parameters. See the example [example.yaml](config/example.yaml).

Multiple simulations can be run in parallel using the [run.py](eval/run.py). An example
using a single input is given below

```
python3 eval/run.py config/example.yaml
```

The results of a run can be plotted using [plot.py](eval/plot.py)
```
python3 eval/plot.py config/example.yaml
```

To run and plot in sequence, utilize [evaluate.py](eval/evaluate.py)
```
python3 eval/evaluate.py config/example.yaml
```

## References
1. J. Hartzer and S. Saripalli, "
   Online Multi Camera-IMU Calibration", 
   IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR), 2022