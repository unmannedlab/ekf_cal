Getting Started {#tutorial}
============


# Dependencies
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

# Build

Building can be done simply with the following command:

```
colcon build --symlink-install --packages-select ekf_cal
```

## Docker
Alternatively, a Dockerfile is provided, which can be used either inside a VS Code [devcontainer](https://code.visualstudio.com/docs/devcontainers/containers), or a standalone container.

# Input Files

This repository offers two main ways to utilize the Kalman filter framework: a simulation and ROS2 node.
Both the simulation and ROS node are configurable and runnable using identically formatted YAML
files. The basic structure of the YAML files is a set of ROS parameters, lists that define the
sensors to use, and the sensor definitions themselves. The top-level of the YAML is the definition
of the node name `/EkfCalNode` and a ROS2-specific implementation for `ros__parameters`. Structuring
the yaml in this way allows the node to utilize these parameters. The `SimParams` section includes
parameters that are only relevant to the simulation.

```YAML
/EkfCalNode:
    ros__parameters:
        Debug_Log_Level: 2                 # Debug_Log_Level:
                                           #   1: FATAL
                                           #   2: ERROR
                                           #   3: WARN
                                           #   4: INFO
                                           #   5: DEBUG
        Data_Logging_On: True              # Flag to enable data logging
        Body_Data_Rate: 100.0              # EKF body data logging rate
        SimParams:
            Seed: 0.0                      # Seed to provide to random number generator
            UseSeed: True                  # Flag to use seed (required for deterministic runs)
            MaxTime: 10.0                  # Maximum simulation time
            NumberOfRuns: 10               # Number of runs for Multi-Threaded Simulation
            RunNumber: 0                   # Start run number (also used to initialize seed)
            NoErrors: False                # Flag to disable errors
            stationary_time: 5.0           # Time to be stationary before motion begins
            TruthType: "Cyclic"            # TruthType: 
                                           #   "Cyclic"
                                           #   "Spline"
            # Cyclic Parameters:
            PosFrequency: [0.3, 0.5, 0.7]  # Cyclic position frequencies
            AngFrequency: [0.2, 0.3, 0.4]  # Cyclic angular frequencies
            PosOffset: [0.0, 0.0, 0.0]     # Position offset for cyclic position

            # Spline Parameters:
            positions:
                - [0.0, 0.0, 0.0]
                - [0.1, 0.1, 0.1]
                - [-0.1, -0.1, -0.1]
                - [0.0, 0.0, 0.0]
            angles:
                - [0.00, 0.00, 0.00]
                - [0.01, 0.01, 0.01]
                - [0.00, 0.00, 0.00]
```

Sensors and parameters can be dynamically declared with a bit of tomfoolery. To achieve this, we
must read from a known list to the declare and read unknown lists. Therefore, the following lists
must exist, but can be empty. These define the parameter sections for each of their respective
categories.

```YAML
/EkfCalNode:
    ros__parameters:

        ...

        IMU_list:            # List can be empty, but is required to exist
            - "example_imu"

        Camera_list:         # List can be empty, but is required to exist
            - "example_cam"

        Tracker_list:        # List can be empty, but is required to exist
            - "ORB"
```

The following is an example of an IMU input configuration.

```YAML
/EkfCalNode:
    ros__parameters:

        ...

        IMU:
            example_imu:
                UseForPrediction: False  # Flag to use IMU in the prediction step of filter
                is_extrinsic: false         # Flag to use IMU frame as base body frame
                is_intrinsic: False         # Flag to calibrate IMU intrinsics
                Rate: 400.0              # Update rate
                Topic: "/example_imu"    # ROS topic
                # Initial variance
                VarInit: [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]
                PosOffInit: [0.0, 0.0, 0.0]             # Initial position offset estimate
                AngOffInit: [1.0, 0.0, 0.0, 0.0]        # Initial angular offset estimate
                AccBiasInit: [0.0, 0.0, 0.0]            # Initial accelerometer bias estimate
                OmgBiasInit: [0.0, 0.0, 0.0]            # Initial gyroscope bias estimate
                SimParams:
                    timeBias: 0.0                       # Measurement time bias
                    timeSkew: 0.0                       # Measurement time skew
                    timeError: 1.0e-6                   # Measurement time error
                    accBias: [0.0, 0.0, 0.0]            # True accelerometer bias
                    accError: [1.0e-3, 1.0e-3, 1.0e-3]  # Accelerometer error
                    omgBias: [0.0, 0.0, 0.0]            # True gyroscope bias
                    omgError: [1.0e-2, 1.0e-2, 1.0e-2]  # Gyroscope error
                    posOffset: [0.0, 0.0, 0.0]          # True position offset
                    angOffset: [1.0, 0.0, 0.0, 0.0]     # True angular offset
```

The following is an example of an camera input configuration.

```YAML
/EkfCalNode:
    ros__parameters:

        ...

        Camera:
            example_cam:
                Rate: 20.0                               # Sensor rate
                Topic:  "/example_cam"                   # ROS Topic
                PosOffInit: [0.0, 0.0, 0.0]              # Initial position offset estimate
                AngOffInit: [0.5, -0.5, 0.5, -0.5]       # Initial angular offset estimate
                VarInit: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # Initial variance
                Tracker: "ORB"                           # Tracker to use
                SimParams:
                    timeBias: 0.0                        # Measurement time bias
                    timeSkew: 0.0                        # Measurement time skew
                    timeError: 1.0e-6                    # Measurement time error
                    posOffset: [0.0, 0.0, 0.0]           # True position offset
                    angOffset: [0.5, -0.5, 0.5, -0.5]    # True angular offset
```

The following is an example of a tracker input configuration.

```YAML
/EkfCalNode:
    ros__parameters:

        ...

        Tracker:
            ORB:
                FeatureDetector: 4       # Feature Detector:
                                         #   0: BRISK
                                         #   1: FAST,
                                         #   2: GFTT,
                                         #   3: MSER,
                                         #   4: ORB,
                                         #   5: SIFT,
                DescriptorExtractor: 0   # Descriptor Extractor:
                                         #   0: ORB
                                         #   1: SIFT
                DescriptorMatcher: 0     # DescriptorMatcher:
                                         #   0: BRUTE_FORCE
                                         #   1: FLANN
                DetectorThreshold: 10.0  # Threshold used for detection (if available)
                PixelError: 1.0          # Average pixel error
                SimParams:
                    featureCount: 100    # Total number of trackable features
                    roomSize: 10.0       # Maximum distance of features
```



# Simulation

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

This will generate and run the requested number of simulation runs for the specified run time and 
produce plots of the Monte Carlo data. Examples of the resulting plots are shown below:


![body_acceleration_error](images/body_acceleration_error.png)

![body_acceleration_error](images/body_position_covariance.png)

![body_acceleration_error](images/imu_2_residuals.png)

![body_acceleration_error](images/imu_2_position.png)


# Launch ROS2 Node

For an example of a filter node launch file, see [example.launch.py](launch/example.launch.py)

In particular, note the configuration file [example.yaml](config/example.yaml).

The configuration file specifies which sensor topics should to use and the initialization values.
Once built, the ROS node can be started by running the following command

```
ros2 launch example.launch
```


# Documentation

Documentation can be generated using the following command:
```
doxygen .doxyfile
```

A single pdf can be generated of the documentation using the following command
```
doxygen .doxyfile && cd docs/doxygen/latex && make
```


# Testing & Static Analysis

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
