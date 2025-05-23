Parameter Definitions {#parameters}
============

## Main parameters

The basic structure of the YAML files is a set of ROS parameters, lists that define the sensors to use, and the sensor definitions themselves. The top-level of the YAML is the definition of the node name `/EkfCalNode` and a ROS2-specific implementation for `ros__parameters`. Structuring the yaml in this way allows the node to utilize these parameters. The `sim_params` section includes parameters that are only relevant to the simulation. The main simulation and node parameters are as follows

```{.py}
/EkfCalNode:
    ros__parameters:
        debug_log_level: 2                    # Debug_Log_Level:
                                              #   1: FATAL
                                              #   2: ERROR
                                              #   3: WARN
                                              #   4: INFO
                                              #   5: DEBUG
        data_logging_on: true                 # Flag to enable data logging
        data_log_rate: 10.0                   # EKF body data logging rate
        augmenting_type: 0                    # Camera state augmentation type
                                              #   0: No camera frames are augmented
                                              #   1: All camera frames are augmented
                                              #   2: Only the primary camera frames are augmented
                                              #   3: Time-based frame augmentation
                                              #   4: Linear Error-based frame augmentation
        augmenting_delta_time: 0.             # Maximum time between augments
        augmenting_pos_error: 0.1             # Maximum position error between augments
        augmenting_ang_error: 0.1             # Maximum angular error between augments
        process_noise:
            pos: 1.0e-0                       # Position process noise
            vel: 1.0e-1                       # Velocity process noise
            ang_pos: 1.0e-0                   # Angular Position process noise

        pos_l_in_g: [0.0, 0.0, 0.0]           # Local frame position in global frame
        ang_l_to_g: 0.0                       # Local frame heading in global frame
        gps_init_type: 0                      # GPS Initialization Type
                                              #   0: CONSTANT
                                              #   1: BASELINE_DIST
                                              #   2: ERROR_THRESHOLD
        gps_init_baseline_dist: 100.0         # Baseline distance threshold to initialize
        gps_init_pos_thresh: 1.0              # Local frame position error threshold
        gps_init_ang_thresh: 0.1              # Local frame heading error threshold
        use_root_covariance: true             # Flag to utilize the square root form of covariance

        sim_params:
            seed: 0.0                         # Seed to provide to random number generator
            use_seed: true                    # Flag to use seed (required for deterministic runs)
            max_time: 10.0                    # Maximum simulation time
            number_of_runs: 10                # Number of runs for Multi-Threaded Simulation
            run_number: 0                     # Start run number (also used to initialize seed)
            stationary_time: 0.1              # Time to be stationary before motion begins
            truth_type: "cyclic"              # TruthType:
                                              #   "cyclic"
                                              #   "spline"
            # Cyclic Truth Parameters
            pos_frequency: [0.3, 0.5, 0.7]    # Cyclic position frequencies
            ang_frequency: [0.2, 0.3, 0.4]    # Cyclic angular frequencies
            pos_offset: [0.0, 0.0, 0.0]       # Position offset for cyclic position
            ang_offset: [0.0, 0.0, 0.0]       # Position offset for cyclic position
            ang_amplitude: 0.1                # Amplitude of translational motion
            pos_amplitude: 1.0                # Amplitude of rotational motion
            pos_b_in_l_error: [0.5, 0.5, 0.5] # Initial body position error
            ang_b_to_l_error: [0.3, 0.3, 0.3] # Initial body angular error

            # Spline Truth Parameters:
            positions:                        # A list of translational vectors
                - [0.0, 0.0, 0.0]
                - [0.1, 0.1, 0.1]
                - [-0.1, -0.1, -0.1]
                - [0.0, 0.0, 0.0]
            angles:                           # A list of rotational vectors
                - [0.00, 0.00, 0.00]
                - [0.01, 0.01, 0.01]
                - [0.00, 0.00, 0.00]
            pos_errors: [1.0, 1.0, 1.0]       # Error in achieving translational vectors
            ang_errors: [0.1, 0.1, 0.1]       # Error in achieving rotational vectors

            pos_l_in_g_error: [1.0, 1.0, 1.0] # Local frame position error
            ang_l_to_g_error: 0.1             # Local frame heading error
```

Sensors and parameters can be dynamically declared with a bit of tomfoolery. To achieve this, we must read from a known list to the declare and read unknown lists. Therefore, the following lists must exist, but can be empty. These define the parameter sections for each of their respective categories. Ensure that each of the sensors used in a configuration are declared here and defined later:

```{.py}
/EkfCalNode:
    ros__parameters:

        ...

        imu_list:       # List of imu. Can be empty but list must exist
            - "imu_1"

        camera_list:    # List of camera. Can be empty but list must exist
            - "cam_1"

        tracker_list:   # List of tracker. Can be empty but list must exist
            - "orb"

        fiducial_list:  # List of fiducial. Can be empty but list must exist
            - "charuco"

        gps_list:       # List of gps. Can be empty but list must exist
            - "gps_1"
```

## IMU parameters

The following is an example of an IMU input configuration.

```{.py}
/EkfCalNode:
    ros__parameters:

        ...

        imu:
            imu_1:
                is_extrinsic: false                      # Flag to calibrate IMU extrinsics
                is_intrinsic: false                      # Flag to calibrate IMU intrinsics
                rate: 1000.0                             # Measurement update rate
                topic: "/imu_1"                          # ROS topic to subscribe to
                variance: [
                    0.01, 0.01, 0.01,                    # Position
                    0.01, 0.01, 0.01,                    # Orientation
                    0.01, 0.01, 0.01,                    # Accelerometer bias
                    0.01, 0.01, 0.01                     # Gyroscope bias
                ]
                pos_i_in_b: [0.0, 0.0, 0.0]              # Initial position offset estimate
                ang_i_to_b: [1.0, 0.0, 0.0, 0.0]         # Initial angular offset estimate
                acc_bias: [0.0, 0.0, 0.0]                # Initial accelerometer bias estimate
                omg_bias: [0.0, 0.0, 0.0]                # Initial gyroscope bias estimate
                pos_stability: 1.0e-2                    # Position stability
                ang_stability: 1.0e-2                    # Orientation stability
                acc_bias_stability: 1.0e-3               # Accelerometer bias stability
                omg_bias_stability: 1.0e-3               # Gyroscope bias stability
                data_log_rate: 10.0                      # Data logging rate
                sim_params:
                    no_errors: false                     # Flag to disable errors in simulation
                    time_bias_error: 1.0e-3              # Measurement time bias
                    time_error: 1.0e-6                   # Measurement time error
                    pos_error: [0.0, 0.0, 0.0]           # Error in position estimate
                    ang_error: [0.0, 0.0, 0.0]           # Error in orientation estimate
                    acc_error: [1.0e-3, 1.0e-3, 1.0e-3]  # Accelerometer error
                    omg_error: [1.0e-2, 1.0e-2, 1.0e-2]  # Gyroscope error
                    acc_bias_error: [0.0, 0.0, 0.0]      # Accelerometer bias error
                    omg_bias_error: [0.0, 0.0, 0.0]      # Gyroscope bias error
```

## Camera parameters

The following is an example of an camera input configuration.

```{.py}
/EkfCalNode:
    ros__parameters:

        ...

        camera:
            cam_1:
                rate: 20.0                          # Sensor Rate
                topic:  "/cam_1"                    # ROS topic
                pos_c_in_b: [0.0, 0.0, 0.0]         # Position in body frame
                ang_c_to_b: [-0.5, 0.5, -0.5, 0.5]  # Orientation in body frame
                variance: [
                    0.1, 0.1, 0.1,                  # Position
                    0.1, 0.1, 0.1                   # Orientation
                    ]
                tracker: "orb"                      # Tracker to use
                fiducial: "charuco"                 # Fiducial to use
                pos_stability: 1.0e-9               # Position stability
                ang_stability: 1.0e-9               # Orientation stability
                data_log_rate: 10.0                 # Data logging rate
                intrinsics:
                    f_x: 0.01                       # X focal length [m]
                    f_y: 0.01                       # Y focal length [m]
                    k_1: 0.0                        # Radial coefficient 1
                    k_2: 0.0                        # Radial coefficient 2
                    p_1: 0.0                        # Tangential coefficient 1
                    p_2: 0.0                        # Tangential coefficient 1
                    pixel_size: 5.0e-6              # Pixel size
                    width: 1920                     # Image width
                    height: 1080                    # Image height
                is_extrinsic: false                 # Flag to calibrate camera extrinsics
                sim_params:
                    no_errors: false                # Flag to disable errors in simulation
                    time_bias_error: 1.0e-3         # Measurement time bias error
                    time_error: 1.0e-6              # Measurement time error
                    pos_error: [0.0, 0.0, 0.0]      # Position error
                    ang_error: [0.0, 0.0, 0.0]      # Orientation error
```
## Tracker parameters

The following is an example of a tracker input configuration.

```{.py}
/EkfCalNode:
    ros__parameters:

        ...

        tracker:
            orb:
                feature_detector: 2         # Feature Detector:
                                            #   0: BRISK
                                            #   1: FAST,
                                            #   2: GFTT,
                                            #   3: MSER,
                                            #   4: ORB,
                                            #   5: SIFT,
                descriptor_extractor: 1     # Descriptor Extractor:
                                            #   0: ORB
                                            #   1: SIFT
                descriptor_matcher: 0       # DescriptorMatcher:
                                            #   0: BRUTE_FORCE
                                            #   1: FLANN
                detector_threshold: 10.0    # Threshold used for detection (if available)
                pixel_error: 0.5            # Average pixel error
                min_feature_distance: 1.0   # Minimum feature distance to consider
                min_track_length: 2         # Minimum track length
                max_track_length: 20        # Maximum track length
                sim_params:
                    no_errors: false        # Flag to disable errors in simulation
                    feature_count: 500      # Total number of trackable features
                    room_size: 5.0          # Maximum distance of features
```

## Fiducial parameters

The following is an example of a fiducial input configuration.

```{.py}
/EkfCalNode:
    ros__parameters:

        ...

        fiducial:
            charuco:
                fiducial_type: 1                            # fiducial_type:
                                                            #   0: ARUCO_BOARD
                                                            #   1: CHARUCO_BOARD
                squares_x: 5                                # Board squares in x
                squares_y: 7                                # Board squares in y
                square_length: 0.04                         # Square length
                marker_length: 0.02                         # Marker length
                id: 0                                       # Initial marker ID
                pos_f_in_l: [5.0, 0.0, 0.0]                 # Board position in local frame
                ang_f_to_l: [1.0, 0.0, 0.0, 0.0]            # Board orientation in local frame
                variance: [
                    0.1, 0.1, 0.1,                          # Position
                    0.1, 0.1, 0.1                           # Orientation
                ]
                min_track_length: 0                         # Minimum track length
                max_track_length: 1                         # Maximum track length
                is_extrinsic: false                         # Flag to calibrate fiducial extrinsics
                sim_params:
                    no_errors: false                        # Flag to disable errors in simulation
                    pos_error: [0.0, 0.0, 0.0]              # Position error
                    ang_error: [0.0, 0.0, 0.0]              # Orientation error
                    t_vec_error: [1.0e-2, 1.0e-2, 1.0e-2]   # Translation measurement error
                    r_vec_error: [1.0e-2, 1.0e-2, 1.0e-2]   # Rotation measurement error
```

## GPS parameters

The following is an example of a GPS input configuration.

```{.py}
/EkfCalNode:
    ros__parameters:

        ...

        gps:
            gps_1:
                rate: 5.0                            # Sensor Rate
                topic: "/gps_1"                      # ROS topic
                pos_a_in_b: [0.0, 0, 0]              # Antenna position in body frame
                variance: [5.0, 5.0, 5.0]            # Antenna position variance
                data_log_rate: 5.0                   # Data log rate
                is_extrinsic: false                  # Flag to calibrate GPS extrinsics
                sim_params:
                    no_errors: false                 # Flag to disable errors in simulation
                    time_bias_error: 0.0             # Time bias error
                    time_error: 1.0e-9               # Measurement time error
                    lla_error:  [5.0, 5.0, 5.0]      # LLA measurement error
                    pos_a_in_b_err: [0.0, 0.0, 0.0]  # Antenna position error
```

## Parameter Adjustment

As with most Kalman Filter tunings the two primary knobs to turn are the measurement and process noise variables.

The measurement noise should be set to the sensor manufacturer's specified value if it has not yet been verified. Ideally, depending on the sensor, a calibration routine should be performed to find a more accurate measurement noise value.

The process noise can be harder to tune, especially when balancing such a large number of filter states, as is the case with `ekf_cal`. In simulation, one can make use of the [NEES](https://kalman-filter.com/normalized-estimation-error-squared/) plots that are produced to determine if the filter is being efficient in its estimations. The process noise should be changed to roughly keep the NEES within the specified confidence interval.

Finally, depending on the setup, it can be difficult to determine what is driving low filter performance. In complex cases, it is easiest to disable entire sensors or disable a sensors extrinsic/intrinsic calibration in steps in order to determine which is driving the largest errors.
