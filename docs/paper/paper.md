---
title: 'EKF_CAL: Extended Kalman Filter-based Calibration and Localization'
tags:
  - C++
  - ROS
  - Kalman Filtering
  - Monte Carlo Simulation
authors:
  - name: Jacob Hartzer
    orcid: 0000-0002-3051-2213
    corresponding: true
    affiliation: 1
  - name: Srikanth Saripalli
    orcid: 0000-0002-3906-7574
    corresponding: false
    affiliation: 1
affiliations:
 - name: Texas A&M University, USA
   index: 1
date: 01 January 2023
bibliography: paper.bib
---

# Summary

The increasing complexity of autonomous systems operating in challenging environments demands robust and accurate calibration methods. Modern autonomous systems use multiple types of sensors to handle a wide variety of environments. To improve system robustness in these challenging environments, redundant sensors are often used to protect against erroneous sensor measurements or outright sensor failures. However, the underlying navigation system must be designed in such a way as to handle these duplicate measurement streams and the addition of each sensor introduces parameters that must be properly calibrated before these sensors' measurements can be effectively used. Moreover, these underlying calibration parameters can change, and often must be estimated *online*. Kalman filters are a common and near-optimal method to estimate a system's state given a series of measurements and transitions through time. Improper tuning can lead to filter instability and poor performance. As such, it is important to verify a filters performance and stability across a large set of runs in the desired operating environment with various initial errors in state estimates.

This software package, `ekf_cal`, seeks to provide a framework for testing multi-sensor Kalman filters using Monte Carlo techniques. By running a given sensor configuration through a large sample of runs, each with varying initial conditions and errors, stability is verified. By looking at the resulting error distributions in any of the system states, the expected system bias and error are estimated. In addition to the simulation running framework, additional scripts are provided that calculate the distribution of errors for a Monte Carlo and generate a report showing state values and errors across the run to aid in analysis. The Multi-State Constraint Kalman Filter (MSCKF) formulation used in this work was originally proposed proposed in [@MSCKF]. Additionally, this work heavily utilizes the Eigen [@eigen] and OpenCV libraries [@opencv]. Lastly, this software package provides example implementations of state-of-the-art Kalman filter-based calibration routines and is currently used for ongoing research @2022_Multi_Cam and @2023_Multi_IMU.

# Statement of need

Currently, there are not many software packages for prototyping and evaluating Kalman Filter-based calibration and localization algorithms. The most popular, which also includes a simulation architecture, is Open-VINS [@Geneva2020ICRA], which makes use of interpolated B-splines to define true motion data. Inertial measurements are sampled from the second derivative of the spline path, and bearing camera measurements are generated using global fixed features which are projected onto the camera sensor frame. The primary limitation of Open-VINS is the limited types of sensors available and the constraint to a single IMU for filtering.

There exist other batch-based (offline) calibration packages, such as the Matlab Camera Calibration Toolbox [@matlab-cam-calib] for mono and stereo camera setups, where the primary limitations are the limited sensor types and closed-source codebase. Additionally, there are batch-based optimizers such as Kalibr [@Rehder] or OpenCalib [@opencalib] that provide tools for performing calibration of multi-sensor systems but do not provide these calibrations online. These software applications are therefore susceptible to changing calibration parameters, such as drifts or shifts in mounting.

The `ekf_cal` package, in contrast, seeks to provide additional avenues for testing Kalman filter based calibration and localization techniques while supporting multi-IMU filtering, GNSS sensors, and fiducial markers. Similar to existing work, this is all done on top of a B-spline based truth model and global feature points.

In summary, this package

- Provides examples of the filtering techniques outlined in @2022_Multi_Cam and @2023_Multi_IMU
- Provides a Monte Carlo architecture for testing filter-based calibration techniques
- Provides analysis tools for plotting results and evaluating performance statistics

# Capabilities

The `ekf_cal` package supports any number or combination of the sensors listed in the following sections. Errors in sensor measurements and calibrations are varied across Monte Carlo runs, which allow for more robust testing of calibration and localization algorithms. The key parameters supported by all sensors are

| Sensor Parameter   | Description                   |
| ------------------ | ----------------------------- |
| `name`             | Identifying sensor name       |
| `topic`            | ROS topic to subscribe to     |
| `rate`             | Sensor update rate            |
| `data_logging_on`  | Flag to enable data logging   |
| `data_log_rate`    | Data logging rate             |
| `output_directory` | Data logging output directory |

### IMU

The `ekf_cal` package supports the use of multiple IMU for updating the state estimate of acceleration and angular rates. A single IMU can be selected to provide state predictions, or all IMU can be used to provide state updates within an Extended Kalman Filter framework. The key IMU parameters supported are


| IMU Parameter        | Description                                 |
| -------------------- | ------------------------------------------- |
| `acc_bias_stability` | Intrinsic accelerometer bias stability      |
| `acc_bias`           | Accelerometer bias vector                   |
| `ang_i_to_b`         | IMU Angular offset quaternion in body frame |
| `ang_stability`      | Extrinsic angular stability                 |
| `is_extrinsic`       | Flag to enable extrinsic calibration        |
| `is_intrinsic`       | Flag to enable intrinsic calibration        |
| `omg_bias_stability` | Intrinsic gyroscope bias stability          |
| `omg_bias`           | Gyroscope bias vector                       |
| `pos_i_in_b`         | IMU Position offset vector in body frame    |
| `pos_stability`      | Extrinsic position stability                |
| `variance`           | Initial state variance                      |

### Cameras

The `ekf_cal` package supports the use of multiple cameras, which can simultaneously use MSCKF-based feature tracking and/or fiducial marker tracking for measurement updates. For visual inertial odometry, the package is designed to support the use of any feature tracker, descriptor, or matcher made available through the OpenCV package [@opencv]. For fiducial measurements, the package supports the use of Aruco or Charuco grid boards. The key camera parameters supported are

| Camera Parameter | Description                                    |
| ---------------- | ---------------------------------------------- |
| `ang_c_to_b`     | Camera angular offset quaternion in body frame |
| `ang_stability`  | Extrinsic angular stability                    |
| `fiducial`       | Name of Fiducial to load                       |
| `intrinsics`     | Camera intrinsics                              |
| `pos_c_in_b`     | Camera position offset vector in body frame    |
| `pos_stability`  | Extrinsic position stability                   |
| `tracker`        | Name of Tracker to load                        |
| `variance`       | Initial state variance                         |

### GPS

The `ekf_cal` package supports the use of multiple GPS antenna for updating the state estimate of position in the global frame. The currently implemented filter can utilize these measurements to estimate the initial global to local frame transformation as well as provide online estimates of the heading of the local frame. The key GPS parameters supported are

| GPS Parameter   | Description                                      |
| --------------- | ------------------------------------------------ |
| `ang_l_to_g`    | Local angle to global frame                      |
| `is_extrinsic`  | Flag to enable extrinsic calibration             |
| `pos_a_in_b`    | GPS antenna position offset vector in body frame |
| `pos_l_in_g`    | Local LLA position in global frame               |
| `pos_stability` | Extrinsic position stability                     |
| `variance`      | Initial state variance                           |

### Plotting and Analysis

`ekf_cal` provides various tools for evaluating the performance of multi-sensor Kalman filters. These include Monte Carlo testing, report generation, and statistical summary functions that simplify the analysis of algorithm changes and development. Perturbations are automatically added to not only sensor measurements, but the underlying truth model. This allows for variations in paths taken, sensor measurement times, and measurement values themselves, among others. An example of perturbations in true body positions is shown in Figure \ref{pos}.

![Simulated positions generated from spline inputs.\label{pos}](png/body-pos.png)

The main goal of this software is the evaluation of calibration kalman filters for stability and accuracy. Therefore, error plots are generated with respect to truth to show convergence over time for key parameters such as shown in Figure \ref{ant}

![Convergence of filtered GPS antenna position over course of multiple Monte Carlo runs.\label{ant}](png/gps-ant-pos-err.png)

# Conclusion

In conclusion, the `ekf_cal` package provides a comprehensive framework for testing multi-sensor Kalman filters using Monte Carlo techniques. By running a given sensor configuration through a large sample of runs, each with varying initial conditions and errors, stability is verified. The package's capabilities include the provision of example implementations of state-of-the-art Kalman filter-based calibration routines and analysis tools for plotting results and evaluating performance statistics. In contrast to existing software packages that are limited by their scope or batch-based optimization methods, `ekf_cal` offers a flexible and online calibration framework that can be tailored to specific research needs. As demonstrated by its successful application in past and ongoing research projects, the `ekf_cal` package has the potential to significantly improve the robustness and accuracy of autonomous systems operating in challenging environments. Future work will focus on expanding the package's capabilities, including the integration of new sensors and optimization methods, as well as exploring its applications in various domains.

# References
