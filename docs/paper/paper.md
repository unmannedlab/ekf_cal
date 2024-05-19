---
title: 'EKF_CAL: Extended Kalman Filter-based Calibration and Localization'
tags:
  - C++
  - ROS
authors:
  - name: Jacob M. Hartzer
    orcid: 0000-0002-3051-2213
    corresponding: true
    affiliation: 1
affiliations:
 - name: Texas A&M University, USA
   index: 1
date: 01 January 2023
bibliography: paper.bib
---

# Summary

Navigation of modern autonomous systems requires the use of multiple types of sensors to handle a wide variety of environments. To improve system robustness in these challenging environments, redundant sensors are often used where each additional sensor introduces parameters that must be properly calibrated before these sensors' measurements can be effectively used.

This software package provides an example implementation of Kalman filter-based calibration routines as well as the simulation capability to show stability and convergence through Monte Carlo techniques.

# Statement of need

This package does the following
- Provides examples of the filtering techniques outlined in [@2023-MFI,@2022-SSRR]
- Provides a Monte Carlo simulation for filter-based calibration techniques
- Provides plotting and evaluation statistics

Has so far been utilized in:
- [@2022_Multi_Cam]
- [@2023_Multi_IMU]

Existing Work:
- [@Geneva2020ICRA]
- [@Rehder]

# Capabilities
`EKF-CAL` supports any number or combination of the sensors listed in the following sections. Additionally, errors in the calibrations are modelled across Monte Carlo simulations.

## IMU
`EKF-CAL` supports the use of multiple IMU for updating the state estimate of acceleration and angular rates. A single IMU can be selected to provide state predictions, or all IMU can be used to provide state updates within the Extended Kalman Filter framework.

## Cameras
`EKF-CAL` supports the use of multiple camera that can simultaneously use MSCKF-based feature tracking and fiducial marker tracking for state updates.

## GPS
Lorem ipsum.

# References