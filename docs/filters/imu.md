IMU Filtering {#imu}
============

There are multiple methods for combining IMU measurements. Most previous methods require some level of synchronization of the IMUs so that a weighted average of the different measurements can be combined into a single "virtual" IMU. However, this has limitations when combining IMU that have different update rates. In order to overcome this, we previously proposed a update-only IMU filter that does away with the typical method of using the IMU measurements in the update step, but instead extends the state vector to include linear and angular positions, rates, and accelerations, in addition to using IMU measurements only in the Kalman update steps to update these estimates.

To explain this, first consider the measurement model for an IMU that is not at the origin of the body-fixed reference frame

\f{align}{
\boldsymbol{h}(\boldsymbol{x}_{b}) = 
\begin{bmatrix}
    \mathcal{C}(\quat{I_i}{B})^T
    \left(
    \mathcal{C}(\quat{B}{G})^T \boldsymbol{a} +
    \boldsymbol{\alpha} \times \pose{I_i}{B} +
    \boldsymbol{\omega} \times \boldsymbol{\omega} \times \pose{I_i}{B}
    \right)
    \\
    \mathcal{C}(\quat{I_i}{B})^T
    \mathcal{C}(\quat{B}{G})^T
    \boldsymbol{\omega}
\end{bmatrix}
\f}

In order to properly develop residuals for this measurement model in the Kalman update step, it is necessary to have state estimates of the position, orientation and their derivatives. Therefore, the body state of the filter is

\f{align}{
  \boldsymbol{x}_B = 
  \begin{bmatrix}
    \pose{B}{L} &
    \vel{B}{L} &
    \acc{B}{L} &
    \quat{B}{L} &
    \angvel{B}{L} &
    \angacc{B}{L}
  \end{bmatrix}
\f}

With this extension of the state vector, it is possible to incorporate any IMU measurement regardless of timing using the typical Kalman update process. First a linearized state prediction is performed to the current measurement time.

## Body State Prediction

\f{align}{
  \hat{\boldsymbol{x}}_{k|k-1} = \boldsymbol{F} \hat{\boldsymbol{x}}_{k-1|k-1}
\f}

\f{align}{
  \boldsymbol{P}_{k|k-1} =
  \boldsymbol{F}
  \boldsymbol{P}_{k|k-1}
  \boldsymbol{F}^T + 
  \boldsymbol{F}
  \boldsymbol{Q}
  \boldsymbol{F}^T
\f}

where 

\f{align}{
  \boldsymbol{F} = 
  \begin{bmatrix}
    \boldsymbol{I}_3 & \Delta t \boldsymbol{I}_3 & 0 & 0 & 0 & 0 & 0 \\
    0 & \boldsymbol{I}_3 & \Delta t \boldsymbol{I}_3 & 0 & 0 & 0 & 0 \\
    0 & 0 & \boldsymbol{I}_3 & 0 & 0 & 0 & 0 \\
    0 & 0 & 0 & \boldsymbol{I}_3 & \Delta t \boldsymbol{I}_3 & 0 & 0 \\
    0 & 0 & 0 & 0 & \boldsymbol{I}_3 & \Delta t \boldsymbol{I}_3 & 0 \\
    0 & 0 & 0 & 0 & 0 & \boldsymbol{I}_3 & \Delta t \boldsymbol{I}_3
  \end{bmatrix}
\f}

## Body State Update

The Kalman update step is performed in the typical fashion. The measurement residual is outlined in equation TODO. The resultant observation matrix is

The typical Kalman update equations are used for the remainder of the update

\f{align}{
  \boldsymbol{S} = \boldsymbol{H} * \boldsymbol{P}_{k|k-1} * \boldsymbol{H}^T + \boldsymbol{R}
\f}
\f{align}{
  \boldsymbol{K} = \boldsymbol{P}_{k|k-1} * \boldsymbol{H}^T * \boldsymbol{S}^{-1}
\f}
\f{align}{
  \boldsymbol{x}_{k|k} = \boldsymbol{K} * \boldsymbol{x}_{k|k}
\f}
\f{align}{
  \boldsymbol{P}_{k|k} =  (\boldsymbol{I} - \boldsymbol{K} * \boldsymbol{H}) * \boldsymbol{P}_{k|k-1} * (\boldsymbol{I} - \boldsymbol{K} * \boldsymbol{H})^T + \boldsymbol{K} * \boldsymbol{R} * \boldsymbol{K}^T;
\f}

## IMU Error Model

For simulation purposes, the IMU error model is

\f{align}{
    a_m =
    \omega \times \omega \times \pose{I}{B} +
    \alpha \times \pose{I}{B} +
    \mathcal{C}\left(q_i^b\right)
    \left[a + \boldsymbol{g}\right]
\f}

where

- \f$a_m\f$            is the measured acceleration,
- \f$\omega\f$         is the true angular velocity,
- \f$\pose{I}{B}\f$    is the true position of the IMU in the body frame,
- \f$\alpha\f$         is the true angular acceleration,
- \f$\mathcal{C}\f$    is the quaternion to rotation matrix function,
- \f$\quat{B}{I}\f$    is the rotation from the body frame to the IMU frame,
- \f$a_b\f$            is the true body acceleration, and
- \f$\boldsymbol{g}\f$ is the gravity vector.


## Reference

```bibtex
@inproceedings{2023_Multi_IMU,
  title         = {Online Multi-IMU Calibration Using Visual-Inertial Odometry},
  booktitle     = {2023 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI)},
  author        = {Jacob Hartzer and Srikanth Saripalli},
  year          = {2023},
  doi           = {10.1109/SDF-MFI59545.2023.10361310},
  preview       = {multi_imu.png},
  arxiv         = {2310.12411},
}
```