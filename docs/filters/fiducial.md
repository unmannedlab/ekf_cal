Fiducial Filtering {#fiducial}
============

Fiducial markers can be excellent resources in localization and mapping. When visible, they can provide relative position and orientation measurements. While fiducial markers remain remain visible and stationary, they can provide a quick and moderately robust SLAM solution.

Issues can arise, however, when performing online calibration of IMU and Camera sensors or when there are sufficiently large errors in the position and orientation of the fiducial.

As such, it can be beneficial to simultaneously estimate the pose of the fiducial alongside the localization of the moving body. This is done by introducing the fiducial pose states into the overall filter as

\f{equation}{
    \boldsymbol{x}_F = 
    \begin{bmatrix}
        \pose{F}{L} & 
        \quat{F}{L}
    \end{bmatrix}
\f}


## Fiducial State Update

The Kalman update step is performed in the typical fashion. The predicted measurement is 

\f{align}{
  \hat{\boldsymbol{z}} = 
  \begin{bmatrix}
    \quatHat{B}{C} (\quatHat{L}{B} (\poseHat{F}{L} - \poseHat{B}{L}) - \poseHat{C}{B}) \\
    \quatHat{B}{C} \quatHat{L}{B} \quatHat{F}{L}
  \end{bmatrix}
\f}

The measurement residual is 
\f{align}{
  \boldsymbol{y} = \boldsymbol{z} - \hat{\boldsymbol{z}}
\f}

The resultant observation matrix is

<!-- @TODO: Add observation matrix here -->

\f{align}{
  \boldsymbol{H} = 
  \begin{bmatrix}
    0 & 0 & 0 & 0 & 0 & 0 \\
    0 & 0 & 0 & 0 & 0 & 0s
  \end{bmatrix}
\f}

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

<!-- TODO: Add example of drift/errors due to fiducial marker pose error-->

## Fiducial Error Model

The simulation error model used for a fiducial position measurement is

\f{align}{
    \pose{f}{C} =
    R(\quat{C}{B})^T
    \left(
    R(\quat{B}{L})^T
    \left[
    \pose{f}{L} -
    \pose{B}{L}
    \right] -
    \pose{C}{B}
    \right) +
    n_p
\f}

where
- \f$ \pose{f}C   \f$ is the measured position of the fiducial in the camera frame,
- \f$ \quat{C}{B} \f$ is the rotation from the camera frame to the body frame,
- \f$ \quat{B}{L} \f$ is the rotation from the body frame to the local frame,
- \f$ \pose{f}L   \f$ is the position of the fiducial in the local frame,
- \f$ \pose{C}L   \f$ is the position of the camera in the local frame, and
- \f$ n_p         \f$ is the position Gaussian white noise process.

The simulation error model used for a fiducial angular measurement is

\f{align}{
    \quat{f}{C} =
    \begin{bmatrix}
        \cos(n_\alpha) & -\sin(n_\alpha) & 0 \\
        \sin(n_\alpha) &  \cos(n_\alpha) & 0 \\
        0            & 0             & 1
    \end{bmatrix}
    \begin{bmatrix}
         \cos(n_\beta) & 0 & \sin(n_\beta) \\
         0            & 1 & 0          \\
        -\sin(n_\beta) & 0 & \cos(n_\beta)
    \end{bmatrix}
    \begin{bmatrix}
        1 & 0            & 0             \\
        0 & \cos(n_\gamma) & -\sin(n_\gamma) \\
        0 & \sin(n_\gamma) &  \cos(n_\gamma)
    \end{bmatrix}
    \quat{C}{B}^{-1}
    \quat{B}{L}^{-1}
    \quat{F}{L}
\f}

where
- \f$ \quat{f}{C} \f$ is the measured fiducial frame to camera frame rotation
- \f$ n_\alpha    \f$ is the yaw Gaussian white noise,
- \f$ n_\beta     \f$ is the pitch Gaussian white noise,
- \f$ n_\gamma    \f$ is the roll Gaussian white noise,
- \f$ \quat{C}{B} \f$ is the rotation from the camera to the body frame,
- \f$ \quat{B}{L} \f$ is the rotation from the body to the local frame, and
- \f$ \quat{F}{L} \f$ is the rotation from the fiducial frame to the local frame