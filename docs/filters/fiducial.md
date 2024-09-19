Fiducial Filtering {#fiducial}
============

Work in progress

## Fiducial Error Model

The error model used for a FiducialTracker is

\f{align}{
    {}^C p_f =
    R({}^B_C q)^T
    \left(
    R({}^L_B q)^T
    \left[
    {}^L p_f -
    {}^L p_B
    \right] -
    {}^B p_C
    \right) +
    n_p
\f}

where
- \f$ {}^C q_f \f$ is the measured position of the fiducial in the camera frame,
- \f$ {}^B_C q \f$ is the rotation from the camera frame to the body frame,
- \f$ {}^L_B q \f$ is the rotation from the body frame to the local frame,
- \f$ {}^L p_f \f$ is the position of the fiducial in the local frame,
- \f$ {}^L p_C \f$ is the position of the camera in the local frame, and
- \f$ n_p      \f$ is the position Gaussian white noise process.

\f{align}{
    {}^C q_F =
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
    {}^B_C q^{-1}
    {}^L_B q^{-1}
    {}^L_F q
\f}

where
- \f$ {}^C q_F \f$ is the measured fiducial frame to camera frame rotation
- \f$ n_\alpha \f$ is the yaw Gaussian white noise,
- \f$ n_\beta  \f$ is the pitch Gaussian white noise,
- \f$ n_\gamma \f$ is the roll Gaussian white noise,
- \f$ {}^B_C q \f$ is the rotation from the camera to the body frame,
- \f$ {}^L_B q \f$ is the rotation from the body to the local frame, and
- \f$ {}^L_F q \f$ is the rotation from the fiducial frame to the local frame