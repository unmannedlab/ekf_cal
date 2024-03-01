Camera Error Models {#cam_model}
============

# Feature Tracker

The Camera error model used for a FeatureTracker is

\f{align}{
    \begin{bmatrix}
        {}^C p_{f_x} \\
        {}^C p_{f_y} \\
        {}^C p_{f_z}
    \end{bmatrix} =
    R({}^C_G q)(p^G_F - p^G_C)
\f}

\f{align}{
    \begin{bmatrix}
        x_n \\
        y_n
    \end{bmatrix} =
    \begin{bmatrix}
        {}^C p_{f_x} / {}^C p_{f_z} \\
        {}^C p_{f_y} / {}^C p_{f_z}
    \end{bmatrix}
\f}


\f{align}{
    \begin{bmatrix}
        u \\ v
    \end{bmatrix} =
    \begin{bmatrix}
        f_x\left[x_n \left(1 + k_1 r^2 +k_2 r^4\right) + 2p_1 x_n y_n + p_2 \left(4^2 + 2 x_n^2\right)  \right] + c_x \\
        f_y\left[y_n \left(1 + k_1 r^2 +k_2 r^4\right) +  p_1 \left(4^2 + 2 y_n^2\right) + 2 p_2 x_n y_n \right] + c_y
    \end{bmatrix} +
    \begin{bmatrix}
        n_u \\ n_v
    \end{bmatrix}
\f}

where
\f{align}{
    r^2 = x_n^2 + y_n^2
\f}

and \f$n_u\f$ and \f$n_v\f$ are Gaussian white noise processes

# Fiducial Tracker

The Camera error model used for a FiducialTracker is

\f{align}{
    {}^C p_f =
    R({}^B_C q)^T
    \left(
    R({}^G_B q)^T
    \left[
    {}^G p_f -
    {}^G p_B
    \right] -
    {}^B p_C
    \right) +
    n_p
\f}

where
- \f$ {}^C q_f \f$ is the measured position of the fiducial in the camera frame,
- \f$ {}^B_C q \f$ is the rotation from the camera frame to the body frame,
- \f$ {}^G_B q \f$ is the rotation from the body frame to the global frame,
- \f$ {}^G p_f \f$ is the position of the fiducial in the global frame,
- \f$ {}^G p_C \f$ is the position of the camera in the global frame, and
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
    {}^G_B q^{-1}
    {}^G_F q
\f}

where
- \f$ {}^C q_F \f$ is the measured fiducial frame to camera frame rotation
- \f$ n_\alpha \f$ is the yaw Gaussian white noise,
- \f$ n_\beta  \f$ is the pitch Gaussian white noise,
- \f$ n_\gamma \f$ is the roll Gaussian white noise,
- \f$ {}^B_C q \f$ is the rotation from the camera to the body frame,
- \f$ {}^G_B q \f$ is the rotation from the body to the global frame, and
- \f$ {}^G_F q \f$ is the rotation from the fiducial frame to the global frame