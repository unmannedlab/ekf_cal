GPS Filtering {#gps}
============

When available, GPS can provide a drift-free global position measurement.

Since these measurements are produced in a global frame, it is necessary to have a global to local frame transformation to utilize these measurements in a localization system.

## Local Frame Initialization
A least squares method is used to initialize the global to local frame transformation. This requires a prior estimate of the GPS antenna in the body frame \f$ \pose{A}{B} \f$ as well as some form of odometry to predict or estimate body positions in the local frame without GPS-aiding.

Before the frame transformation is initialized, each GPS measurement is saved off along with the current estimate of the body position in the local frame. These measurement/estimate pairs are then used for the following routine.

Given desired translational and rotational errors thresholds, \f$ \epsilon_t \f$ and \f$ \epsilon_r \f$, it is possible to determine when it is best to perform a least squares estimate of the global to local frame rotation. Estimates of the translational and rotational errors are

<!-- @TODO: Add threshold equations here -->

Once these thresholds are reached, the Kabsch-TODO method of frame-to-frame estimation is used.

## Online Calibration

Errors in the initial local frame heading estimate can cause issues with estimates of intrinsic sensor parameters over long trajectories. Therefore, it is also desirable to develop an online estimate of just the local frame heading to further improve accuracy with large distances traveled by the body. Additionally, it is desirable to further refine the estimated location of the GPS antenna in the body frame. As such, the local frame heading is incorporated into the state estimate alongside each antenna position in the body frame.

\f{align}{
    \boldsymbol{x}_G = 
    \begin{bmatrix}
        \ang{G}{L} &
        \pose{A_1}{B} &
        \hdots &
        \pose{A_{N_G}}{B}
    \end{bmatrix}
\f}

## GPS Update Equations

The Kalman update step is performed in the typical fashion. The predicted measurement is 

\f{align}{
  \hat{\boldsymbol{z}} = 
  \begin{bmatrix}
    \poseHat{L}{G} + \quatHat{L}{G} (\poseHat{B}{L} + \quatHat{B}{L} (\poseHat{A}{B}))
  \end{bmatrix}
\f}

The measurement residual is 
\f{align}{
  \boldsymbol{y} = \boldsymbol{z} - \hat{\boldsymbol{z}}
\f}

The resultant observation matrix is

<!-- @TODO: Add observation matrix here -->


## GPS Error Model

The GPS error model used is
\f{align}{
    \begin{bmatrix}
        \phi_m \\
        \lambda_m \\
        h_m
    \end{bmatrix} =
    \begin{bmatrix}
        \phi \\
        \lambda \\
        h
    \end{bmatrix} +
    \begin{bmatrix}
        n_\phi \\
        n_\lambda \\
        n_h
    \end{bmatrix}
\f}
where
- \f$\phi_m\f$      is the measured latitude,
- \f$\lambda_m\f$   is the measured longitude,
- \f$h_m\f$         is the measured altitude,
- \f$\phi\f$        is the true latitude,
- \f$\lambda\f$     is the true longitude,
- \f$h\f$           is the true altitude, and
- \f$n_\phi\f$, \f$n_\lambda\f$, and \f$n_h\f$ are Gaussian, white noise processes

<!-- ## TODO: Reference:  -->
