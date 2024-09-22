GPS Filtering {#gps}
============

When available, GPS can provide a drift-free global position measurement.

Since these measurements are produced in a global frame, it is necessary to have a global to local frame transformation to utilize these measurements in a localization system.

## Local Frame Initialization

## Online Calibration

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
