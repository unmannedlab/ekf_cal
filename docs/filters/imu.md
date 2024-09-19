IMU Filtering {#imu}
============

Work in progress

\f{align}{
\boldsymbol{h}(\boldsymbol{x}_{b}) = 
\begin{bmatrix}
    \mathcal{C}(\prescript{B}{I_i}{q})^T
    \left(
    \mathcal{C}(\prescript{G}{B}{q})^T \boldsymbol{a} +
    \boldsymbol{\alpha} \times \prescript{B}{}{\boldsymbol{p}}_{I_i} +
    \boldsymbol{\omega} \times \boldsymbol{\omega} \times \prescript{B}{}{\boldsymbol{p}}_{I_i}
    \right)
    \\
    \mathcal{C}(\prescript{B}{I_i}{q})^T
    \mathcal{C}(\prescript{G}{B}{q})^T
    \boldsymbol{\omega}
\end{bmatrix}
\f}

## IMU Error Model

The IMU error model used is
\f{align}{
    a_m =
    \omega \times \omega \times p_a +
    \alpha \times p_a +
    \mathcal{C}\left(q_i^b\right)
    \left[a + g\right]
\f}
where
- \f$a_m\f$            is the measured acceleration,
- \f$\omega\f$         is the true angular velocity,
- \f$p_a\f$            is the true position of the accelerometer in the body frame,
- \f$\alpha\f$         is the true angular acceleration,
- \f$\mathcal{C}\f$    is the quaternion to rotation matrix function,
- \f$q_i^b\f$          is the rotation from the body frame to the IMU frame,
- \f$a_b\f$            is the true body acceleration, and
- \f$g\f$              is the gravity vector.


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