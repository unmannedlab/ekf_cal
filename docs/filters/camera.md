Camera Filtering {#camera}
============

Work in progress

## Camera Error Models 

The error model used for a FeatureTracker is

\f{align}{
    \begin{bmatrix}
        {}^C p_{f_x} \\
        {}^C p_{f_y} \\
        {}^C p_{f_z}
    \end{bmatrix} =
    R({}^C_L q)(p^L_F - p^L_C)
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


```bibtex
@inproceedings{2022_Multi_Cal,
  title     = {Online Multi Camera-IMU Calibration},
  booktitle = {2022 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)},
  author    = {Jacob Hartzer and Srikanth Saripalli},
  year      = {2022},
  pages     = {360-365},
  doi       = {10.1109/SSRR56537.2022.10018692},
  preview   = {multi_cam_imu/setup_thumb.png},
  arxiv     = {2209.13821},
}
```