Multi-IMU Filtering {#multi-imu}
============

Work in progress

<!-- \f{align}{
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
\f} -->


```bibtex
@inproceedings{2023_Multi_IMU,
  title         = {Online Multi-IMU Calibration Using Visual-Inertial Odometry},
  booktitle     = {2023 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI)},
  author        = {Jacob Hartzer and Srikanth Saripalli},
  year          = {2023},
  eprint        = {2310.12411},
  archiveprefix = {arXiv},
  primaryclass  = {cs.RO},
  doi           = {10.1109/SDF-MFI59545.2023.10361310},
  selected      = {true},
  preview       = {multi_imu.png},
  arxiv         = {2310.12411},
}
```