# ekf_cal
[![Documentation](https://github.com/unmannedlab/ekf_cal/actions/workflows/documentation.yaml/badge.svg)](https://github.com/unmannedlab/ekf_cal/actions/workflows/documentation.yaml)

Extended Kalman Filter Calibration and Localization: ekf_cal is a package focused on the simulation
and development of a multi-sensor online calibration kalman filter. It combines the architecture of
a Multi-State Constraint Kalman Filter (MSCKF) with a multi-IMU calibration filter to provide
estimates of intrinsic and extrinsic sensor calibration parameters.

For full documentation, please see below:
- [Github Repository](https://github.com/unmannedlab/ekf_cal/)
- [Documentation](https://unmannedlab.org/ekf_cal/)
- [Getting Started](https://unmannedlab.org/ekf_cal/tutorial.html)

## References

```bibtex
@inproceedings{2023_Multi_IMU,
  title         = {Online Multi-IMU Calibration Using Visual-Inertial Odometry},
  booktitle     = {2023 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI)},
  author        = {Jacob Hartzer and Srikanth Saripalli},
  year          = {2023},
  doi           = {10.1109/SDF-MFI59545.2023.10361310},
  arxiv         = {2310.12411},
}
```
```bibtex
@inproceedings{2022_Multi_Cam,
  title     = {Online Multi Camera-IMU Calibration},
  booktitle = {2022 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)},
  author    = {Hartzer, Jacob and Saripalli, Srikanth},
  year      = {2022},
  pages     = {360-365},
  doi       = {10.1109/SSRR56537.2022.10018692},
  arxiv     = {2209.13821},
}
```
