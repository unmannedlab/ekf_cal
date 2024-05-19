# ekf_cal
[![Documentation](https://github.com/unmannedlab/ekf_cal/actions/workflows/documentation.yaml/badge.svg)](https://github.com/unmannedlab/ekf_cal/actions/workflows/documentation.yaml)
[![Plant Tree](https://img.shields.io/badge/dynamic/json?color=brightgreen&label=Plant%20Tree&query=%24.total&url=https%3A%2F%2Fpublic.offset.earth%2Fusers%2Ftreeware%2Ftrees)](https://plant.treeware.earth/unmannedlab/ekf_cal)

Extended Kalman Filter Calibration and Localization: ekf_cal is a package focused on the simulation
and development of a multi-sensor online calibration kalman filter. It combines the architecture of
a Multi-State Constraint Kalman Filter (MSCKF) with a multi-IMU calibration filter to provide
estimates of intrinsic and extrinsic sensor calibration parameters.

For full documentation, please see below:
- [Github Repository](https://github.com/unmannedlab/ekf_cal/)
- [Documentation](https://unmannedlab.org/ekf_cal/)
- [Getting Started](https://unmannedlab.org/ekf_cal/tutorial.html)

## License
This package is [Treeware](https://treeware.earth). If you use it in production, then we ask that you [**buy the world a tree**](https://plant.treeware.earth/unmannedlab/ekf_cal) to thank us for our work. By contributing to the Treeware forest you’ll be creating employment for local families and restoring wildlife habitats.

## References

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
```bibtex
@inproceedings{2022_Multi_Cam,
  author    = {Hartzer, Jacob and Saripalli, Srikanth},
  booktitle = {2022 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)},
  title     = {Online Multi Camera-IMU Calibration},
  year      = {2022},
  pages     = {360-365},
  doi       = {10.1109/SSRR56537.2022.10018692},
  selected  = {true},
  preview   = {multi_cam_imu/setup_thumb.png},
  arxiv     = {2209.13821},
}
```
