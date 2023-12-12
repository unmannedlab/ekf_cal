# EKF_CAL
[![Documentation](https://github.com/unmannedlab/ekf-cal/actions/workflows/documentation.yaml/badge.svg)](https://github.com/unmannedlab/ekf-cal/actions/workflows/documentation.yaml)
[![Plant Tree](https://img.shields.io/badge/dynamic/json?color=brightgreen&label=Plant%20Tree&query=%24.total&url=https%3A%2F%2Fpublic.offset.earth%2Fusers%2Ftreeware%2Ftrees)](https://plant.treeware.earth/ekf-cal)

Extended Kalman Filter Calibration and Localization: EKF-CAL is a package focused on the simulation
and development of a multi-sensor online calibration kalman filter. It combines the architecture of
a Multi-State Constraint Kalman Filter (MSCKF) with a multi-IMU calibration filter to provide
estimates of intrinsic and extrinsic sensor calibration parameters.

For full documentation, please see below:
- [Github Repository](https://github.com/unmannedlab/ekf-cal/)
- [Documentation](https://unmannedlab.org/ekf-cal/)
- [Getting Started](https://unmannedlab.org/ekf-cal/tutorial.html)

## License
This package is [Treeware](https://treeware.earth). If you use it in production, then we ask that you [**buy the world a tree**](https://plant.treeware.earth/ekf-cal) to thank us for our work. By contributing to the Treeware forest you’ll be creating employment for local families and restoring wildlife habitats.

## References

```bibtex
@inproceedings{2022_Multi_Cal,
  author    = {Hartzer, Jacob and Saripalli, Srikanth},
  booktitle = {2022 IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)},
  title     = {Online Multi Camera-IMU Calibration},
  year      = {2022},
  pages     = {360-365},
  doi       = {10.1109/SSRR56537.2022.10018692},
}
```