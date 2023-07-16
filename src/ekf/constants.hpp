// Copyright 2023 Jacob Hartzer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef EKF__CONSTANTS_HPP_
#define EKF__CONSTANTS_HPP_

#include <eigen3/Eigen/Eigen>


static constexpr unsigned int g_body_state_size {18};
static constexpr unsigned int g_imu_state_size {12};
static constexpr unsigned int g_cam_state_size {6};
static constexpr unsigned int g_aug_state_size {12};

const Eigen::Vector3d g_gravity = Eigen::Vector3d(0, 0, 0);
/// @todo add back in here and subtract in body
// const Eigen::Vector3d g_gravity = Eigen::Vector3d(0, 0, 9.80665);

#endif  // EKF__CONSTANTS_HPP_
