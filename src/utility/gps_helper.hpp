// Copyright 2024 Jacob Hartzer
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

#ifndef UTILITY__GPS_HELPER_HPP_
#define UTILITY__GPS_HELPER_HPP_

#include <eigen3/Eigen/Eigen>

Eigen::Vector3d ecef_to_enu(const Eigen::Vector3d & in_ecef, const Eigen::Vector3d & ref_lla);
Eigen::Vector3d ecef_to_lla(const Eigen::Vector3d & ecef);
Eigen::Vector3d enu_to_ecef(const Eigen::Vector3d & in_enu, const Eigen::Vector3d & ref_lla);
Eigen::Vector3d enu_to_lla(const Eigen::Vector3d & enu_in, const Eigen::Vector3d & ref_lla);
Eigen::Vector3d lla_to_ecef(const Eigen::Vector3d & lla);
Eigen::Vector3d lla_to_enu(const Eigen::Vector3d & point_lla, const Eigen::Vector3d & ref_lla);
Eigen::Vector3d local_to_enu(const Eigen::Vector3d & local_in, const double ang_l_to_g);
Eigen::Vector3d enu_to_local(const Eigen::Vector3d & enu_in, const double ang_l_to_g);

#endif  // UTILITY__GPS_HELPER_HPP_
