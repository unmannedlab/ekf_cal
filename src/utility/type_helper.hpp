// Copyright 2022 Jacob Hartzer
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

#ifndef UTILITY__TYPE_HELPER_HPP_
#define UTILITY__TYPE_HELPER_HPP_

#include <eigen3/Eigen/Eigen>

#include <vector>

///
/// @brief Converts std::vector into Eigen Quaternion
/// @param in Input std::vector
/// @return Output Eigen Vector3
///
Eigen::VectorXd StdToEigVec(std::vector<double> const & in);

///
/// @brief Converts std::vector into Eigen Quaternion
/// @param in Input std::vector
/// @return Output Eigen Quaternion
///
Eigen::Quaterniond StdToEigQuat(std::vector<double> const & in);

///
/// @brief Convert rotation vector to quaternion
/// @param rot_vec Input rotation vector
/// @return Rotation quaternion
///
Eigen::Quaterniond RotVecToQuat(Eigen::Vector3d rot_vec);

///
/// @brief Convert quaternion to rotation vector
/// @param quat Input rotation quaternion
/// @return Rotation vector
///
Eigen::Vector3d QuatToRotVec(Eigen::Quaterniond quat);


///
/// @brief Convert Euler angles to quaternion
/// @param euler_angles Input Euler angles
/// @return Resulting quaternion
///
Eigen::Quaterniond EigVecToQuat(Eigen::Vector3d euler_angles);

#endif  // UTILITY__TYPE_HELPER_HPP_
