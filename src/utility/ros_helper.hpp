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

#ifndef UTILITY__ROS_HELPER_HPP_
#define UTILITY__ROS_HELPER_HPP_

#include <eigen3/Eigen/Eigen>

#include <array>

#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>

///
/// @brief Convert ROS header to time
/// @param header ROS header
/// @return Time as a double
///
double RosHeaderToTime(std_msgs::msg::Header header);

///
/// @brief Convert ROS vector to Eigen
/// @param msg ROS message
/// @return Eigen vector
///
Eigen::Vector3d RosToEigen(geometry_msgs::msg::Vector3 msg);

///
/// @brief Convert ROS matrix to Eigen
/// @param msg ROS message
/// @return Eigen matrix
///
Eigen::Matrix3d RosToEigen(std::array<double, 9> msg);

#endif  // UTILITY__ROS_HELPER_HPP_
