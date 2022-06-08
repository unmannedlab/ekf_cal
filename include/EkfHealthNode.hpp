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

#ifndef EKFHEALTHNODE_HPP_
#define EKFHEALTHNODE_HPP_

#include <rclcpp/rclcpp.hpp>

///
/// @class EkfHealthNode: A node for monitoring the health of sensor
/// calibrations
/// @todo IMU error tracking
/// @todo Implement 3 sensor error tracking for camera model
/// @todo LiDAR error tracking
///
class EkfHealthNode : public rclcpp::Node
{
public:
  ///
  /// @brief Constructor for the EKF Health Node
  ///
  EkfHealthNode();
};

#endif  // EKFHEALTHNODE_HPP_
