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

#ifndef SENSORS__ROS__ROS_IMU_HPP_
#define SENSORS__ROS__ROS_IMU_HPP_

#include <memory>

#include "sensors/imu.hpp"
#include "sensors/ros/ros_imu_message.hpp"


///
/// @class RosIMU
/// @brief IMU Sensor Class
///
class RosIMU : public IMU
{
public:
  using IMU::IMU;

  ///
  /// @brief RosIMU callback method
  /// @param ros_imu_message ROS IMU message
  ///
  void Callback(const RosImuMessage & ros_imu_message);
};

#endif  // SENSORS__ROS__ROS_IMU_HPP_
