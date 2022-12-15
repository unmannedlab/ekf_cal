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

#ifndef SENSORS__ROS__ROSIMU_HPP_
#define SENSORS__ROS__ROSIMU_HPP_

#include <sensor_msgs/msg/imu.hpp>

#include <string>

#include "infrastructure/Logger.hpp"
#include "sensors/IMU.hpp"
#include "sensors/Sensor.hpp"

///
/// @class RosIMU
/// @brief IMU Sensor Class
/// @todo Add parameter input/defaults for covariance
///
class RosIMU : public IMU
{
public:
  using IMU::IMU;

  ///
  /// @brief RosIMU callback method
  /// @param msg ROS IMU measurement
  ///
  void Callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

#endif  // SENSORS__ROS__ROSIMU_HPP_
