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


#ifndef SENSORS__ROS__SIMIMU_HPP_
#define SENSORS__ROS__SIMIMU_HPP_

#include <string>
#include <sensor_msgs/msg/imu.hpp>
#include <eigen3/Eigen/Eigen>

#include "infrastructure/Logger.hpp"
#include "sensors/IMU.hpp"
#include "sensors/Sensor.hpp"
#include "sensors/sim/SimTypes.hpp"


class SimImuMessage : public SimMessage
{
public:
  SimImuMessage() {}

  Eigen::Vector3d acceleration;
  Eigen::Vector3d angularRate;
};

///
/// @class SimIMU
/// @brief Simulated IMU Sensor Class
///
class SimIMU : public IMU
{
public:
  using IMU::IMU;

  SimImuMessage generateMeasurement(double measurementTime);
};


#endif  // SENSORS__ROS__SIMIMU_HPP_
