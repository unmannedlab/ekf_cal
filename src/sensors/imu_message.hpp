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


#ifndef SENSORS__IMU_MESSAGE_HPP_
#define SENSORS__IMU_MESSAGE_HPP_

#include <eigen3/Eigen/Eigen>

#include "sensors/sensor_message.hpp"

///
/// @class ImuMessage
/// @brief Data class for IMU messages
///
class ImuMessage : public SensorMessage
{
public:
  ImuMessage() {}
  Eigen::Vector3d acceleration;              ///< @brief IMU acceleration
  Eigen::Vector3d angular_rate;              ///< @brief IMU angular rate
  Eigen::Matrix3d acceleration_covariance;   ///< @brief IMU acceleration covariance
  Eigen::Matrix3d angular_rate_covariance;   ///< @brief IMU angular rate covariance
};

#endif  // SENSORS__IMU_MESSAGE_HPP_
