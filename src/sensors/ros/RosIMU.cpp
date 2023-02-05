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

#include "sensors/ros/RosIMU.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "sensors/Sensor.hpp"
#include "utility/MathHelper.hpp"
#include "utility/RosHelper.hpp"
#include "utility/TypeHelper.hpp"


void RosIMU::callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double time = rosHeaderToTime(msg->header);
  Eigen::Vector3d acc = rosToEigen(msg->linear_acceleration);
  Eigen::Vector3d omg = rosToEigen(msg->angular_velocity);
  Eigen::Matrix3d acc_cov = rosToEigen(msg->linear_acceleration_covariance);
  Eigen::Matrix3d omg_cov = rosToEigen(msg->angular_velocity_covariance);

  IMU::callback(time, acc, acc_cov, omg, omg_cov);
}
