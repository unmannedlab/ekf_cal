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

#include "sensors/ros/RosCamera.hpp"

#include <sensor_msgs/msg/image.hpp>

#include "sensors/Camera.hpp"
#include "utility/RosHelper.hpp"
#include "utility/TypeHelper.hpp"


// RosCamera::RosCamera(Camera::Params params)
// : Camera(params) {}

void RosCamera::Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  double time = RosHelper::RosHeaderToTime(msg->header);

  m_Logger->log(LogLevel::INFO, "IMU Callback: " + m_name + std::to_string(time));

  Camera::Callback(time);
}
