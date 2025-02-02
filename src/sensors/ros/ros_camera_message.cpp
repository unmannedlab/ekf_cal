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

#include "sensors/ros/ros_camera_message.hpp"

#include <memory>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "sensors/camera_message.hpp"
#include "utility/ros_helper.hpp"


RosCameraMessage::RosCameraMessage(const sensor_msgs::msg::Image::SharedPtr msg)
: CameraMessage(cv_bridge::toCvCopy(msg)->image)
{
  time = RosHeaderToTime(msg->header);
  sensor_type = SensorType::Camera;
}
