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

#ifndef SENSORS__ROS__ROS_CAMERA_MESSAGE_HPP_
#define SENSORS__ROS__ROS_CAMERA_MESSAGE_HPP_

#include <sensor_msgs/msg/image.hpp>

#include "sensors/camera_message.hpp"

///
/// @class RosCameraMessage
/// @brief Class for ROS image message data structure
///
class RosCameraMessage : public CameraMessage
{
public:
  ///
  /// @brief RosCameraMessage constructor
  /// @param msg Ros image message input
  ///
  explicit RosCameraMessage(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif  // SENSORS__ROS__ROS_CAMERA_MESSAGE_HPP_
