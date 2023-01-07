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

#ifndef SENSORS__ROS__ROSCAMERA_HPP_
#define SENSORS__ROS__ROSCAMERA_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>

#include "sensors/Camera.hpp"

///
/// @class RosCamera
/// @brief ROS camera sensor class
/// @todo Implement callback methods
///
class RosCamera : public Camera
{
public:
  // using Camera::Camera;
  explicit RosCamera(Camera::Params params);

  ///
  /// @brief RosCamera callback method
  /// @param msg ROS image message
  ///
  void Callback(const sensor_msgs::msg::Image::SharedPtr msg);

private:
  rclcpp::Node::SharedPtr m_node;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_imgPublisher;
};

#endif  // SENSORS__ROS__ROSCAMERA_HPP_
