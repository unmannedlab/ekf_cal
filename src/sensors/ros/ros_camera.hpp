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

#ifndef SENSORS__ROS__ROS_CAMERA_HPP_
#define SENSORS__ROS__ROS_CAMERA_HPP_

#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "infrastructure/debug_logger.hpp"
#include "sensors/ros/ros_camera_message.hpp"
#include "sensors/camera.hpp"

///
/// @class RosCamera
/// @brief ROS camera sensor class
/// @todo Implement callback methods
///
class RosCamera : public Camera
{
public:
  ///
  /// @brief RosCamera constructor
  /// @param camera_parameters Camera parameters
  ///
  explicit RosCamera(Camera::Parameters camera_parameters);

  ///
  /// @brief RosCamera callback method
  /// @param ros_camera_message ROS image message
  ///
  void Callback(std::shared_ptr<RosCameraMessage> ros_camera_message);

  ///
  /// @brief Camera output ROS image getter method
  /// @return Camera output ROS image
  ///
  sensor_msgs::msg::Image::SharedPtr GetRosImage();

  ///
  /// @brief Method to add tracker object to ros camera sensor
  /// @param tracker Tracker pointer for ros camera to use during callbacks
  ///
  void AddTracker(std::shared_ptr<FeatureTracker> tracker);

private:
  sensor_msgs::msg::Image::SharedPtr m_out_ros_img;
};

#endif  // SENSORS__ROS__ROS_CAMERA_HPP_
