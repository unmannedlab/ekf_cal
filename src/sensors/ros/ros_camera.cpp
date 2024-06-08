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

#include "sensors/ros/ros_camera.hpp"


#include <memory>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "infrastructure/debug_logger.hpp"
#include "sensors/camera.hpp"
#include "sensors/ros/ros_camera_message.hpp"


RosCamera::RosCamera(Camera::Parameters camera_parameters)
: Camera(camera_parameters) {}

void RosCamera::Callback(std::shared_ptr<RosCameraMessage> ros_camera_message)
{
  auto camera_message = std::make_shared<CameraMessage>(ros_camera_message->image);
  camera_message->time = ros_camera_message->time;
  camera_message->sensor_id = ros_camera_message->sensor_id;
  camera_message->sensor_type = ros_camera_message->sensor_type;
  Camera::Callback(camera_message);

  m_logger->Log(LogLevel::DEBUG, "Image publish ROS");

  m_out_ros_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", m_out_img).toImageMsg();
}

sensor_msgs::msg::Image::SharedPtr RosCamera::GetRosImage()
{
  return m_out_ros_img;
}

void RosCamera::AddTracker(std::shared_ptr<FeatureTracker> tracker)
{
  Camera::AddTracker(tracker);
}
