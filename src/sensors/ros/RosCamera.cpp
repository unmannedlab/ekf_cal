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

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include "sensors/Camera.hpp"
#include "utility/RosHelper.hpp"
#include "utility/TypeHelper.hpp"


RosCamera::RosCamera(Camera::Params params)
: Camera(params)
{
  m_node = rclcpp::Node::make_shared("ImgNode");
  m_imgPublisher = m_node->create_publisher<sensor_msgs::msg::Image>("outImg", 10);
}

void RosCamera::Callback(const sensor_msgs::msg::Image::SharedPtr inMsg)
{
  double time = RosHelper::RosHeaderToTime(inMsg->header);

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(inMsg);

  Camera::Callback(time, cv_ptr->image);

  m_logger->log(LogLevel::INFO, "Image publish ROS");

  sensor_msgs::msg::Image::SharedPtr outMsg = cv_bridge::CvImage(
    std_msgs::msg::Header(), "bgr8", m_outImg).toImageMsg();

  m_imgPublisher->publish(*outMsg.get());
}
