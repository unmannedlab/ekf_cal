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

#include "infrastructure/Logger.hpp"
#include "sensors/Camera.hpp"
#include "sensors/Tracker.hpp"
#include "utility/RosHelper.hpp"
#include "utility/TypeHelper.hpp"


RosCamera::RosCamera(Camera::Params cParams, Tracker::Params tParams)
: Camera(cParams, tParams) {}

void RosCamera::callback(const sensor_msgs::msg::Image::SharedPtr inMsg)
{
  double time = rosHeaderToTime(inMsg->header);

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(inMsg);

  Camera::callback(time, cv_ptr->image);

  m_logger->log(LogLevel::DEBUG, "Image publish ROS");

  m_outRosImg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", m_outImg).toImageMsg();
}

sensor_msgs::msg::Image::SharedPtr RosCamera::getRosImage()
{
  return m_outRosImg;
}
