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

#include <gtest/gtest.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/msg/image.hpp>

#include "sensors/ros/ros_camera.hpp"
#include "trackers/feature_tracker.hpp"

TEST(test_ros_camera, constructor) {
  Camera::Parameters cParams;
  cParams.name = "test_Camera";
  RosCamera rosCamera(cParams);
  EXPECT_TRUE(true);
}

TEST(test_ros_camera, ros_camera_message) {
  auto image_message = std::make_shared<sensor_msgs::msg::Image>();
  image_message->encoding = "bgr8";
  RosCameraMessage ros_camera_message(image_message);
}

TEST(test_ros_camera, ros_camera_callback) {
  auto image_message = std::make_shared<sensor_msgs::msg::Image>();
  image_message->encoding = "bgr8";
  cv::Mat img = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
  cv_bridge::CvImage cv_image_bridge;

  auto ros_camera_message = std::make_shared<RosCameraMessage>(image_message);
  ros_camera_message->image = img;

  Camera::Parameters cParams;
  cParams.name = "test_Camera";
  RosCamera rosCamera(cParams);
  rosCamera.Callback(ros_camera_message);

  FeatureTracker::Parameters tParams;
  auto feature_tracker = std::make_shared<FeatureTracker>(tParams);
  rosCamera.AddTracker(feature_tracker);
  rosCamera.Callback(ros_camera_message);
}
