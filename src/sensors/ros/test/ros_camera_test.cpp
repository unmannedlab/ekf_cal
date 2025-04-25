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

#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "sensors/camera.hpp"
#include "sensors/ros/ros_camera_message.hpp"
#include "sensors/ros/ros_camera.hpp"
#include "trackers/feature_tracker.hpp"

TEST(test_ros_camera, constructor) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  Camera::Parameters cam_params;
  cam_params.name = "test_Camera";
  cam_params.ekf = ekf;
  RosCamera rosCamera(cam_params);
  EXPECT_TRUE(true);
}

TEST(test_ros_camera, ros_camera_message) {
  auto image_message = std::make_shared<sensor_msgs::msg::Image>();
  image_message->encoding = "bgr8";
  RosCameraMessage ros_camera_message(image_message);
  EXPECT_TRUE(true);
}

TEST(test_ros_camera, ros_camera_callback) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  auto image_message = std::make_shared<sensor_msgs::msg::Image>();
  image_message->encoding = "bgr8";
  cv::Mat img = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  cv_bridge::CvImage cv_image_bridge;

  RosCameraMessage ros_camera_message(image_message);
  ros_camera_message.image = img;

  Camera::Parameters cam_params;
  cam_params.name = "test_Camera";
  cam_params.ekf = ekf;
  cam_params.logger = ekf_params.debug_logger;
  RosCamera rosCamera(cam_params);
  rosCamera.Callback(ros_camera_message);

  FeatureTracker::Parameters tracker_params;
  tracker_params.ekf = ekf;
  tracker_params.logger = ekf_params.debug_logger;
  auto feature_tracker = std::make_shared<FeatureTracker>(tracker_params);
  rosCamera.AddTracker(feature_tracker);
  rosCamera.Callback(ros_camera_message);

  sensor_msgs::msg::Image::SharedPtr image_message_out = rosCamera.GetRosImage();
  EXPECT_TRUE(true);
}
