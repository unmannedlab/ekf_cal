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
  image_message->encoding = "mono16";
  RosCameraMessage ros_camera_message(image_message);
}
