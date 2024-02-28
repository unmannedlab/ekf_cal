// Copyright 2024 Jacob Hartzer
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

#include "sensors/camera_message.hpp"
#include "sensors/imu_message.hpp"
#include "sensors/sensor_message.hpp"
#include "sensors/sensor.hpp"


TEST(test_sensor, Constructor) {
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  Sensor sensor("", logger);
}

TEST(test_sensor, MessageCompare) {
  cv::Mat cam_img = cv::Mat::zeros(cv::Size(640, 480), CV_8UC3);
  auto camera_message = std::make_shared<CameraMessage>(cam_img);
  camera_message->m_time = 0.0;

  auto imu_message = std::make_shared<ImuMessage>();
  imu_message->m_time = 0.1;

  EXPECT_TRUE(MessageCompare(camera_message, imu_message));
  EXPECT_FALSE(MessageCompare(imu_message, camera_message));
}
