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

#include "sensors/imu.hpp"
#include "sensors/ros/ros_imu.hpp"
#include "sensors/ros/ros_imu_message.hpp"


TEST(test_RosIMU, Constructor) {
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(logger, 0.0, false, "");
  IMU::Parameters ros_imu_params;
  ros_imu_params.logger = logger;
  ros_imu_params.ekf = ekf;
  RosIMU rosIMU(ros_imu_params);
}

TEST(test_RosGPS, ros_gps_message) {
  RosImuMessage ros_imu_message;
  EXPECT_TRUE(true);
}
