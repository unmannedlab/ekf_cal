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

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "sensors/gps.hpp"
#include "sensors/ros/ros_gps_message.hpp"
#include "sensors/ros/ros_gps.hpp"


TEST(test_RosGPS, Constructor) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  GPS::Parameters ros_gps_params;
  ros_gps_params.logger = ekf_params.debug_logger;
  ros_gps_params.ekf = ekf;
  RosGPS rosGPS(ros_gps_params);
}

TEST(test_RosGPS, ros_gps_message) {
  auto nav_sat_fix_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
  RosGpsMessage ros_gps_message(nav_sat_fix_msg);
  EXPECT_TRUE(true);
}

TEST(test_ros_camera, ros_gps_callback) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  GPS::Parameters ros_gps_params;
  ros_gps_params.logger = ekf_params.debug_logger;
  ros_gps_params.ekf = ekf;
  RosGPS rosGPS(ros_gps_params);


  auto nav_sat_fix_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
  nav_sat_fix_msg->header.stamp.sec = 0;
  nav_sat_fix_msg->header.stamp.nanosec = 0;
  nav_sat_fix_msg->latitude = 0.0;
  nav_sat_fix_msg->longitude = 0.0;
  nav_sat_fix_msg->altitude = 0.0;

  RosGpsMessage ros_gps_message(nav_sat_fix_msg);

  rosGPS.Callback(ros_gps_message);
}
