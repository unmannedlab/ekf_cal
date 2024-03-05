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

#include <stddef.h>

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

#include "application/ros/node/ekf_cal_node.hpp"

/// @todo Write these integration tests
class EkfCalNode_test : public ::testing::Test
{
protected:
  virtual void SetUp() {rclcpp::init(0, NULL);}  ///< @brief EKF CAL Test node set up method
  virtual void TearDown() {rclcpp::shutdown();}  ///< @brief EKF CAL Test node tear down method
};

TEST_F(EkfCalNode_test, hello_world)
{
  EkfCalNode node;

  node.set_parameter(rclcpp::Parameter("debug_log_level", 1));
  node.set_parameter(rclcpp::Parameter("data_logging_on", true));
  node.set_parameter(rclcpp::Parameter("imu_list", std::vector<std::string>{"TestImu"}));
  node.set_parameter(rclcpp::Parameter("camera_list", std::vector<std::string>{"TestCamera"}));
  node.set_parameter(rclcpp::Parameter("tracker_list", std::vector<std::string>{"TestTracker"}));
  node.set_parameter(rclcpp::Parameter("gps_list", std::vector<std::string>{"TestGps"}));

  node.Initialize();
  node.DeclareSensors();

  node.set_parameter(rclcpp::Parameter("imu.TestImu.is_extrinsic", false));
  node.set_parameter(rclcpp::Parameter("imu.TestImu.is_intrinsic", false));
  node.set_parameter(rclcpp::Parameter("imu.TestImu.rate", 400.0));
  node.set_parameter(rclcpp::Parameter("imu.TestImu.topic", "/ImuTopic"));
  node.set_parameter(
    rclcpp::Parameter(
      "imu.TestImu.variance",
      std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01}));
  node.set_parameter(
    rclcpp::Parameter(
      "imu.TestImu.pos_i_in_b",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "imu.TestImu.ang_i_to_b",
      std::vector<double>{1.0, 0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "imu.TestImu.acc_bias", std::vector<double>{0.0, 0.0,
        0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "imu.TestImu.omg_bias", std::vector<double>{0.0, 0.0,
        0.0}));

  node.set_parameter(rclcpp::Parameter("camera.TestCamera.rate", 5.0));
  node.set_parameter(rclcpp::Parameter("camera.TestCamera.topic", "/CameraTopic"));
  node.set_parameter(
    rclcpp::Parameter(
      "camera.TestCamera.pos_c_in_b",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "camera.TestCamera.ang_c_to_b",
      std::vector<double>{1.0, 0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "camera.TestCamera.variance",
      std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1}));
  node.set_parameter(rclcpp::Parameter("camera.TestCamera.tracker", "TestTracker"));

  node.set_parameter(rclcpp::Parameter("tracker.TestTracker.feature_detector", 4));
  node.set_parameter(rclcpp::Parameter("tracker.TestTracker.descriptor_extractor", 0));
  node.set_parameter(rclcpp::Parameter("tracker.TestTracker.descriptor_matcher", 0));
  node.set_parameter(rclcpp::Parameter("tracker.TestTracker.detector_threshold", 10.0));

  node.set_parameter(rclcpp::Parameter("GPS.TestGps.Topic", "/gps1"));
  node.set_parameter(rclcpp::Parameter("GPS.TestGps.Rate", 10.0));
  node.set_parameter(rclcpp::Parameter("GPS.TestGps.VarInit", std::vector<double>{1.0, 1.0, 1.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "GPS.TestGps.pos_a_in_b",
      std::vector<double>{0.0, 0.0, 0.0}));

  node.LoadSensors();
  auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
  unsigned int imu_id = 1;
  imu_msg->header.stamp.sec = 0;
  imu_msg->header.stamp.nanosec = 0;
  imu_msg->linear_acceleration.x = 0.0;
  imu_msg->linear_acceleration.y = 0.0;
  imu_msg->linear_acceleration.z = 0.0;
  imu_msg->angular_velocity.x = 0.0;
  imu_msg->angular_velocity.y = 0.0;
  imu_msg->angular_velocity.z = 0.0;
  imu_msg->linear_acceleration_covariance.fill(0);
  imu_msg->angular_velocity_covariance.fill(0);

  node.ImuCallback(imu_msg, imu_id);
  EXPECT_TRUE(true);

  imu_msg->header.stamp.nanosec = 500000000;
  node.ImuCallback(imu_msg, imu_id);
  EXPECT_TRUE(true);

  /// @todo add image to callback
  // sensor_msgs::msg::Image::SharedPtr camMsg{};
  // unsigned int camID = 2;
  // node.cameraCallback(camMsg, camID);

  EXPECT_TRUE(true);
}
