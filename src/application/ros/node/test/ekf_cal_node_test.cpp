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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

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

  node.set_parameter(rclcpp::Parameter("Debug_Log_Level", 1));
  node.set_parameter(rclcpp::Parameter("IMU_list", std::vector<std::string>{"TestImu"}));
  node.set_parameter(rclcpp::Parameter("Camera_list", std::vector<std::string>{"TestCamera"}));
  node.set_parameter(rclcpp::Parameter("Tracker_list", std::vector<std::string>{"TestTracker"}));

  node.Initialize();
  node.DeclareSensors();

  node.set_parameter(rclcpp::Parameter("IMU.TestImu.BaseSensor", true));
  node.set_parameter(rclcpp::Parameter("IMU.TestImu.Intrinsic", false));
  node.set_parameter(rclcpp::Parameter("IMU.TestImu.Rate", 400.0));
  node.set_parameter(rclcpp::Parameter("IMU.TestImu.Topic", "/ImuTopic"));
  node.set_parameter(
    rclcpp::Parameter(
      "IMU.TestImu.VarInit",
      std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01}));
  node.set_parameter(
    rclcpp::Parameter(
      "IMU.TestImu.PosOffInit",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "IMU.TestImu.AngOffInit",
      std::vector<double>{1.0, 0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "IMU.TestImu.AccBiasInit", std::vector<double>{0.0, 0.0,
        0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "IMU.TestImu.OmgBiasInit", std::vector<double>{0.0, 0.0,
        0.0}));

  node.set_parameter(rclcpp::Parameter("Camera.TestCamera.Rate", 5.0));
  node.set_parameter(rclcpp::Parameter("Camera.TestCamera.Topic", "/CameraTopic"));
  node.set_parameter(
    rclcpp::Parameter(
      "Camera.TestCamera.PosOffInit",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "Camera.TestCamera.AngOffInit",
      std::vector<double>{1.0, 0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "Camera.TestCamera.VarInit",
      std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1}));
  node.set_parameter(rclcpp::Parameter("Camera.TestCamera.Tracker", "TestTracker"));

  node.set_parameter(rclcpp::Parameter("Tracker.TestTracker.FeatureDetector", 4));
  node.set_parameter(rclcpp::Parameter("Tracker.TestTracker.DescriptorExtractor", 0));
  node.set_parameter(rclcpp::Parameter("Tracker.TestTracker.DescriptorMatcher", 0));
  node.set_parameter(rclcpp::Parameter("Tracker.TestTracker.DetectorThreshold", 10.0));

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
