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

#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>

#include "application/ros/node/ekf_cal_node.hpp"

///
/// @class EkfCalNode_test
/// @brief Testing class for EkfCalNode
///
class EkfCalNode_test : public ::testing::Test
{
protected:
  virtual void SetUp() {rclcpp::init(0, nullptr);}  ///< @brief EKF CAL Test node set up method
  virtual void TearDown() {rclcpp::shutdown();}     ///< @brief EKF CAL Test node tear down method
};

TEST_F(EkfCalNode_test, hello_world)
{
  EkfCalNode node;

  node.set_parameter(rclcpp::Parameter("debug_log_level", 1));
  node.set_parameter(rclcpp::Parameter("data_logging_on", true));
  node.set_parameter(rclcpp::Parameter("imu_list", std::vector<std::string>{"imu_1"}));
  node.set_parameter(rclcpp::Parameter("camera_list", std::vector<std::string>{"cam_2"}));
  node.set_parameter(rclcpp::Parameter("tracker_list", std::vector<std::string>{"tracker_3"}));
  node.set_parameter(rclcpp::Parameter("gps_list", std::vector<std::string>{"gps_4"}));
  node.set_parameter(rclcpp::Parameter("fiducial_list", std::vector<std::string>{"fiducial_5"}));

  node.Initialize();
  node.DeclareSensors();

  node.set_parameter(rclcpp::Parameter("imu.imu_1.is_extrinsic", false));
  node.set_parameter(rclcpp::Parameter("imu.imu_1.is_intrinsic", false));
  node.set_parameter(rclcpp::Parameter("imu.imu_1.rate", 400.0));
  node.set_parameter(rclcpp::Parameter("imu.imu_1.topic", "/ImuTopic"));
  node.set_parameter(
    rclcpp::Parameter(
      "imu.imu_1.variance",
      std::vector<double>{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01}));
  node.set_parameter(
    rclcpp::Parameter(
      "imu.imu_1.pos_i_in_b",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "imu.imu_1.ang_i_to_b",
      std::vector<double>{1.0, 0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "imu.imu_1.acc_bias", std::vector<double>{0.0, 0.0,
        0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "imu.imu_1.omg_bias", std::vector<double>{0.0, 0.0,
        0.0}));

  node.set_parameter(rclcpp::Parameter("camera.cam_2.rate", 5.0));
  node.set_parameter(rclcpp::Parameter("camera.cam_2.topic", "/CameraTopic"));
  node.set_parameter(
    rclcpp::Parameter(
      "camera.cam_2.pos_c_in_b",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "camera.cam_2.ang_c_to_b",
      std::vector<double>{1.0, 0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "camera.cam_2.variance",
      std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1}));
  node.set_parameter(rclcpp::Parameter("camera.cam_2.tracker", "tracker_3"));
  node.set_parameter(rclcpp::Parameter("camera.cam_2.fiducial", "fiducial_5"));

  node.set_parameter(rclcpp::Parameter("tracker.tracker_3.feature_detector", 4));
  node.set_parameter(rclcpp::Parameter("tracker.tracker_3.descriptor_extractor", 0));
  node.set_parameter(rclcpp::Parameter("tracker.tracker_3.descriptor_matcher", 0));
  node.set_parameter(rclcpp::Parameter("tracker.tracker_3.detector_threshold", 10.0));

  node.set_parameter(rclcpp::Parameter("gps.gps_4.topic", "/gps1"));
  node.set_parameter(rclcpp::Parameter("gps.gps_4.rate", 10.0));
  node.set_parameter(rclcpp::Parameter("gps.gps_4.variance", std::vector<double>{1.0, 1.0, 1.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "gps.gps_4.pos_a_in_b",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "gps.gps_4.pos_l_in_g",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(rclcpp::Parameter("gps.gps_4.ang_l_to_g", 0.0));


  node.set_parameter(rclcpp::Parameter("fiducial.fiducial_5.fiducial_type", 1));
  node.set_parameter(rclcpp::Parameter("fiducial.fiducial_5.squares_x", 5));
  node.set_parameter(rclcpp::Parameter("fiducial.fiducial_5.squares_y", 7));
  node.set_parameter(rclcpp::Parameter("fiducial.fiducial_5.square_length", 0.04));
  node.set_parameter(rclcpp::Parameter("fiducial.fiducial_5.marker_length", 0.02));
  node.set_parameter(rclcpp::Parameter("fiducial.fiducial_5.min_track_length", 0));
  node.set_parameter(rclcpp::Parameter("fiducial.fiducial_5.max_track_length", 1));
  node.set_parameter(rclcpp::Parameter("fiducial.fiducial_5.is_extrinsic", false));
  node.set_parameter(rclcpp::Parameter("fiducial.fiducial_5.data_log_rate", 20.0));
  node.set_parameter(
    rclcpp::Parameter(
      "fiducial.fiducial_5.pos_f_in_l",
      std::vector<double>{0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "fiducial.fiducial_5.ang_f_to_l",
      std::vector<double>{1.0, 0.0, 0.0, 0.0}));
  node.set_parameter(
    rclcpp::Parameter(
      "fiducial.fiducial_5.variance",
      std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1}));

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

  imu_msg->header.stamp.nanosec = 500000000;
  node.ImuCallback(imu_msg, imu_id);

  unsigned int cam_id = 2;
  auto cam_msg = std::make_shared<sensor_msgs::msg::Image>();
  cv::Mat cv_image = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  cam_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "8UC1", cv_image).toImageMsg();

  node.CameraCallback(cam_msg, cam_id);

  unsigned int gps_id = 3;
  auto gps_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
  gps_msg->altitude = 0.0;
  gps_msg->latitude = 0.0;
  gps_msg->longitude = 0.0;

  node.GpsCallback(gps_msg, gps_id);

  node.PublishState();

  EXPECT_TRUE(true);
}
