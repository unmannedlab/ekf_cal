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

#include "EkfCalNode.hpp"

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "TypeHelper.hpp"
#include "ekf/EKF.hpp"
#include "ekf/sensors/Camera.hpp"
#include "ekf/sensors/Imu.hpp"
#include "ekf/sensors/Lidar.hpp"

using std::placeholders::_1;

EkfCalNode::EkfCalNode()
: Node("EkfCalNode")
{
  // Declare Parameters
  this->declare_parameter("IMU_list");
  this->declare_parameter("Camera_list");
  this->declare_parameter("LIDAR_list");

  // Load lists of sensors
  std::vector<std::string> imuList = this->get_parameter("IMU_list").as_string_array();
  std::vector<std::string> camList = this->get_parameter("Camera_list").as_string_array();
  std::vector<std::string> lidarList = this->get_parameter("LIDAR_list").as_string_array();

  // Load Imu sensor parameters
  for (std::string & imuName : imuList) {
    LoadImu(imuName);
  }
  if (m_baseImuAssigned == false) {
    RCLCPP_WARN(get_logger(), "Base IMU should be set for filter stability");
  }

  // Load Camera sensor parameters
  for (std::string & camName : camList) {
    LoadCamera(camName);
  }

  // Load Lidar sensor parameters
  for (std::string & lidarName : lidarList) {
    LoadLidar(lidarName);
  }

  // Create publishers
  pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
  twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/twist", 10);
}

void EkfCalNode::LoadImu(std::string imuName)
{
  // Declare parameters
  std::string imuPrefix = "IMUs." + imuName;
  this->declare_parameter(imuPrefix + ".BaseSensor");
  this->declare_parameter(imuPrefix + ".Intrinsic");
  this->declare_parameter(imuPrefix + ".Rate");
  this->declare_parameter(imuPrefix + ".Topic");

  // Load parameters
  bool baseSensor = this->get_parameter(imuPrefix + ".BaseSensor").as_bool();
  bool intrinsic = this->get_parameter(imuPrefix + ".Intrinsic").as_bool();
  double rate = this->get_parameter(imuPrefix + ".Rate").as_double();
  std::string topic = this->get_parameter(imuPrefix + ".Topic").as_string();
  std::vector<double> posOff {0, 0, 0};
  std::vector<double> angOff {0, 0, 0, 0};
  std::vector<double> accBias {0, 0, 0};
  std::vector<double> omgBias {0, 0, 0};

  // Only calibrate offsets if not the base IMU
  if (baseSensor == false) {
    this->declare_parameter(imuPrefix + ".PosOffInit");
    this->declare_parameter(imuPrefix + ".AngOffInit");
    posOff = this->get_parameter(imuPrefix + ".PosOffInit").as_double_array();
    angOff = this->get_parameter(imuPrefix + ".AngOffInit").as_double_array();
  }

  // Only calibrate intrinsics if flag is set
  if (intrinsic == true) {
    this->declare_parameter(imuPrefix + ".AccBiasInit");
    this->declare_parameter(imuPrefix + ".OmgBiasInit");
    accBias = this->get_parameter(imuPrefix + ".AccBiasInit").as_double_array();
    omgBias = this->get_parameter(imuPrefix + ".OmgBiasInit").as_double_array();
  }

  if ((baseSensor == false) || (intrinsic == true)) {
    this->declare_parameter(imuPrefix + ".VarInit");
    std::vector<double> variance = this->get_parameter(imuPrefix + ".VarInit").as_double_array();
  }

  // Assign parameters to struct
  Imu::Params imuParams;
  imuParams.name = imuName;
  imuParams.baseSensor = baseSensor;
  imuParams.intrinsic = intrinsic;
  imuParams.rate = rate;
  imuParams.posOffset = TypeHelper::StdToEigVec(posOff);
  imuParams.angOffset = TypeHelper::StdToEigQuat(angOff);
  imuParams.accBias = TypeHelper::StdToEigVec(accBias);
  imuParams.omgBias = TypeHelper::StdToEigVec(omgBias);

  // Register IMU and bind callback to ID
  unsigned int id = m_ekf.RegisterSensor(imuParams);
  std::function<void(std::shared_ptr<sensor_msgs::msg::Imu>)> function;
  function = std::bind(&EkfCalNode::ImuCallback, this, _1, id);
  m_ImuSubs.push_back(this->create_subscription<sensor_msgs::msg::Imu>(topic, 10, function));

  if (imuParams.baseSensor) {
    m_baseImuAssigned = true;
  }
  RCLCPP_INFO(get_logger(), "Loaded IMU: '%s'", imuName.c_str());
}

void EkfCalNode::LoadCamera(std::string camName)
{
  RCLCPP_INFO(get_logger(), "Camera not Loaded: '%s'", camName.c_str());
}

void EkfCalNode::LoadLidar(std::string lidarName)
{
  RCLCPP_INFO(get_logger(), "LIDAR not Loaded: '%s'", lidarName.c_str());
}

void EkfCalNode::ImuCallback(
  const sensor_msgs::msg::Imu::SharedPtr msg,
  unsigned int id)
{
  double time = TypeHelper::RosHeaderToTime(msg->header);
  Eigen::Vector3d acc = TypeHelper::RosToEigen(msg->linear_acceleration);
  Eigen::Vector3d omg = TypeHelper::RosToEigen(msg->angular_velocity);
  Eigen::Matrix3d acc_cov = TypeHelper::RosToEigen(msg->linear_acceleration_covariance);
  Eigen::Matrix3d omg_cov = TypeHelper::RosToEigen(msg->angular_velocity_covariance);

  m_ekf.ImuCallback(id, time, acc, acc_cov, omg, omg_cov);
  PublishState();
}

void EkfCalNode::CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg, unsigned int id)
{
  double time = TypeHelper::RosHeaderToTime(msg->header);

  m_ekf.CameraCallback(id, time);
  PublishState();
}

void EkfCalNode::LidarCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg, unsigned int id)
{
  double time = TypeHelper::RosHeaderToTime(msg->header);
  m_ekf.LidarCallback(id, time);
  PublishState();
}

void EkfCalNode::PublishState()
{
  auto pose_msg = geometry_msgs::msg::PoseStamped();
  auto twist_msg = geometry_msgs::msg::TwistStamped();

  Eigen::VectorXd state = m_ekf.GetState();

  // Position
  pose_msg.pose.position.x = state(0);
  pose_msg.pose.position.y = state(1);
  pose_msg.pose.position.z = state(2);

  // Orientation
  Eigen::Quaterniond quat = TypeHelper::RotVecToQuat(state.segment(9, 3));
  pose_msg.pose.orientation.w = quat.w();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();

  // Linear Velocity
  twist_msg.twist.linear.x = state(3);
  twist_msg.twist.linear.y = state(4);
  twist_msg.twist.linear.z = state(5);

  // Angular Velocity
  twist_msg.twist.angular.x = state(12);
  twist_msg.twist.angular.y = state(13);
  twist_msg.twist.angular.z = state(14);


  rclcpp::Time now = this->get_clock()->now();
  pose_msg.header.stamp = now;
  twist_msg.header.stamp = now;

  pose_pub->publish(pose_msg);
  twist_pub->publish(twist_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfCalNode>());
  rclcpp::shutdown();

  return 0;
}
