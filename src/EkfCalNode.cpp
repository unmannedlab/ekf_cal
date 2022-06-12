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
}

void EkfCalNode::LoadImu(std::string imuName)
{
  // Declare parameters
  std::string imuPrefix = "IMUs." + imuName;
  this->declare_parameter(imuPrefix + ".BaseSensor");
  this->declare_parameter(imuPrefix + ".Intrinsic");
  this->declare_parameter(imuPrefix + ".Rate");
  this->declare_parameter(imuPrefix + ".Topic");
  this->declare_parameter(imuPrefix + ".PosOffInit");
  this->declare_parameter(imuPrefix + ".QuatOffInit");
  this->declare_parameter(imuPrefix + ".AccBiasInit");
  this->declare_parameter(imuPrefix + ".OmgBiasInit");

  // Load parameters
  bool baseSensor = this->get_parameter(imuPrefix + ".BaseSensor").as_bool();
  bool intrinsic = this->get_parameter(imuPrefix + ".Intrinsic").as_bool();
  double rate = this->get_parameter(imuPrefix + ".Rate").as_double();
  std::string topic = this->get_parameter(imuPrefix + ".Topic").as_string();
  std::vector<double> posOff = this->get_parameter(imuPrefix + ".PosOffInit").as_double_array();
  std::vector<double> quatOff = this->get_parameter(imuPrefix + ".QuatOffInit").as_double_array();
  std::vector<double> accBias = this->get_parameter(imuPrefix + ".AccBiasInit").as_double_array();
  std::vector<double> omgBias = this->get_parameter(imuPrefix + ".OmgBiasInit").as_double_array();

  // Assign parameters to struct
  Imu::Params imuParams;
  imuParams.name = imuName;
  imuParams.baseSensor = baseSensor;
  imuParams.intrinsic = intrinsic;
  imuParams.rate = rate;
  imuParams.posOffset = TypeHelper::StdToEigVec(posOff);
  imuParams.angOffset = TypeHelper::StdToEigQuat(quatOff);
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
}

void EkfCalNode::CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg, unsigned int id)
{
  double time = TypeHelper::RosHeaderToTime(msg->header);

  m_ekf.CameraCallback(id, time);
}

void EkfCalNode::LidarCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg, unsigned int id)
{
  double time = TypeHelper::RosHeaderToTime(msg->header);
  m_ekf.LidarCallback(id, time);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfCalNode>());
  rclcpp::shutdown();

  return 0;
}
