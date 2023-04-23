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

#include "ekf_cal_node.hpp"

#include <eigen3/Eigen/Eigen>

#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include "utility/type_helper.hpp"
#include "ekf/ekf.hpp"
#include "sensors/camera.hpp"
#include "sensors/imu.hpp"
#include "sensors/ros/ros_camera.hpp"
#include "sensors/ros/ros_imu.hpp"
#include "infrastructure/debug_logger.hpp"

using std::placeholders::_1;

EkfCalNode::EkfCalNode()
: Node("EkfCalNode")
{
  // Declare Parameters
  this->declare_parameter("Debug_Log_Level", 0);
  this->declare_parameter("Data_Log_Level", 0);
  this->declare_parameter("IMU_list", std::vector<std::string>{});
  this->declare_parameter("Camera_list", std::vector<std::string>{});
  this->declare_parameter("Tracker_list", std::vector<std::string>{});

  Initialize();
  DeclareSensors();
  LoadSensors();
}

void EkfCalNode::Initialize()
{
  // Set logging
  unsigned int log_level =
    static_cast<unsigned int>(this->get_parameter("Debug_Log_Level").as_int());
  m_logger->SetLogLevel(log_level);

  // Load lists of sensors
  m_imu_list = this->get_parameter("IMU_list").as_string_array();
  m_camera_list = this->get_parameter("Camera_list").as_string_array();
  m_tracker_list = this->get_parameter("Tracker_list").as_string_array();
}

void EkfCalNode::DeclareSensors()
{
  for (std::string & imu_name : m_imu_list) {
    DeclareImuParameters(imu_name);
  }
  for (std::string & camera_name : m_camera_list) {
    DeclareCameraParameters(camera_name);
  }
  for (std::string & tracker_name : m_tracker_list) {
    DeclareTrackerParameters(tracker_name);
  }
}


void EkfCalNode::LoadSensors()
{
  // Load IMU sensor parameters
  for (std::string & imu_name : m_imu_list) {
    LoadIMU(imu_name);
  }

  // Load Camera sensor parameters
  for (std::string & cam_name : m_camera_list) {
    LoadCamera(cam_name);
  }

  // Create publishers
  m_img_publisher = this->create_publisher<sensor_msgs::msg::Image>("~/outImg", 10);
}

void EkfCalNode::DeclareImuParameters(std::string imu_name)
{
  m_logger->Log(LogLevel::INFO, "Declare IMU: " + imu_name);

  // Declare parameters
  std::string imu_prefix = "IMU." + imu_name;
  this->declare_parameter(imu_prefix + ".BaseSensor", false);
  this->declare_parameter(imu_prefix + ".Intrinsic", false);
  this->declare_parameter(imu_prefix + ".Rate", 1.0);
  this->declare_parameter(imu_prefix + ".Topic", "Topic");
  this->declare_parameter(
    imu_prefix + ".VarInit", std::vector<double>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  this->declare_parameter(imu_prefix + ".PosOffInit", std::vector<double>{0, 0, 0});
  this->declare_parameter(imu_prefix + ".AngOffInit", std::vector<double>{1, 0, 0, 0});
  this->declare_parameter(imu_prefix + ".AccBiasInit", std::vector<double>{0, 0, 0});
  this->declare_parameter(imu_prefix + ".OmgBiasInit", std::vector<double>{0, 0, 0});
}

/// @todo Move these getters into ROS helper?
IMU::Parameters EkfCalNode::GetImuParameters(std::string imu_name)
{
  // Load parameters
  std::string imu_prefix = "IMU." + imu_name;
  bool base_sensor = this->get_parameter(imu_prefix + ".BaseSensor").as_bool();
  bool intrinsic = this->get_parameter(imu_prefix + ".Intrinsic").as_bool();
  double rate = this->get_parameter(imu_prefix + ".Rate").as_double();
  std::string topic = this->get_parameter(imu_prefix + ".Topic").as_string();
  std::vector<double> variance = this->get_parameter(imu_prefix + ".VarInit").as_double_array();
  std::vector<double> pos_off = this->get_parameter(imu_prefix + ".PosOffInit").as_double_array();
  std::vector<double> ang_off = this->get_parameter(imu_prefix + ".AngOffInit").as_double_array();
  std::vector<double> acc_bias = this->get_parameter(imu_prefix + ".AccBiasInit").as_double_array();
  std::vector<double> omg_bias = this->get_parameter(imu_prefix + ".OmgBiasInit").as_double_array();

  // Assign parameters to struct
  IMU::Parameters imuParams;
  imuParams.name = imu_name;
  imuParams.topic = topic;
  imuParams.base_sensor = base_sensor;
  imuParams.intrinsic = intrinsic;
  imuParams.rate = rate;
  imuParams.variance = StdToEigVec(variance);
  imuParams.pos_offset = StdToEigVec(pos_off);
  imuParams.ang_offset = StdToEigQuat(ang_off);
  imuParams.acc_bias = StdToEigVec(acc_bias);
  imuParams.omg_bias = StdToEigVec(omg_bias);
  return imuParams;
}

void EkfCalNode::DeclareCameraParameters(std::string camera_name)
{
  // Declare parameters
  std::string cam_prefix = "Camera." + camera_name;
  this->declare_parameter(cam_prefix + ".Rate", 1.0);
  this->declare_parameter(cam_prefix + ".Topic", "Topic");
  this->declare_parameter(cam_prefix + ".PosOffInit", std::vector<double>{0, 0, 0});
  this->declare_parameter(cam_prefix + ".AngOffInit", std::vector<double>{1, 0, 0, 0});
  this->declare_parameter(cam_prefix + ".VarInit", std::vector<double>{1, 1, 1, 1, 1, 1});
  this->declare_parameter(cam_prefix + ".Tracker", "Tracker");
}

Camera::Parameters EkfCalNode::GetCameraParameters(std::string camera_name)
{
  // Load parameters
  std::string cam_prefix = "Camera." + camera_name;
  double rate = this->get_parameter(cam_prefix + ".Rate").as_double();
  std::string topic = this->get_parameter(cam_prefix + ".Topic").as_string();
  std::vector<double> pos_off = this->get_parameter(cam_prefix + ".PosOffInit").as_double_array();
  std::vector<double> ang_off = this->get_parameter(cam_prefix + ".AngOffInit").as_double_array();
  std::vector<double> variance = this->get_parameter(cam_prefix + ".VarInit").as_double_array();
  std::string tracker_name = this->get_parameter(cam_prefix + ".Tracker").as_string();

  // Assign parameters to struct
  Camera::Parameters camera_params;
  camera_params.name = camera_name;
  camera_params.topic = topic;
  camera_params.rate = rate;
  camera_params.pos_offset = StdToEigVec(pos_off);
  camera_params.ang_offset = StdToEigQuat(ang_off);
  camera_params.variance = StdToEigVec(variance);
  camera_params.tracker = tracker_name;
  return camera_params;
}

/// @todo Change Feature Detector et. al. to be strings?
void EkfCalNode::DeclareTrackerParameters(std::string tracker_name)
{
  // Declare parameters
  std::string tracker_prefix = "Tracker." + tracker_name;
  this->declare_parameter(tracker_prefix + ".FeatureDetector", 0);
  this->declare_parameter(tracker_prefix + ".DescriptorExtractor", 0);
  this->declare_parameter(tracker_prefix + ".DescriptorMatcher", 0);
  this->declare_parameter(tracker_prefix + ".DetectorThreshold", 20.0);
}

FeatureTracker::Parameters EkfCalNode::GetTrackerParameters(std::string tracker_name)
{
  // Get parameters
  std::string tracker_prefix = "Tracker." + tracker_name;
  int detector = this->get_parameter(tracker_prefix + ".FeatureDetector").as_int();
  int extractor = this->get_parameter(tracker_prefix + ".DescriptorExtractor").as_int();
  int matcher = this->get_parameter(tracker_prefix + ".DescriptorMatcher").as_int();

  FeatureTracker::Parameters tracker_params;
  tracker_params.detector = static_cast<FeatureTracker::FeatureDetectorEnum>(detector);
  tracker_params.descriptor = static_cast<FeatureTracker::DescriptorExtractorEnum>(extractor);
  tracker_params.matcher = static_cast<FeatureTracker::DescriptorMatcherEnum>(matcher);
  tracker_params.threshold = this->get_parameter(tracker_prefix + ".DetectorThreshold").as_double();
  return tracker_params;
}


void EkfCalNode::LoadIMU(std::string imu_name)
{
  IMU::Parameters imu_params = GetImuParameters(imu_name);
  m_logger->Log(LogLevel::INFO, "Loaded IMU: " + imu_name);

  // Create new RosIMU and and bind callback to ID
  std::shared_ptr<RosIMU> imu_ptr = std::make_shared<RosIMU>(imu_params);

  RegisterImu(imu_ptr, imu_params.topic);
}

void EkfCalNode::RegisterImu(std::shared_ptr<RosIMU> imu_ptr, std::string topic)
{
  m_map_imu[imu_ptr->GetId()] = imu_ptr;

  std::function<void(std::shared_ptr<sensor_msgs::msg::Imu>)> function;
  function = std::bind(&EkfCalNode::ImuCallback, this, _1, imu_ptr->GetId());

  auto sub = this->create_subscription<sensor_msgs::msg::Imu>(topic, 10, function);
  m_imu_subs.push_back(sub);

  // if (imu_params.baseSensor) {
  //   m_baseIMUAssigned = true;
  // }
  m_logger->Log(
    LogLevel::INFO, "Registered IMU " + std::to_string(
      imu_ptr->GetId()) + ": " + imu_ptr->GetName());
}

void EkfCalNode::LoadCamera(std::string camera_name)
{
  // Load parameters
  Camera::Parameters camera_params = GetCameraParameters(camera_name);
  FeatureTracker::Parameters tParams = GetTrackerParameters(camera_params.tracker);
  m_logger->Log(LogLevel::INFO, "Loaded Camera: " + camera_name);

  // Create new RosCamera and bind callback to ID
  std::shared_ptr<RosCamera> camera_ptr = std::make_shared<RosCamera>(camera_params);
  std::shared_ptr<FeatureTracker> trkPtr = std::make_shared<FeatureTracker>(tParams);
  camera_ptr->AddTracker(trkPtr);

  RegisterCamera(camera_ptr, camera_params.topic);
}

void EkfCalNode::RegisterCamera(std::shared_ptr<RosCamera> camera_ptr, std::string topic)
{
  m_map_camera[camera_ptr->GetId()] = camera_ptr;

  std::function<void(std::shared_ptr<sensor_msgs::msg::Image>)> function;
  function = std::bind(&EkfCalNode::CameraCallback, this, _1, camera_ptr->GetId());
  auto sub = this->create_subscription<sensor_msgs::msg::Image>(topic, 10, function);
  m_camera_subs.push_back(sub);

  m_logger->Log(
    LogLevel::INFO, "Registered Camera " + std::to_string(
      camera_ptr->GetId()) + ": " + camera_ptr->GetName());
}


void EkfCalNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id)
{
  auto ros_imu_iter = m_map_imu.find(id);
  if (ros_imu_iter != m_map_imu.end()) {
    auto ros_imu_message = std::make_shared<RosImuMessage>(msg);
    ros_imu_message->m_sensor_id = id;
    ros_imu_iter->second->Callback(ros_imu_message);
  } else {
    m_logger->Log(LogLevel::WARN, "IMU ID Not Found: " + std::to_string(id));
  }
}

void EkfCalNode::CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg, unsigned int id)
{
  auto rosCamIter = m_map_camera.find(id);
  if (rosCamIter != m_map_camera.end()) {
    auto ros_camera_message = std::make_shared<RosCameraMessage>(msg);
    ros_camera_message->m_sensor_id = id;
    rosCamIter->second->Callback(ros_camera_message);
    m_img_publisher->publish(*rosCamIter->second->GetRosImage().get());
  } else {
    m_logger->Log(LogLevel::WARN, "Camera ID Not Found: " + std::to_string(id));
  }
}
