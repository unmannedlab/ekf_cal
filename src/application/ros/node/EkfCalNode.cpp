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

#include "utility/TypeHelper.hpp"
#include "ekf/EKF.hpp"
#include "sensors/Camera.hpp"
#include "sensors/IMU.hpp"
#include "sensors/ros/RosCamera.hpp"
#include "sensors/ros/RosIMU.hpp"
#include "infrastructure/Logger.hpp"

using std::placeholders::_1;

EkfCalNode::EkfCalNode()
: Node("EkfCalNode")
{
  // Declare Parameters
  this->declare_parameter("Log_Level", 2);
  this->declare_parameter("IMU_list", std::vector<std::string>{});
  this->declare_parameter("Camera_list", std::vector<std::string>{});
  this->declare_parameter("Tracker_list", std::vector<std::string>{});

  initialize();
  declareSensors();
  loadSensors();
}

void EkfCalNode::initialize()
{
  // Set logging
  unsigned int logLevel = static_cast<unsigned int>(this->get_parameter("Log_Level").as_int());
  m_logger->setLogLevel(logLevel);

  // Load lists of sensors
  m_ImuList = this->get_parameter("IMU_list").as_string_array();
  m_CameraList = this->get_parameter("Camera_list").as_string_array();
  m_TrackerList = this->get_parameter("Tracker_list").as_string_array();
}

void EkfCalNode::declareSensors()
{
  for (std::string & imuName : m_ImuList) {
    declareImuParameters(imuName);
  }
  for (std::string & cameraName : m_CameraList) {
    declareCameraParameters(cameraName);
  }
  for (std::string & trackerName : m_TrackerList) {
    declareTrackerParameters(trackerName);
  }
}


void EkfCalNode::loadSensors()
{
  // Load IMU sensor parameters
  for (std::string & imuName : m_ImuList) {
    loadIMU(imuName);
  }

  // Load Camera sensor parameters
  for (std::string & camName : m_CameraList) {
    loadCamera(camName);
  }

  // Create publishers
  m_imgPublisher = this->create_publisher<sensor_msgs::msg::Image>("~/outImg", 10);
}

void EkfCalNode::declareImuParameters(std::string imuName)
{
  m_logger->log(LogLevel::INFO, "Declare IMU: " + imuName);

  // Declare parameters
  std::string imuPrefix = "IMU." + imuName;
  this->declare_parameter(imuPrefix + ".BaseSensor", false);
  this->declare_parameter(imuPrefix + ".Intrinsic", false);
  this->declare_parameter(imuPrefix + ".Rate", 1.0);
  this->declare_parameter(imuPrefix + ".Topic", "Topic");
  this->declare_parameter(
    imuPrefix + ".VarInit", std::vector<double>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  this->declare_parameter(imuPrefix + ".PosOffInit", std::vector<double>{0, 0, 0});
  this->declare_parameter(imuPrefix + ".AngOffInit", std::vector<double>{1, 0, 0, 0});
  this->declare_parameter(imuPrefix + ".AccBiasInit", std::vector<double>{0, 0, 0});
  this->declare_parameter(imuPrefix + ".OmgBiasInit", std::vector<double>{0, 0, 0});
}

/// @todo Move these getters into ROS helper?
IMU::Params EkfCalNode::getImuParameters(std::string imuName)
{
  // Load parameters
  std::string imuPrefix = "IMU." + imuName;
  bool baseSensor = this->get_parameter(imuPrefix + ".BaseSensor").as_bool();
  bool intrinsic = this->get_parameter(imuPrefix + ".Intrinsic").as_bool();
  double rate = this->get_parameter(imuPrefix + ".Rate").as_double();
  std::string topic = this->get_parameter(imuPrefix + ".Topic").as_string();
  std::vector<double> variance = this->get_parameter(imuPrefix + ".VarInit").as_double_array();
  std::vector<double> posOff = this->get_parameter(imuPrefix + ".PosOffInit").as_double_array();
  std::vector<double> angOff = this->get_parameter(imuPrefix + ".AngOffInit").as_double_array();
  std::vector<double> accBias = this->get_parameter(imuPrefix + ".AccBiasInit").as_double_array();
  std::vector<double> omgBias = this->get_parameter(imuPrefix + ".OmgBiasInit").as_double_array();

  // Assign parameters to struct
  IMU::Params imuParams;
  imuParams.name = imuName;
  imuParams.topic = topic;
  imuParams.baseSensor = baseSensor;
  imuParams.intrinsic = intrinsic;
  imuParams.rate = rate;
  imuParams.variance = stdToEigVec(variance);
  imuParams.posOffset = stdToEigVec(posOff);
  imuParams.angOffset = stdToEigQuat(angOff);
  imuParams.accBias = stdToEigVec(accBias);
  imuParams.omgBias = stdToEigVec(omgBias);
  return imuParams;
}

void EkfCalNode::declareCameraParameters(std::string cameraName)
{
  // Declare parameters
  std::string camPrefix = "Camera." + cameraName;
  this->declare_parameter(camPrefix + ".Rate", 1.0);
  this->declare_parameter(camPrefix + ".Topic", "Topic");
  this->declare_parameter(camPrefix + ".PosOffInit", std::vector<double>{0, 0, 0});
  this->declare_parameter(camPrefix + ".AngOffInit", std::vector<double>{1, 0, 0, 0});
  this->declare_parameter(camPrefix + ".VarInit", std::vector<double>{1, 1, 1, 1, 1, 1});
  this->declare_parameter(camPrefix + ".Tracker", "Tracker");
}

Camera::Params EkfCalNode::getCameraParameters(std::string cameraName)
{
  // Load parameters
  std::string camPrefix = "Camera." + cameraName;
  double rate = this->get_parameter(camPrefix + ".Rate").as_double();
  std::string topic = this->get_parameter(camPrefix + ".Topic").as_string();
  std::vector<double> posOff = this->get_parameter(camPrefix + ".PosOffInit").as_double_array();
  std::vector<double> angOff = this->get_parameter(camPrefix + ".AngOffInit").as_double_array();
  std::vector<double> variance = this->get_parameter(camPrefix + ".VarInit").as_double_array();
  std::string trackerName = this->get_parameter(camPrefix + ".Tracker").as_string();

  // Assign parameters to struct
  Camera::Params cameraParams;
  cameraParams.name = cameraName;
  cameraParams.topic = topic;
  cameraParams.rate = rate;
  cameraParams.posOffset = stdToEigVec(posOff);
  cameraParams.angOffset = stdToEigQuat(angOff);
  cameraParams.variance = stdToEigVec(variance);
  cameraParams.tracker = trackerName;
  return cameraParams;
}

/// @todo Change Feature Detector et. al. to be strings?
void EkfCalNode::declareTrackerParameters(std::string trackerName)
{
  // Declare parameters
  std::string trackerPrefix = "Tracker." + trackerName;
  this->declare_parameter(trackerPrefix + ".FeatureDetector", 0);
  this->declare_parameter(trackerPrefix + ".DescriptorExtractor", 0);
  this->declare_parameter(trackerPrefix + ".DescriptorMatcher", 0);
  this->declare_parameter(trackerPrefix + ".DetectorThreshold", 20.0);
}

Tracker::Params EkfCalNode::getTrackerParameters(std::string trackerName)
{
  // Get parameters
  std::string trackerPrefix = "Tracker." + trackerName;
  int fDetector = this->get_parameter(trackerPrefix + ".FeatureDetector").as_int();
  int dExtractor = this->get_parameter(trackerPrefix + ".DescriptorExtractor").as_int();
  int dMatcher = this->get_parameter(trackerPrefix + ".DescriptorMatcher").as_int();

  Tracker::Params trackerParams;
  trackerParams.detector = static_cast<Tracker::FeatureDetectorEnum>(fDetector);
  trackerParams.descriptor = static_cast<Tracker::DescriptorExtractorEnum>(dExtractor);
  trackerParams.matcher = static_cast<Tracker::DescriptorMatcherEnum>(dMatcher);
  trackerParams.threshold = this->get_parameter(trackerPrefix + ".DetectorThreshold").as_double();
  return trackerParams;
}


void EkfCalNode::loadIMU(std::string imuName)
{
  IMU::Params iParams = getImuParameters(imuName);
  m_logger->log(LogLevel::INFO, "Loaded IMU: " + imuName);

  // Create new RosIMU and and bind callback to ID
  std::shared_ptr<RosIMU> imuPtr = std::make_shared<RosIMU>(iParams);

  registerImu(imuPtr, iParams.topic);
}

void EkfCalNode::registerImu(std::shared_ptr<RosIMU> imuPtr, std::string topic)
{
  m_mapIMU[imuPtr->getId()] = imuPtr;

  std::function<void(std::shared_ptr<sensor_msgs::msg::Imu>)> function;
  function = std::bind(&EkfCalNode::imuCallback, this, _1, imuPtr->getId());

  auto sub = this->create_subscription<sensor_msgs::msg::Imu>(topic, 10, function);
  m_IMUSubs.push_back(sub);

  // if (iParams.baseSensor) {
  //   m_baseIMUAssigned = true;
  // }
  m_logger->log(
    LogLevel::INFO, "Registered IMU " + std::to_string(
      imuPtr->getId()) + ": " + imuPtr->getName());
}

void EkfCalNode::loadCamera(std::string cameraName)
{
  // Load parameters
  Camera::Params cParams = getCameraParameters(cameraName);
  Tracker::Params tParams = getTrackerParameters(cParams.tracker);
  m_logger->log(LogLevel::INFO, "Loaded Camera: " + cameraName);

  // Create new RosCamera and bind callback to ID
  std::shared_ptr<RosCamera> camPtr = std::make_shared<RosCamera>(cParams, tParams);

  registerCamera(camPtr, cParams.topic);
}

void EkfCalNode::registerCamera(std::shared_ptr<RosCamera> camPtr, std::string topic)
{
  m_mapCamera[camPtr->getId()] = camPtr;

  std::function<void(std::shared_ptr<sensor_msgs::msg::Image>)> function;
  function = std::bind(&EkfCalNode::cameraCallback, this, _1, camPtr->getId());
  auto sub = this->create_subscription<sensor_msgs::msg::Image>(topic, 10, function);
  m_CameraSubs.push_back(sub);

  m_logger->log(
    LogLevel::INFO, "Registered Camera " + std::to_string(
      camPtr->getId()) + ": " + camPtr->getName());
}


void EkfCalNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id)
{
  auto rosImuIter = m_mapIMU.find(id);
  if (rosImuIter != m_mapIMU.end()) {
    rosImuIter->second->callback(msg);
  } else {
    m_logger->log(LogLevel::WARN, "IMU ID Not Found: " + std::to_string(id));
  }
}

void EkfCalNode::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg, unsigned int id)
{
  auto rosCamIter = m_mapCamera.find(id);
  if (rosCamIter != m_mapCamera.end()) {
    rosCamIter->second->callback(msg);
    m_imgPublisher->publish(*rosCamIter->second->getRosImage().get());
  } else {
    m_logger->log(LogLevel::WARN, "Camera ID Not Found: " + std::to_string(id));
  }
}
