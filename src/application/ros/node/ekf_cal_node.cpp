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

#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "infrastructure/ekf_cal_version.hpp"
#include "sensors/camera.hpp"
#include "sensors/imu.hpp"
#include "sensors/ros/ros_camera_message.hpp"
#include "sensors/ros/ros_camera.hpp"
#include "sensors/ros/ros_imu_message.hpp"
#include "sensors/ros/ros_imu.hpp"
#include "utility/string_helper.hpp"
#include "utility/type_helper.hpp"

using std::placeholders::_1;

EkfCalNode::EkfCalNode()
: Node("EkfCalNode")
{
  // Declare Parameters
  this->declare_parameter("debug_log_level", 0);
  this->declare_parameter("data_logging_on", false);
  this->declare_parameter("imu_list", std::vector<std::string>{});
  this->declare_parameter("camera_list", std::vector<std::string>{});
  this->declare_parameter("tracker_list", std::vector<std::string>{});

  m_state_pub_timer =
    this->create_wall_timer(std::chrono::seconds(1), std::bind(&EkfCalNode::PublishState, this));
}

void EkfCalNode::Initialize()
{
  // Set logging
  int debug_log_level = this->get_parameter("debug_log_level").as_int();
  bool data_logging_on = this->get_parameter("data_logging_on").as_bool();
  m_logger->SetLogLevel(static_cast<unsigned int>(debug_log_level));
  m_logger->Log(LogLevel::INFO, "EKF CAL Version: " + std::string(EKF_CAL_VERSION));
  m_state_data_logger.SetLogging(data_logging_on);
  m_state_data_logger.SetOutputDirectory("~/log/");
  m_state_data_logger.SetOutputFileName("state_vector.csv");
  m_state_data_logger.DefineHeader("\n");

  // Load lists of sensors
  m_imu_list = this->get_parameter("imu_list").as_string_array();
  m_camera_list = this->get_parameter("camera_list").as_string_array();
  m_tracker_list = this->get_parameter("tracker_list").as_string_array();
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
  m_img_publisher = this->create_publisher<sensor_msgs::msg::Image>("~/OutImg", 10);
  m_body_state_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("~/BodyState", 10);
  m_imu_state_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("~/ImuState", 10);
}

void EkfCalNode::DeclareImuParameters(std::string imu_name)
{
  m_logger->Log(LogLevel::INFO, "Declare IMU: " + imu_name);

  // Declare parameters
  std::string imu_prefix = "imu." + imu_name;
  this->declare_parameter(imu_prefix + ".is_extrinsic", false);
  this->declare_parameter(imu_prefix + ".is_intrinsic", false);
  this->declare_parameter(imu_prefix + ".use_for_prediction", false);
  this->declare_parameter(imu_prefix + ".rate", 1.0);
  this->declare_parameter(imu_prefix + ".topic", "");
  this->declare_parameter(
    imu_prefix + ".variance", std::vector<double>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  this->declare_parameter(imu_prefix + ".pos_i_in_b", std::vector<double>{0, 0, 0});
  this->declare_parameter(imu_prefix + ".ang_i_to_b", std::vector<double>{1, 0, 0, 0});
  this->declare_parameter(imu_prefix + ".acc_bias", std::vector<double>{0, 0, 0});
  this->declare_parameter(imu_prefix + ".omg_bias", std::vector<double>{0, 0, 0});
  this->declare_parameter(imu_prefix + ".pos_stability", 1e-9);
  this->declare_parameter(imu_prefix + ".ang_stability", 1e-9);
  this->declare_parameter(imu_prefix + ".acc_bias_stability", 1e-9);
  this->declare_parameter(imu_prefix + ".omg_bias_stability", 1e-9);
}

/// @todo Move these getters into ROS helper?
IMU::Parameters EkfCalNode::GetImuParameters(std::string imu_name)
{
  // Load parameters
  std::string imu_prefix = "imu." + imu_name;
  bool is_extrinsic = this->get_parameter(imu_prefix + ".is_extrinsic").as_bool();
  bool is_intrinsic = this->get_parameter(imu_prefix + ".is_intrinsic").as_bool();
  bool use_for_prediction = this->get_parameter(imu_prefix + ".use_for_prediction").as_bool();
  double rate = this->get_parameter(imu_prefix + ".rate").as_double();
  std::string topic = this->get_parameter(imu_prefix + ".topic").as_string();
  std::vector<double> variance = this->get_parameter(imu_prefix + ".variance").as_double_array();
  std::vector<double> pos_b_in_g =
    this->get_parameter(imu_prefix + ".pos_i_in_b").as_double_array();
  std::vector<double> ang_b_to_g =
    this->get_parameter(imu_prefix + ".ang_i_to_b").as_double_array();
  std::vector<double> acc_bias = this->get_parameter(imu_prefix + ".acc_bias").as_double_array();
  std::vector<double> omg_bias = this->get_parameter(imu_prefix + ".omg_bias").as_double_array();
  double pos_stability = this->get_parameter(imu_prefix + ".pos_stability").as_double();
  double ang_stability = this->get_parameter(imu_prefix + ".ang_stability").as_double();
  double acc_bias_stability = this->get_parameter(imu_prefix + ".acc_bias_stability").as_double();
  double omg_bias_stability = this->get_parameter(imu_prefix + ".omg_bias_stability").as_double();

  // Assign parameters to struct
  IMU::Parameters imu_params;
  imu_params.name = imu_name;
  imu_params.topic = topic;
  imu_params.is_extrinsic = is_extrinsic;
  imu_params.is_intrinsic = is_intrinsic;
  imu_params.use_for_prediction = use_for_prediction;
  imu_params.rate = rate;
  imu_params.variance = StdToEigVec(variance);
  imu_params.pos_i_in_b = StdToEigVec(pos_b_in_g);
  imu_params.ang_i_to_b = StdToEigQuat(ang_b_to_g);
  imu_params.acc_bias = StdToEigVec(acc_bias);
  imu_params.omg_bias = StdToEigVec(omg_bias);
  imu_params.pos_stability = pos_stability;
  imu_params.ang_stability = ang_stability;
  imu_params.acc_bias_stability = acc_bias_stability;
  imu_params.omg_bias_stability = omg_bias_stability;
  return imu_params;
}

void EkfCalNode::DeclareCameraParameters(std::string camera_name)
{
  // Declare parameters
  std::string cam_prefix = "camera." + camera_name;
  this->declare_parameter(cam_prefix + ".rate", 1.0);
  this->declare_parameter(cam_prefix + ".topic", "");
  this->declare_parameter(cam_prefix + ".pos_c_in_b", std::vector<double>{0, 0, 0});
  this->declare_parameter(cam_prefix + ".ang_c_to_b", std::vector<double>{1, 0, 0, 0});
  this->declare_parameter(cam_prefix + ".variance", std::vector<double>{1, 1, 1, 1, 1, 1});
  this->declare_parameter(cam_prefix + ".tracker", "");
}

Camera::Parameters EkfCalNode::GetCameraParameters(std::string camera_name)
{
  // Load parameters
  std::string cam_prefix = "camera." + camera_name;
  double rate = this->get_parameter(cam_prefix + ".rate").as_double();
  std::string topic = this->get_parameter(cam_prefix + ".topic").as_string();
  std::vector<double> pos_b_in_g =
    this->get_parameter(cam_prefix + ".pos_c_in_b").as_double_array();
  std::vector<double> ang_b_to_g =
    this->get_parameter(cam_prefix + ".ang_c_to_b").as_double_array();
  std::vector<double> variance = this->get_parameter(cam_prefix + ".variance").as_double_array();
  std::string tracker_name = this->get_parameter(cam_prefix + ".tracker").as_string();

  // Assign parameters to struct
  Camera::Parameters camera_params;
  camera_params.name = camera_name;
  camera_params.topic = topic;
  camera_params.rate = rate;
  camera_params.pos_c_in_b = StdToEigVec(pos_b_in_g);
  camera_params.ang_c_to_b = StdToEigQuat(ang_b_to_g);
  camera_params.variance = StdToEigVec(variance);
  camera_params.tracker = tracker_name;
  return camera_params;
}

/// @todo Change Feature Detector et. al. to be strings?
void EkfCalNode::DeclareTrackerParameters(std::string tracker_name)
{
  // Declare parameters
  std::string tracker_prefix = "tracker." + tracker_name;
  this->declare_parameter(tracker_prefix + ".feature_detector", 0);
  this->declare_parameter(tracker_prefix + ".descriptor_extractor", 0);
  this->declare_parameter(tracker_prefix + ".descriptor_matcher", 0);
  this->declare_parameter(tracker_prefix + ".detector_threshold", 20.0);
}

FeatureTracker::Parameters EkfCalNode::GetTrackerParameters(std::string tracker_name)
{
  // Get parameters
  std::string tracker_prefix = "tracker." + tracker_name;
  int detector = this->get_parameter(tracker_prefix + ".feature_detector").as_int();
  int extractor = this->get_parameter(tracker_prefix + ".descriptor_extractor").as_int();
  int matcher = this->get_parameter(tracker_prefix + ".descriptor_matcher").as_int();

  FeatureTracker::Parameters tracker_params;
  tracker_params.detector = static_cast<FeatureTracker::FeatureDetectorEnum>(detector);
  tracker_params.descriptor = static_cast<FeatureTracker::DescriptorExtractorEnum>(extractor);
  tracker_params.matcher = static_cast<FeatureTracker::DescriptorMatcherEnum>(matcher);
  tracker_params.threshold =
    this->get_parameter(tracker_prefix + ".detector_threshold").as_double();
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
  tParams.sensor_id = camera_ptr->GetId();
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


void EkfCalNode::PublishState()
{
  // Body State
  Eigen::VectorXd body_state_vector = m_ekf->GetBodyState().ToVector();
  auto body_state_vec_msg = std_msgs::msg::Float64MultiArray();

  for (auto & element : body_state_vector) {
    body_state_vec_msg.data.push_back(element);
  }
  m_body_state_pub->publish(body_state_vec_msg);

  // IMU States
  Eigen::VectorXd imu_state_vector = m_ekf->GetImuState(1).ToVector();
  auto imu_state_vec_msg = std_msgs::msg::Float64MultiArray();

  for (auto & element : imu_state_vector) {
    imu_state_vec_msg.data.push_back(element);
  }
  m_imu_state_pub->publish(imu_state_vec_msg);

  std::stringstream msg;
  Eigen::VectorXd state_vector = m_ekf->GetState().ToVector();
  msg << VectorToCommaString(state_vector) << std::endl;
  m_state_data_logger.Log(msg.str());
}
