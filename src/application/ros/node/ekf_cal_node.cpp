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
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "infrastructure/ekf_cal_version.hpp"
#include "sensors/camera.hpp"
#include "sensors/gps.hpp"
#include "sensors/imu.hpp"
#include "sensors/ros/ros_camera_message.hpp"
#include "sensors/ros/ros_camera.hpp"
#include "sensors/ros/ros_gps_message.hpp"
#include "sensors/ros/ros_gps.hpp"
#include "sensors/ros/ros_imu_message.hpp"
#include "sensors/ros/ros_imu.hpp"
#include "utility/string_helper.hpp"
#include "utility/type_helper.hpp"

using std::placeholders::_1;

EkfCalNode::EkfCalNode()
: Node("EkfCalNode")
{
  // Declare EKF Parameters
  declare_parameter("log_directory", "~/log/");
  declare_parameter("debug_log_level", 0);
  declare_parameter("data_log_rate", 0.0);
  declare_parameter("augmenting_type", 0);
  declare_parameter("augmenting_delta_time", 0.0);
  declare_parameter("augmenting_pos_error", 0.0);
  declare_parameter("augmenting_ang_error", 0.0);
  declare_parameter("process_noise.pos", 1.0e-2);
  declare_parameter("process_noise.vel", 1.0e-2);
  declare_parameter("process_noise.ang_pos", 1.0e-2);
  declare_parameter("pos_b_in_l", std::vector<double>{0, 0, 0});
  declare_parameter("ang_b_to_l", std::vector<double>{0, 0, 0, 0});
  declare_parameter("pos_e_in_g", std::vector<double>{0, 0, 0});
  declare_parameter("ang_l_to_e", 0.0);
  declare_parameter("gps_init_type", 0);
  declare_parameter("gps_init_baseline_dist", 1.0);
  declare_parameter("gps_init_pos_thresh", 1.0);
  declare_parameter("gps_init_ang_thresh", 1.0);
  declare_parameter("motion_detection_chi_squared", 1.0);
  declare_parameter("imu_noise_scale_factor", 100.0);
  declare_parameter("use_root_covariance", true);
  declare_parameter("use_first_estimate_jacobian", false);

  // Declare Sensor Lists
  declare_parameter("imu_list", std::vector<std::string>{});
  declare_parameter("camera_list", std::vector<std::string>{});
  declare_parameter("tracker_list", std::vector<std::string>{});
  declare_parameter("fiducial_list", std::vector<std::string>{});
  declare_parameter("gps_list", std::vector<std::string>{});

  m_state_pub_timer =
    create_wall_timer(std::chrono::seconds(1), std::bind(&EkfCalNode::PublishState, this));
}

void EkfCalNode::Initialize()
{
  // Set logging
  auto debug_log_level = static_cast<unsigned int>(get_parameter("debug_log_level").as_int());
  double data_log_rate = get_parameter("data_log_rate").as_double();
  m_log_directory = get_parameter("log_directory").as_string();

  m_debug_logger = std::make_shared<DebugLogger>(debug_log_level, m_log_directory);
  if (data_log_rate) {m_state_data_logger.EnableLogging();}
  m_state_data_logger.SetOutputDirectory(m_log_directory);
  m_state_data_logger.SetOutputFileName("state_vector.csv");
  m_state_data_logger.DefineHeader("");
  m_debug_logger->Log(LogLevel::INFO, "EKF CAL Version: " + std::string(EKF_CAL_VERSION));
  m_debug_logger->Log(LogLevel::INFO, "Log Directory: " + m_log_directory);
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = m_debug_logger;
  ekf_params.data_log_rate = data_log_rate;
  ekf_params.log_directory = m_log_directory;
  ekf_params.augmenting_type =
    static_cast<AugmentationType>(get_parameter("augmenting_type").as_int());
  ekf_params.augmenting_delta_time = get_parameter("augmenting_delta_time").as_double();
  ekf_params.augmenting_pos_error = get_parameter("augmenting_pos_error").as_double();
  ekf_params.augmenting_ang_error = get_parameter("augmenting_ang_error").as_double();
  ekf_params.process_noise = LoadProcessNoise();
  ekf_params.pos_b_in_l = StdToEigVec(get_parameter("pos_b_in_l").as_double_array());
  ekf_params.ang_b_to_l = StdToEigQuat(get_parameter("ang_b_to_l").as_double_array());
  ekf_params.pos_e_in_g = StdToEigVec(get_parameter("pos_e_in_g").as_double_array());
  ekf_params.ang_l_to_e = get_parameter("ang_l_to_e").as_double();
  ekf_params.gps_init_type = static_cast<GpsInitType>(get_parameter("gps_init_type").as_int());
  ekf_params.gps_init_baseline_dist = get_parameter("gps_init_baseline_dist").as_double();
  ekf_params.gps_init_pos_thresh = get_parameter("gps_init_pos_thresh").as_double();
  ekf_params.gps_init_ang_thresh = get_parameter("gps_init_ang_thresh").as_double();
  ekf_params.motion_detection_chi_squared =
    get_parameter("motion_detection_chi_squared").as_double();
  ekf_params.use_root_covariance = get_parameter("use_root_covariance").as_bool();
  ekf_params.use_first_estimate_jacobian =
    get_parameter("use_first_estimate_jacobian").as_bool();
  ekf_params.imu_noise_scale_factor = get_parameter("imu_noise_scale_factor").as_double();

  // Load lists of sensors
  m_imu_list = get_parameter("imu_list").as_string_array();
  m_camera_list = get_parameter("camera_list").as_string_array();
  m_tracker_list = get_parameter("tracker_list").as_string_array();
  m_fiducial_list = get_parameter("fiducial_list").as_string_array();
  m_gps_list = get_parameter("gps_list").as_string_array();

  if (m_tracker_list.size() == 0) {
    ekf_params.augmenting_type = AugmentationType::NONE;
  }

  m_ekf = std::make_shared<EKF>(ekf_params);
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
  for (std::string & fiducial_name : m_fiducial_list) {
    DeclareFiducialParameters(fiducial_name);
  }
  for (std::string & gps_name : m_gps_list) {
    DeclareGpsParameters(gps_name);
  }
}

void EkfCalNode::DeclareSensorParameters(const std::string & prefix)
{
  declare_parameter(prefix + ".topic", "");
  declare_parameter(prefix + ".rate", 1.0);
  declare_parameter(prefix + ".data_log_rate", 0.0);
}

void EkfCalNode::LoadSensorParameters(
  Sensor::Parameters & params,
  const std::string & prefix,
  const std::string & name
)
{
  params.topic = get_parameter(prefix + ".topic").as_string();
  params.rate = get_parameter(prefix + ".rate").as_double();
  params.data_log_rate = get_parameter(prefix + ".data_log_rate").as_double();
  params.name = name;
  params.log_directory = m_log_directory;
  params.ekf = m_ekf;
  params.logger = m_debug_logger;
}

void EkfCalNode::LoadSensors()
{
  // Load IMU sensor parameters
  for (std::string & imu_name : m_imu_list) {
    LoadImu(imu_name);
  }

  // Load Camera sensor parameters
  for (std::string & cam_name : m_camera_list) {
    LoadCamera(cam_name);
  }

  // Load Camera sensor parameters
  for (std::string & gps_name : m_gps_list) {
    LoadGps(gps_name);
  }

  // Create publishers
  m_body_state_pub = create_publisher<std_msgs::msg::Float64MultiArray>("~/BodyState", 10);
  m_imu_state_pub = create_publisher<std_msgs::msg::Float64MultiArray>("~/ImuState", 10);
}

void EkfCalNode::DeclareImuParameters(const std::string & imu_name)
{
  // Declare parameters
  std::string imu_prefix = "imu." + imu_name;
  DeclareSensorParameters(imu_prefix);
  declare_parameter(imu_prefix + ".is_extrinsic", false);
  declare_parameter(imu_prefix + ".is_intrinsic", false);
  declare_parameter(
    imu_prefix + ".variance", std::vector<double>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1});
  declare_parameter(imu_prefix + ".pos_i_in_b", std::vector<double>{0, 0, 0});
  declare_parameter(imu_prefix + ".ang_i_to_b", std::vector<double>{1, 0, 0, 0});
  declare_parameter(imu_prefix + ".acc_bias", std::vector<double>{0, 0, 0});
  declare_parameter(imu_prefix + ".omg_bias", std::vector<double>{0, 0, 0});
  declare_parameter(imu_prefix + ".pos_stability", 1e-9);
  declare_parameter(imu_prefix + ".ang_stability", 1e-9);
  declare_parameter(imu_prefix + ".acc_bias_stability", 1e-9);
  declare_parameter(imu_prefix + ".omg_bias_stability", 1e-9);
}

IMU::Parameters EkfCalNode::GetImuParameters(const std::string & imu_name)
{
  // Load parameters
  std::string imu_prefix = "imu." + imu_name;
  bool is_extrinsic = get_parameter(imu_prefix + ".is_extrinsic").as_bool();
  bool is_intrinsic = get_parameter(imu_prefix + ".is_intrinsic").as_bool();
  std::vector<double> variance = get_parameter(imu_prefix + ".variance").as_double_array();
  std::vector<double> pos_i_in_b = get_parameter(imu_prefix + ".pos_i_in_b").as_double_array();
  std::vector<double> ang_i_to_b = get_parameter(imu_prefix + ".ang_i_to_b").as_double_array();
  std::vector<double> acc_bias = get_parameter(imu_prefix + ".acc_bias").as_double_array();
  std::vector<double> omg_bias = get_parameter(imu_prefix + ".omg_bias").as_double_array();
  double pos_stability = get_parameter(imu_prefix + ".pos_stability").as_double();
  double ang_stability = get_parameter(imu_prefix + ".ang_stability").as_double();
  double acc_bias_stability = get_parameter(imu_prefix + ".acc_bias_stability").as_double();
  double omg_bias_stability = get_parameter(imu_prefix + ".omg_bias_stability").as_double();

  // Assign parameters to struct
  IMU::Parameters imu_params;
  LoadSensorParameters(imu_params, imu_prefix, imu_name);
  imu_params.is_extrinsic = is_extrinsic;
  imu_params.is_intrinsic = is_intrinsic;
  imu_params.variance = StdToEigVec(variance);
  imu_params.pos_i_in_b = StdToEigVec(pos_i_in_b);
  imu_params.ang_i_to_b = StdToEigQuat(ang_i_to_b);
  imu_params.acc_bias = StdToEigVec(acc_bias);
  imu_params.omg_bias = StdToEigVec(omg_bias);
  imu_params.pos_stability = pos_stability;
  imu_params.ang_stability = ang_stability;
  imu_params.acc_bias_stability = acc_bias_stability;
  imu_params.omg_bias_stability = omg_bias_stability;
  return imu_params;
}

void EkfCalNode::DeclareIntrinsicParameters(const std::string & intrinsics_prefix)
{
  declare_parameter(intrinsics_prefix + ".f_x", 0.01);
  declare_parameter(intrinsics_prefix + ".f_y", 0.01);
  declare_parameter(intrinsics_prefix + ".k_1", 0.0);
  declare_parameter(intrinsics_prefix + ".k_2", 0.0);
  declare_parameter(intrinsics_prefix + ".p_1", 0.0);
  declare_parameter(intrinsics_prefix + ".p_2", 0.0);
  declare_parameter(intrinsics_prefix + ".pixel_size", 5.0e-6);
  declare_parameter(intrinsics_prefix + ".width", 640.0);
  declare_parameter(intrinsics_prefix + ".height", 480.0);
}

Intrinsics EkfCalNode::GetIntrinsicParameters(const std::string & intrinsics_prefix)
{
  Intrinsics intrinsics;

  intrinsics.f_x = get_parameter(intrinsics_prefix + ".f_x").as_double();
  intrinsics.f_y = get_parameter(intrinsics_prefix + ".f_y").as_double();
  intrinsics.k_1 = get_parameter(intrinsics_prefix + ".k_1").as_double();
  intrinsics.k_2 = get_parameter(intrinsics_prefix + ".k_2").as_double();
  intrinsics.p_1 = get_parameter(intrinsics_prefix + ".p_1").as_double();
  intrinsics.p_2 = get_parameter(intrinsics_prefix + ".p_2").as_double();
  intrinsics.pixel_size = get_parameter(intrinsics_prefix + ".pixel_size").as_double();
  intrinsics.width = get_parameter(intrinsics_prefix + ".width").as_double();
  intrinsics.height = get_parameter(intrinsics_prefix + ".height").as_double();

  return intrinsics;
}

void EkfCalNode::DeclareCameraParameters(const std::string & camera_name)
{
  // Declare parameters
  std::string cam_prefix = "camera." + camera_name;
  DeclareSensorParameters(cam_prefix);
  declare_parameter(cam_prefix + ".pos_c_in_b", std::vector<double>{0, 0, 0});
  declare_parameter(cam_prefix + ".ang_c_to_b", std::vector<double>{1, 0, 0, 0});
  declare_parameter(cam_prefix + ".variance", std::vector<double>{1, 1, 1, 1, 1, 1});
  declare_parameter(cam_prefix + ".tracker", "");
  declare_parameter(cam_prefix + ".fiducial", "");
  declare_parameter(cam_prefix + ".pos_stability", 1e-9);
  declare_parameter(cam_prefix + ".ang_stability", 1e-9);
  declare_parameter(cam_prefix + ".is_extrinsic", false);
  DeclareIntrinsicParameters(cam_prefix + ".intrinsics");
}

Camera::Parameters EkfCalNode::GetCameraParameters(const std::string & camera_name)
{
  // Load parameters
  std::string cam_prefix = "camera." + camera_name;
  std::vector<double> pos_c_in_b = get_parameter(cam_prefix + ".pos_c_in_b").as_double_array();
  std::vector<double> ang_c_to_b = get_parameter(cam_prefix + ".ang_c_to_b").as_double_array();
  std::vector<double> variance = get_parameter(cam_prefix + ".variance").as_double_array();
  std::string tracker_name = get_parameter(cam_prefix + ".tracker").as_string();
  std::string fiducial_name = get_parameter(cam_prefix + ".fiducial").as_string();
  double pos_stability = get_parameter(cam_prefix + ".pos_stability").as_double();
  double ang_stability = get_parameter(cam_prefix + ".ang_stability").as_double();
  bool is_extrinsic = get_parameter(cam_prefix + ".is_extrinsic").as_bool();

  // Assign parameters to struct
  Camera::Parameters camera_params;
  LoadSensorParameters(camera_params, cam_prefix, camera_name);
  camera_params.pos_c_in_b = StdToEigVec(pos_c_in_b);
  camera_params.ang_c_to_b = StdToEigQuat(ang_c_to_b);
  camera_params.variance = StdToEigVec(variance);
  camera_params.tracker = tracker_name;
  camera_params.fiducial = fiducial_name;
  camera_params.pos_stability = pos_stability;
  camera_params.ang_stability = ang_stability;
  camera_params.is_extrinsic = is_extrinsic;
  camera_params.intrinsics = GetIntrinsicParameters(cam_prefix + ".intrinsics");
  return camera_params;
}

void EkfCalNode::DeclareTrackerParameters(const std::string & tracker_name)
{
  // Declare parameters
  std::string tracker_prefix = "tracker." + tracker_name;
  declare_parameter(tracker_prefix + ".feature_detector", 0);
  declare_parameter(tracker_prefix + ".descriptor_extractor", 0);
  declare_parameter(tracker_prefix + ".descriptor_matcher", 0);
  declare_parameter(tracker_prefix + ".detector_threshold", 20.0);
  declare_parameter(tracker_prefix + ".pixel_error", 1.0);
  declare_parameter(tracker_prefix + ".down_sample_height", 480.0);
  declare_parameter(tracker_prefix + ".down_sample_width", 640.0);
  declare_parameter(tracker_prefix + ".min_feature_distance", 1.0);
  declare_parameter(tracker_prefix + ".min_track_length", 2);
  declare_parameter(tracker_prefix + ".max_track_length", 20);
}

FeatureTracker::Parameters EkfCalNode::GetTrackerParameters(const std::string & tracker_name)
{
  // Get parameters
  std::string tracker_prefix = "tracker." + tracker_name;
  int detector = get_parameter(tracker_prefix + ".feature_detector").as_int();
  int extractor = get_parameter(tracker_prefix + ".descriptor_extractor").as_int();
  int matcher = get_parameter(tracker_prefix + ".descriptor_matcher").as_int();
  double threshold = get_parameter(tracker_prefix + ".detector_threshold").as_double();
  double px_error = get_parameter(tracker_prefix + ".pixel_error").as_double();
  double down_height = get_parameter(tracker_prefix + ".down_sample_height").as_double();
  double down_width = get_parameter(tracker_prefix + ".down_sample_width").as_double();
  double min_feat_dist = get_parameter(tracker_prefix + ".min_feature_distance").as_double();
  int min_track_length = get_parameter(tracker_prefix + ".min_track_length").as_int();
  int max_track_length = get_parameter(tracker_prefix + ".max_track_length").as_int();

  FeatureTracker::Parameters tracker_params;
  tracker_params.detector = static_cast<Detector>(detector);
  tracker_params.descriptor = static_cast<Descriptor>(extractor);
  tracker_params.matcher = static_cast<Matcher>(matcher);
  tracker_params.threshold = threshold;
  tracker_params.px_error = px_error;
  tracker_params.down_sample_height = down_height;
  tracker_params.down_sample_width = down_width;
  tracker_params.min_feat_dist = min_feat_dist;
  tracker_params.min_track_length = min_track_length;
  tracker_params.max_track_length = max_track_length;
  tracker_params.ekf = m_ekf;
  tracker_params.logger = m_debug_logger;
  tracker_params.log_directory = m_log_directory;
  return tracker_params;
}

void EkfCalNode::DeclareFiducialParameters(const std::string & fid_name)
{
  // Declare parameters
  std::string fiducial_prefix = "fiducial." + fid_name;
  declare_parameter(fiducial_prefix + ".fiducial_type", 0);
  declare_parameter(fiducial_prefix + ".squares_x", 0);
  declare_parameter(fiducial_prefix + ".squares_y", 0);
  declare_parameter(fiducial_prefix + ".square_length", 0.0);
  declare_parameter(fiducial_prefix + ".marker_length", 0.0);
  declare_parameter(fiducial_prefix + ".id", 0);
  declare_parameter(fiducial_prefix + ".pos_f_in_l", std::vector<double>{0, 0, 0});
  declare_parameter(fiducial_prefix + ".ang_f_to_l", std::vector<double>{1, 0, 0, 0});
  declare_parameter(fiducial_prefix + ".variance", std::vector<double>{0, 0, 0, 0, 0, 0});
  declare_parameter(fiducial_prefix + ".min_track_length", 2);
  declare_parameter(fiducial_prefix + ".max_track_length", 20);
  declare_parameter(fiducial_prefix + ".is_extrinsic", false);
}

FiducialTracker::Parameters EkfCalNode::GetFiducialParameters(const std::string & fiducial_name)
{
  // Get parameters
  std::string fiducial_prefix = "fiducial." + fiducial_name;
  auto fiducial_type = get_parameter(fiducial_prefix + ".fiducial_type").as_int();
  auto squares_x = get_parameter(fiducial_prefix + ".squares_x").as_int();
  auto squares_y = get_parameter(fiducial_prefix + ".squares_y").as_int();
  auto square_length = get_parameter(fiducial_prefix + ".square_length").as_double();
  auto marker_length = get_parameter(fiducial_prefix + ".marker_length").as_double();
  auto id = get_parameter(fiducial_prefix + ".id").as_int();
  auto pos_f_in_l = get_parameter(fiducial_prefix + ".pos_f_in_l").as_double_array();
  auto ang_f_to_l = get_parameter(fiducial_prefix + ".ang_f_to_l").as_double_array();
  auto variance = get_parameter(fiducial_prefix + ".variance").as_double_array();
  auto min_track_length = get_parameter(fiducial_prefix + ".min_track_length").as_int();
  auto max_track_length = get_parameter(fiducial_prefix + ".max_track_length").as_int();
  auto is_extrinsic = get_parameter(fiducial_prefix + ".is_extrinsic").as_bool();

  FiducialTracker::Parameters fiducial_params;
  fiducial_params.detector_type = static_cast<FiducialType>(fiducial_type);
  fiducial_params.squares_x = squares_x;
  fiducial_params.squares_y = squares_y;
  fiducial_params.square_length = square_length;
  fiducial_params.marker_length = marker_length;
  fiducial_params.id = static_cast<unsigned int>(id);
  fiducial_params.pos_f_in_l = StdToEigVec(pos_f_in_l);
  fiducial_params.ang_f_to_l = StdToEigQuat(ang_f_to_l);
  fiducial_params.variance = StdToEigVec(variance);
  fiducial_params.min_track_length = min_track_length;
  fiducial_params.max_track_length = max_track_length;
  fiducial_params.is_extrinsic = is_extrinsic;
  fiducial_params.ekf = m_ekf;
  fiducial_params.logger = m_debug_logger;
  fiducial_params.log_directory = m_log_directory;
  return fiducial_params;
}

void EkfCalNode::DeclareGpsParameters(const std::string & gps_name)
{
  // Declare parameters
  std::string gps_prefix = "gps." + gps_name;
  DeclareSensorParameters(gps_prefix);
  declare_parameter(gps_prefix + ".pos_a_in_b", std::vector<double>{0, 0, 0});
  declare_parameter(gps_prefix + ".pos_stability", 1e-9);
  declare_parameter(gps_prefix + ".variance", std::vector<double>{1, 1, 1});
}

GPS::Parameters EkfCalNode::GetGpsParameters(const std::string & gps_name)
{
  // Get parameters
  std::string gps_prefix = "gps." + gps_name;
  std::vector<double> pos_a_in_b = get_parameter(gps_prefix + ".pos_a_in_b").as_double_array();
  std::vector<double> variance = get_parameter(gps_prefix + ".variance").as_double_array();
  double pos_stability = get_parameter(gps_prefix + ".pos_stability").as_double();

  GPS::Parameters gps_params;
  LoadSensorParameters(gps_params, gps_prefix, gps_name);
  gps_params.pos_a_in_b = StdToEigVec(pos_a_in_b);
  gps_params.variance = StdToEigVec(variance);
  gps_params.pos_stability = pos_stability;
  return gps_params;
}

void EkfCalNode::LoadImu(const std::string & imu_name)
{
  IMU::Parameters imu_params = GetImuParameters(imu_name);
  m_debug_logger->Log(LogLevel::INFO, "Loaded IMU: " + imu_name);

  // Create new RosIMU and and bind callback to ID
  std::shared_ptr<RosIMU> imu_ptr = std::make_shared<RosIMU>(imu_params);

  RegisterImu(imu_ptr, imu_params.topic);
}

void EkfCalNode::RegisterImu(std::shared_ptr<RosIMU> imu_ptr, const std::string & topic)
{
  m_map_imu[imu_ptr->GetId()] = imu_ptr;

  std::function<void(std::shared_ptr<sensor_msgs::msg::Imu>)> function;
  function = std::bind(&EkfCalNode::ImuCallback, this, _1, imu_ptr->GetId());

  auto sub = create_subscription<sensor_msgs::msg::Imu>(topic, 10, function);
  m_imu_subs.push_back(sub);

  std::stringstream log_msg;
  log_msg << "Registered IMU " << imu_ptr->GetId() << ": " << imu_ptr->GetName();
  m_debug_logger->Log(LogLevel::INFO, log_msg.str());
}

void EkfCalNode::LoadCamera(const std::string & camera_name)
{
  // Load camera parameters
  Camera::Parameters camera_params = GetCameraParameters(camera_name);
  std::shared_ptr<RosCamera> camera_ptr = std::make_shared<RosCamera>(camera_params);
  m_debug_logger->Log(LogLevel::INFO, "Loaded Camera: " + camera_name);

  if (!camera_params.tracker.empty()) {
    FeatureTracker::Parameters trk_params = GetTrackerParameters(camera_params.tracker);
    trk_params.camera_id = camera_ptr->GetId();
    trk_params.data_log_rate = camera_params.data_log_rate;
    std::shared_ptr<FeatureTracker> trk_ptr = std::make_shared<FeatureTracker>(trk_params);
    camera_ptr->AddTracker(trk_ptr);
  }

  if (!camera_params.fiducial.empty()) {
    FiducialTracker::Parameters fid_params = GetFiducialParameters(camera_params.fiducial);
    fid_params.camera_id = camera_ptr->GetId();
    fid_params.data_log_rate = camera_params.data_log_rate;
    std::shared_ptr<FiducialTracker> fid_ptr = std::make_shared<FiducialTracker>(fid_params);
    camera_ptr->AddFiducial(fid_ptr);
  }

  // Create new RosCamera and bind callback to ID
  RegisterCamera(camera_ptr, camera_params.topic);

  // Create image re-publisher
  m_map_image_publishers[camera_ptr->GetId()] =
    create_publisher<sensor_msgs::msg::Image>("~/" + camera_name, 10);
}

void EkfCalNode::RegisterCamera(std::shared_ptr<RosCamera> camera_ptr, const std::string & topic)
{
  m_map_camera[camera_ptr->GetId()] = camera_ptr;

  std::function<void(std::shared_ptr<sensor_msgs::msg::Image>)> function;
  function = std::bind(&EkfCalNode::CameraCallback, this, _1, camera_ptr->GetId());
  auto sub = create_subscription<sensor_msgs::msg::Image>(topic, 10, function);
  m_camera_subs.push_back(sub);

  std::stringstream log_msg;
  log_msg << "Registered Camera " << camera_ptr->GetId() << ": " << camera_ptr->GetName();
  m_debug_logger->Log(LogLevel::INFO, log_msg.str());
}

void EkfCalNode::LoadGps(const std::string & gps_name)
{
  GPS::Parameters gps_params = GetGpsParameters(gps_name);
  m_debug_logger->Log(LogLevel::INFO, "Loaded GPS: " + gps_name);

  // Create new RosGPS and and bind callback to ID
  std::shared_ptr<RosGPS> gps_ptr = std::make_shared<RosGPS>(gps_params);

  RegisterGps(gps_ptr, gps_params.topic);
}

void EkfCalNode::RegisterGps(std::shared_ptr<RosGPS> gps_ptr, const std::string & topic)
{
  m_map_gps[gps_ptr->GetId()] = gps_ptr;

  std::function<void(std::shared_ptr<sensor_msgs::msg::NavSatFix>)> function;
  function = std::bind(&EkfCalNode::GpsCallback, this, _1, gps_ptr->GetId());
  auto sub = create_subscription<sensor_msgs::msg::NavSatFix>(topic, 10, function);
  m_gps_subs.push_back(sub);

  std::stringstream log_msg;
  log_msg << "Registered GPS " << gps_ptr->GetId() << ": " << gps_ptr->GetName();
  m_debug_logger->Log(LogLevel::INFO, log_msg.str());
}

void EkfCalNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id)
{
  auto ros_imu_iter = m_map_imu.find(id);
  if (ros_imu_iter != m_map_imu.end()) {
    auto ros_imu_message = std::make_shared<RosImuMessage>(msg);
    ros_imu_message->sensor_id = id;
    ros_imu_iter->second->Callback(ros_imu_message);
  } else {
    m_debug_logger->Log(LogLevel::WARN, "IMU ID Not Found: " + std::to_string(id));
  }
}

void EkfCalNode::CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg, unsigned int id)
{
  auto ros_cam_iter = m_map_camera.find(id);
  if (ros_cam_iter != m_map_camera.end()) {
    auto ros_camera_message = std::make_shared<RosCameraMessage>(msg);
    ros_camera_message->sensor_id = id;
    ros_cam_iter->second->Callback(ros_camera_message);
    m_map_image_publishers[id]->publish(*ros_cam_iter->second->GetRosImage().get());
  } else {
    m_debug_logger->Log(LogLevel::WARN, "Camera ID Not Found: " + std::to_string(id));
  }
}

void EkfCalNode::GpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg, unsigned int id)
{
  auto ros_gps_iter = m_map_gps.find(id);
  if (ros_gps_iter != m_map_gps.end()) {
    auto ros_gps_message = std::make_shared<RosGpsMessage>(msg);
    ros_gps_message->sensor_id = id;
    ros_gps_iter->second->Callback(ros_gps_message);
  } else {
    m_debug_logger->Log(LogLevel::WARN, "GPS ID Not Found: " + std::to_string(id));
  }
}

void EkfCalNode::PublishState()
{
  /// @todo Copy states before converting to vectors due to race conditions.
  // Body State
  Eigen::VectorXd body_state_vector = m_ekf->m_state.body_state.ToVector();
  auto body_state_vec_msg = std_msgs::msg::Float64MultiArray();

  for (auto & element : body_state_vector) {
    body_state_vec_msg.data.push_back(element);
  }
  m_body_state_pub->publish(body_state_vec_msg);

  // // IMU States
  // Eigen::VectorXd imu_state_vector = m_ekf->GetImuState(1).ToVector();
  // auto imu_state_vec_msg = std_msgs::msg::Float64MultiArray();

  // for (auto & element : imu_state_vector) {
  //   imu_state_vec_msg.data.push_back(element);
  // }
  // m_imu_state_pub->publish(imu_state_vec_msg);

  // std::stringstream msg;
  // Eigen::VectorXd state_vector = m_ekf->m_state.ToVector();
  // msg << VectorToCommaString(state_vector);
  // m_state_data_logger.Log(msg.str());
}

Eigen::VectorXd EkfCalNode::LoadProcessNoise()
{
  double pos_noise = get_parameter("process_noise.pos").as_double();
  double vel_noise = get_parameter("process_noise.vel").as_double();
  double ang_pos_noise = get_parameter("process_noise.ang_pos").as_double();

  Eigen::VectorXd process_noise(g_body_state_size);
  process_noise.segment<3>(0) = Eigen::Vector3d::Ones() * pos_noise;
  process_noise.segment<3>(3) = Eigen::Vector3d::Ones() * vel_noise;
  process_noise.segment<3>(6) = Eigen::Vector3d::Ones() * ang_pos_noise;

  return process_noise;
}
