// Copyright 2023 Jacob Hartzer
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

#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "infrastructure/data_logger.hpp"
#include "infrastructure/debug_logger.hpp"
#include "infrastructure/ekf_cal_version.hpp"
#include "infrastructure/sim/truth_engine_cyclic.hpp"
#include "infrastructure/sim/truth_engine_smoother.hpp"
#include "infrastructure/sim/truth_engine_spline.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/camera.hpp"
#include "sensors/imu.hpp"
#include "sensors/sensor_message.hpp"
#include "sensors/sensor.hpp"
#include "sensors/sim/sim_camera_message.hpp"
#include "sensors/sim/sim_camera.hpp"
#include "sensors/sim/sim_gps_message.hpp"
#include "sensors/sim/sim_gps.hpp"
#include "sensors/sim/sim_imu_message.hpp"
#include "sensors/sim/sim_imu.hpp"
#include "trackers/feature_tracker.hpp"
#include "trackers/sim/sim_feature_tracker.hpp"
#include "trackers/sim/sim_fiducial_tracker.hpp"
#include "utility/gps_helper.hpp"
#include "utility/sim/sim_rng.hpp"
#include "utility/string_helper.hpp"
#include "utility/type_helper.hpp"


std::vector<std::string> LoadNodeList(YAML::Node node)
{
  std::vector<std::string> string_list;
  for (unsigned int i = 0; i < node.size(); ++i) {
    string_list.push_back(node[i].as<std::string>());
  }
  return string_list;
}

void LoadSensorParams(
  Sensor::Parameters & params,
  YAML::Node node,
  const std::string & name,
  const std::string & log_directory,
  std::shared_ptr<EKF> ekf,
  std::shared_ptr<DebugLogger> debug_logger
)
{
  params.topic = node["topic"].as<std::string>("");
  params.rate = node["rate"].as<double>(1.0);
  params.data_log_rate = node["data_log_rate"].as<double>(0.0);
  params.name = name;
  params.log_directory = log_directory;
  params.ekf = ekf;
  params.logger = debug_logger;
}

void LoadSimSensorParams(
  SimSensor::Parameters & params,
  YAML::Node node)
{
  params.no_errors = node["no_errors"].as<bool>(false);
  params.time_error = node["time_error"].as<double>(0.0);
}

void LoadTrackerParams(
  Tracker::Parameters & params,
  const std::string & name,
  const std::string & log_directory,
  std::shared_ptr<EKF> ekf,
  std::shared_ptr<DebugLogger> debug_logger
)
{
  params.name = name;
  params.log_directory = log_directory;
  params.ekf = ekf;
  params.logger = debug_logger;
}

Eigen::VectorXd LoadProcessNoise(YAML::Node process_noise_node)
{
  double pos_noise = process_noise_node["pos"].as<double>(1.0e-2);
  double vel_noise = process_noise_node["vel"].as<double>(1.0e-2);
  double acc_noise = process_noise_node["acc"].as<double>(1.0e-2);
  double ang_pos_noise = process_noise_node["ang_pos"].as<double>(1.0e-2);
  double ang_vel_noise = process_noise_node["ang_vel"].as<double>(1.0e-2);
  double ang_acc_noise = process_noise_node["ang_acc"].as<double>(1.0e-2);

  Eigen::VectorXd process_noise(g_body_state_size);
  process_noise.segment<3>(0) = Eigen::Vector3d::Ones() * pos_noise;
  process_noise.segment<3>(3) = Eigen::Vector3d::Ones() * vel_noise;
  process_noise.segment<3>(6) = Eigen::Vector3d::Ones() * acc_noise;
  process_noise.segment<3>(9) = Eigen::Vector3d::Ones() * ang_pos_noise;
  process_noise.segment<3>(12) = Eigen::Vector3d::Ones() * ang_vel_noise;
  process_noise.segment<3>(15) = Eigen::Vector3d::Ones() * ang_acc_noise;

  return process_noise;
}

int main(int argc, char * argv[])
{
  const cv::String keys =
    "{@config        | <none> | Input YAML configuration file }"
    "{@log_directory | ~/log/ | Output directory for logs     }"
    "{help h usage ? |        | print this help message       }"
  ;

  cv::CommandLineParser parser(argc, argv, keys);
  parser.about("ekf_cal Simulation: " + std::string(EKF_CAL_VERSION));
  if (parser.has("help") || (!parser.has("@config") || !parser.has("@log_directory"))) {
    parser.printMessage();
    return 0;
  }

  std::string config = parser.get<std::string>("@config");
  std::string log_directory = parser.get<std::string>("@log_directory");

  // Define sensors to use (load config from yaml)
  YAML::Node root = YAML::LoadFile(config);
  auto imus = LoadNodeList(root["/EkfCalNode"]["ros__parameters"]["imu_list"]);
  auto cameras = LoadNodeList(root["/EkfCalNode"]["ros__parameters"]["camera_list"]);
  auto trackers = LoadNodeList(root["/EkfCalNode"]["ros__parameters"]["tracker_list"]);
  auto fiducials = LoadNodeList(root["/EkfCalNode"]["ros__parameters"]["fiducial_list"]);
  auto gps_list = LoadNodeList(root["/EkfCalNode"]["ros__parameters"]["gps_list"]);

  // Construct sensors and EKF
  std::map<unsigned int, std::shared_ptr<Sensor>> sensor_map;
  std::vector<std::shared_ptr<SensorMessage>> messages;

  // Define default values
  std::vector<double> def_vec{0.0, 0.0, 0.0};
  std::vector<double> def_quat{1.0, 0.0, 0.0, 0.0};

  // Logging parameters
  EKF::Parameters ekf_params;
  YAML::Node ros_params = root["/EkfCalNode"]["ros__parameters"];
  unsigned int debug_log_level = ros_params["debug_log_level"].as<unsigned int>(0);
  auto debug_logger = std::make_shared<DebugLogger>(debug_log_level, log_directory);
  debug_logger->Log(LogLevel::INFO, "EKF CAL Version: " + std::string(EKF_CAL_VERSION));

  // EKF parameters
  double data_log_rate = ros_params["data_log_rate"].as<double>(0.0);
  ekf_params.debug_logger = debug_logger;
  ekf_params.data_log_rate = data_log_rate;
  ekf_params.log_directory = log_directory;
  ekf_params.augmenting_type =
    static_cast<AugmentationType>(ros_params["augmenting_type"].as<unsigned int>(1));
  ekf_params.augmenting_delta_time = ros_params["augmenting_delta_time"].as<double>(1.0);
  ekf_params.augmenting_pos_error = ros_params["augmenting_pos_error"].as<double>(0.1);
  ekf_params.augmenting_ang_error = ros_params["augmenting_ang_error"].as<double>(0.1);
  ekf_params.process_noise = LoadProcessNoise(ros_params["process_noise"]);
  ekf_params.pos_b_in_l = StdToEigVec(ros_params["pos_b_in_l"].as<std::vector<double>>(def_vec));
  ekf_params.ang_b_to_l = StdToEigQuat(ros_params["ang_b_to_l"].as<std::vector<double>>(def_quat));
  ekf_params.pos_e_in_g = StdToEigVec(ros_params["pos_e_in_g"].as<std::vector<double>>(def_vec));
  ekf_params.ang_l_to_e = ros_params["ang_l_to_e"].as<double>(0.0);
  ekf_params.gps_init_type =
    static_cast<GpsInitType>(ros_params["gps_init_type"].as<unsigned int>(0));
  ekf_params.gps_init_baseline_dist = ros_params["gps_init_baseline_dist"].as<double>(1.0);
  ekf_params.gps_init_pos_thresh = ros_params["gps_init_pos_thresh"].as<double>(1.0);
  ekf_params.gps_init_ang_thresh = ros_params["gps_init_ang_thresh"].as<double>(1.0);
  ekf_params.motion_detection_chi_squared =
    ros_params["motion_detection_chi_squared"].as<double>(1.0);
  ekf_params.imu_noise_scale_factor = ros_params["imu_noise_scale_factor"].as<double>(100.0);
  ekf_params.use_root_covariance = ros_params["use_root_covariance"].as<bool>(true);
  ekf_params.use_first_estimate_jacobian =
    ros_params["use_first_estimate_jacobian"].as<bool>(false);
  if (trackers.empty()) {
    ekf_params.augmenting_type = AugmentationType::NONE;
  }
  auto ekf = std::make_shared<EKF>(ekf_params);

  // Simulation parameters
  /// @todo: Add overriding no_errors option
  YAML::Node sim_params = ros_params["sim_params"];
  unsigned int rng_seed = sim_params["seed"].as<unsigned int>(1);
  double max_time = sim_params["max_time"].as<double>(10.0);

  if (rng_seed > 0) {
    SimRNG::SetSeed(rng_seed);
  }

  // Truth parameters
  std::string truth_type = sim_params["truth_type"].as<std::string>("cyclic");
  double stationary_time = std::max(sim_params["stationary_time"].as<double>(0.1), 0.1);
  std::shared_ptr<TruthEngine> truth_engine;
  if (truth_type == "cyclic") {
    auto pos_frequency = StdToEigVec(sim_params["pos_frequency"].as<std::vector<double>>(def_vec));
    auto ang_frequency = StdToEigVec(sim_params["ang_frequency"].as<std::vector<double>>(def_vec));
    auto pos_offset = StdToEigVec(sim_params["pos_offset"].as<std::vector<double>>(def_vec));
    auto ang_offset = StdToEigVec(sim_params["ang_offset"].as<std::vector<double>>(def_vec));
    double pos_amplitude = sim_params["pos_amplitude"].as<double>(1.0);
    double ang_amplitude = sim_params["ang_amplitude"].as<double>(0.1);
    auto truth_engine_cyclic = std::make_shared<TruthEngineCyclic>(
      pos_frequency,
      ang_frequency,
      pos_offset,
      ang_offset,
      pos_amplitude,
      ang_amplitude,
      stationary_time,
      max_time,
      debug_logger
    );
    truth_engine = std::static_pointer_cast<TruthEngine>(truth_engine_cyclic);
  } else if (truth_type == "smoother") {
    auto times = sim_params["times"].as<std::vector<double>>(def_vec);
    auto positions = sim_params["positions"].as<std::vector<double>>(def_vec);
    auto angles = sim_params["angles"].as<std::vector<double>>(def_vec);
    auto pos_errs = sim_params["pos_errors"].as<std::vector<double>>(def_vec);
    auto ang_errs = sim_params["ang_errors"].as<std::vector<double>>(def_vec);
    auto truth_engine_spline = std::make_shared<TruthEngineSmoother>(
      times, positions, angles, pos_errs, ang_errs,
      stationary_time, max_time, 1000.0, debug_logger);
    truth_engine = std::static_pointer_cast<TruthEngine>(truth_engine_spline);
  } else if (truth_type == "spline") {
    auto positions = sim_params["positions"].as<std::vector<double>>(def_vec);
    auto angles = sim_params["angles"].as<std::vector<double>>(def_vec);
    auto pos_errs = sim_params["pos_errors"].as<std::vector<double>>(def_vec);
    auto ang_errs = sim_params["ang_errors"].as<std::vector<double>>(def_vec);
    auto truth_engine_spline = std::make_shared<TruthEngineSpline>(
      positions, angles, pos_errs, ang_errs, stationary_time, max_time, debug_logger);
    truth_engine = std::static_pointer_cast<TruthEngine>(truth_engine_spline);
  } else {
    std::stringstream msg;
    msg << "Unknown truth engine type: " << truth_type;
    debug_logger->Log(LogLevel::ERROR, msg.str());
  }

  // Local Position Error
  auto pos_b_in_l_err =
    StdToEigVec(sim_params["pos_b_in_l_error"].as<std::vector<double>>(def_vec));
  auto ang_b_to_l_err =
    StdToEigVec(sim_params["ang_b_to_l_error"].as<std::vector<double>>(def_vec));
  BodyState initial_state;
  initial_state.pos_b_in_l = SimRNG::VecNormRand(ekf_params.pos_b_in_l, pos_b_in_l_err);
  initial_state.ang_b_to_l =
    Eigen::AngleAxisd(SimRNG::NormRand(0, ang_b_to_l_err[0]), Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(SimRNG::NormRand(0, ang_b_to_l_err[1]), Eigen::Vector3d::UnitY()) *
    ekf_params.ang_b_to_l;

  ekf->Initialize(0.0, initial_state);

  // Global Position Error
  auto pos_e_in_g_err =
    StdToEigVec(sim_params["pos_e_in_g_error"].as<std::vector<double>>(def_vec));
  auto ang_l_to_e_err = sim_params["ang_l_to_e_error"].as<double>(0.0);

  Eigen::Vector3d pos_e_in_g_true;
  pos_e_in_g_true(0) = SimRNG::NormRand(
    ekf_params.pos_e_in_g(0), wgs84_m_to_deg(pos_e_in_g_err(0)));
  pos_e_in_g_true(1) = SimRNG::NormRand(
    ekf_params.pos_e_in_g(1), wgs84_m_to_deg(pos_e_in_g_err(1)));
  pos_e_in_g_true(2) = SimRNG::NormRand(ekf_params.pos_e_in_g(2), pos_e_in_g_err(2));
  double ang_l_to_e_true = SimRNG::NormRand(ekf_params.ang_l_to_e, ang_l_to_e_err);
  truth_engine->SetLocalPosition(pos_e_in_g_true);
  truth_engine->SetLocalHeading(ang_l_to_e_true);

  // Load IMUs and generate measurements
  debug_logger->Log(LogLevel::INFO, "Loading IMUs");
  for (unsigned int i = 0; i < imus.size(); ++i) {
    YAML::Node imu_node = root["/EkfCalNode"]["ros__parameters"]["imu"][imus[i]];
    YAML::Node sim_node = imu_node["sim_params"];

    IMU::Parameters imu_params;
    LoadSensorParams(imu_params, imu_node, imus[i], log_directory, ekf, debug_logger);
    imu_params.is_extrinsic = imu_node["is_extrinsic"].as<bool>(false);
    imu_params.is_intrinsic = imu_node["is_intrinsic"].as<bool>(false);
    imu_params.variance = StdToEigVec(imu_node["variance"].as<std::vector<double>>(def_vec));
    imu_params.pos_i_in_b = StdToEigVec(imu_node["pos_i_in_b"].as<std::vector<double>>(def_vec));
    imu_params.ang_i_to_b = StdToEigQuat(imu_node["ang_i_to_b"].as<std::vector<double>>(def_quat));
    imu_params.acc_bias = StdToEigVec(imu_node["acc_bias"].as<std::vector<double>>(def_vec));
    imu_params.omg_bias = StdToEigVec(imu_node["omg_bias"].as<std::vector<double>>(def_vec));
    imu_params.pos_stability = imu_node["pos_stability"].as<double>(1.0e-9);
    imu_params.ang_stability = imu_node["ang_stability"].as<double>(1.0e-9);
    imu_params.acc_bias_stability = imu_node["acc_bias_stability"].as<double>(1.0e-9);
    imu_params.omg_bias_stability = imu_node["omg_bias_stability"].as<double>(1.0e-9);

    // SimParams
    SimIMU::Parameters sim_imu_params;
    LoadSimSensorParams(sim_imu_params, sim_node);
    sim_imu_params.imu_params = imu_params;
    sim_imu_params.acc_error = StdToEigVec(sim_node["acc_error"].as<std::vector<double>>(def_vec));
    sim_imu_params.omg_error = StdToEigVec(sim_node["omg_error"].as<std::vector<double>>(def_vec));
    sim_imu_params.pos_error = StdToEigVec(sim_node["pos_error"].as<std::vector<double>>(def_vec));
    sim_imu_params.ang_error = StdToEigVec(sim_node["ang_error"].as<std::vector<double>>(def_vec));
    sim_imu_params.acc_bias_error =
      StdToEigVec(sim_node["acc_bias_error"].as<std::vector<double>>(def_vec));
    sim_imu_params.omg_bias_error =
      StdToEigVec(sim_node["omg_bias_error"].as<std::vector<double>>(def_vec));

    // Add sensor to map
    auto imu = std::make_shared<SimIMU>(sim_imu_params, truth_engine);
    sensor_map[imu->GetId()] = imu;

    // Calculate sensor measurements
    auto imu_messages = imu->GenerateMessages();
    messages.insert(messages.end(), imu_messages.begin(), imu_messages.end());
  }

  // Load tracker parameters
  unsigned int max_track_length {0};
  debug_logger->Log(LogLevel::INFO, "Loading Trackers");
  std::map<std::string, SimFeatureTracker::Parameters> tracker_map;
  for (unsigned int i = 0; i < trackers.size(); ++i) {
    YAML::Node trk_node = root["/EkfCalNode"]["ros__parameters"]["tracker"][trackers[i]];
    YAML::Node sim_node = trk_node["sim_params"];

    FeatureTracker::Parameters track_params;
    LoadTrackerParams(
      track_params, trackers[i], log_directory, ekf, debug_logger);
    track_params.px_error = trk_node["pixel_error"].as<double>(1.0);
    track_params.min_track_length = trk_node["min_track_length"].as<unsigned int>(2);
    track_params.max_track_length = trk_node["max_track_length"].as<unsigned int>(20);
    track_params.min_feat_dist = trk_node["min_feat_dist"].as<double>(1.0);
    track_params.down_sample = trk_node["down_sample"].as<bool>(false);
    track_params.down_sample_height = trk_node["down_sample_height"].as<int>(480);
    track_params.down_sample_width = trk_node["down_sample_width"].as<int>(640);
    max_track_length = std::max(max_track_length, track_params.max_track_length);

    SimFeatureTracker::Parameters sim_tracker_params;
    sim_tracker_params.feature_count = sim_node["feature_count"].as<unsigned int>(100);
    sim_tracker_params.room_size = sim_node["room_size"].as<double>(10.0);
    sim_tracker_params.no_errors = sim_node["no_errors"].as<bool>(false);
    sim_tracker_params.tracker_params = track_params;

    tracker_map[track_params.name] = sim_tracker_params;
  }

  // Load board detectors
  debug_logger->Log(LogLevel::INFO, "Loading Board Detectors");
  std::map<std::string, SimFiducialTracker::Parameters> fiducial_map;
  for (unsigned int i = 0; i < fiducials.size(); ++i) {
    YAML::Node fid_node = root["/EkfCalNode"]["ros__parameters"]["fiducial"][fiducials[i]];
    YAML::Node sim_node = fid_node["sim_params"];

    FiducialTracker::Parameters fiducial_params;
    LoadTrackerParams(
      fiducial_params, fiducials[i], log_directory, ekf, debug_logger);
    fiducial_params.pos_f_in_l =
      StdToEigVec(fid_node["pos_f_in_l"].as<std::vector<double>>(def_vec));
    fiducial_params.ang_f_to_l =
      StdToEigQuat(fid_node["ang_f_to_l"].as<std::vector<double>>(def_quat));
    fiducial_params.variance = StdToEigVec(fid_node["variance"].as<std::vector<double>>(def_vec));
    fiducial_params.squares_x = fid_node["squares_x"].as<unsigned int>(1);
    fiducial_params.squares_y = fid_node["squares_y"].as<unsigned int>(1);
    fiducial_params.square_length = fid_node["square_length"].as<double>(0.0);
    fiducial_params.marker_length = fid_node["marker_length"].as<double>(0.0);
    fiducial_params.id = fid_node["id"].as<unsigned int>(0);
    fiducial_params.is_extrinsic = fid_node["is_extrinsic"].as<bool>(false);
    fiducial_params.ekf = ekf;
    max_track_length = std::max(max_track_length, fiducial_params.max_track_length);

    SimFiducialTracker::Parameters sim_fiducial_params;
    sim_fiducial_params.pos_error =
      StdToEigVec(sim_node["pos_error"].as<std::vector<double>>(def_vec));
    sim_fiducial_params.ang_error =
      StdToEigVec(sim_node["ang_error"].as<std::vector<double>>(def_vec));
    sim_fiducial_params.t_vec_error =
      StdToEigVec(sim_node["t_vec_error"].as<std::vector<double>>(def_vec));
    sim_fiducial_params.r_vec_error =
      StdToEigVec(sim_node["r_vec_error"].as<std::vector<double>>(def_vec));
    sim_fiducial_params.no_errors = sim_node["no_errors"].as<bool>(false);
    sim_fiducial_params.fiducial_params = fiducial_params;

    fiducial_map[fiducial_params.name] = sim_fiducial_params;
  }
  ekf->SetMaxTrackLength(max_track_length);

  // Load cameras and generate measurements
  debug_logger->Log(LogLevel::INFO, "Loading Cameras");
  for (unsigned int i = 0; i < cameras.size(); ++i) {
    YAML::Node cam_node = root["/EkfCalNode"]["ros__parameters"]["camera"][cameras[i]];
    YAML::Node sim_node = cam_node["sim_params"];

    Camera::Parameters cam_params;
    LoadSensorParams(cam_params, cam_node, cameras[i], log_directory, ekf, debug_logger);
    cam_params.variance = StdToEigVec(cam_node["variance"].as<std::vector<double>>(def_vec));
    cam_params.pos_c_in_b = StdToEigVec(cam_node["pos_c_in_b"].as<std::vector<double>>(def_vec));
    cam_params.ang_c_to_b = StdToEigQuat(cam_node["ang_c_to_b"].as<std::vector<double>>(def_quat));
    cam_params.is_extrinsic = cam_node["is_extrinsic"].as<bool>(false);
    cam_params.pos_stability = cam_node["pos_stability"].as<double>(1.0e-9);
    cam_params.ang_stability = cam_node["ang_stability"].as<double>(1.0e-9);
    cam_params.tracker = cam_node["tracker"].as<std::string>("");
    cam_params.fiducial = cam_node["fiducial"].as<std::string>("");
    cam_params.intrinsics.f_x = cam_node["intrinsics"]["f_x"].as<double>(0.01);
    cam_params.intrinsics.f_y = cam_node["intrinsics"]["f_y"].as<double>(0.01);
    cam_params.intrinsics.k_1 = cam_node["intrinsics"]["k_1"].as<double>(0.0);
    cam_params.intrinsics.k_2 = cam_node["intrinsics"]["k_2"].as<double>(0.0);
    cam_params.intrinsics.p_1 = cam_node["intrinsics"]["p_1"].as<double>(0.0);
    cam_params.intrinsics.p_2 = cam_node["intrinsics"]["p_2"].as<double>(0.0);
    cam_params.intrinsics.width = cam_node["intrinsics"]["width"].as<double>(640.0);
    cam_params.intrinsics.height = cam_node["intrinsics"]["height"].as<double>(480.0);
    cam_params.intrinsics.pixel_size = cam_node["intrinsics"]["pixel_size"].as<double>(5.0e-6);

    // SimCamera::Parameters
    SimCamera::Parameters sim_cam_params;
    LoadSimSensorParams(sim_cam_params, sim_node);
    sim_cam_params.pos_error = StdToEigVec(sim_node["pos_error"].as<std::vector<double>>(def_vec));
    sim_cam_params.ang_error = StdToEigVec(sim_node["ang_error"].as<std::vector<double>>(def_vec));
    sim_cam_params.cam_params = cam_params;

    // Add sensor to map
    auto cam = std::make_shared<SimCamera>(sim_cam_params, truth_engine);
    if (!cam_params.tracker.empty()) {
      auto trk_params = tracker_map[cam_params.tracker];
      trk_params.tracker_params.camera_id = cam->GetId();
      trk_params.tracker_params.intrinsics = cam_params.intrinsics;
      trk_params.tracker_params.is_cam_extrinsic = cam_params.is_extrinsic;
      trk_params.tracker_params.data_log_rate = cam_params.data_log_rate;
      trk_params.no_errors = trk_params.no_errors || sim_cam_params.no_errors;
      auto trk = std::make_shared<SimFeatureTracker>(trk_params, truth_engine);
      cam->AddTracker(trk);
    }
    if (!cam_params.fiducial.empty()) {
      auto fid_params = fiducial_map[cam_params.fiducial];
      fid_params.fiducial_params.camera_id = cam->GetId();
      fid_params.fiducial_params.intrinsics = cam_params.intrinsics;
      fid_params.fiducial_params.is_cam_extrinsic = cam_params.is_extrinsic;
      fid_params.fiducial_params.data_log_rate = cam_params.data_log_rate;
      fid_params.no_errors = fid_params.no_errors || sim_cam_params.no_errors;
      auto fid = std::make_shared<SimFiducialTracker>(fid_params, truth_engine);
      cam->AddFiducial(fid);
    }

    sensor_map[cam->GetId()] = cam;

    // Calculate sensor measurements
    auto cam_messages = cam->GenerateMessages();
    messages.insert(messages.end(), cam_messages.begin(), cam_messages.end());
  }

  // Load GPSs and generate measurements
  debug_logger->Log(LogLevel::INFO, "Loading GPSs");
  for (unsigned int i = 0; i < gps_list.size(); ++i) {
    YAML::Node gps_node = root["/EkfCalNode"]["ros__parameters"]["gps"][gps_list[i]];
    YAML::Node sim_node = gps_node["sim_params"];

    GPS::Parameters gps_params;
    LoadSensorParams(
      gps_params, gps_node, gps_list[i], log_directory, ekf, debug_logger);
    gps_params.variance = StdToEigVec(gps_node["variance"].as<std::vector<double>>(def_vec));
    gps_params.pos_a_in_b = StdToEigVec(gps_node["pos_a_in_b"].as<std::vector<double>>(def_vec));
    gps_params.pos_stability = gps_node["pos_stability"].as<double>(0.0);
    gps_params.is_extrinsic = gps_node["is_extrinsic"].as<bool>(false);

    // SimParams
    SimGPS::Parameters sim_gps_params;
    LoadSimSensorParams(sim_gps_params, sim_node);
    sim_gps_params.gps_params = gps_params;
    sim_gps_params.lla_error = StdToEigVec(sim_node["lla_error"].as<std::vector<double>>(def_vec));
    sim_gps_params.pos_a_in_b_err =
      StdToEigVec(sim_node["pos_a_in_b_error"].as<std::vector<double>>(def_vec));
    sim_gps_params.pos_e_in_g_err =
      StdToEigVec(sim_node["pos_e_in_g_error"].as<std::vector<double>>(def_vec));
    sim_gps_params.ang_l_to_e_err = sim_node["ang_l_to_e_error"].as<double>(1e-9);

    // Add sensor to map
    auto gps = std::make_shared<SimGPS>(sim_gps_params, truth_engine);
    sensor_map[gps->GetId()] = gps;

    // Calculate sensor measurements
    auto gps_messages = gps->GenerateMessages();
    messages.insert(messages.end(), gps_messages.begin(), gps_messages.end());
  }

  // Log truth data
  if (data_log_rate != 0.0) {truth_engine->WriteTruthData(data_log_rate, log_directory);}

  // Sort Measurements
  sort(messages.begin(), messages.end(), MessageCompare);

  // Run measurements through sensors and EKF
  /// @todo: Add high-level execution timing vs. simulation time.
  debug_logger->Log(LogLevel::INFO, "Begin Simulation");
  for (auto message : messages) {
    auto sensor_iter = sensor_map.find(message->sensor_id);
    if (sensor_iter != sensor_map.end()) {
      if (message->sensor_type == SensorType::IMU) {
        auto imu = std::static_pointer_cast<SimIMU>(sensor_iter->second);
        auto msg = std::static_pointer_cast<SimImuMessage>(message);
        imu->Callback(*msg);
      } else if (message->sensor_type == SensorType::Camera) {
        auto cam = std::static_pointer_cast<SimCamera>(sensor_iter->second);
        auto msg = std::static_pointer_cast<SimCameraMessage>(message);
        cam->Callback(*msg);
      } else if (message->sensor_type == SensorType::GPS) {
        auto gps = std::static_pointer_cast<SimGPS>(sensor_iter->second);
        auto msg = std::static_pointer_cast<SimGpsMessage>(message);
        gps->Callback(*msg);
      } else {
        debug_logger->Log(LogLevel::WARN, "Unknown Message Type");
      }
    }
  }
  debug_logger->Log(LogLevel::INFO, "End Simulation");

  return 0;
}
