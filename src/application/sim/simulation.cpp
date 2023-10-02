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

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <memory>
#include <string>
#include <opencv2/core/utility.hpp>

#include "infrastructure/ekf_cal_version.hpp"
#include "infrastructure/sim/truth_engine_cyclic.hpp"
#include "infrastructure/sim/truth_engine_spline.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/imu.hpp"
#include "sensors/sim/sim_camera_message.hpp"
#include "sensors/sim/sim_camera.hpp"
#include "sensors/sim/sim_imu_message.hpp"
#include "sensors/sim/sim_imu.hpp"
#include "trackers/sim/sim_feature_tracker.hpp"
#include "utility/type_helper.hpp"
#include "utility/string_helper.hpp"


std::vector<std::string> LoadNodeList(YAML::Node node)
{
  std::vector<std::string> string_list;
  for (unsigned int i = 0; i < node.size(); ++i) {
    string_list.push_back(node[i].as<std::string>());
  }
  return string_list;
}

void WriteTruthData(
  std::shared_ptr<TruthEngine> truth_engine,
  double body_data_rate,
  double max_time,
  std::string output_directory,
  bool data_logging_on)
{
  DataLogger data_logger;
  data_logger.SetLogging(data_logging_on);
  data_logger.SetOutputFileName("body_truth.csv");
  data_logger.SetOutputDirectory(output_directory);

  std::stringstream header;
  header << "time";
  header << EnumerateHeader("body_pos", 3);
  header << EnumerateHeader("body_vel", 3);
  header << EnumerateHeader("body_acc", 3);
  header << EnumerateHeader("body_ang_pos", 4);
  header << EnumerateHeader("body_ang_vel", 3);
  header << EnumerateHeader("body_ang_acc", 3);
  header << std::endl;
  data_logger.DefineHeader(header.str());

  unsigned int num_measurements = static_cast<int>(std::floor(max_time * body_data_rate)) + 100;
  for (unsigned int i = 0; i < num_measurements; ++i) {
    double time = static_cast<double>(i) / body_data_rate;
    Eigen::Vector3d body_pos = truth_engine->GetBodyPosition(time);
    Eigen::Vector3d body_vel = truth_engine->GetBodyVelocity(time);
    Eigen::Vector3d body_acc = truth_engine->GetBodyAcceleration(time);
    Eigen::Quaterniond body_ang_pos = truth_engine->GetBodyAngularPosition(time);
    Eigen::Vector3d body_ang_vel = truth_engine->GetBodyAngularRate(time);
    Eigen::Vector3d body_ang_acc = truth_engine->GetBodyAngularAcceleration(time);

    std::stringstream msg;
    msg << time;
    msg << VectorToCommaString(body_pos);
    msg << VectorToCommaString(body_vel);
    msg << VectorToCommaString(body_acc);
    msg << QuaternionToCommaString(body_ang_pos);
    msg << VectorToCommaString(body_ang_vel);
    msg << VectorToCommaString(body_ang_acc);
    msg << std::endl;

    data_logger.Log(msg.str());
  }
}

int main(int argc, char * argv[])
{
  const cv::String keys =
    "{@config        |<none>| Input YAML configuration file }"
    "{@out_dir       |<none>| Output directory for logs     }"
    "{help h usage ? |      | print this help message       }"
  ;

  cv::CommandLineParser parser(argc, argv, keys);
  /// @todo(jhartzer): Get this value from include
  parser.about("EKF-CAL 0.2.0");

  /// @todo(jhartzer): Cannot access help function
  if (parser.has("help")) {
    parser.printMessage();
    return 0;
  }

  std::string config = parser.get<std::string>("@config");
  std::string out_dir = parser.get<std::string>("@out_dir");

  // Define sensors to use (load config from yaml)
  YAML::Node root = YAML::LoadFile(config);
  auto imus = LoadNodeList(root["/EkfCalNode"]["ros__parameters"]["imu_list"]);
  auto cameras = LoadNodeList(root["/EkfCalNode"]["ros__parameters"]["camera_list"]);
  auto trackers = LoadNodeList(root["/EkfCalNode"]["ros__parameters"]["tracker_list"]);

  // Construct sensors and EKF
  std::map<unsigned int, std::shared_ptr<Sensor>> sensor_map;
  std::vector<std::shared_ptr<SensorMessage>> messages;

  // Logging parameters
  YAML::Node ros_params = root["/EkfCalNode"]["ros__parameters"];
  unsigned int debug_log_level = ros_params["debug_log_level"].as<unsigned int>();
  bool data_logging_on = ros_params["data_logging_on"].as<bool>();
  double body_data_rate = ros_params["body_data_rate"].as<double>();

  // Set EKF parameters
  EKF * ekf = EKF::GetInstance();
  ekf->SetBodyDataRate(body_data_rate);
  ekf->SetDataLogging(data_logging_on);
  ekf->m_data_logger.SetOutputDirectory(out_dir);
  ekf->m_data_logger.SetOutputFileName("body_state.csv");

  // Simulation parameters
  YAML::Node sim_params = ros_params["sim_params"];
  double rng_seed = sim_params["seed"].as<double>();
  bool use_seed = sim_params["use_seed"].as<bool>();
  bool no_errors = sim_params["no_errors"].as<bool>();
  Eigen::Vector3d pos_frequency =
    StdToEigVec(sim_params["pos_frequency"].as<std::vector<double>>());
  Eigen::Vector3d ang_frequency =
    StdToEigVec(sim_params["ang_frequency"].as<std::vector<double>>());
  Eigen::Vector3d pos_b_in_g = StdToEigVec(sim_params["pos_offset"].as<std::vector<double>>());
  double max_time = sim_params["max_time"].as<double>();

  DebugLogger * logger = DebugLogger::GetInstance();
  logger->SetOutputDirectory(out_dir);
  logger->SetLogLevel(debug_log_level);
  logger->Log(LogLevel::INFO, "EKF CAL Version: " + std::string(EKF_CAL_VERSION));

  SimRNG rng;
  if (use_seed) {
    rng.SetSeed(rng_seed);
  }

  /// @todo Select type of truth engine using parameters
  auto truth_engine_cyclic = std::make_shared<TruthEngineCyclic>(
    pos_frequency,
    ang_frequency,
    pos_b_in_g);
  auto truth_engine = std::static_pointer_cast<TruthEngine>(truth_engine_cyclic);

  WriteTruthData(truth_engine, body_data_rate, max_time, out_dir, data_logging_on);

  // Load IMUs and generate measurements
  bool using_any_imu_for_prediction {false};
  logger->Log(LogLevel::INFO, "Loading IMUs");
  for (unsigned int i = 0; i < imus.size(); ++i) {
    YAML::Node imu_node = root["/EkfCalNode"]["ros__parameters"]["imu"][imus[i]];
    YAML::Node sim_node = imu_node["sim_params"];

    IMU::Parameters imu_params;
    imu_params.name = imus[i];
    imu_params.base_sensor = imu_node["base_sensor"].as<bool>();
    imu_params.intrinsic = imu_node["intrinsic"].as<bool>();
    imu_params.rate = imu_node["rate"].as<double>();
    imu_params.topic = imu_node["topic"].as<std::string>();
    imu_params.variance = StdToEigVec(imu_node["variance"].as<std::vector<double>>());
    imu_params.pos_i_in_b = StdToEigVec(imu_node["pos_i_in_b"].as<std::vector<double>>());
    imu_params.ang_i_to_b = StdToEigQuat(imu_node["ang_i_to_b"].as<std::vector<double>>());
    imu_params.acc_bias = StdToEigVec(imu_node["acc_bias"].as<std::vector<double>>());
    imu_params.omg_bias = StdToEigVec(imu_node["omg_bias"].as<std::vector<double>>());
    imu_params.output_directory = out_dir;
    imu_params.data_logging_on = data_logging_on;
    imu_params.use_for_prediction = imu_node["use_for_prediction"].as<bool>();
    using_any_imu_for_prediction = using_any_imu_for_prediction || imu_params.use_for_prediction;

    // SimParams
    SimIMU::Parameters sim_imu_params;
    sim_imu_params.imu_params = imu_params;
    sim_imu_params.time_bias_error = sim_node["time_bias_error"].as<double>();
    sim_imu_params.time_skew_error = sim_node["time_skew_error"].as<double>();
    sim_imu_params.time_error = sim_node["time_error"].as<double>();
    sim_imu_params.acc_error = StdToEigVec(sim_node["acc_error"].as<std::vector<double>>());
    sim_imu_params.omg_error = StdToEigVec(sim_node["omg_error"].as<std::vector<double>>());
    sim_imu_params.acc_bias_error =
      StdToEigVec(sim_node["acc_bias_error"].as<std::vector<double>>());
    sim_imu_params.omg_bias_error =
      StdToEigVec(sim_node["omg_bias_error"].as<std::vector<double>>());
    sim_imu_params.no_errors = no_errors;

    // Add sensor to map
    auto imu = std::make_shared<SimIMU>(sim_imu_params, truth_engine);
    sensor_map[imu->GetId()] = imu;

    // Calculate sensor measurements
    auto imu_messages = imu->GenerateMessages(max_time);
    messages.insert(messages.end(), imu_messages.begin(), imu_messages.end());
  }

  if (using_any_imu_for_prediction && (imus.size() > 1)) {
    std::cerr << "Configuration Error: Cannot use multiple IMUs and IMU prediction" << std::endl;
    return -1;
  }

  // Load tracker parameters
  logger->Log(LogLevel::INFO, "Loading Trackers");
  std::map<std::string, SimFeatureTracker::Parameters> trackerMap;
  for (unsigned int i = 0; i < trackers.size(); ++i) {
    YAML::Node trk_node = root["/EkfCalNode"]["ros__parameters"]["tracker"][trackers[i]];
    YAML::Node sim_node = trk_node["sim_params"];

    FeatureTracker::Parameters track_params;
    track_params.name = trackers[i];
    track_params.output_directory = out_dir;
    track_params.data_logging_on = data_logging_on;
    track_params.px_error = trk_node["pixel_error"].as<double>();

    SimFeatureTracker::Parameters sim_tracker_params;
    sim_tracker_params.feature_count = sim_node["feature_count"].as<unsigned int>();
    sim_tracker_params.room_size = sim_node["room_size"].as<double>();
    sim_tracker_params.tracker_params = track_params;
    sim_tracker_params.no_errors = no_errors;

    trackerMap[track_params.name] = sim_tracker_params;
  }

  // Load cameras and generate measurements
  logger->Log(LogLevel::INFO, "Loading Cameras");
  for (unsigned int i = 0; i < cameras.size(); ++i) {
    YAML::Node camNode = root["/EkfCalNode"]["ros__parameters"]["camera"][cameras[i]];
    YAML::Node sim_node = camNode["sim_params"];

    Camera::Parameters cam_params;
    cam_params.name = cameras[i];
    cam_params.rate = camNode["rate"].as<double>();
    cam_params.variance = StdToEigVec(camNode["variance"].as<std::vector<double>>());
    cam_params.pos_c_in_b = StdToEigVec(camNode["pos_c_in_b"].as<std::vector<double>>());
    cam_params.ang_c_to_b = StdToEigQuat(camNode["ang_c_to_b"].as<std::vector<double>>());
    cam_params.output_directory = out_dir;
    cam_params.data_logging_on = data_logging_on;
    cam_params.tracker = camNode["tracker"].as<std::string>();

    // SimCamera::Parameters
    SimCamera::Parameters sim_cam_params;
    sim_cam_params.time_bias_error = sim_node["time_bias_error"].as<double>();
    sim_cam_params.time_skew_error = sim_node["time_skew_error"].as<double>();
    sim_cam_params.time_error = sim_node["time_error"].as<double>();
    sim_cam_params.cam_params = cam_params;
    sim_cam_params.no_errors = no_errors;

    // Add sensor to map
    auto cam = std::make_shared<SimCamera>(sim_cam_params, truth_engine);
    auto trk_params = trackerMap[cam_params.tracker];
    trk_params.tracker_params.sensor_id = cam->GetId();
    auto trk = std::make_shared<SimFeatureTracker>(
      trk_params, truth_engine, out_dir, data_logging_on);
    cam->AddTracker(trk);
    sensor_map[cam->GetId()] = cam;

    // Calculate sensor measurements
    auto imu_messages = cam->GenerateMessages(max_time);
    messages.insert(messages.end(), imu_messages.begin(), imu_messages.end());
  }

  // Sort Measurements
  sort(messages.begin(), messages.end(), MessageCompare);

  // Run measurements through sensors and EKF
  logger->Log(LogLevel::INFO, "Begin Simulation");
  for (auto message : messages) {
    auto it = sensor_map.find(message->m_sensor_id);
    if (it != sensor_map.end()) {
      if (message->m_sensor_type == SensorType::IMU) {
        auto imu = std::static_pointer_cast<SimIMU>(it->second);
        auto msg = std::static_pointer_cast<SimImuMessage>(message);
        imu->Callback(msg);
      } else if (message->m_sensor_type == SensorType::Camera) {
        auto cam = std::static_pointer_cast<SimCamera>(it->second);
        auto msg = std::static_pointer_cast<SimCameraMessage>(message);
        cam->Callback(msg);
      } else {
        logger->Log(LogLevel::WARN, "Unknown Message Type");
      }
    }
  }
  logger->Log(LogLevel::INFO, "End Simulation");

  return 0;
}
