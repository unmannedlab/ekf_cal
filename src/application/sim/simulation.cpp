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
#include <opencv2/core/utility.hpp>

#include "infrastructure/sim/truth_engine.hpp"
#include "infrastructure/sim/truth_engine_spline.hpp"
#include "infrastructure/sim/truth_engine_cyclic.hpp"
#include "sensors/imu.hpp"
#include "sensors/sim/sim_imu.hpp"
#include "sensors/sim/sim_camera.hpp"
#include "trackers/sim/sim_feature_tracker.hpp"
#include "utility/type_helper.hpp"


/// @todo read input YAML and output directory from arguments
int main(int argc, char * argv[])
{
  const cv::String keys =
    "{help h usage ? |      | print this help message       }"
    "{@config        |<none>| Input YAML configuration file }"
    "{@out_dir       |<none>| Output directory for logs     }"
  ;

  cv::CommandLineParser parser(argc, argv, keys);

  std::string config = parser.get<std::string>("@config");
  std::string out_dir = parser.get<std::string>("@out_dir");

  // Define sensors to use (load config from yaml)
  YAML::Node root = YAML::LoadFile(config);
  const auto imus = root["/EkfCalNode"]["ros__parameters"]["IMU"];
  const auto cameras = root["/EkfCalNode"]["ros__parameters"]["Camera"];
  const auto trackers = root["/EkfCalNode"]["ros__parameters"]["Tracker"];

  // Construct sensors and EKF
  std::map<unsigned int, std::shared_ptr<Sensor>> sensor_map;

  /// @todo Select type of truth engine using parameters
  auto truth_engine = std::static_pointer_cast<TruthEngine>(std::make_shared<TruthEngineCyclic>());
  std::vector<std::shared_ptr<SensorMessage>> messages;

  double max_time = 10;

  // Logging parameters
  unsigned int debug_log_level =
    root["/EkfCalNode"]["ros__parameters"]["Debug_Log_Level"].as<unsigned int>();
  bool data_logging_on = root["/EkfCalNode"]["ros__parameters"]["Data_Logging_On"].as<bool>();
  double rng_seed = root["/EkfCalNode"]["ros__parameters"]["SimParams"]["Seed"].as<double>();
  DebugLogger * logger = DebugLogger::GetInstance();
  logger->SetOutputDirectory(out_dir);
  logger->SetLogLevel(debug_log_level);
  SimRNG rng;
  rng.SetSeed(rng_seed);

  // Load IMUs and generate measurements
  logger->Log(LogLevel::INFO, "Loading IMUs");
  if (imus) {
    for (auto it = imus.begin(); it != imus.end(); ++it) {
      YAML::Node imu_node = it->second;
      YAML::Node sim_node = imu_node["SimParams"];

      IMU::Parameters imu_params;
      imu_params.name = it->first.as<std::string>();
      imu_params.base_sensor = imu_node["BaseSensor"].as<bool>();
      imu_params.intrinsic = imu_node["Intrinsic"].as<bool>();
      imu_params.rate = imu_node["Rate"].as<double>();
      imu_params.topic = imu_node["Topic"].as<std::string>();
      imu_params.variance = StdToEigVec(imu_node["VarInit"].as<std::vector<double>>());
      imu_params.pos_offset = StdToEigVec(imu_node["PosOffInit"].as<std::vector<double>>());
      imu_params.ang_offset = StdToEigQuat(imu_node["AngOffInit"].as<std::vector<double>>());
      imu_params.acc_bias = StdToEigVec(imu_node["AccBiasInit"].as<std::vector<double>>());
      imu_params.omg_bias = StdToEigVec(imu_node["OmgBiasInit"].as<std::vector<double>>());
      imu_params.output_directory = out_dir;
      imu_params.data_logging_on = data_logging_on;

      // SimParams
      SimIMU::Parameters sim_imu_params;
      sim_imu_params.imu_params = imu_params;
      sim_imu_params.time_bias = sim_node["timeBias"].as<double>();
      sim_imu_params.time_skew = sim_node["timeSkew"].as<double>();
      sim_imu_params.time_error = sim_node["timeError"].as<double>();
      sim_imu_params.acc_bias = StdToEigVec(sim_node["accBias"].as<std::vector<double>>());
      sim_imu_params.acc_error = StdToEigVec(sim_node["accError"].as<std::vector<double>>());
      sim_imu_params.omg_bias = StdToEigVec(sim_node["omgBias"].as<std::vector<double>>());
      sim_imu_params.omg_error = StdToEigVec(sim_node["omgError"].as<std::vector<double>>());
      sim_imu_params.pos_offset = StdToEigVec(sim_node["posOffset"].as<std::vector<double>>());
      sim_imu_params.ang_offset = StdToEigQuat(sim_node["angOffset"].as<std::vector<double>>());

      // Add sensor to map
      auto imu = std::make_shared<SimIMU>(sim_imu_params, truth_engine);
      sensor_map[imu->GetId()] = imu;

      // Calculate sensor measurements
      auto imu_messages = imu->GenerateMessages(max_time);
      messages.insert(messages.end(), imu_messages.begin(), imu_messages.end());
    }
  }

  // Load tracker parameters
  logger->Log(LogLevel::INFO, "Loading Trackers");
  std::map<std::string, SimFeatureTracker::Parameters> trackerMap;
  if (trackers) {
    for (auto it = trackers.begin(); it != trackers.end(); ++it) {
      YAML::Node trk_node = it->second;
      YAML::Node sim_node = trk_node["SimParams"];

      FeatureTracker::Parameters trkParams;
      trkParams.name = it->first.as<std::string>();
      trkParams.output_directory = out_dir;
      trkParams.data_logging_on = data_logging_on;

      SimFeatureTracker::Parameters simTrkParams;
      simTrkParams.feature_count = sim_node["featureCount"].as<unsigned int>();
      simTrkParams.room_size = sim_node["roomSize"].as<double>();
      simTrkParams.tracker_params = trkParams;

      trackerMap[trkParams.name] = simTrkParams;
    }
  }

  // Load cameras and generate measurements
  logger->Log(LogLevel::INFO, "Loading Cameras");
  if (cameras) {
    for (auto it = cameras.begin(); it != cameras.end(); ++it) {
      YAML::Node camNode = it->second;
      YAML::Node sim_node = camNode["SimParams"];

      Camera::Parameters cam_params;
      cam_params.name = it->first.as<std::string>();
      cam_params.rate = camNode["Rate"].as<double>();
      cam_params.variance = StdToEigVec(camNode["VarInit"].as<std::vector<double>>());
      cam_params.pos_offset = StdToEigVec(camNode["PosOffInit"].as<std::vector<double>>());
      cam_params.ang_offset = StdToEigQuat(camNode["AngOffInit"].as<std::vector<double>>());
      cam_params.output_directory = out_dir;
      cam_params.data_logging_on = data_logging_on;
      cam_params.tracker = camNode["Tracker"].as<std::string>();

      // SimCamera::Parameters
      SimCamera::Parameters sim_cam_params;
      sim_cam_params.time_bias = sim_node["timeBias"].as<double>();
      sim_cam_params.time_skew = sim_node["timeSkew"].as<double>();
      sim_cam_params.time_error = sim_node["timeError"].as<double>();
      sim_cam_params.pos_offset = StdToEigVec(sim_node["posOffset"].as<std::vector<double>>());
      sim_cam_params.ang_offset = StdToEigQuat(sim_node["angOffset"].as<std::vector<double>>());
      sim_cam_params.cam_params = cam_params;

      // Add sensor to map
      auto cam = std::make_shared<SimCamera>(sim_cam_params, truth_engine);
      auto trk_params = trackerMap[cam_params.tracker];
      trk_params.tracker_params.sensor_id = cam->GetId();
      auto trk = std::make_shared<SimFeatureTracker>(trk_params, truth_engine);
      cam->AddTracker(trk);
      sensor_map[cam->GetId()] = cam;

      // Calculate sensor measurements
      auto imu_messages = cam->GenerateMessages(max_time);
      messages.insert(messages.end(), imu_messages.begin(), imu_messages.end());
    }
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

  // Return
  /// @todo "Uses" input parameters to suppress compiler warning
  (void)argv[argc - 1];
  return 0;
}