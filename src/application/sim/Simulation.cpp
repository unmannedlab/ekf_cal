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

#include "infrastructure/sim/TruthEngine.hpp"
#include "infrastructure/sim/TruthEngineSpline.hpp"
#include "infrastructure/sim/TruthEngineCyclic.hpp"
#include "sensors/IMU.hpp"
#include "sensors/sim/SimIMU.hpp"
#include "sensors/sim/SimCamera.hpp"
#include "trackers/sim/SimFeatureTracker.hpp"
#include "utility/TypeHelper.hpp"


/// @todo read input YAML and output directory from arguments
int main(int argc, char * argv[])
{
  const cv::String keys =
    "{help h usage ? |      | print this help message       }"
    "{@config        |<none>| Input YAML configuration file }"
    "{@outDir        |<none>| Output directory for logs     }"
  ;

  cv::CommandLineParser parser(argc, argv, keys);

  std::string config = parser.get<std::string>("@config");
  std::string outDir = parser.get<std::string>("@outDir");

  // Define sensors to use (load config from yaml)
  YAML::Node root = YAML::LoadFile(config);
  std::cout << "root size:" << root.size() << std::endl;
  const auto imus = root["/EkfCalNode"]["ros__parameters"]["IMU"];
  const auto cams = root["/EkfCalNode"]["ros__parameters"]["Camera"];
  const auto trks = root["/EkfCalNode"]["ros__parameters"]["Tracker"];

  // Construct sensors and EKF
  std::map<unsigned int, std::shared_ptr<Sensor>> sensorMap;

  /// @todo Select type of truth engine using parameters
  auto truthEngine = std::static_pointer_cast<TruthEngine>(std::make_shared<TruthEngineCyclic>());
  std::vector<std::shared_ptr<SensorMessage>> messages;

  double maxTime = 10;

  // Logging parameters
  unsigned int debugLogLevel =
    root["/EkfCalNode"]["ros__parameters"]["Debug_Log_Level"].as<unsigned int>();
  bool dataLoggingOn = root["/EkfCalNode"]["ros__parameters"]["Data_Logging_On"].as<bool>();
  DebugLogger * logger = DebugLogger::getInstance();
  logger->setOutputDirectory(outDir);
  logger->setLogLevel(debugLogLevel);

  // Load IMUs and generate measurements
  if (imus) {
    for (auto it = imus.begin(); it != imus.end(); ++it) {
      YAML::Node imuNode = it->second;
      YAML::Node simNode = imuNode["SimParams"];

      IMU::Parameters imuParams;
      imuParams.name = it->first.as<std::string>();
      imuParams.baseSensor = imuNode["BaseSensor"].as<bool>();
      imuParams.intrinsic = imuNode["Intrinsic"].as<bool>();
      imuParams.rate = imuNode["Rate"].as<double>();
      imuParams.topic = imuNode["Topic"].as<std::string>();
      imuParams.variance = stdToEigVec(imuNode["VarInit"].as<std::vector<double>>());
      imuParams.posOffset = stdToEigVec(imuNode["PosOffInit"].as<std::vector<double>>());
      imuParams.angOffset = stdToEigQuat(imuNode["AngOffInit"].as<std::vector<double>>());
      imuParams.accBias = stdToEigVec(imuNode["AccBiasInit"].as<std::vector<double>>());
      imuParams.omgBias = stdToEigVec(imuNode["OmgBiasInit"].as<std::vector<double>>());
      imuParams.outputDirectory = outDir;
      imuParams.dataLoggingOn = dataLoggingOn;

      // SimParams
      SimIMU::Parameters simImuParams;
      simImuParams.imuParams = imuParams;
      simImuParams.tBias = simNode["timeBias"].as<double>();
      simImuParams.tSkew = simNode["timeSkew"].as<double>();
      simImuParams.tError = simNode["timeError"].as<double>();
      simImuParams.accBias = stdToEigVec(simNode["accBias"].as<std::vector<double>>());
      simImuParams.accError = stdToEigVec(simNode["accError"].as<std::vector<double>>());
      simImuParams.omgBias = stdToEigVec(simNode["omgBias"].as<std::vector<double>>());
      simImuParams.omgError = stdToEigVec(simNode["omgError"].as<std::vector<double>>());
      simImuParams.posOffset = stdToEigVec(simNode["posOffset"].as<std::vector<double>>());
      simImuParams.angOffset = stdToEigQuat(simNode["angOffset"].as<std::vector<double>>());

      // Add sensor to map
      auto imu = std::make_shared<SimIMU>(simImuParams, truthEngine);
      sensorMap[imu->getId()] = imu;

      // Calculate sensor measurements
      auto imuMessages = imu->generateMessages(maxTime);
      messages.insert(messages.end(), imuMessages.begin(), imuMessages.end());
    }
  }

  // Load tracker parameters
  std::map<std::string, SimFeatureTracker::Parameters> trackerMap;
  if (trks) {
    for (auto it = trks.begin(); it != trks.end(); ++it) {
      YAML::Node trkNode = it->second;
      YAML::Node simNode = trkNode["SimParams"];

      FeatureTracker::Parameters trkParams;
      trkParams.name = it->first.as<std::string>();
      trkParams.outputDirectory = outDir;
      trkParams.dataLoggingOn = dataLoggingOn;

      SimFeatureTracker::Parameters simTrkParams;
      simTrkParams.featureCount = simNode["featureCount"].as<unsigned int>();
      simTrkParams.roomSize = simNode["roomSize"].as<double>();
      simTrkParams.trackerParams = trkParams;

      trackerMap[trkParams.name] = simTrkParams;
    }
  }

  // Load cameras and generate measurements
  if (cams) {
    for (auto it = cams.begin(); it != cams.end(); ++it) {
      YAML::Node camNode = it->second;
      YAML::Node simNode = camNode["SimParams"];

      Camera::Parameters camParams;
      camParams.name = it->first.as<std::string>();
      camParams.rate = camNode["Rate"].as<double>();
      camParams.variance = stdToEigVec(camNode["VarInit"].as<std::vector<double>>());
      camParams.posOffset = stdToEigVec(camNode["PosOffInit"].as<std::vector<double>>());
      camParams.angOffset = stdToEigQuat(camNode["AngOffInit"].as<std::vector<double>>());
      camParams.outputDirectory = outDir;
      camParams.dataLoggingOn = dataLoggingOn;
      camParams.tracker = camNode["Tracker"].as<std::string>();

      // SimCamera::Parameters
      SimCamera::Parameters simCamParams;
      simCamParams.tBias = simNode["timeBias"].as<double>();
      simCamParams.tSkew = simNode["timeSkew"].as<double>();
      simCamParams.tError = simNode["timeError"].as<double>();
      simCamParams.posOffset = stdToEigVec(simNode["posOffset"].as<std::vector<double>>());
      simCamParams.angOffset = stdToEigQuat(simNode["angOffset"].as<std::vector<double>>());
      simCamParams.camParams = camParams;

      // Add sensor to map
      auto cam = std::make_shared<SimCamera>(simCamParams, truthEngine);
      auto trkParams = trackerMap[camParams.tracker];
      trkParams.trackerParams.sensorID = cam->getId();
      auto trk = std::make_shared<SimFeatureTracker>(trkParams, truthEngine);
      cam->addTracker(trk);
      sensorMap[cam->getId()] = cam;

      // Calculate sensor measurements
      auto imuMessages = cam->generateMessages(maxTime);
      messages.insert(messages.end(), imuMessages.begin(), imuMessages.end());
    }
  }

  // Sort Measurements
  sort(messages.begin(), messages.end(), messageCompare);

  // Run measurements through sensors and EKF
  for (auto message : messages) {
    auto it = sensorMap.find(message->sensorID);
    if (it != sensorMap.end()) {
      if (message->sensorType == SensorType::IMU) {
        auto imu = std::static_pointer_cast<SimIMU>(it->second);
        auto msg = std::static_pointer_cast<SimImuMessage>(message);
        imu->callback(msg);
      } else if (message->sensorType == SensorType::Camera) {
        auto cam = std::static_pointer_cast<SimCamera>(it->second);
        auto msg = std::static_pointer_cast<SimCameraMessage>(message);
        cam->callback(msg);
      } else {
        std::cout << "Unknown Message Type" << std::endl;
      }
    }
  }

  // Return
  /// @todo "Uses" input parameters to suppress compiler warning
  (void)argv[argc - 1];
  return 0;
}
