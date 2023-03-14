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

#include "sensors/sim/SimIMU.hpp"
#include "sensors/IMU.hpp"
#include "infrastructure/sim/TruthEngine.hpp"
#include "utility/TypeHelper.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>

/// @todo read input YAML and output directory from arguments
int main(int argc, char * argv[])
{
  // Define sensors to use (load config from yaml)
  YAML::Node root = YAML::LoadFile("/home/jacob/proj/ekf_cal_ws/src/ekf_cal/config/imu-cam.yaml");
  std::cout << "root size:" << root.size() << std::endl;
  const auto imus = root["/EkfCalNode"]["ros__parameters"]["IMU"];

  // Construct sensors and EKF
  std::map<unsigned int, std::shared_ptr<Sensor>> sensorMap;
  auto truthEngine = std::make_shared<TruthEngine>();
  std::vector<std::shared_ptr<SensorMessage>> messages;

  double maxTime = 10;

  // Logging parameters
  unsigned int LogLevel =
    root["/EkfCalNode"]["ros__parameters"]["Debug_Log_Level"].as<unsigned int>();
  unsigned int performanceLogLevel =
    root["/EkfCalNode"]["ros__parameters"]["Data_Log_Level"].as<unsigned int>();
  DebugLogger * logger = DebugLogger::getInstance();
  logger->setLogLevel(LogLevel);

  if (imus) {
    for (auto it = imus.begin(); it != imus.end(); ++it) {
      YAML::Node imuNode = it->second;
      YAML::Node simNode = imuNode["SimParams"];

      IMU::Params imuParams;
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

      // SimParams
      SimImuParams simImuParams;
      simImuParams.imuParams = imuParams;
      simImuParams.tBias = simNode["timeBias"].as<double>();
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
      std::vector<std::shared_ptr<SimImuMessage>> imuMessages = imu->generateMessages(maxTime);
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
      } else if (message->sensorType == SensorType::Tracker) {
        /// @todo Tracker sim callback
      } else {
        std::cout << "Unknown Message Type" << std::endl;
      }
    }
  }

  // Return
  (void)argv[argc - 1]; /// @todo "Uses" input parameters to suppress compiler warning
  return 0;
}
