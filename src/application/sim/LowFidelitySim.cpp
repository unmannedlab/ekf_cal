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
#include "infrastructure/sim/TruthEngine.hpp"


int main(int argc, char * argv[])
{
  // Define sensors to use (load config from yaml)

  double maxTime = 10;

  // Construct sensors and EKF
  auto truthEngine = std::make_shared<TruthEngine>();
  std::map<unsigned int, std::shared_ptr<Sensor>> sensorMap;

  SimImuParams imuParams1;
  imuParams1.accError = 1e-3;
  imuParams1.omgError = 1e-3;
  auto imu1 = std::make_shared<SimIMU>(imuParams1, truthEngine);
  sensorMap[imu1->getId()] = imu1;

  SimImuParams imuParams2;
  imuParams2.accError = 1e-3;
  imuParams2.omgError = 1e-3;
  imuParams2.posOffset = Eigen::Vector3d(0.1, 0, 0);
  auto imu2 = std::make_shared<SimIMU>(imuParams2, truthEngine);
  sensorMap[imu2->getId()] = imu2;

  // Calculate sensor measurements
  std::vector<SensorMessage> measurements;
  std::vector<SimImuMessage> imu1Measurements = imu1->generateMeasurements(maxTime);
  std::vector<SimImuMessage> imu2Measurements = imu2->generateMeasurements(maxTime);

  measurements.insert(measurements.end(), imu1Measurements.begin(), imu1Measurements.end());
  measurements.insert(measurements.end(), imu2Measurements.begin(), imu2Measurements.end());

  // Sort Measurements
  sort(measurements.begin(), measurements.end());

  // Run measurements through sensors and EKF
  for (auto measurement : measurements) {
    auto it = sensorMap.find(measurement.sensorID);
    if (it != sensorMap.end()) {
      /// @todo This doesn't call the child class callback
      it->second->callback(measurement);
    }
  }

  // Plot results
  return 0;
}
