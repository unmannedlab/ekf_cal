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


#ifndef SENSORS__SIM__SIMIMU_HPP_
#define SENSORS__SIM__SIMIMU_HPP_

#include <string>
#include <vector>
#include <eigen3/Eigen/Eigen>

#include "ekf/Types.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "infrastructure/sim/TruthEngine.hpp"
#include "sensors/IMU.hpp"
#include "sensors/Sensor.hpp"
#include "utility/sim/SimRNG.hpp"

class SimImuMessage : public ImuMessage
{
public:
  using ImuMessage::ImuMessage;
};

typedef struct SimImuParams
{
  double tBias {0.0};
  double tError {1e-9};
  Eigen::Vector3d accBias {0.0, 0.0, 0.0};
  Eigen::Vector3d accError {1e-9, 1e-9, 1e-9};
  Eigen::Vector3d omgBias {0.0, 0.0, 0.0};
  Eigen::Vector3d omgError {1e-9, 1e-9, 1e-9};
  Eigen::Vector3d posOffset {0.0, 0.0, 0.0};
  Eigen::Quaterniond angOffset {1.0, 0.0, 0.0, 0.0};
  IMU::Params imuParams;
} SimImuParams;

///
/// @class SimIMU
/// @brief Simulated IMU Sensor Class
///
class SimIMU : public IMU
{
public:
  SimIMU(SimImuParams params, std::shared_ptr<TruthEngine> truthEngine);

  std::vector<std::shared_ptr<SimImuMessage>> generateMessages(double maxTime);

private:
  double m_tBias{0.0};
  double m_tSkew{0.0};
  double m_tError{1e-9};
  Eigen::Vector3d m_accBias{0.0, 0.0, 0.0};
  Eigen::Vector3d m_accError{1e-9, 1e-9, 1e-9};
  Eigen::Vector3d m_omgBias{0.0, 0.0, 0.0};
  Eigen::Vector3d m_omgError{1e-9, 1e-9, 1e-9};
  Eigen::Vector3d m_posOffset{0.0, 0.0, 0.0};
  Eigen::Quaterniond m_angOffset{1.0, 0.0, 0.0, 0.0};
  SimRNG m_rng;
  std::shared_ptr<TruthEngine> m_truth;
};


#endif  // SENSORS__SIM__SIMIMU_HPP_
