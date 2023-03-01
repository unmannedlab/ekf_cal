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
#include <eigen3/Eigen/Eigen>

#include "infrastructure/Logger.hpp"
#include "infrastructure/sim/TruthEngine.hpp"
#include "sensors/IMU.hpp"
#include "sensors/Sensor.hpp"
#include "sensors/sim/SimTypes.hpp"
#include "utility/sim/SimRNG.hpp"

class SimImuMessage : public SimMessage
{
public:
  SimImuMessage() {}

  Eigen::Vector3d acceleration;
  Eigen::Matrix3d accelerationCovariance;
  Eigen::Vector3d angularRate;
  Eigen::Matrix3d angularRateCovariance;
};

typedef struct SimImuParams
{
  double tBias {0};
  double tError {1e-9};
  double accBias {0};
  double accError {1e-9};
  double omgBias {0};
  double omgError {1e-9};
  Eigen::Vector3d posOffset {0, 0, 0};
  Eigen::Quaterniond angOffset {1, 0, 0, 0};
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
  SimImuMessage generateMeasurement(double measurementTime);

private:
  double m_tBias{0};
  double m_tError{1e-9};
  double m_accBias{0};
  double m_accError{1};
  double m_omgBias{0};
  double m_omgError{1};
  Eigen::Vector3d m_posOffset;
  Eigen::Quaterniond m_angOffset;
  SimRNG m_rng;
  std::shared_ptr<TruthEngine> m_truth;
};


#endif  // SENSORS__SIM__SIMIMU_HPP_
