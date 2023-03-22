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

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>
#include <vector>

#include "ekf/Types.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "infrastructure/sim/TruthEngine.hpp"
#include "sensors/IMU.hpp"
#include "sensors/Sensor.hpp"
#include "utility/sim/SimRNG.hpp"

class SimImuMessage : public ImuMessage
{
public:
  ///
  /// @brief Define SimImuMessage constructor with ImuMessage's
  ///
  using ImuMessage::ImuMessage;
};

///
/// @class SimIMU
/// @brief Simulated IMU Sensor Class
///
class SimIMU : public IMU
{
public:
  ///
  /// @brief Sim IMU initialization parameters structure
  ///
  typedef struct SimImuParams
  {
    double tBias {0.0};                                 ///< @brief Time offset bias
    double tSkew {1.0};                               ///< @brief Time offset error
    double tError {1e-9};                               ///< @brief Time offset error
    Eigen::Vector3d accBias {0.0, 0.0, 0.0};            ///< @brief Acceleration bias
    Eigen::Vector3d accError {1e-9, 1e-9, 1e-9};        ///< @brief Acceleration error
    Eigen::Vector3d omgBias {0.0, 0.0, 0.0};            ///< @brief Angular rate bias
    Eigen::Vector3d omgError {1e-9, 1e-9, 1e-9};        ///< @brief Angular rate error
    Eigen::Vector3d posOffset {0.0, 0.0, 0.0};          ///< @brief Sensor position offset
    Eigen::Quaterniond angOffset {1.0, 0.0, 0.0, 0.0};  ///< @brief Sensor angular offset
    std::string outputDirectory;                        ///< @brief Output directory path
    IMU::Params imuParams;                              ///< @brief IMU sensor parameters
  } SimImuParams;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truthEngine Truth engine
  ///
  SimIMU(SimImuParams params, std::shared_ptr<TruthEngine> truthEngine);

  ///
  /// @brief Generate simulated IMU messages
  /// @param maxTime Maximum time of generated messages
  ///
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
