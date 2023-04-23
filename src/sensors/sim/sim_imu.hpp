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


#ifndef SENSORS__SIM__SIM_IMU_HPP_
#define SENSORS__SIM__SIM_IMU_HPP_

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>
#include <vector>

#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/imu.hpp"
#include "sensors/sensor.hpp"
#include "utility/sim/sim_rng.hpp"

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
  typedef struct Parameters
  {
    double time_bias {0.0};                                 ///< @brief Time offset bias
    double time_skew {1.0};                                 ///< @brief Time offset error
    double time_error {1e-9};                               ///< @brief Time offset error
    Eigen::Vector3d acc_bias {0.0, 0.0, 0.0};            ///< @brief Acceleration bias
    Eigen::Vector3d acc_error {1e-9, 1e-9, 1e-9};        ///< @brief Acceleration error
    Eigen::Vector3d omg_bias {0.0, 0.0, 0.0};            ///< @brief Angular rate bias
    Eigen::Vector3d omg_error {1e-9, 1e-9, 1e-9};        ///< @brief Angular rate error
    Eigen::Vector3d pos_offset {0.0, 0.0, 0.0};          ///< @brief Sensor position offset
    Eigen::Quaterniond ang_offset {1.0, 0.0, 0.0, 0.0};  ///< @brief Sensor angular offset
    IMU::Parameters imu_params;                          ///< @brief IMU sensor parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truth_engine Truth engine
  ///
  SimIMU(SimIMU::Parameters params, std::shared_ptr<TruthEngine> truth_engine);

  ///
  /// @brief Generate simulated IMU messages
  /// @param max_time Maximum time of generated messages
  ///
  std::vector<std::shared_ptr<SimImuMessage>> GenerateMessages(double max_time);

private:
  double m_time_bias{0.0};
  double m_time_skew{0.0};
  double m_time_error{1e-9};
  Eigen::Vector3d m_acc_bias{0.0, 0.0, 0.0};
  Eigen::Vector3d m_acc_error{1e-9, 1e-9, 1e-9};
  Eigen::Vector3d m_omg_bias{0.0, 0.0, 0.0};
  Eigen::Vector3d m_omg_error{1e-9, 1e-9, 1e-9};
  Eigen::Vector3d m_pos_offset{0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_offset{1.0, 0.0, 0.0, 0.0};
  SimRNG m_rng;
  std::shared_ptr<TruthEngine> m_truth;
};


#endif  // SENSORS__SIM__SIM_IMU_HPP_
