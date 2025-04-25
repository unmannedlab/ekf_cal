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
#include <vector>

#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/imu.hpp"
#include "sensors/sim/sim_imu_message.hpp"
#include "sensors/sim/sim_sensor.hpp"
#include "utility/sim/sim_rng.hpp"


///
/// @class SimIMU
/// @brief Simulated IMU Sensor Class
///
class SimIMU : public IMU, public SimSensor
{
public:
  ///
  /// @brief Sim IMU initialization parameters structure
  ///
  typedef struct Parameters : public SimSensor::Parameters
  {
    Eigen::Vector3d acc_error {0.0, 0.0, 0.0};       ///< @brief Acceleration error
    Eigen::Vector3d omg_error {0.0, 0.0, 0.0};       ///< @brief Angular rate error
    Eigen::Vector3d pos_error {0.0, 0.0, 0.0};       ///< @brief Position offset error
    Eigen::Vector3d ang_error {0.0, 0.0, 0.0};       ///< @brief Angular offset error
    Eigen::Vector3d acc_bias_error {0.0, 0.0, 0.0};  ///< @brief Acceleration bias
    Eigen::Vector3d omg_bias_error {0.0, 0.0, 0.0};  ///< @brief Angular rate bias
    IMU::Parameters imu_params;                      ///< @brief IMU sensor parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truth_engine Truth engine
  ///
  SimIMU(SimIMU::Parameters params, std::shared_ptr<TruthEngine> truth_engine);

  ///
  /// @brief Generate simulated IMU messages
  /// @return Generated IMU messages
  ///
  std::vector<std::shared_ptr<SimImuMessage>> GenerateMessages() const;

private:
  Eigen::Vector3d m_acc_error;
  Eigen::Vector3d m_omg_error;
  Eigen::Vector3d m_pos_error;
  Eigen::Vector3d m_ang_error;
  Eigen::Vector3d m_acc_bias_error;
  Eigen::Vector3d m_omg_bias_error;
};


#endif  // SENSORS__SIM__SIM_IMU_HPP_
