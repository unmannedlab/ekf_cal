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

#ifndef SENSORS__IMU_HPP_
#define SENSORS__IMU_HPP_

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>

#include "ekf/update/imu_updater.hpp"
#include "sensors/imu_message.hpp"
#include "sensors/sensor.hpp"

///
/// @class IMU
/// @brief IMU Sensor Class
/// @todo Add parameter input/defaults for covariance
/// @todo Bias Stability and Noise process inputs for IMUs
///
class IMU : public Sensor
{
public:
  ///
  /// @brief IMU initialization parameters structure
  /// @todo remove topic from parameters
  ///
  typedef struct Parameters
  {
    std::string name {"Name"};                   ///< @brief IMU name
    std::string topic {"Topic"};                 ///< @brief IMU topic
    bool base_sensor{true};                      ///< @brief IMU base sensor flag
    bool intrinsic{false};                       ///< @brief IMU intrinsic calibration
    double rate{1.0};                            ///< @brief IMU update rate
    Eigen::Vector3d pos_i_in_b {0, 0, 0};        ///< @brief IMU position offset vector
    Eigen::Quaterniond ang_i_to_b {1, 0, 0, 0};  ///< @brief IMU angular offset quaternion
    Eigen::Vector3d acc_bias {0, 0, 0};          ///< @brief IMU accelerometer bias vector
    Eigen::Vector3d omg_bias {0, 0, 0};          ///< @brief IMU gyroscope bias vector
    double acc_bias_stability {0};               ///< @brief IMU accelerometer bias stability
    double omg_bias_stability {0};               ///< @brief IMU gyroscope bias stability
    std::string output_directory {""};           ///< @brief IMU data logging directory
    bool data_logging_on {false};                ///< @brief IMU data logging flag
    bool use_for_prediction {false};             ///< @brief Flag to use measurements for prediction
    /// @brief Initial state variance
    Eigen::VectorXd variance {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
  } Parameters;

  ///
  /// @brief IMU class constructor
  /// @param params IMU sensor parameters
  ///
  explicit IMU(IMU::Parameters params);

  ///
  /// @brief Callback method for IMU measurements
  /// @param imu_message IMU measurement message
  ///
  void Callback(std::shared_ptr<ImuMessage> imu_message);

private:
  bool m_is_base_sensor;
  bool m_is_intrinsic;
  bool m_use_for_prediction;

  ImuUpdater m_imu_updater;
};

#endif  // SENSORS__IMU_HPP_
