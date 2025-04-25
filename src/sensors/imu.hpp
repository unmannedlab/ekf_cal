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
///
class IMU : public Sensor
{
public:
  ///
  /// @brief IMU initialization parameters structure
  ///
  typedef struct Parameters : public Sensor::Parameters
  {
    bool is_extrinsic{false};                    ///< @brief Extrinsic calibration
    bool is_intrinsic{false};                    ///< @brief Intrinsic calibration
    Eigen::Vector3d pos_i_in_b {0, 0, 0};        ///< @brief Position offset vector
    Eigen::Quaterniond ang_i_to_b {1, 0, 0, 0};  ///< @brief Angular offset quaternion
    Eigen::Vector3d acc_bias {0, 0, 0};          ///< @brief Accelerometer bias vector
    Eigen::Vector3d omg_bias {0, 0, 0};          ///< @brief Gyroscope bias vector
    double pos_stability {1e-9};                 ///< @brief Position stability
    double ang_stability {1e-9};                 ///< @brief Angular stability
    double acc_bias_stability {1e-9};            ///< @brief Accelerometer bias stability
    double omg_bias_stability {1e-9};            ///< @brief Gyroscope bias stability
    double motion_detection_threshold{1.0};      ///< @brief Motion detection chi-Squared threshold
    double noise_scale_factor{1.0};              ///< @brief Stationary noise scale factor
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
  void Callback(ImuMessage & imu_message);

private:
  bool m_is_extrinsic;
  bool m_is_intrinsic;
  std::shared_ptr<EKF> m_ekf;
  ImuUpdater m_imu_updater;
};

#endif  // SENSORS__IMU_HPP_
