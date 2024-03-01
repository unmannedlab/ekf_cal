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

#ifndef EKF__UPDATE__IMU_UPDATER_HPP_
#define EKF__UPDATE__IMU_UPDATER_HPP_

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>

#include "ekf/update/updater.hpp"
#include "infrastructure/data_logger.hpp"

///
/// @class ImuUpdater
/// @brief EKF Updater Class for IMU Sensors
///
class ImuUpdater : public Updater
{
public:
  ///
  /// @brief IMU EKF Updater Constructor
  /// @param imu_id IMU Sensor ID
  /// @param is_extrinsic switch if this is the base sensor
  /// @param is_intrinsic switch if imu intrinsics should be calibrated
  /// @param log_file_directory Logging file directory
  /// @param data_logging_on Logging flag
  /// @param data_log_rate Maximum average rate to log data
  ///
  ImuUpdater(
    unsigned int imu_id,
    bool is_extrinsic,
    bool is_intrinsic,
    std::string log_file_directory,
    bool data_logging_on,
    double data_log_rate,
    std::shared_ptr<DebugLogger> logger
  );

  ///
  /// @brief Predict measurement method
  /// @return Predicted measurement vector
  ///
  Eigen::VectorXd PredictMeasurement();

  ///
  /// @brief Measurement Jacobian method
  /// @return Measurement Jacobian matrix
  ///
  Eigen::MatrixXd GetMeasurementJacobian();

  ///
  /// @brief EKF update method for IMU measurements
  /// @param time Measurement time
  /// @param acceleration Measured acceleration
  /// @param acceleration_covariance Estimated acceleration error
  /// @param angular_rate Measured angular rate
  /// @param angular_rate_covariance Estimated angular rate error
  /// @param use_as_predictor switch to use IMU as a prediction step
  ///
  void UpdateEKF(
    std::shared_ptr<EKF> ekf,
    double time, Eigen::Vector3d acceleration, Eigen::Matrix3d acceleration_covariance,
    Eigen::Vector3d angular_rate, Eigen::Matrix3d angular_rate_covariance, bool use_as_predictor);

private:
  Eigen::Vector3d m_body_pos {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_vel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_acc {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_b_to_g {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_ang_vel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_ang_acc {0.0, 0.0, 0.0};
  Eigen::Vector3d m_pos_i_in_g {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_i_to_b {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_acc_bias {0.0, 0.0, 0.0};
  Eigen::Vector3d m_omg_bias {0.0, 0.0, 0.0};
  bool m_is_extrinsic;
  bool m_is_intrinsic;

  DataLogger m_data_logger;
};

#endif  // EKF__UPDATE__IMU_UPDATER_HPP_
