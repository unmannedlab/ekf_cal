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
  /// @param data_log_rate Maximum average rate to log data
  /// @param logger Debug logger pointer
  ///
  ImuUpdater(
    unsigned int imu_id,
    bool is_extrinsic,
    bool is_intrinsic,
    const std::string & log_file_directory,
    double data_log_rate,
    std::shared_ptr<DebugLogger> logger
  );

  ///
  /// @brief Predict acceleration measurement
  /// @param ekf EKF address
  /// @return Predicted measurement vector
  ///
  Eigen::VectorXd PredictMeasurement(EKF & ekf) const;

  ///
  /// @brief Zero acceleration Jacobian method
  /// @param ekf EKF address
  /// @return Measurement Jacobian matrix
  ///
  Eigen::MatrixXd GetZeroAccelerationJacobian(EKF & ekf) const;

  ///
  /// @brief EKF update method for IMU measurements
  /// @param ekf EKF address
  /// @param time Measurement time
  /// @param acceleration Measured acceleration
  /// @param acceleration_covariance Estimated acceleration error
  /// @param angular_rate Measured angular rate
  /// @param angular_rate_covariance Estimated angular rate error
  ///
  void UpdateEKF(
    EKF & ekf,
    const double time,
    const Eigen::Vector3d & acceleration,
    const Eigen::Matrix3d & acceleration_covariance,
    const Eigen::Vector3d & angular_rate,
    const Eigen::Matrix3d & angular_rate_covariance
  );

  ///
  /// @brief Check for and perform a zero-acceleration update
  /// @param ekf EKF address
  /// @param local_time Measurement in local EKF time
  /// @param acceleration Measured acceleration
  /// @param acceleration_covariance Estimated acceleration error
  /// @param angular_rate Measured angular rate
  /// @param angular_rate_covariance Estimated angular rate error
  ///
  bool ZeroAccelerationUpdate(
    EKF & ekf,
    double local_time,
    const Eigen::Vector3d & acceleration,
    const Eigen::Matrix3d & acceleration_covariance,
    const Eigen::Vector3d & angular_rate,
    const Eigen::Matrix3d & angular_rate_covariance
  );

  ///
  /// @brief Calculate IMU measurement jacobian
  /// @param ekf EKF address
  /// @return Measurement jacobian
  ///
  Eigen::MatrixXd GetMeasurementJacobian(EKF & ekf) const;

  ///
  /// @brief IMU state angular rate updater
  /// @param ekf EKF address
  /// @param angular_rate Angular rate measurement
  /// @param angular_rate_covariance Angular rate covariance
  ///
  void AngularUpdate(
    EKF & ekf,
    const Eigen::Vector3d & angular_rate,
    const Eigen::Matrix3d & angular_rate_covariance
  ) const;

private:
  Eigen::Vector3d m_pos_i_in_g {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_i_to_b {1.0, 0.0, 0.0, 0.0};
  bool m_is_extrinsic;
  bool m_is_intrinsic;
  bool m_initial_motion_detected{false};
  // bool m_correct_heading_rotation{true};

  DataLogger m_data_logger;
};

#endif  // EKF__UPDATE__IMU_UPDATER_HPP_
