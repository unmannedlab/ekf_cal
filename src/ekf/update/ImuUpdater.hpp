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

#ifndef EKF__UPDATE__IMUUPDATER_HPP_
#define EKF__UPDATE__IMUUPDATER_HPP_

#include <string>

#include "ekf/EKF.hpp"
#include "ekf/update/Updater.hpp"
#include "infrastructure/DataLogger.hpp"

///
/// @class ImuUpdater
/// @brief EKF Updater Class for IMU Sensors
///
class ImuUpdater : public Updater
{
public:
  ///
  /// @brief IMU EKF Updater Constructor
  /// @param imuID IMU Sensor ID
  /// @param accBiasStability
  /// @param omgBiasStability
  /// @param logFileDirectory
  ///
  ImuUpdater(
    unsigned int imuID,
    double accBiasStability,
    double omgBiasStability,
    std::string logFileDirectory,
    bool dataLoggingOn
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
  Eigen::MatrixXd GetMeasurementJacobian(bool isBaseSensor, bool isIntrinsic);

  ///
  /// @brief EKF update method for IMU measurements
  /// @param time Measurement time
  /// @param acceleration Measured acceleration
  /// @param accelerationCovariance Estimated acceleration error
  /// @param angularRate Measured angular rate
  /// @param angularRateCovariance Estimated angular rate error
  /// @param isBaseSensor switch if this is the base sensor
  /// @param isIntrinsic switch if imu intrinsics should be calibrated
  ///
  void UpdateEKF(
    double time, Eigen::Vector3d acceleration, Eigen::Matrix3d accelerationCovariance,
    Eigen::Vector3d angularRate, Eigen::Matrix3d angularRateCovariance, bool isBaseSensor,
    bool isIntrinsic);

  ///
  /// @brief Refresh internal states with EKF values
  ///
  void RefreshStates();

private:
  Eigen::Vector3d m_body_pos {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_vel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_acc {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_body_ang_pos {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_ang_vel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_ang_acc {0.0, 0.0, 0.0};
  Eigen::Vector3d m_pos_offset {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_offset {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_acc_bias {0.0, 0.0, 0.0};
  Eigen::Vector3d m_omg_bias {0.0, 0.0, 0.0};
  double m_acc_bias_stability;
  double m_omg_bias_stability;
  DataLogger m_data_logger;
};

#endif  // EKF__UPDATE__IMUUPDATER_HPP_
