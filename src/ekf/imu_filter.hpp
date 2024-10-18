// Copyright 2024 Jacob Hartzer
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

#ifndef EKF__IMU_FILTER_HPP_
#define EKF__IMU_FILTER_HPP_

#include <eigen3/Eigen/Eigen>

#include "ekf/constants.hpp"

///
/// @class ImuFilter
/// @brief Class for filtering streams of IMU measurements into a single virtual IMU
///
class ImuFilter
{
public:
  ///
  /// @brief IMU filter class constructor
  ///
  ImuFilter();

  ///
  /// @brief Set the number of IMUs being filter
  /// @param imu_count Current IMU count
  ///
  void SetImuCount(unsigned int imu_count);

  ///
  /// @brief Predict acceleration measurement
  /// @param pos_i_in_b IMU position in the body frame
  /// @param ang_i_to_b IMU orientation to the body frame
  /// @param acc_bias Accelerometer bias
  /// @param omg_bias Angular rate bias
  /// @param ang_b_to_l Body orientation to the local frame
  /// @return Predicted acceleration measurement
  ///
  Eigen::VectorXd PredictMeasurement(
    Eigen::Vector3d pos_i_in_b,
    Eigen::Quaterniond ang_i_to_b,
    Eigen::Vector3d acc_bias,
    Eigen::Vector3d omg_bias,
    Eigen::Quaterniond ang_b_to_l
  );

  ///
  /// @brief Calculate measurement jacobian
  /// @param pos_i_in_b IMU position in the body frame
  /// @param ang_i_to_b IMU orientation to the body frame
  /// @param ang_b_to_l Body orientation to the local frame
  /// @return Measurement jacobian
  ///
  Eigen::MatrixXd GetMeasurementJacobian(
    Eigen::Vector3d pos_i_in_b,
    Eigen::Quaterniond ang_i_to_b,
    Eigen::Quaterniond ang_b_to_l
  );

  ///
  /// @brief IMU state updater
  /// @param acceleration Acceleration measurement
  /// @param angular_rate Angular rate measurement
  /// @param acceleration_covariance Acceleration covariance
  /// @param angular_rate_covariance Angular rate covariance
  /// @param pos_i_in_b IMU position in the body frame
  /// @param ang_i_to_b IMU orientation to the body frame
  /// @param acc_bias Accelerometer bias
  /// @param omg_bias Angular rate bias
  /// @param ang_b_to_l Body orientation to the local frame
  ///
  void Update(
    Eigen::Vector3d acceleration,
    Eigen::Vector3d angular_rate,
    Eigen::Matrix3d acceleration_covariance,
    Eigen::Matrix3d angular_rate_covariance,
    Eigen::Vector3d pos_i_in_b,
    Eigen::Quaterniond ang_i_to_b,
    Eigen::Vector3d acc_bias,
    Eigen::Vector3d omg_bias,
    Eigen::Quaterniond ang_b_to_l
  );

  ///
  /// @brief Filtered acceleration getter
  /// @return Filtered acceleration
  ///
  Eigen::Vector3d GetAcc();

  ///
  /// @brief Filtered angular rate getter
  /// @return Filtered angular rate
  ///
  Eigen::Vector3d GetAngVel();

  ///
  /// @brief Filtered angular acceleration getter
  /// @return Filtered angular acceleration
  ///
  Eigen::Vector3d GetAngAcc();

private:
  unsigned int m_imu_count{0};                       ///< @brief Number of IMUs in filter
  Eigen::MatrixXd m_cov;                             ///< @brief IMU state covariance
  Eigen::Vector3d m_acc_in_b{g_gravity};             ///< @brief Body acceleration
  Eigen::Vector3d m_ang_vel_in_b{0.0, 0.0, 0.0};     ///< @brief Body angular rate
  Eigen::Vector3d m_ang_acc_in_b{0.0, 0.0, 0.0};     ///< @brief Body angular acceleration
};

#endif  // EKF__IMU_FILTER_HPP_
