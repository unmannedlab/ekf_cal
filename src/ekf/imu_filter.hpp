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
/// @class EKF
/// @brief Calibration EKF class
/// @todo Implement check for correlation coefficients to be between +/- 1
/// @todo Add gravity initialization/check
/// @todo Create generic function to update(r,H,R)
///
class ImuFilter
{
public:
  // ///
  // /// @brief IMU filter class parameters
  // ///
  // typedef struct Parameters
  // {

  // } Parameters;

  ///
  /// @brief IMU filter class constructor
  ///
  ImuFilter();

  ///
  /// @brief Get EKF state as a vector
  /// @return EKF state as a vector
  ///
  Eigen::VectorXd ToVector() const;

  ///
  /// @brief Function to set state using vector
  /// @param state vector for setting body state
  ///
  // void SetState(Eigen::VectorXd state);

  Eigen::VectorXd PredictMeasurement(
    Eigen::Vector3d pos_i_in_b,
    Eigen::Quaterniond ang_i_to_b,
    Eigen::Vector3d acc_bias,
    Eigen::Vector3d omg_bias,
    Eigen::Quaterniond ang_b_to_l
  );

  Eigen::MatrixXd GetMeasurementJacobian(
    Eigen::Vector3d pos_i_in_b,
    Eigen::Quaterniond ang_i_to_b,
    Eigen::Quaterniond ang_b_to_l
  );


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

  Eigen::Vector3d GetAcc();
  Eigen::Vector3d GetAngVel();
  Eigen::Vector3d GetAngAcc();

private:
  Eigen::MatrixXd m_cov;                          ///< @brief IMU state covariance
  Eigen::Vector3d m_acc_in_b{g_gravity};          ///< @brief Body acceleration
  Eigen::Vector3d m_ang_vel_in_b{0.0, 0.0, 0.0};  ///< @brief Body angular rate
  Eigen::Vector3d m_ang_acc_in_b{0.0, 0.0, 0.0};  ///< @brief Body angular acceleration
};

#endif  // EKF__IMU_FILTER_HPP_
