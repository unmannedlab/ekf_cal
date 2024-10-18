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

#include "ekf/imu_filter.hpp"

#include "utility/math_helper.hpp"


ImuFilter::ImuFilter()
{
  m_cov = Eigen::MatrixXd::Identity(g_imu_state_size, g_imu_state_size);
}

Eigen::VectorXd ImuFilter::PredictMeasurement(
  Eigen::Vector3d pos_i_in_b,
  Eigen::Quaterniond ang_i_to_b,
  Eigen::Vector3d acc_bias,
  Eigen::Vector3d omg_bias,
  Eigen::Quaterniond ang_b_to_l
)
{
  Eigen::VectorXd predicted_measurement(6);

  // Transform acceleration to IMU location
  Eigen::Vector3d imu_acc_b =
    ang_b_to_l.inverse() * m_acc_in_b +
    m_ang_acc_in_b.cross(pos_i_in_b) +
    m_ang_vel_in_b.cross((m_ang_vel_in_b.cross(pos_i_in_b)));

  // Rotate measurements in place
  predicted_measurement.segment<3>(0) = acc_bias + ang_i_to_b.inverse() * imu_acc_b;
  predicted_measurement.segment<3>(3) = omg_bias + ang_i_to_b.inverse() * m_ang_vel_in_b;

  return predicted_measurement;
}

Eigen::MatrixXd ImuFilter::GetMeasurementJacobian(
  Eigen::Vector3d pos_i_in_b,
  Eigen::Quaterniond ang_i_to_b,
  Eigen::Quaterniond ang_b_to_l
)
{
  Eigen::MatrixXd measurement_jacobian = Eigen::MatrixXd::Zero(6, g_imu_state_size);

  // Body Acceleration
  measurement_jacobian.block<3, 3>(0, 0) = ang_i_to_b.inverse().toRotationMatrix() *
    ang_b_to_l.inverse().toRotationMatrix();

  // Body Angular Velocity
  measurement_jacobian.block<3, 3>(0, 3) = ang_i_to_b.inverse().toRotationMatrix() * (
    SkewSymmetric(m_ang_vel_in_b) *
    SkewSymmetric(pos_i_in_b).transpose() +
    SkewSymmetric(m_ang_vel_in_b.cross(pos_i_in_b)).transpose()
  );

  // Body Angular Acceleration
  measurement_jacobian.block<3, 3>(0, 6) = ang_i_to_b.inverse().toRotationMatrix() *
    SkewSymmetric(pos_i_in_b);

  // Body Angular Velocity
  measurement_jacobian.block<3, 3>(3, 3) = ang_i_to_b.inverse().toRotationMatrix() *
    ang_b_to_l.inverse().toRotationMatrix();

  return measurement_jacobian;
}

void ImuFilter::Update(
  Eigen::Vector3d acceleration,
  Eigen::Vector3d angular_rate,
  Eigen::Matrix3d acceleration_covariance,
  Eigen::Matrix3d angular_rate_covariance,
  Eigen::Vector3d pos_i_in_b,
  Eigen::Quaterniond ang_i_to_b,
  Eigen::Vector3d acc_bias,
  Eigen::Vector3d omg_bias,
  Eigen::Quaterniond ang_b_to_l
)
{
  if (m_imu_count == 1) {
    m_acc_in_b = ang_i_to_b * acceleration;
    m_ang_vel_in_b = ang_i_to_b * angular_rate;
    m_ang_acc_in_b = Eigen::Vector3d::Zero();
  } else {
    Eigen::VectorXd z(acceleration.size() + angular_rate.size());
    z.segment<3>(0) = acceleration;
    z.segment<3>(3) = angular_rate;

    Eigen::VectorXd z_pred =
      PredictMeasurement(pos_i_in_b, ang_i_to_b, acc_bias, omg_bias, ang_b_to_l);
    Eigen::VectorXd resid = z - z_pred;

    Eigen::MatrixXd H = GetMeasurementJacobian(pos_i_in_b, ang_i_to_b, ang_b_to_l);

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
    R.block<3, 3>(0, 0) = acceleration_covariance;
    R.block<3, 3>(3, 3) = angular_rate_covariance;

    // Apply Kalman update
    Eigen::MatrixXd S, G, K;
    R = R.cwiseSqrt();
    G = QR_r(m_cov * H.transpose(), R);
    K = (G.inverse() * ((G.transpose()).inverse() * H) * m_cov.transpose() * m_cov).transpose();

    Eigen::VectorXd update = K * resid;

    m_acc_in_b += update.segment<3>(0);
    m_ang_vel_in_b += update.segment<3>(3);
    m_ang_acc_in_b += update.segment<3>(6);

    m_cov = QR_r(
      m_cov * (Eigen::MatrixXd::Identity(g_imu_state_size, g_imu_state_size) - K * H).transpose(),
      R * K.transpose());
  }
}

Eigen::Vector3d ImuFilter::GetAcc()
{
  return m_acc_in_b;
}

Eigen::Vector3d ImuFilter::GetAngVel()
{
  return m_ang_vel_in_b;
}

Eigen::Vector3d ImuFilter::GetAngAcc()
{
  return m_ang_acc_in_b;
}

void ImuFilter::SetImuCount(unsigned int imu_count)
{
  m_imu_count = imu_count;
}
