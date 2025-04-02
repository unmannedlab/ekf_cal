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

#include "ekf/update/updater.hpp"

#include <memory>

#include "utility/math_helper.hpp"

Updater::Updater(unsigned int sensor_id, std::shared_ptr<DebugLogger> logger)
: m_id(sensor_id), m_logger(logger) {}


void Updater::KalmanUpdate(
  EKF & ekf,
  const Eigen::MatrixXd & jacobian,
  const Eigen::VectorXd & residual,
  const Eigen::MatrixXd & measurement_noise
)
{
  // Calculate Kalman gain
  Eigen::MatrixXd S, G, K, R;
  if (ekf.GetUseRootCovariance()) {
    R = measurement_noise.cwiseSqrt();
    G = QR_r(ekf.m_cov * jacobian.transpose(), R);
    K = ekf.m_cov.transpose() * ekf.m_cov * jacobian.transpose() * (G.transpose() * G).inverse();
  } else {
    R = measurement_noise;
    S = jacobian * ekf.m_cov * jacobian.transpose() + R;
    K = ekf.m_cov * jacobian.transpose() * S.inverse();
  }

  Eigen::VectorXd update = K * residual;
  ekf.m_state += update;

  unsigned int rows = ekf.m_cov.rows();
  unsigned int cols = ekf.m_cov.cols();
  if (ekf.GetUseRootCovariance()) {
    ekf.m_cov = QR_r(
      ekf.m_cov * (Eigen::MatrixXd::Identity(rows, cols) -
      K * jacobian).transpose(), R * K.transpose());
  } else {
    ekf.m_cov =
      (Eigen::MatrixXd::Identity(rows, cols) - K * jacobian) * ekf.m_cov *
      (Eigen::MatrixXd::Identity(rows, cols) - K * jacobian).transpose() +
      K * R * K.transpose();
  }
}
