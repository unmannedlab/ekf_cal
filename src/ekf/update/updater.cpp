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
  Eigen::MatrixXd innovation;
  Eigen::MatrixXd gain;
  Eigen::MatrixXd observation_noise;

  if (ekf.GetUseRootCovariance()) {
    observation_noise = measurement_noise.cwiseSqrt();
    innovation = QR_r(ekf.m_cov * jacobian.transpose(), observation_noise);
    gain = ekf.m_cov.transpose() * ekf.m_cov * jacobian.transpose() *
      (innovation.transpose() * innovation).inverse();
  } else {
    observation_noise = measurement_noise;
    innovation = jacobian * ekf.m_cov * jacobian.transpose() + observation_noise;
    gain = ekf.m_cov * jacobian.transpose() * innovation.inverse();
  }

  Eigen::VectorXd update = gain * residual;
  ekf.m_state += update;

  unsigned int rows = static_cast<unsigned int>(ekf.m_cov.rows());
  unsigned int cols = static_cast<unsigned int>(ekf.m_cov.cols());
  if (ekf.GetUseRootCovariance()) {
    ekf.m_cov = QR_r(
      ekf.m_cov * (Eigen::MatrixXd::Identity(rows, cols) -
      gain * jacobian).transpose(), observation_noise * gain.transpose());
  } else {
    ekf.m_cov =
      (Eigen::MatrixXd::Identity(rows, cols) - gain * jacobian) * ekf.m_cov *
      (Eigen::MatrixXd::Identity(rows, cols) - gain * jacobian).transpose() +
      gain * observation_noise * gain.transpose();
  }
}
