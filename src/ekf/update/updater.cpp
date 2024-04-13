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

Updater::Updater(unsigned int sensor_id, std::shared_ptr<DebugLogger> logger)
: m_id(sensor_id), m_logger(logger) {}


void Updater::KalmanUpdate(
  std::shared_ptr<EKF> ekf,
  const Eigen::MatrixXd & jacobian,
  const Eigen::VectorXd & residual,
  const Eigen::MatrixXd & measurement_noise
)
{
  // Calculate Kalman gain
  Eigen::MatrixXd S = jacobian * ekf->GetCov() * jacobian.transpose() + measurement_noise;
  Eigen::MatrixXd K = ekf->GetCov() * jacobian.transpose() * S.inverse();
  Eigen::VectorXd update = K * residual;

  ekf->GetState() += update;
  ekf->GetCov() =
    (Eigen::MatrixXd::Identity(update.size(), update.size()) - K * jacobian) * ekf->GetCov() *
    (Eigen::MatrixXd::Identity(update.size(), update.size()) - K * jacobian).transpose() +
    K * measurement_noise * K.transpose();
}
