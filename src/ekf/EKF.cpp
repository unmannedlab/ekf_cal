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

#include "ekf/EKF.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "ekf/sensors/Imu.hpp"

EKF::EKF() {}

unsigned int EKF::RegisterSensor(typename Imu::Params params)
{
  std::shared_ptr<Imu> sensor_ptr = std::make_shared<Imu>(params);
  m_mapImu[sensor_ptr->GetId()] = sensor_ptr;
  sensor_ptr->SetStateStartIndex(m_stateSize);

  m_stateSize += sensor_ptr->GetStateSize();

  m_state.conservativeResize(m_stateSize);
  m_cov.conservativeResize(m_stateSize, m_stateSize);

  return sensor_ptr->GetId();
}

unsigned int EKF::RegisterSensor(typename Camera::Params params)
{
  std::shared_ptr<Camera> sensor_ptr = std::make_shared<Camera>(params);
  m_mapCamera[sensor_ptr->GetId()] = sensor_ptr;
  sensor_ptr->SetStateStartIndex(m_stateSize);

  m_stateSize += sensor_ptr->GetStateSize();

  m_state.conservativeResize(m_stateSize);
  m_cov.conservativeResize(m_stateSize, m_stateSize);

  return sensor_ptr->GetId();
}

unsigned int EKF::RegisterSensor(typename Lidar::Params params)
{
  std::shared_ptr<Lidar> sensor_ptr = std::make_shared<Lidar>(params);
  m_mapLidar[sensor_ptr->GetId()] = sensor_ptr;
  sensor_ptr->SetStateStartIndex(m_stateSize);

  m_stateSize += sensor_ptr->GetStateSize();

  m_state.conservativeResize(m_stateSize);
  m_cov.conservativeResize(m_stateSize, m_stateSize);

  return sensor_ptr->GetId();
}

Eigen::MatrixXd EKF::GetStateTransition(double dT)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize);
  F.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(3, 6) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dT;
  return F;
}

Eigen::MatrixXd EKF::GetProcessInput()
{
  Eigen::MatrixXd G = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize);
  return G;
}

Eigen::MatrixXd EKF::GetProcessNoise()
{
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(m_stateSize, m_stateSize);
  unsigned int stateStart {0};
  double accBiasStability {0};
  double omgBiasStability {0};

  for (auto const & iter : m_mapImu) {
    if (iter.second->IsIntrinsic()) {
      stateStart = iter.second->GetStateStartIndex();
      accBiasStability = iter.second->GetAccBiasStability();
      omgBiasStability = iter.second->GetOmgBiasStability();

      Q.block<3, 3>(
        stateStart + 6,
        stateStart + 6) = Eigen::MatrixXd::Identity(3, 3) * accBiasStability;
      Q.block<3, 3>(
        stateStart + 9,
        stateStart + 9) = Eigen::MatrixXd::Identity(3, 3) * omgBiasStability;
    }
  }

  return Q;
}

void EKF::Predict(double time)
{
  if (time < m_currentTime) {
    RCLCPP_WARN(rclcpp::get_logger("EKF"), "Requested time in the past");
    return;
  }
  double dT = m_currentTime - time;

  Eigen::MatrixXd F = GetStateTransition(dT);
  Eigen::MatrixXd G = GetProcessInput();
  Eigen::MatrixXd Q = GetProcessNoise();

  m_state = F * m_state;
  m_cov = F * m_cov * F + F * G * Q * G * F;
  m_currentTime = time;
}

void EKF::ImuCallback(
  unsigned int id, double time, Eigen::Vector3d acceleration,
  Eigen::Matrix3d accelerationCovariance, Eigen::Vector3d angularRate,
  Eigen::Matrix3d angularRateCovariance)
{
  auto iter = m_mapImu.find(id);
  Predict(time);

  Eigen::VectorXd z(acceleration.size() + angularRate.size());
  z << acceleration, angularRate;
  Eigen::VectorXd z_pred = iter->second->PredictMeasurement();
  Eigen::VectorXd resid = z - z_pred;

  /// @todo fix this jacobian
  unsigned int stateSize = iter->second->GetStateSize();
  unsigned int stateStartIndex = iter->second->GetStateStartIndex();
  Eigen::MatrixXd subH = iter->second->GetMeasurementJacobian();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, m_stateSize);
  // H.block<6, 18>(0, 0) = subH.block<6, 18>(0, 0);
  // H.block<6, stateSize>(0, stateStartIndex) = subH.block<6, stateSize>(0, 18);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
  R.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * accelerationCovariance;
  R.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * angularRateCovariance;

  Eigen::MatrixXd S = H * m_cov * H.transpose() + R;
  Eigen::MatrixXd K = m_cov * H.transpose() * S.inverse();

  m_state = m_state + K * resid;
  m_cov = (Eigen::MatrixXd::Identity(6, 6) - K * H) * m_cov;

  iter->second->SetState(m_state.segment(stateStartIndex, stateSize));
}

void EKF::CameraCallback(unsigned int id, double time)
{
  RCLCPP_WARN(
    rclcpp::get_logger("EKF"),
    "Camera callback for '%u' at '%g' not implemented", id, time);
}

void EKF::LidarCallback(unsigned int id, double time)
{
  RCLCPP_WARN(
    rclcpp::get_logger("EKF"),
    "Lidar callback for '%u' at '%g' not implemented", id, time);
}
