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

#include <memory>
#include <string>
#include <vector>


// #include "sensors/ros/RosImu.hpp"
// #include "infrastructure/Logger.hpp"

// initializing instancePointer with NULL
EKF * EKF::instancePointer = NULL;

// EKF::EKF()
// : m_Logger(LogLevel::DEBUG)
// {}

// unsigned int EKF::RegisterSensor(typename Imu::Params params)
// {
//   std::shared_ptr<Imu> sensor_ptr = std::make_shared<Imu>(params);
//   m_mapImu[sensor_ptr->GetId()] = sensor_ptr;
//   sensor_ptr->SetStateStartIndex(m_stateSize);

//   if (sensor_ptr->GetStateSize() != 0) {
//     ExtendState(sensor_ptr->GetStateSize(), sensor_ptr->GetState(), sensor_ptr->GetCov());
//   }
//   return sensor_ptr->GetId();
// }

// unsigned int EKF::RegisterSensor(typename Camera::Params params)
// {
//   std::shared_ptr<Camera> sensor_ptr = std::make_shared<Camera>(params);
//   m_mapCamera[sensor_ptr->GetId()] = sensor_ptr;
//   sensor_ptr->SetStateStartIndex(m_stateSize);

//   if (sensor_ptr->GetStateSize() != 0) {
//     ExtendState(sensor_ptr->GetStateSize(), sensor_ptr->GetState(), sensor_ptr->GetCov());
//   }

//   return sensor_ptr->GetId();
// }

void EKF::ExtendState(
  unsigned int sensorStateSize, Eigen::VectorXd sensorState,
  Eigen::MatrixXd sensorCov)
{
  // Resize State and Covariance
  m_state.conservativeResize(m_stateSize + sensorStateSize);
  m_cov.conservativeResize(m_stateSize + sensorStateSize, m_stateSize + sensorStateSize);

  // Set new states to zero
  m_state.segment(m_stateSize, sensorStateSize) = sensorState;

  // Set new covariance to identity and cross-covariance to zero
  m_cov.block(m_stateSize, m_stateSize, sensorStateSize, sensorStateSize) = sensorCov;
  m_cov.block(0, m_stateSize, m_stateSize, sensorStateSize).setZero();
  m_cov.block(m_stateSize, 0, sensorStateSize, m_stateSize).setZero();

  m_stateSize += sensorStateSize;
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

  // for (auto const & iter : m_mapImu) {
  //   if (iter.second->IsIntrinsic()) {
  //     stateStart = iter.second->GetStateStartIndex();
  //     accBiasStability = iter.second->GetAccBiasStability();
  //     omgBiasStability = iter.second->GetOmgBiasStability();

  //     Q.block<3, 3>(
  //       stateStart + 6,
  //       stateStart + 6) = Eigen::MatrixXd::Identity(3, 3) * accBiasStability;
  //     Q.block<3, 3>(
  //       stateStart + 9,
  //       stateStart + 9) = Eigen::MatrixXd::Identity(3, 3) * omgBiasStability;
  //   }
  // }

  return Q;
}

void EKF::Predict(double time)
{
  // Don't predict if time is not initialized
  if (!m_timeInitialized) {
    m_currentTime = time;
    m_timeInitialized = true;
    return;
  }

  if (time < m_currentTime) {
    // m_Logger->log(LogLevel::WARN, "Requested time in the past");
    return;
  }

  double dT = time - m_currentTime;

  Eigen::MatrixXd F = GetStateTransition(dT);
  Eigen::MatrixXd G = GetProcessInput();
  Eigen::MatrixXd Q = GetProcessNoise();

  /// @todo Should create convolution function to handle quaternion multiplication
  m_state = F * m_state;
  m_cov = F * m_cov * F.transpose() + F * G * Q * G.transpose() * F.transpose();
  m_currentTime = time;
  // Sensor::SetBodyState(m_state.segment<18>(0));
}

Eigen::VectorXd EKF::GetState()
{
  return m_state;
}

Eigen::MatrixXd EKF::GetCov()
{
  return m_cov;
}

unsigned int EKF::GetStateSize()
{
  return m_stateSize;
}

void EKF::InitializeBodyState(double timeInit, Eigen::VectorXd bodyStateInit)
{
  m_currentTime = timeInit;
  m_timeInitialized = true;
  m_state.segment<18>(0) = bodyStateInit;
  // Sensor::SetBodyState(bodyStateInit);
}
