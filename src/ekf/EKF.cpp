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

#include "infrastructure/Logger.hpp"

// initializing instancePointer with NULL
EKF * EKF::instancePointer = NULL;

/// @todo Extend state with process noise (and inputs?)
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

void EKF::Predict(double time)
{
  m_logger->log(LogLevel::DEBUG, "EKF::Predict at t=" + std::to_string(time));

  // Don't predict if time is not initialized
  if (!m_timeInitialized) {
    m_currentTime = time;
    m_timeInitialized = true;
    m_logger->log(LogLevel::INFO, "EKF::Predict initialize time at t=" + std::to_string(time));
    return;
  }

  if (time < m_currentTime) {
    m_logger->log(
      LogLevel::WARN, "Requested time in the past. Current t=" +
      std::to_string(time) + ", Requested t=" +
      std::to_string(m_currentTime));
    return;
  }

  double dT = time - m_currentTime;

  Eigen::MatrixXd F = GetStateTransition(dT);

  m_state = F * m_state;
  m_cov = F * m_cov * F.transpose() + F * m_processInput * m_processNoise *
    m_processInput.transpose() * F.transpose();
  m_currentTime = time;
}

Eigen::VectorXd & EKF::GetState()
{
  return m_state;
}

Eigen::VectorXd EKF::GetBodyState()
{
  Eigen::VectorXd bodyState = m_state.segment(0, 18);
  return bodyState;
}

Eigen::MatrixXd & EKF::GetCov()
{
  return m_cov;
}

unsigned int EKF::GetStateSize()
{
  return m_stateSize;
}

void EKF::Initialize(double timeInit, Eigen::VectorXd bodyStateInit)
{
  m_currentTime = timeInit;
  m_timeInitialized = true;
  m_state.segment<18>(0) = bodyStateInit;
}
