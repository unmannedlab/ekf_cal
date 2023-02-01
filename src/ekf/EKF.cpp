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

#include "ekf/Types.hpp"
#include "infrastructure/Logger.hpp"

// initializing instancePointer with NULL
EKF * EKF::instancePointer = NULL;

/// @todo Extend state with process noise (and inputs?)
void EKF::extendState(
  unsigned int sensorStateSize, Eigen::VectorXd sensorState,
  Eigen::MatrixXd sensorCov)
{
  /// @todo Reimplement these lines
  // // Resize State and Covariance
  // m_state.conservativeResize(m_stateSize + sensorStateSize);
  // m_cov.conservativeResize(m_stateSize + sensorStateSize, m_stateSize + sensorStateSize);

  // // Set new states to zero
  // m_state.segment(m_stateSize, sensorStateSize) = sensorState;

  // // Set new covariance to identity and cross-covariance to zero
  // m_cov.block(m_stateSize, m_stateSize, sensorStateSize, sensorStateSize) = sensorCov;
  // m_cov.block(0, m_stateSize, m_stateSize, sensorStateSize).setZero();
  // m_cov.block(m_stateSize, 0, sensorStateSize, m_stateSize).setZero();

  m_stateSize += sensorStateSize;
}

Eigen::MatrixXd EKF::getStateTransition(double dT)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize);
  F.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(3, 6) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dT;
  return F;
}

void EKF::predict(double time)
{
  m_logger->log(LogLevel::DEBUG, "EKF::Predict at t=" + std::to_string(time));

  // Don't predict if time is not initialized
  if (!m_timeInitialized) {
    m_currentTime = time;
    m_timeInitialized = true;
    m_logger->log(LogLevel::INFO, "EKF::Predict initialized time at t=" + std::to_string(time));
    return;
  }

  if (time < m_currentTime) {
    m_logger->log(
      LogLevel::WARN, "Requested prediction to time in the past. Current t=" +
      std::to_string(m_currentTime) + ", Requested t=" +
      std::to_string(time));
    return;
  }

  double dT = time - m_currentTime;

  Eigen::MatrixXd F = getStateTransition(dT);

  /// @todo Reimplement these lines
  /// @todo use convolution function instead for rotation update or RK4
  // m_state = F * m_state;
  // m_cov = F * m_cov * F.transpose() + F * m_processInput * m_processNoise *
  //   m_processInput.transpose() * F.transpose();
  m_currentTime = time;
}

State & EKF::getState()
{
  return m_state;
}

BodyState EKF::getBodyState()
{
  return m_state.bodyState;
}

Eigen::MatrixXd & EKF::getCov()
{
  return m_cov;
}

unsigned int EKF::getStateSize()
{
  /// @todo update size count for map
  unsigned int stateSize = 18 +
    m_state.imuStates.size() * 12 +
    m_state.camStates.size() * 6;
  return stateSize;
}

void EKF::initialize(double timeInit, BodyState bodyStateInit)
{
  m_currentTime = timeInit;
  m_timeInitialized = true;
  m_state.bodyState = bodyStateInit;
}

void EKF::augmentState(unsigned int cameraID)
{
  // Create a copy of the current estimate of the camera pose
  // add pose reference to list per camera
}

void EKF::update_msckf(FeatureTracks featureTracks)
{
  // MSCKF Update
}


void EKF::registerIMU(unsigned int imuID, ImuState imuState)
{
  /// @todo check that id hasn't been used before
  m_state.imuStates[imuID] = imuState;
}

void EKF::registerCamera(unsigned int camID, CamState camState)
{
  /// @todo check that id hasn't been used before
  m_state.camStates[camID] = camState;
}
