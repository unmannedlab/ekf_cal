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

Eigen::MatrixXd EKF::getStateTransition(double dT)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize);
  F.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(3, 6) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * dT;
  F.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dT;
  return F;
}

void EKF::processModel(double time)
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

  Eigen::VectorXd processUpdate = F * m_state.toVector();
  m_state += processUpdate;
  m_cov = F * m_cov * F.transpose() + F * m_processInput * m_processNoise *
    m_processInput.transpose() * F.transpose();
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

void EKF::initialize(double timeInit, BodyState bodyStateInit)
{
  m_currentTime = timeInit;
  m_timeInitialized = true;
  m_state.bodyState = bodyStateInit;
}

void EKF::augmentState(unsigned int cameraID, unsigned int frameID)
{
  AugmentedState augState;
  augState.frameID = frameID;

  augState.position = m_state.bodyState.position +
    m_state.bodyState.orientation * m_state.camStates[cameraID].position;
  augState.orientation = m_state.camStates[cameraID].orientation * m_state.bodyState.orientation;
  m_state.camStates[cameraID].augmentedStates.push_back(augState);
}

void EKF::update_msckf(FeatureTracks featureTracks)
{
  // MSCKF Update
}

void EKF::registerIMU(unsigned int imuID, ImuState imuState, Eigen::MatrixXd covariance)
{
  /// @todo check that id hasn't been used before
  m_state.imuStates[imuID] = imuState;
  unsigned int imuStateSize = 18 + 12 * m_state.imuStates.size();

  Eigen::MatrixXd newCov = Eigen::MatrixXd::Zero(m_stateSize + 12, m_stateSize + 12);

  newCov.block(0, 0, imuStateSize, imuStateSize) = m_cov.block(0, 0, imuStateSize, imuStateSize);
  newCov.block(
    imuStateSize + 12, imuStateSize + 12, m_stateSize + 12,
    m_stateSize + 12) = m_cov.block(imuStateSize, imuStateSize, m_stateSize, m_stateSize);
  newCov.block(imuStateSize + 12, imuStateSize + 12, 12, 12) = covariance;

  m_cov = newCov;
  m_stateSize += 12;
}

void EKF::registerCamera(unsigned int camID, CamState camState, Eigen::MatrixXd covariance)
{
  /// @todo check that id hasn't been used before
  m_state.camStates[camID] = camState;

  Eigen::MatrixXd newCov = Eigen::MatrixXd::Zero(m_stateSize + 6, m_stateSize + 6);
  newCov.block(0, 0, m_stateSize, m_stateSize) = m_cov;
  newCov.block(m_stateSize, m_stateSize, 6, 6) = covariance;
  m_cov = newCov;
  m_stateSize += 6;
}


unsigned int EKF::getImuStateStartIndex(unsigned int m_id)
{
  unsigned int stateStartIndex = 18;
  for (auto const & imuIter : m_state.imuStates) {
    if (imuIter.first == m_id) {
      break;
    } else {
      stateStartIndex += 12;
    }
  }
}

unsigned int EKF::getCamStateStartIndex(unsigned int m_id)
{
  unsigned int stateStartIndex = 18;
  stateStartIndex += 12 * m_state.imuStates.size();
  for (auto const & camIter : m_state.camStates) {
    if (camIter.first == m_id) {
      break;
    } else {
      stateStartIndex += 6;
    }
  }
}
