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

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include "ekf/Types.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "utility/MathHelper.hpp"

// initializing instancePointer with NULL
EKF * EKF::instancePointer = NULL;

Eigen::MatrixXd EKF::getStateTransition(double dT)
{
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(BODY_STATE_SIZE, BODY_STATE_SIZE);
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
    m_logger->log(
      LogLevel::INFO,
      "EKF::Predict initialized time at t=" + std::to_string(time));
    return;
  }

  if (time <= m_currentTime) {
    m_logger->log(
      LogLevel::WARN, "Requested prediction to time in the past. Current t=" +
      std::to_string(m_currentTime) + ", Requested t=" +
      std::to_string(time));
    return;
  }

  double dT = time - m_currentTime;

  Eigen::MatrixXd F = getStateTransition(dT);

  Eigen::VectorXd processUpdate = F * m_state.bodyState.toVector();

  m_state.bodyState.SetState(processUpdate);

  // Process input matrix is just identity
  m_cov.block<BODY_STATE_SIZE, BODY_STATE_SIZE>(0, 0) =
    F * (m_cov.block<BODY_STATE_SIZE, BODY_STATE_SIZE>(0, 0) + m_processNoise) * F.transpose();

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

ImuState EKF::getImuState(unsigned int imuID)
{
  return m_state.imuStates[imuID];
}

CamState EKF::getCamState(unsigned int camID)
{
  return m_state.camStates[camID];
}

unsigned int EKF::getImuCount()
{
  return m_state.imuStates.size();
}

unsigned int EKF::getCamCount()
{
  return m_state.camStates.size();
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

void EKF::registerIMU(unsigned int imuID, ImuState imuState, Eigen::MatrixXd covariance)
{
  /// @todo check that id hasn't been used before
  /// @todo replace 12s with constants from IMU class
  unsigned int imuStateSize = BODY_STATE_SIZE + 12 * m_state.imuStates.size();

  Eigen::MatrixXd newCov = Eigen::MatrixXd::Zero(m_stateSize + 12, m_stateSize + 12);

  newCov.block(0, 0, imuStateSize, imuStateSize) = m_cov.block(0, 0, imuStateSize, imuStateSize);
  newCov.block(imuStateSize, imuStateSize, 12, 12) = covariance;
  newCov.block(
    imuStateSize + 12, imuStateSize + 12, m_stateSize - imuStateSize,
    m_stateSize - imuStateSize) = m_cov.block(
    imuStateSize, imuStateSize,
    m_stateSize - imuStateSize,
    m_stateSize - imuStateSize);

  m_cov = newCov;
  m_state.imuStates[imuID] = imuState;
  m_stateSize += 12;

  m_logger->log(
    LogLevel::DEBUG, "Register IMU: " + std::to_string(
      imuID) + ", stateSize: " + std::to_string(m_stateSize));
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

  m_logger->log(
    LogLevel::DEBUG, "Register Cam: " + std::to_string(
      camID) + ", stateSize: " + std::to_string(m_stateSize));
}

/// @todo Replace this lookup with a map
unsigned int EKF::getImuStateStartIndex(unsigned int imuID)
{
  unsigned int stateStartIndex = BODY_STATE_SIZE;
  for (auto const & imuIter : m_state.imuStates) {
    if (imuIter.first == imuID) {
      break;
    } else {
      stateStartIndex += 12;
    }
  }
  return stateStartIndex;
}

/// @todo Replace this lookup with a map
unsigned int EKF::getCamStateStartIndex(unsigned int camID)
{
  unsigned int stateStartIndex = BODY_STATE_SIZE;
  stateStartIndex += 12 * m_state.imuStates.size();
  for (auto const & camIter : m_state.camStates) {
    if (camIter.first == camID) {
      break;
    } else {
      stateStartIndex += 6 + 12 * camIter.second.augmentedStates.size();
    }
  }
  return stateStartIndex;
}

/// @todo Replace this lookup with a map
unsigned int EKF::getAugStateStartIndex(unsigned int camID, unsigned int frameID)
{
  unsigned int stateStartIndex = BODY_STATE_SIZE;
  stateStartIndex += (12 * m_state.imuStates.size());
  for (auto const & camIter : m_state.camStates) {
    stateStartIndex += 6;
    for (auto const & augIter : camIter.second.augmentedStates) {
      if ((camIter.first == camID) && (augIter.frameID == frameID)) {
        return stateStartIndex;
      } else {
        stateStartIndex += 12;
      }
    }
  }

  return stateStartIndex;
}


void EKF::augmentState(unsigned int cameraID, unsigned int frameID)
{
  AugmentedState augState;
  augState.frameID = frameID;

  augState.imuPosition = m_state.bodyState.position;
  augState.imuOrientation = m_state.bodyState.orientation;
  augState.position = m_state.bodyState.position +
    m_state.bodyState.orientation * m_state.camStates[cameraID].position;
  augState.orientation = m_state.camStates[cameraID].orientation * m_state.bodyState.orientation;
  m_state.camStates[cameraID].augmentedStates.push_back(augState);

  /// @todo Augment covariance with Jacobian

  unsigned int augStateStart = getAugStateStartIndex(cameraID, frameID);
  Eigen::MatrixXd newCov = Eigen::MatrixXd::Zero(m_stateSize + 12, m_stateSize + 12);

  /// @todo Math helper function to insert sub-matrix block
  newCov.block(
    0, 0, augStateStart,
    augStateStart) = m_cov.block(0, 0, augStateStart, augStateStart);
  newCov.block(augStateStart, augStateStart, 12, 12) = Eigen::MatrixXd::Identity(12, 12);
  newCov.block(
    augStateStart + 12, augStateStart + 12, m_stateSize - augStateStart,
    m_stateSize - augStateStart) = m_cov.block(
    augStateStart, augStateStart,
    m_stateSize - augStateStart,
    m_stateSize - augStateStart);

  m_cov = newCov;
  m_stateSize += 12;
}
