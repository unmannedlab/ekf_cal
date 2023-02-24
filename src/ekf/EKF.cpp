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
#include <sstream>

#include "ekf/Types.hpp"
#include "infrastructure/Logger.hpp"
#include "utility/MathHelper.hpp"

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

  if (time <= m_currentTime) {
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

ImuState EKF::getImuState(unsigned int imuID)
{
  return m_state.imuStates[imuID];
}

CamState EKF::getCamState(unsigned int camID)
{
  return m_state.camStates[camID];
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
  unsigned int imuStateSize = 18 + 12 * m_state.imuStates.size();

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
  m_processNoise = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize) * 1e-3;
  m_processInput = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize);
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
  m_processNoise = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize) * 1e-3;
  m_processInput = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize);
}

/// @todo Replace this lookup with a map
unsigned int EKF::getImuStateStartIndex(unsigned int imuID)
{
  unsigned int stateStartIndex = 18;
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
  unsigned int stateStartIndex = 18;
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
  m_logger->log(LogLevel::DEBUG, "imu size " + std::to_string(m_state.imuStates.size()));

  unsigned int stateStartIndex = 18;
  stateStartIndex += (12 * m_state.imuStates.size());
  for (auto const & camIter : m_state.camStates) {
    stateStartIndex += 6;
    m_logger->log(LogLevel::DEBUG, "camera " + std::to_string(camIter.first));
    for (auto const & augIter : camIter.second.augmentedStates) {
      m_logger->log(LogLevel::DEBUG, "frame " + std::to_string(augIter.frameID));
      if (augIter.frameID == frameID) {
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

  m_logger->log(
    LogLevel::DEBUG, "Augment State 1: " + std::to_string(
      cameraID) + ", " + std::to_string(frameID));

  /// @todo Augment covariance with Jacobian

  unsigned int augStateStart = getAugStateStartIndex(cameraID, frameID);
  Eigen::MatrixXd newCov = Eigen::MatrixXd::Zero(m_stateSize + 12, m_stateSize + 12);


  m_logger->log(
    LogLevel::DEBUG, "Augment State 2:" + std::to_string(
      augStateStart) + ", " + std::to_string(m_stateSize));

  /// @todo Math helper function to insert sub-matrix block
  newCov.block(
    0, 0, augStateStart,
    augStateStart) = m_cov.block(0, 0, augStateStart, augStateStart);
  m_logger->log(LogLevel::DEBUG, "Augment State 3");
  newCov.block(augStateStart, augStateStart, 12, 12) = Eigen::MatrixXd::Identity(12, 12);
  m_logger->log(LogLevel::DEBUG, "Augment State 4");
  newCov.block(
    augStateStart + 12, augStateStart + 12, m_stateSize - augStateStart,
    m_stateSize - augStateStart) = m_cov.block(
    augStateStart, augStateStart,
    m_stateSize - augStateStart,
    m_stateSize - augStateStart);

  m_cov = newCov;
  m_stateSize += 12;
  m_processNoise = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize) * 1e-3;
  m_processInput = Eigen::MatrixXd::Identity(m_stateSize, m_stateSize);
}
