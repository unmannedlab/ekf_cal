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

  augState.imuPosition = m_state.bodyState.position;
  augState.imuOrientation = m_state.bodyState.orientation;
  augState.position = m_state.bodyState.position +
    m_state.bodyState.orientation * m_state.camStates[cameraID].position;
  augState.orientation = m_state.camStates[cameraID].orientation * m_state.bodyState.orientation;
  m_state.camStates[cameraID].augmentedStates.push_back(augState);

  /// @todo Augment covariance with Jacobian
}

AugmentedState EKF::matchState(std::vector<AugmentedState> augmentedStates, unsigned int frameID)
{
  AugmentedState augStateMatch;

  for (auto & augState : augmentedStates) {
    if (augState.frameID == frameID) {
      augStateMatch = augState;
      break;
    }
  }

  return augStateMatch;
}

void EKF::update_msckf(unsigned int cameraID, FeatureTracks featureTracks)
{
  // MSCKF Update
  for (auto & featureTrack : featureTracks) {
    const std::vector<AugmentedState> * augStates = &m_state.camStates[cameraID].augmentedStates;

    AugmentedState augStateMatch = matchState(*augStates, featureTrack[0].frameID);

    // 3D Cartesian Triangulation
    /// @todo Move into separate function
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    unsigned int total_meas = 1;
    unsigned int total_hx = 6;


    const Eigen::Matrix<double, 3, 3> & R_GtoA = augStateMatch.orientation.toRotationMatrix();
    const Eigen::Matrix<double, 3, 1> & p_AinG = augStateMatch.position;

    for (unsigned int i = 1; i < featureTrack.size(); ++i) {
      augStateMatch = matchState(*augStates, featureTrack[i].frameID);

      const Eigen::Matrix<double, 3, 3> R_GtoCi = augStateMatch.orientation.toRotationMatrix();
      const Eigen::Vector3d p_CiinG = augStateMatch.position;

      // Convert current position relative to anchor
      Eigen::Matrix3d R_AtoCi = R_GtoCi * R_GtoA.transpose();
      Eigen::Vector3d p_CiinA = R_GtoA * (p_CiinG - p_AinG);

      // Get the UV coordinate normal
      Eigen::Vector3d b_i;
      b_i << featureTrack[i].keyPoint.pt.x, featureTrack[i].keyPoint.pt.y, 1;
      b_i = R_AtoCi.transpose() * b_i;
      b_i = b_i / b_i.norm();
      Eigen::Matrix3d bSkew = skewSymmetric(b_i);
      Eigen::Matrix3d Ai = bSkew.transpose() * bSkew;
      A += Ai;
      b += Ai * p_CiinA;
      total_meas += 1;
      total_hx += 6;
    }

    // Solve linear triangulation for 3D cartesian estimate of feature position
    Eigen::VectorXd p_FinA = A.colPivHouseholderQr().solve(b);

    /// @todo Additional non-linear optimization

    /// @todo Get Jacobian
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;

    // Calculate the position of this feature in the global frame
    // Get calibration for our anchor camera
    Eigen::Matrix3d R_ItoC = augStates->at(0).orientation.toRotationMatrix();
    Eigen::Vector3d p_IinC = augStates->at(0).position;
    // Anchor pose orientation and position
    Eigen::Matrix3d R_GtoI = augStates->at(0).imuOrientation.toRotationMatrix();
    Eigen::Vector3d p_IinG = augStates->at(0).imuPosition;
    // Feature in the global frame
    Eigen::Vector3d p_FinG = R_GtoI.transpose() * R_ItoC.transpose() * (p_FinA - p_IinC) +
      p_IinG;

    // Allocate our residual and Jacobians
    int c = 0;
    int jacobsize = 3;
    res = Eigen::VectorXd::Zero(2 * total_meas);
    H_f = Eigen::MatrixXd::Zero(2 * total_meas, jacobsize);
    H_x = Eigen::MatrixXd::Zero(2 * total_meas, total_hx);

    // Derivative of p_FinG in respect to feature representation.
    // This only needs to be computed once and thus we pull it out of the loop
    Eigen::MatrixXd dPFG_dLambda;
    Eigen::Matrix3d R_CtoG = R_GtoI.transpose() * R_ItoC.transpose();

    // Jacobian for our anchor pose
    Eigen::Matrix<double, 3, 6> H_anc;
    H_anc.block(
      0, 0, 3,
      3).noalias() = -R_GtoI.transpose() * skewSymmetric(R_ItoC.transpose() * (p_FinA - p_IinC));
    H_anc.block(0, 3, 3, 3).setIdentity();

    // Add anchor Jacobians to our return vector

    // Get calibration Jacobians (for anchor clone)
    Eigen::Matrix<double, 3, 6> H_calib;
    H_calib.block(0, 0, 3, 3).noalias() = -R_CtoG * skewSymmetric(p_FinA - p_IinC);
    H_calib.block(0, 3, 3, 3) = -R_CtoG;

    dPFG_dLambda = R_CtoG;


    // Loop through each camera for this feature
    for (unsigned int i = 1; i < featureTrack.size(); ++i) {
      augStateMatch = matchState(*augStates, featureTrack[i].frameID);

      // Our calibration between the IMU and CAMi frames
      Eigen::Matrix3d R_ItoC = augStateMatch.orientation.toRotationMatrix();
      Eigen::Vector3d p_IinC = augStateMatch.position;

      // Get current IMU clone state
      Eigen::Matrix3d R_GtoIi = augStateMatch.imuOrientation.toRotationMatrix();
      Eigen::Vector3d p_Ii_inG = augStateMatch.imuPosition;

      // Get current feature in the IMU
      Eigen::Vector3d p_FinIi = R_GtoIi * (p_FinG - p_Ii_inG);

      // Project the current feature into the current frame of reference
      Eigen::Vector3d p_FinCi = R_ItoC * p_FinIi + p_IinC;
      Eigen::Vector2d uv_norm;
      uv_norm << p_FinCi(0) / p_FinCi(2), p_FinCi(1) / p_FinCi(2);

      // Normalized coordinates in respect to projection function
      Eigen::MatrixXd dzn_dPFC = Eigen::MatrixXd::Zero(2, 3);
      dzn_dPFC << 1 / p_FinCi(2), 0, -p_FinCi(0) / (p_FinCi(2) * p_FinCi(2)), 0, 1 / p_FinCi(2),
        -p_FinCi(1) / (p_FinCi(2) * p_FinCi(2));

      // Derivative of p_FinCi in respect to p_FinIi
      Eigen::MatrixXd dPFC_dPFG = R_ItoC * R_GtoIi;

      // Derivative of p_FinCi in respect to camera clone state
      Eigen::MatrixXd dPFC_dClone = Eigen::MatrixXd::Zero(3, 6);
      dPFC_dClone.block(0, 0, 3, 3) = R_ItoC * skewSymmetric(p_FinIi);
      dPFC_dClone.block(0, 3, 3, 3) = -dPFC_dPFG;

      unsigned int camStateStart = getCamStateStartIndex(cameraID);
      unsigned int augStateStart = getAugStateStartIndex(cameraID, featureTrack[i].frameID);

      // Precompute some matrices
      Eigen::MatrixXd dz_dPFC = dzn_dPFC;
      Eigen::MatrixXd dz_dPFG = dPFC_dPFG;

      // CHAINRULE: get the total feature Jacobian
      H_f.block(2 * c, 0, 2, H_f.cols()) = dz_dPFG * dPFG_dLambda;

      // CHAINRULE: get state clone Jacobian
      H_x.block(2 * c, camStateStart, 2, 6) = dz_dPFC * dPFC_dClone;

      // CHAINRULE: loop through all extra states and add their
      // NOTE: we add the Jacobian here as we might be in the anchoring pose for this measurement
      H_x.block(2 * c, augStateStart, 2, 6) += dz_dPFG * H_anc;
      H_x.block(2 * c, augStateStart + 6, 2, 6) += dz_dPFG * H_calib;

      // Derivative of p_FinCi in respect to camera calibration (R_ItoC, p_IinC)

      // Calculate the Jacobian
      Eigen::MatrixXd dPFC_dCalib = Eigen::MatrixXd::Zero(3, 6);
      dPFC_dCalib.block(0, 0, 3, 3) = skewSymmetric(p_FinCi - p_IinC);
      dPFC_dCalib.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

      // Chainrule it and add it to the big jacobian
      H_x.block(2 * c, camStateStart + 6, 2, 6) += dz_dPFC * dPFC_dCalib;

      // Move the Jacobian and residual index forward
      c++;
    }


    /// @todo Get the Jacobian for this feature
    /// @todo Nullspace projection
  }
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

unsigned int EKF::getAugStateStartIndex(unsigned int camID, unsigned int frameID)
{
  unsigned int stateStartIndex = 18;
  stateStartIndex += 12 * m_state.imuStates.size();
  for (auto const & camIter : m_state.camStates) {
    stateStartIndex += 6;
    for (auto const & augIter : camIter.second.augmentedStates) {
      if (augIter.frameID == frameID) {
        break;
      } else {
        stateStartIndex += 12;
      }
    }
  }

  return stateStartIndex;
}
