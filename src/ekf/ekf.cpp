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

#include "ekf/ekf.hpp"

#include <eigen3/Eigen/Eigen>

#include <string>
#include <vector>
#include <sstream>

#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "utility/math_helper.hpp"

// initializing instance_pointer with NULL
EKF * EKF::instance_pointer = NULL;

Eigen::MatrixXd EKF::GetStateTransition(double dT)
{
  Eigen::MatrixXd state_transition =
    Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size);
  state_transition.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dT;
  state_transition.block<3, 3>(3, 6) = Eigen::MatrixXd::Identity(3, 3) * dT;
  state_transition.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * dT;
  state_transition.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dT;
  return state_transition;
}

void EKF::ProcessModel(double time)
{
  m_logger->Log(LogLevel::DEBUG, "EKF::Predict at t=" + std::to_string(time));

  // Don't predict if time is not initialized
  if (!m_time_initialized) {
    m_current_time = time;
    m_time_initialized = true;
    m_logger->Log(
      LogLevel::INFO,
      "EKF::Predict initialized time at t=" + std::to_string(time));
    return;
  }

  if (time <= m_current_time) {
    m_logger->Log(
      LogLevel::WARN, "Requested prediction to time in the past. Current t=" +
      std::to_string(m_current_time) + ", Requested t=" +
      std::to_string(time));
    return;
  }

  double dT = time - m_current_time;

  Eigen::MatrixXd F = GetStateTransition(dT);

  Eigen::VectorXd process_update = F * m_state.m_body_state.ToVector();

  m_state.m_body_state.SetState(process_update);

  // Process input matrix is just identity
  m_cov.block<g_body_state_size, g_body_state_size>(0, 0) =
    F * (m_cov.block<g_body_state_size, g_body_state_size>(0, 0) + m_process_noise) * F.transpose();

  m_current_time = time;
}

State & EKF::GetState()
{
  return m_state;
}

BodyState EKF::GetBodyState()
{
  return m_state.m_body_state;
}

ImuState EKF::GetImuState(unsigned int imu_id)
{
  return m_state.m_imu_states[imu_id];
}

CamState EKF::GetCamState(unsigned int cam_id)
{
  return m_state.m_cam_states[cam_id];
}

unsigned int EKF::GetImuCount()
{
  return m_state.m_imu_states.size();
}

unsigned int EKF::GetCamCount()
{
  return m_state.m_cam_states.size();
}

Eigen::MatrixXd & EKF::GetCov()
{
  return m_cov;
}

void EKF::Initialize(double timeInit, BodyState body_state_init)
{
  m_current_time = timeInit;
  m_time_initialized = true;
  m_state.m_body_state = body_state_init;
}

void EKF::RegisterIMU(unsigned int imu_id, ImuState imu_state, Eigen::MatrixXd covariance)
{
  /// @todo check that id hasn't been used before
  /// @todo replace 12s with constants from IMU class
  unsigned int imu_state_start = g_body_state_size + 12 * m_state.m_imu_states.size();

  m_cov = InsertInMatrix(covariance, m_cov, imu_state_start, imu_state_start);
  m_state.m_imu_states[imu_id] = imu_state;
  m_stateSize += 12;

  m_logger->Log(
    LogLevel::DEBUG, "Register IMU: " + std::to_string(
      imu_id) + ", stateSize: " + std::to_string(m_stateSize));
}

void EKF::RegisterCamera(unsigned int cam_id, CamState cam_state, Eigen::MatrixXd covariance)
{
  /// @todo check that id hasn't been used before
  m_state.m_cam_states[cam_id] = cam_state;
  m_cov = InsertInMatrix(covariance, m_cov, 6, 6);
  m_stateSize += 6;

  m_logger->Log(
    LogLevel::DEBUG, "Register Cam: " + std::to_string(
      cam_id) + ", stateSize: " + std::to_string(m_stateSize));
}

/// @todo Replace this lookup with a map
unsigned int EKF::GetImuStateStartIndex(unsigned int imu_id)
{
  unsigned int stateStartIndex = g_body_state_size;
  for (auto const & imuIter : m_state.m_imu_states) {
    if (imuIter.first == imu_id) {
      break;
    } else {
      stateStartIndex += 12;
    }
  }
  return stateStartIndex;
}

/// @todo Replace this lookup with a map
unsigned int EKF::GetCamStateStartIndex(unsigned int cam_id)
{
  unsigned int stateStartIndex = g_body_state_size;
  stateStartIndex += 12 * m_state.m_imu_states.size();
  for (auto const & camIter : m_state.m_cam_states) {
    if (camIter.first == cam_id) {
      break;
    } else {
      stateStartIndex += 6 + 12 * camIter.second.augmented_states.size();
    }
  }
  return stateStartIndex;
}

/// @todo Replace this lookup with a map
unsigned int EKF::GetAugStateStartIndex(unsigned int cam_id, unsigned int frame_id)
{
  unsigned int stateStartIndex = g_body_state_size;
  stateStartIndex += (12 * m_state.m_imu_states.size());
  for (auto const & camIter : m_state.m_cam_states) {
    stateStartIndex += 6;
    for (auto const & augIter : camIter.second.augmented_states) {
      if ((camIter.first == cam_id) && (augIter.frame_id == frame_id)) {
        return stateStartIndex;
      } else {
        stateStartIndex += 12;
      }
    }
  }

  return stateStartIndex;
}

Eigen::MatrixXd EKF::AugmentJacobian(
  unsigned int cam_state_start,
  unsigned int camera_id,
  unsigned int aug_state_start)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(m_stateSize, m_stateSize - 12);

  unsigned int after_start = aug_state_start;
  unsigned int after_size = m_stateSize - aug_state_start - 12;

  // Before augmented state jacobian
  jacobian.block(0, 0, aug_state_start, aug_state_start) =
    Eigen::MatrixXd::Identity(aug_state_start, aug_state_start);

  // After augmented state jacobian
  if (after_size > 0) {
    jacobian.block(after_start, after_start, after_size, after_size) =
      Eigen::MatrixXd::Identity(after_size, after_size);
  }

  // IMU Global Position
  jacobian.block(aug_state_start + 0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

  // IMU Global Orientation
  jacobian.block(aug_state_start + 3, 9, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

  // Camera Global Position
  jacobian.block(aug_state_start + 6, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  jacobian.block(aug_state_start + 6, 9, 3, 3) =
    SkewSymmetric(m_state.m_body_state.m_orientation * m_state.m_cam_states[camera_id].position);
  jacobian.block(aug_state_start + 6, cam_state_start, 3, 3) =
    m_state.m_body_state.m_orientation.toRotationMatrix();

  // Camera Global Orientation
  jacobian.block(aug_state_start + 6, 9, 3, 3) =
    m_state.m_cam_states[camera_id].orientation.toRotationMatrix();
  jacobian.block(aug_state_start + 6, cam_state_start + 3, 3, 3) =
    m_state.m_body_state.m_orientation.toRotationMatrix();

  return jacobian;
}

/// @todo Augment covariance with Jacobian
void EKF::AugmentState(unsigned int camera_id, unsigned int frame_id)
{
  AugmentedState aug_state;
  aug_state.frame_id = frame_id;
  Eigen::Vector3d pos_i_in_g = m_state.m_body_state.m_position;
  Eigen::Quaterniond ang_i_to_g = m_state.m_body_state.m_orientation;

  aug_state.imu_position = pos_i_in_g;
  aug_state.imu_orientation = ang_i_to_g;

  aug_state.position = pos_i_in_g + ang_i_to_g * m_state.m_cam_states[camera_id].position;
  aug_state.orientation = m_state.m_cam_states[camera_id].orientation * ang_i_to_g;
  m_state.m_cam_states[camera_id].augmented_states.push_back(aug_state);

  unsigned int aug_state_start;
  unsigned int cam_state_start = GetCamStateStartIndex(camera_id);

  // Limit augmented states to 20
  if (m_state.m_cam_states[camera_id].augmented_states.size() <= 20) {
    aug_state_start = GetAugStateStartIndex(camera_id, frame_id);

    m_stateSize += 12;
  } else {
    // Remove second element from state
    m_state.m_cam_states[camera_id].augmented_states.erase(
      m_state.m_cam_states[camera_id].augmented_states.begin() + 1);

    // Remove second element from covariance
    m_cov = RemoveFromMatrix(m_cov, cam_state_start + 24, cam_state_start + 24, 12);

    aug_state_start = GetAugStateStartIndex(camera_id, frame_id);
  }

  /// @todo next: Use jacobian to initialize covariance
  Eigen::MatrixXd augment_jacobian = AugmentJacobian(cam_state_start, camera_id, aug_state_start);
  m_cov = (augment_jacobian * m_cov * augment_jacobian.transpose()).eval();
}
