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
#include "infrastructure/data_logger.hpp"
#include "infrastructure/debug_logger.hpp"
#include "utility/math_helper.hpp"
#include "utility/string_helper.hpp"

// initializing instance_pointer with NULL
EKF * EKF::instance_pointer = NULL;

EKF::EKF()
{
  std::stringstream msg;
  msg << "time";
  msg << EnumerateHeader("body_state", g_body_state_size);
  msg << EnumerateHeader("body_cov", g_body_state_size);
  msg << "\n";

  m_data_logger.DefineHeader(msg.str());
  m_data_logger.SetLogging(m_data_logging_on);
}

Eigen::MatrixXd EKF::GetStateTransition(double dT)
{
  Eigen::MatrixXd state_transition =
    Eigen::MatrixXd::Zero(g_body_state_size, g_body_state_size);
  state_transition.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dT;
  state_transition.block<3, 3>(3, 6) = Eigen::MatrixXd::Identity(3, 3) * dT;
  /// This assumes the state is body angular rates and accelerations. Others assume global frame
  state_transition.block<3, 3>(9, 12) =
    m_state.m_body_state.m_orientation.toRotationMatrix() * dT;
  state_transition.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dT;
  return state_transition;
}

void EKF::LogBodyStateIfNeeded()
{
  /// @todo Use modulo for determining log
  if ((m_data_logging_on) &&
    (m_body_data_rate) &&
    (m_current_time > m_prev_log_time + 1 / m_body_data_rate))
  {
    m_prev_log_time = m_current_time;
    std::stringstream msg;
    Eigen::VectorXd body_state_vec = GetState().m_body_state.ToVector();
    Eigen::VectorXd body_cov =
      GetCov().block(0, 0, g_body_state_size, g_body_state_size).diagonal();
    msg << m_current_time;
    msg << VectorToCommaString(body_state_vec);
    msg << VectorToCommaString(body_cov);
    msg << "\n";
    m_data_logger.Log(msg.str());
  }
}

void EKF::ProcessModel(double time)
{
  m_logger->Log(LogLevel::DEBUG, "ProcessModel at t=" + std::to_string(time));

  // Don't predict if time is not initialized
  if (!m_time_initialized) {
    m_current_time = time;
    m_time_initialized = true;
    m_logger->Log(
      LogLevel::INFO,
      "ProcessModel initialized time at t=" + std::to_string(time));
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

  Eigen::MatrixXd dF = GetStateTransition(dT);
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size) + dF;

  Eigen::VectorXd process_update = dF * m_state.m_body_state.ToVector();

  m_state.m_body_state += process_update;

  // Process input matrix is just identity
  m_cov.block<g_body_state_size, g_body_state_size>(0, 0) =
    F * (m_cov.block<g_body_state_size, g_body_state_size>(0, 0) + m_process_noise) * F.transpose();

  m_current_time = time;

  LogBodyStateIfNeeded();
}

void EKF::PredictModel(
  double time,
  Eigen::Vector3d acceleration,
  Eigen::Matrix3d accelerationCovariance,
  Eigen::Vector3d angularRate,
  Eigen::Matrix3d angularRateCovariance)
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

  Eigen::Vector3d acceleration_body =
    m_state.m_body_state.m_orientation.inverse() * acceleration;
  Eigen::Matrix3d accelerationCovariance_body =
    m_state.m_body_state.m_orientation.inverse() * accelerationCovariance;
  Eigen::Vector3d angularRate_body =
    m_state.m_body_state.m_orientation.inverse() * angularRate;
  Eigen::Matrix3d angularRateCovariance_body =
    m_state.m_body_state.m_orientation.inverse() * angularRateCovariance;

  Eigen::Quaterniond d_quat {
    1.0,
    angularRate[0] * dT / 2,
    angularRate[1] * dT / 2,
    angularRate[2] * dT / 2
  };

  m_state.m_body_state.m_position +=
    dT * m_state.m_body_state.m_velocity +
    dT * dT / 2 * acceleration_body;
  m_state.m_body_state.m_velocity += dT * acceleration_body;
  m_state.m_body_state.m_acceleration = acceleration_body;
  m_state.m_body_state.m_orientation = m_state.m_body_state.m_orientation * d_quat;
  m_state.m_body_state.m_angular_velocity = angularRate_body;
  /// @todo replace with smoothing filter?
  m_state.m_body_state.m_angular_acceleration.setZero();

  Eigen::MatrixXd dF = GetStateTransition(dT);
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size) + dF;

  Eigen::MatrixXd process_noise = Eigen::MatrixXd::Zero(g_body_state_size, g_body_state_size);
  process_noise.block<3, 3>(6, 6) = accelerationCovariance_body;
  process_noise.block<3, 3>(12, 12) = angularRateCovariance_body;

  // Process input matrix is just identity
  m_cov.block<g_body_state_size, g_body_state_size>(0, 0) =
    F * (m_cov.block<g_body_state_size, g_body_state_size>(0, 0) + process_noise) * F.transpose();

  m_current_time = time;

  LogBodyStateIfNeeded();
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

  /// @todo check size of matrix being inserted
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

/// @todo Don't return a jacobian but rather just apply a jacobian to the covariance
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

void EKF::AugmentState(unsigned int camera_id, unsigned int frame_id)
{
  std::stringstream msg;
  msg << "Aug State Frame: " << std::to_string(frame_id);
  m_logger->Log(LogLevel::DEBUG, msg.str());
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

  // Limit augmented states to 20 + 1
  if (m_state.m_cam_states[camera_id].augmented_states.size() <= 21) {
    aug_state_start = GetAugStateStartIndex(camera_id, frame_id);

    m_stateSize += 12;
  } else {
    // Remove first element from state
    m_state.m_cam_states[camera_id].augmented_states.erase(
      m_state.m_cam_states[camera_id].augmented_states.begin());

    // Remove first element from covariance
    m_cov = RemoveFromMatrix(m_cov, cam_state_start + 12, cam_state_start + 12, 12);

    aug_state_start = GetAugStateStartIndex(camera_id, frame_id);
  }

  Eigen::MatrixXd augment_jacobian = AugmentJacobian(cam_state_start, camera_id, aug_state_start);
  /// @todo doing this is very expensive. Apply Jacobian in place without large multiplications
  /// Most elements are identity/zeros anyways
  m_cov = (augment_jacobian * m_cov * augment_jacobian.transpose()).eval();
}

void EKF::SetBodyDataRate(double rate)
{
  m_body_data_rate = rate;
}

void EKF::SetDataLogging(bool value)
{
  m_data_logging_on = value;
  m_data_logger.SetLogging(m_data_logging_on);
}
