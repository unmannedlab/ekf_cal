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

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ekf/types.hpp"
#include "infrastructure/data_logger.hpp"
#include "infrastructure/debug_logger.hpp"
#include "utility/math_helper.hpp"
#include "utility/string_helper.hpp"
#include "utility/type_helper.hpp"

EKF::EKF(
  std::shared_ptr<DebugLogger> logger,
  double body_data_rate,
  bool data_logging_on,
  std::string log_directory
)
: m_logger(logger),
  m_data_logging_on(data_logging_on),
  m_data_logger(log_directory, "body_state.csv")
{
  std::stringstream header;
  header << "time";
  header << EnumerateHeader("body_pos", 3);
  header << EnumerateHeader("body_vel", 3);
  header << EnumerateHeader("body_acc", 3);
  header << EnumerateHeader("body_ang_pos", 4);
  header << EnumerateHeader("body_ang_vel", 3);
  header << EnumerateHeader("body_ang_acc", 3);
  header << EnumerateHeader("body_cov", g_body_state_size);

  m_data_logger.DefineHeader(header.str());
  m_data_logger.SetLogging(m_data_logging_on);
  m_data_logger.SetLogRate(body_data_rate);
}

Eigen::MatrixXd EKF::GetStateTransition(double dT)
{
  Eigen::MatrixXd state_transition =
    Eigen::MatrixXd::Zero(g_body_state_size, g_body_state_size);
  state_transition.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dT;
  state_transition.block<3, 3>(3, 6) = Eigen::MatrixXd::Identity(3, 3) * dT;
  state_transition.block<3, 3>(9, 12) = Eigen::MatrixXd::Identity(3, 3) * dT;
  state_transition.block<3, 3>(12, 15) = Eigen::MatrixXd::Identity(3, 3) * dT;
  return state_transition;
}

void EKF::LogBodyStateIfNeeded()
{
  if (m_data_logging_on) {
    std::stringstream msg;
    Eigen::VectorXd body_cov =
      GetCov().block<g_body_state_size, g_body_state_size>(0, 0).diagonal();
    msg << m_current_time;
    msg << VectorToCommaString(GetState().m_body_state.m_position);
    msg << VectorToCommaString(GetState().m_body_state.m_velocity);
    msg << VectorToCommaString(GetState().m_body_state.m_acceleration);
    msg << QuaternionToCommaString(GetState().m_body_state.m_ang_b_to_g);
    msg << VectorToCommaString(GetState().m_body_state.m_angular_velocity);
    msg << VectorToCommaString(GetState().m_body_state.m_angular_acceleration);
    msg << VectorToCommaString(body_cov);
    m_data_logger.RateLimitedLog(msg.str(), m_current_time);
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

  if (time < m_current_time) {
    m_logger->Log(
      LogLevel::WARN, "Requested prediction to time in the past. Current t=" +
      std::to_string(m_current_time) + ", Requested t=" +
      std::to_string(time));
    return;
  } else if (time == m_current_time) {
    m_logger->Log(LogLevel::DEBUG, "Requested prediction to current time.");
    return;
  }

  double dT = time - m_current_time;

  Eigen::MatrixXd dF = GetStateTransition(dT);
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size) + dF;

  Eigen::VectorXd process_update = dF * m_state.m_body_state.ToVector();

  m_state.m_body_state += process_update;

  // Process input matrix is just identity
  /// @todo(jhartzer): Limit covariance for angular uncertainty
  /// @todo(jhartzer): Check matrix condition
  m_cov.block<g_body_state_size, g_body_state_size>(0, 0) =
    F * (m_cov.block<g_body_state_size, g_body_state_size>(0, 0)) * F.transpose();

  AddProccessNoise();

  m_current_time = time;

  LogBodyStateIfNeeded();
}

void EKF::PredictModel(
  double time,
  Eigen::Vector3d acceleration,
  Eigen::Matrix3d acceleration_covariance,
  Eigen::Vector3d angular_rate,
  Eigen::Matrix3d angular_rate_covariance)
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

  Eigen::Quaterniond ang_i_to_b = m_state.m_body_state.m_ang_b_to_g;

  Eigen::Vector3d acceleration_global = ang_i_to_b * acceleration;
  Eigen::Vector3d angular_rate_global = ang_i_to_b * angular_rate;
  Eigen::Matrix3d acceleration_covariance_global = ang_i_to_b * acceleration_covariance;
  Eigen::Matrix3d angular_rate_covariance_global = ang_i_to_b * angular_rate_covariance;

  Eigen::Vector3d rot_vec(angular_rate[0] * dT, angular_rate[1] * dT,
    angular_rate[2] * dT);

  m_state.m_body_state.m_position +=
    dT * m_state.m_body_state.m_velocity + dT * dT / 2 * acceleration_global;
  m_state.m_body_state.m_velocity += dT * acceleration_global;
  m_state.m_body_state.m_acceleration = acceleration_global;
  m_state.m_body_state.m_ang_b_to_g = m_state.m_body_state.m_ang_b_to_g * RotVecToQuat(rot_vec);
  m_state.m_body_state.m_angular_velocity = angular_rate_global;
  m_state.m_body_state.m_angular_acceleration.setZero();

  Eigen::MatrixXd dF = GetStateTransition(dT);
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size) + dF;

  AddProccessNoise();

  Eigen::MatrixXd process_noise = Eigen::MatrixXd::Zero(g_body_state_size, g_body_state_size);
  process_noise.block<3, 3>(6, 6) = acceleration_covariance_global;
  process_noise.block<3, 3>(12, 12) = angular_rate_covariance_global;

  // Process input matrix is just identity
  m_cov.block<g_body_state_size, g_body_state_size>(0, 0) =
    F * (m_cov.block<g_body_state_size, g_body_state_size>(0, 0)) * F.transpose() + process_noise;

  m_current_time = time;

  LogBodyStateIfNeeded();
}

/// @todo(jhartzer): Adjust process noise for offsets and biases
void EKF::AddProccessNoise()
{
  m_cov.block<g_body_state_size, g_body_state_size>(0, 0) += m_process_noise;

  for (auto const & imu_iter : m_state.m_imu_states) {
    if (imu_iter.second.is_intrinsic && imu_iter.second.is_extrinsic) {
      unsigned int imu_state_start = GetImuStateStartIndex(imu_iter.first);
      Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(12, 12);

      process_noise.block<3, 3>(0, 0) *= imu_iter.second.pos_stability;
      process_noise.block<3, 3>(3, 3) *= imu_iter.second.ang_stability;
      process_noise.block<3, 3>(6, 6) *= imu_iter.second.acc_bias_stability;
      process_noise.block<3, 3>(9, 9) *= imu_iter.second.omg_bias_stability;

      m_cov.block<12, 12>(imu_state_start, imu_state_start) += process_noise;

    } else if (imu_iter.second.is_intrinsic) {
      unsigned int imu_state_start = GetImuStateStartIndex(imu_iter.first);
      Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(6, 6);

      process_noise.block<3, 3>(0, 0) *= imu_iter.second.acc_bias_stability;
      process_noise.block<3, 3>(3, 3) *= imu_iter.second.omg_bias_stability;

      m_cov.block<6, 6>(imu_state_start, imu_state_start) += process_noise;

    } else if (imu_iter.second.is_extrinsic) {
      unsigned int imu_state_start = GetImuStateStartIndex(imu_iter.first);
      Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(6, 6);

      process_noise.block<3, 3>(0, 0) *= imu_iter.second.pos_stability;
      process_noise.block<3, 3>(3, 3) *= imu_iter.second.ang_stability;

      m_cov.block<6, 6>(imu_state_start, imu_state_start) += process_noise;
    }
  }

  for (auto const & cam_iter : m_state.m_cam_states) {
    unsigned int cam_state_start = GetCamStateStartIndex(cam_iter.first);
    Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(6, 6);

    process_noise.block<3, 3>(0, 0) *= cam_iter.second.pos_stability;
    process_noise.block<3, 3>(3, 3) *= cam_iter.second.ang_stability;

    m_cov.block<6, 6>(cam_state_start, cam_state_start) += process_noise;
  }
}

State & EKF::GetState()
{
  return m_state;
}

unsigned int EKF::GetStateSize()
{
  return m_state_size;
}

BodyState EKF::GetBodyState()
{
  return m_state.m_body_state;
}

ImuState EKF::GetImuState(unsigned int imu_id)
{
  return m_state.m_imu_states[imu_id];
}

GpsState EKF::GetGpsState(unsigned int gps_id)
{
  return m_state.m_gps_states[gps_id];
}

CamState EKF::GetCamState(unsigned int cam_id)
{
  return m_state.m_cam_states[cam_id];
}

unsigned int EKF::GetImuCount()
{
  return m_state.m_imu_states.size();
}

unsigned int EKF::GetImuStateSize()
{
  unsigned int imu_state_size {0};
  for (auto & imu_iter : m_state.m_imu_states) {
    unsigned int imu_id = imu_iter.first;
    if (m_state.m_imu_states[imu_id].is_extrinsic) {imu_state_size += g_imu_extrinsic_state_size;}
    if (m_state.m_imu_states[imu_id].is_intrinsic) {imu_state_size += g_imu_intrinsic_state_size;}
  }
  return imu_state_size;
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
  // Check that ID hasn't been used before
  if (m_state.m_imu_states.find(imu_id) != m_state.m_imu_states.end()) {
    std::stringstream imu_id_used_warning;
    imu_id_used_warning << "IMU ID " << imu_id << " has already been registered.";
    m_logger->Log(LogLevel::WARN, imu_id_used_warning.str());
  }

  unsigned int imu_state_start = g_body_state_size + GetImuStateSize();

  /// @todo check size of matrix being inserted
  m_cov = InsertInMatrix(covariance, m_cov, imu_state_start, imu_state_start);
  m_state.m_imu_states[imu_id] = imu_state;
  if (imu_state.is_extrinsic) {m_state_size += g_imu_extrinsic_state_size;}
  if (imu_state.is_intrinsic) {m_state_size += g_imu_intrinsic_state_size;}

  std::stringstream log_msg;
  log_msg << "Register IMU: " << imu_id << ", stateSize: " << m_state_size;
  m_logger->Log(LogLevel::INFO, log_msg.str());
}

void EKF::RegisterGPS(unsigned int gps_id, GpsState gps_state, Eigen::Matrix3d covariance)
{
  // Check that ID hasn't been used before
  if (m_state.m_gps_states.find(gps_id) != m_state.m_gps_states.end()) {
    std::stringstream gps_id_used_warning;
    gps_id_used_warning << "GPS ID " << gps_id << " has already been registered.";
    m_logger->Log(LogLevel::WARN, gps_id_used_warning.str());
  }

  m_state.m_gps_states[gps_id] = gps_state;
  m_cov = InsertInMatrix(covariance, m_cov, g_gps_state_size, g_gps_state_size);
  m_state_size += g_gps_state_size;

  std::stringstream log_msg;
  log_msg << "Register GPS: " << gps_id << ", stateSize: " << m_state_size;
  m_logger->Log(LogLevel::INFO, log_msg.str());
}

void EKF::RegisterCamera(unsigned int cam_id, CamState cam_state, Eigen::MatrixXd covariance)
{
  // Check that ID hasn't been used before
  if (m_state.m_cam_states.find(cam_id) != m_state.m_cam_states.end()) {
    std::stringstream cam_id_used_warning;
    cam_id_used_warning << "Camera ID " << cam_id << " has already been registered.";
    m_logger->Log(LogLevel::WARN, cam_id_used_warning.str());
  }

  m_state.m_cam_states[cam_id] = cam_state;
  m_cov = InsertInMatrix(covariance, m_cov, g_cam_state_size, g_cam_state_size);
  m_state_size += g_cam_state_size;

  std::stringstream log_msg;
  log_msg << "Register Camera: " << cam_id << ", stateSize: " << m_state_size;
  m_logger->Log(LogLevel::INFO, log_msg.str());
}

/// @todo Replace this lookup with a map
unsigned int EKF::GetImuStateStartIndex(unsigned int imu_id)
{
  unsigned int state_start_index = g_body_state_size;
  for (auto const & imu_iter : m_state.m_imu_states) {
    if (imu_iter.first == imu_id) {
      break;
    } else {
      if (m_state.m_imu_states[imu_iter.first].is_extrinsic) {
        state_start_index += g_imu_extrinsic_state_size;
      }
      if (m_state.m_imu_states[imu_iter.first].is_intrinsic) {
        state_start_index += g_imu_intrinsic_state_size;
      }
    }
  }
  return state_start_index;
}

/// @todo Replace this lookup with a map
unsigned int EKF::GetGpsStateStartIndex(unsigned int gps_id)
{
  unsigned int state_start_index = g_body_state_size;
  for (auto const & gps_iter : m_state.m_gps_states) {
    if (gps_iter.first == gps_id) {
      break;
    } else {
      state_start_index += g_gps_state_size;
    }
  }
  return state_start_index;
}

/// @todo Replace this lookup with a map
unsigned int EKF::GetCamStateStartIndex(unsigned int cam_id)
{
  unsigned int state_start_index = g_body_state_size;
  state_start_index += GetImuStateSize();
  for (auto const & cam_iter : m_state.m_cam_states) {
    if (cam_iter.first == cam_id) {
      break;
    } else {
      state_start_index += g_cam_state_size +
        g_aug_state_size * cam_iter.second.augmented_states.size();
    }
  }
  return state_start_index;
}

/// @todo Replace this lookup with a map
unsigned int EKF::GetAugStateStartIndex(unsigned int cam_id, int frame_id)
{
  unsigned int state_start_index = g_body_state_size;
  state_start_index += GetImuStateSize();
  for (auto const & cam_iter : m_state.m_cam_states) {
    state_start_index += g_cam_state_size;
    for (auto const & augIter : cam_iter.second.augmented_states) {
      if ((cam_iter.first == cam_id) && (augIter.frame_id == frame_id)) {
        return state_start_index;
      } else {
        state_start_index += g_aug_state_size;
      }
    }
  }

  return state_start_index;
}

/// @todo Don't return a Jacobian but rather just apply a Jacobian to the covariance
Eigen::MatrixXd EKF::AugmentJacobian(
  unsigned int cam_state_start,
  unsigned int aug_state_start)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(m_state_size, m_state_size - g_aug_state_size);

  unsigned int after_start = aug_state_start;
  unsigned int after_size = m_state_size - aug_state_start - g_aug_state_size;

  // Before augmented state Jacobian
  jacobian.block(0, 0, aug_state_start, aug_state_start) =
    Eigen::MatrixXd::Identity(aug_state_start, aug_state_start);

  // After augmented state Jacobian
  if (after_size > 0) {
    jacobian.block(after_start, after_start, after_size, after_size) =
      Eigen::MatrixXd::Identity(after_size, after_size);
  }

  // Body Position in Global Frame
  jacobian.block<3, 3>(aug_state_start + 0, 0) = Eigen::MatrixXd::Identity(3, 3);

  // Body Orientation to Global Frame
  jacobian.block<3, 3>(aug_state_start + 3, 9) = Eigen::MatrixXd::Identity(3, 3);

  // Camera Position in IMU Frame
  jacobian.block<3, 3>(aug_state_start + 6, cam_state_start + 0) = Eigen::MatrixXd::Identity(3, 3);

  // Camera Orientation to IMU Frame
  jacobian.block<3, 3>(aug_state_start + 9, cam_state_start + 3) = Eigen::MatrixXd::Identity(3, 3);

  return jacobian;
}

void EKF::AugmentState(unsigned int camera_id, int frame_id)
{
  std::stringstream msg;
  msg << "Aug State Frame: " << std::to_string(frame_id);
  m_logger->Log(LogLevel::DEBUG, msg.str());
  AugmentedState aug_state;
  aug_state.frame_id = frame_id;
  Eigen::Vector3d pos_b_in_g = m_state.m_body_state.m_position;
  Eigen::Quaterniond ang_b_to_g = m_state.m_body_state.m_ang_b_to_g;

  aug_state.pos_b_in_g = pos_b_in_g;
  aug_state.ang_b_to_g = ang_b_to_g;
  aug_state.pos_c_in_b = m_state.m_cam_states[camera_id].pos_c_in_b;
  aug_state.ang_c_to_b = m_state.m_cam_states[camera_id].ang_c_to_b;
  m_state.m_cam_states[camera_id].augmented_states.push_back(aug_state);

  unsigned int aug_state_start;
  unsigned int cam_state_start = GetCamStateStartIndex(camera_id);

  // Limit augmented states to m_max_track_length
  if (m_state.m_cam_states[camera_id].augmented_states.size() <= m_max_track_length) {
    aug_state_start = GetAugStateStartIndex(camera_id, frame_id);

    m_state_size += g_aug_state_size;
  } else {
    /// @todo(jhartzer): Evaluate switching to second element / creating map
    // Remove first element from state
    m_state.m_cam_states[camera_id].augmented_states.erase(
      m_state.m_cam_states[camera_id].augmented_states.begin());

    // Remove first element from covariance
    m_cov = RemoveFromMatrix(
      m_cov, cam_state_start + g_cam_state_size,
      cam_state_start + g_cam_state_size, g_aug_state_size);

    aug_state_start = GetAugStateStartIndex(camera_id, frame_id);
  }

  Eigen::MatrixXd augment_jacobian = AugmentJacobian(cam_state_start, aug_state_start);
  /// @todo doing this is very expensive. Apply Jacobian in place without large multiplications
  /// Most elements are identity/zeros anyways
  m_cov = (augment_jacobian * m_cov * augment_jacobian.transpose()).eval();
}

void EKF::SetProcessNoise(Eigen::VectorXd process_noise)
{
  m_process_noise = process_noise.asDiagonal();
}

AugmentedState EKF::MatchState(int camera_id, int frame_id)
{
  AugmentedState aug_state_match;

  for (auto & aug_state : m_state.m_cam_states[camera_id].augmented_states) {
    if (aug_state.frame_id == frame_id) {
      return aug_state;
    }
  }

  std::stringstream warning_msg;
  warning_msg << "No matching augmented state for frame " << frame_id;
  m_logger->Log(LogLevel::WARN, warning_msg.str());
  return aug_state_match;
}

void EKF::SetMaxTrackLength(unsigned int max_track_length)
{
  m_max_track_length = max_track_length;
}

void EKF::SetGpsReference(Eigen::VectorXd reference_lla, double ang_l_to_g)
{
  m_reference_lla = reference_lla;
  m_ang_l_to_g = ang_l_to_g;
  m_is_lla_initialized = true;
}

Eigen::VectorXd EKF::GetReferenceLLA()
{
  if (!m_is_lla_initialized) {
    m_logger->Log(LogLevel::WARN, "LLA is being accessed before initialization!");
  }
  return m_reference_lla;
}

double EKF::GetReferenceAngle()
{
  return m_ang_l_to_g;
}

bool EKF::IsLlaInitialized()
{
  return m_is_lla_initialized;
}
