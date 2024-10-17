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

#include <algorithm>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ekf/constants.hpp"
#include "ekf/types.hpp"
#include "infrastructure/data_logger.hpp"
#include "infrastructure/debug_logger.hpp"
#include "utility/gps_helper.hpp"
#include "utility/math_helper.hpp"
#include "utility/string_helper.hpp"
#include "utility/type_helper.hpp"

EKF::EKF(Parameters params)
: m_debug_logger(params.debug_logger),
  m_data_logging_on(params.data_logging_on),
  m_data_logger(params.log_directory, "body_state.csv"),
  m_gps_init_type(params.gps_init_type),
  m_gps_init_pos_thresh(params.gps_init_pos_thresh),
  m_gps_init_ang_thresh(params.gps_init_ang_thresh),
  m_gps_init_baseline_dist(params.gps_init_baseline_dist),
  m_pos_l_in_g(params.pos_l_in_g),
  m_ang_l_to_g(params.ang_l_to_g),
  m_augmenting_type(params.augmenting_type),
  m_augmenting_delta_time(params.augmenting_delta_time),
  m_augmenting_pos_error(params.augmenting_pos_error),
  m_augmenting_ang_error(params.augmenting_ang_error),
  m_motion_detection_chi_squared(params.motion_detection_chi_squared),
  m_imu_noise_scale_factor(params.imu_noise_scale_factor),
  m_use_root_covariance(params.use_root_covariance),
  m_use_first_estimate_jacobian(params.use_first_estimate_jacobian)
{
  std::stringstream header;
  header << "time";
  header << EnumerateHeader("body_pos", 3);
  header << EnumerateHeader("body_vel", 3);
  header << EnumerateHeader("body_ang_pos", 4);
  header << EnumerateHeader("body_cov", g_body_state_size);
  header << EnumerateHeader("duration", 1);

  m_data_logger.DefineHeader(header.str());
  m_data_logger.SetLogging(m_data_logging_on);
  m_data_logger.SetLogRate(params.body_data_rate);
  SetBodyProcessNoise(params.process_noise);

  if (params.gps_init_type == GpsInitType::CONSTANT) {
    m_is_lla_initialized = true;
  } else {
    m_is_lla_initialized = false;
  }
}

Eigen::MatrixXd EKF::GetStateTransition(double dT)
{
  Eigen::MatrixXd state_transition =
    Eigen::MatrixXd::Zero(g_body_state_size, g_body_state_size);
  state_transition.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3) * dT;
  return state_transition;
}

void EKF::LogBodyStateIfNeeded(int execution_count)
{
  if (m_data_logging_on) {
    std::stringstream msg;
    Eigen::VectorXd body_cov = m_cov.block<g_body_state_size, g_body_state_size>(0, 0).diagonal();
    msg << m_current_time;
    msg << VectorToCommaString(m_state.body_state.pos_b_in_l);
    msg << VectorToCommaString(m_state.body_state.vel_b_in_l);
    msg << QuaternionToCommaString(m_state.body_state.ang_b_to_l);
    msg << VectorToCommaString(body_cov);
    msg << "," << execution_count;
    m_data_logger.RateLimitedLog(msg.str(), m_current_time);
  }
}

/// @todo: Use RK4 or other higher-order prediction step
void EKF::PredictModel(double time)
{
  m_debug_logger->Log(LogLevel::DEBUG, "EKF::Predict at t=" + std::to_string(time));

  // Don't predict if time is not initialized
  if (!m_time_initialized) {
    m_current_time = time;
    m_time_initialized = true;
    m_debug_logger->Log(
      LogLevel::INFO,
      "EKF::Predict initialized time at t=" + std::to_string(time));
    return;
  }

  if (time < m_current_time) {
    m_debug_logger->Log(
      LogLevel::INFO, "Requested prediction to time in the past. Current t=" +
      std::to_string(m_current_time) + ", Requested t=" +
      std::to_string(time));
    return;
  }

  auto t_start = std::chrono::high_resolution_clock::now();

  double dT = time - m_current_time;

  Eigen::Quaterniond ang_b_to_l = m_state.body_state.ang_b_to_l;
  Eigen::Vector3d acc_in_b = m_imu_filter.GetAcc();
  Eigen::Vector3d ang_vel_in_b = m_imu_filter.GetAngVel();

  Eigen::Vector3d acceleration_local = (ang_b_to_l * acc_in_b) - g_gravity;

  Eigen::Vector3d rot_vec(ang_vel_in_b[0] * dT, ang_vel_in_b[1] * dT, ang_vel_in_b[2] * dT);

  m_state.body_state.vel_b_in_l += dT * acceleration_local;
  m_state.body_state.pos_b_in_l += dT * m_state.body_state.vel_b_in_l;
  m_state.body_state.ang_b_to_l = m_state.body_state.ang_b_to_l * RotVecToQuat(rot_vec);

  Eigen::MatrixXd dF = GetStateTransition(dT);
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size) + dF;

  if (m_use_root_covariance) {
    m_cov.block<g_body_state_size, g_body_state_size>(0, 0) = QR_r(
      m_cov.block<g_body_state_size, g_body_state_size>(0, 0) * F.transpose(),
      m_process_noise.block<g_body_state_size, g_body_state_size>(0, 0) * std::sqrt(dT));
  } else {
    m_cov.block<g_body_state_size, g_body_state_size>(0, 0) =
      F * (m_cov.block<g_body_state_size, g_body_state_size>(0, 0)) * F.transpose() +
      m_process_noise.block<g_body_state_size, g_body_state_size>(0, 0) * dT;
  }

  m_current_time = time;

  AugmentStateIfNeeded();

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  LogBodyStateIfNeeded(t_execution.count());
}

unsigned int EKF::GetStateSize()
{
  return m_state_size;
}

ImuState EKF::GetImuState(unsigned int imu_id)
{
  return m_state.imu_states[imu_id];
}

GpsState EKF::GetGpsState(unsigned int gps_id)
{
  return m_state.gps_states[gps_id];
}

CamState EKF::GetCamState(unsigned int cam_id)
{
  return m_state.cam_states[cam_id];
}

unsigned int EKF::GetImuCount()
{
  return m_state.imu_states.size();
}

unsigned int EKF::GetImuStateSize()
{
  return m_imu_state_size;
}

unsigned int EKF::GetGpsCount()
{
  return m_state.gps_states.size();
}

unsigned int EKF::GetGpsStateSize()
{
  return m_gps_state_size;
}

unsigned int EKF::GetCamStateSize()
{
  return m_cam_state_size;
}

unsigned int EKF::GetCamCount()
{
  return m_state.cam_states.size();
}

unsigned int EKF::GetAugStateSize()
{
  return m_aug_state_size;
}

void EKF::Initialize(double timeInit, BodyState body_state_init)
{
  m_current_time = timeInit;
  m_time_initialized = true;
  m_state.body_state = body_state_init;
}

void EKF::RegisterIMU(unsigned int imu_id, ImuState imu_state, Eigen::MatrixXd covariance)
{
  // Check that ID hasn't been used before
  if (m_state.imu_states.find(imu_id) != m_state.imu_states.end()) {
    std::stringstream imu_id_used_warning;
    imu_id_used_warning << "IMU ID " << imu_id << " has already been registered.";
    m_debug_logger->Log(LogLevel::WARN, imu_id_used_warning.str());
    return;
  }

  unsigned int imu_state_end = g_body_state_size + GetImuStateSize();

  m_state.imu_states[imu_id] = imu_state;
  if (imu_state.GetIsExtrinsic() || imu_state.GetIsIntrinsic()) {
    m_cov = InsertInMatrix(
      covariance.block(0, 0, imu_state.size, imu_state.size), m_cov, imu_state_end, imu_state_end);
  }
  if (imu_state.GetIsExtrinsic()) {
    m_state_size += g_imu_extrinsic_state_size;
    m_imu_state_size += g_imu_extrinsic_state_size;
  }
  if (imu_state.GetIsIntrinsic()) {
    m_state_size += g_imu_intrinsic_state_size;
    m_imu_state_size += g_imu_intrinsic_state_size;
  }

  RefreshIndices();

  std::stringstream log_msg;
  log_msg << "Register IMU: " << imu_id << ", stateSize: " << m_state_size;
  m_debug_logger->Log(LogLevel::INFO, log_msg.str());
}

void EKF::RegisterGPS(unsigned int gps_id, GpsState gps_state, Eigen::Matrix3d covariance)
{
  // Check that ID hasn't been used before
  if (m_state.gps_states.find(gps_id) != m_state.gps_states.end()) {
    std::stringstream gps_id_used_warning;
    gps_id_used_warning << "GPS ID " << gps_id << " has already been registered.";
    m_debug_logger->Log(LogLevel::WARN, gps_id_used_warning.str());
    return;
  }

  unsigned int gps_state_end = g_body_state_size + GetImuStateSize() + GetGpsStateSize();
  m_state.gps_states[gps_id] = gps_state;
  if (gps_state.GetIsExtrinsic()) {
    m_cov = InsertInMatrix(
      covariance.block(0, 0, gps_state.size, gps_state.size), m_cov, gps_state_end, gps_state_end);
    m_state_size += g_gps_extrinsic_state_size;
    m_gps_state_size += g_gps_extrinsic_state_size;
  }

  RefreshIndices();

  std::stringstream log_msg;
  log_msg << "Register GPS: " << gps_id << ", stateSize: " << m_state_size;
  m_debug_logger->Log(LogLevel::INFO, log_msg.str());
}

void EKF::RegisterCamera(unsigned int cam_id, CamState cam_state, Eigen::MatrixXd covariance)
{
  // Check that ID hasn't been used before
  if (m_state.cam_states.find(cam_id) != m_state.cam_states.end()) {
    std::stringstream cam_id_used_warning;
    cam_id_used_warning << "Camera ID " << cam_id << " has already been registered.";
    m_debug_logger->Log(LogLevel::WARN, cam_id_used_warning.str());
    return;
  }

  m_max_frame_period = std::max(m_max_frame_period, 1 / cam_state.rate);
  m_max_track_duration = m_max_frame_period * m_max_track_length;
  m_min_aug_period = std::min(m_min_aug_period, 1 / cam_state.rate);

  unsigned int cam_state_end = g_body_state_size +
    GetImuStateSize() + GetGpsStateSize() + GetCamStateSize();

  m_state.cam_states[cam_id] = cam_state;
  m_cov = InsertInMatrix(
    covariance.block(0, 0, cam_state.size, cam_state.size), m_cov, cam_state_end, cam_state_end);
  m_state_size += g_cam_state_size;
  m_cam_state_size += g_cam_state_size;

  RefreshIndices();

  std::stringstream log_msg;
  log_msg << "Register Camera: " << cam_id << ", stateSize: " << m_state_size;
  m_debug_logger->Log(LogLevel::INFO, log_msg.str());
}

void EKF::RegisterFiducial(FidState fid_state, Eigen::MatrixXd covariance)
{
  // Check that ID hasn't been used before
  if (m_state.fid_states.find(fid_state.id) != m_state.fid_states.end()) {
    std::stringstream fid_id_used_warning;
    fid_id_used_warning << "FID ID " << fid_state.id << " has already been registered.";
    m_debug_logger->Log(LogLevel::WARN, fid_id_used_warning.str());
    return;
  }

  m_state.fid_states[fid_state.id] = fid_state;
  if (fid_state.GetIsExtrinsic()) {
    m_cov = InsertInMatrix(
      covariance.block(0, 0, fid_state.size, fid_state.size), m_cov, m_state_size, m_state_size);
    m_state_size += g_fid_extrinsic_state_size;
    m_fid_state_size += g_fid_extrinsic_state_size;
  }

  std::stringstream log_msg;
  log_msg << "Register Fiducial: " << fid_state.id << ", stateSize: " << m_state_size;
  m_debug_logger->Log(LogLevel::INFO, log_msg.str());
}

Eigen::MatrixXd EKF::AugmentCovariance(Eigen::MatrixXd in_cov, unsigned int index)
{
  unsigned int in_rows = in_cov.rows();
  unsigned int in_cols = in_cov.cols();
  unsigned int out_rows = in_cov.rows() + g_aug_state_size;
  unsigned int out_cols = in_cov.cols() + g_aug_state_size;

  Eigen::MatrixXd out_cov = Eigen::MatrixXd::Zero(out_rows, out_cols);

  // Top Left
  out_cov.block(0, 0, index, index) = in_cov.block(0, 0, index, index);

  // Top Right
  out_cov.block(0, index + g_aug_state_size, index, in_cols - index) =
    in_cov.block(0, index, index, in_cols - index);

  // Bottom Left
  out_cov.block(index + g_aug_state_size, 0, in_rows - index, index) =
    in_cov.block(index, 0, in_rows - index, index);

  // Bottom Right
  out_cov.block(
    index + g_aug_state_size, index + g_aug_state_size, in_rows - index, in_cols - index) =
    in_cov.block(index, index, in_rows - index, in_cols - index);

  // Copy Rows
  out_cov.block(index + 0, 0, 3, out_cols) = out_cov.block(0, 0, 3, out_cols);
  out_cov.block(index + 3, 0, 3, out_cols) = out_cov.block(9, 0, 3, out_cols);

  // Copy Columns
  out_cov.block(0, index + 0, out_rows, 3) = out_cov.block(0, 0, out_rows, 3);
  out_cov.block(0, index + 3, out_rows, 3) = out_cov.block(0, 9, out_rows, 3);

  // Copy diagonal variances
  out_cov.block(index + 0, index + 0, 3, 3) = out_cov.block(0, 0, 3, 3);
  out_cov.block(index + 3, index + 3, 3, 3) = out_cov.block(9, 9, 3, 3);

  // Copy cross-covariances
  out_cov.block(index + 0, 0, 3, 3) = out_cov.block(0, 0, 3, 3);
  out_cov.block(0, index + 0, 3, 3) = out_cov.block(0, 0, 3, 3);
  out_cov.block(index + 3, 9, 3, 3) = out_cov.block(9, 9, 3, 3);
  out_cov.block(9, index + 3, 3, 3) = out_cov.block(9, 9, 3, 3);

  return out_cov;
}

void EKF::AugmentStateIfNeeded()
{
  if (
    m_augmenting_type == AugmentationType::ALL ||
    m_augmenting_type == AugmentationType::PRIMARY)
  {
    return;
  }

  bool augmented_state_needed {false};

  if (m_state.aug_states[0].size() == 0) {
    augmented_state_needed = true;
  } else if (m_augmenting_type == AugmentationType::TIME) {
    if (abs(m_current_time - m_augmenting_prev_time) > m_augmenting_delta_time) {
      augmented_state_needed = true;
    }
  } else if (m_augmenting_type == AugmentationType::ERROR) {
    if ((m_current_time - m_state.aug_states[0].back().time) > m_max_track_duration) {
      augmented_state_needed = true;
    } else if ((m_current_time - m_state.aug_states[0].back().time) > m_min_aug_period) {
      AugState last_aug = m_state.aug_states[0].back();
      double delta_time = m_current_time - last_aug.time;
      Eigen::Vector3d acc_in_b = m_imu_filter.GetAcc();
      Eigen::Vector3d ang_vel_in_b = m_imu_filter.GetAngVel();
      Eigen::Vector3d ang_acc_in_b = m_imu_filter.GetAngAcc();

      Eigen::Vector3d delta_pos = m_state.body_state.pos_b_in_l -
        delta_time * m_state.body_state.vel_b_in_l -
        0.5 * delta_time * delta_time * acc_in_b -
        last_aug.pos_b_in_l;

      Eigen::Vector3d rot_vec(
        ang_vel_in_b[0] * delta_time + ang_acc_in_b[0] * 0.5 * delta_time * delta_time,
        ang_vel_in_b[1] * delta_time + ang_acc_in_b[1] * 0.5 * delta_time * delta_time,
        ang_vel_in_b[2] * delta_time + ang_acc_in_b[2] * 0.5 * delta_time * delta_time);

      Eigen::Quaterniond delta_ang =
        m_state.body_state.ang_b_to_l * RotVecToQuat(rot_vec).inverse() *
        last_aug.ang_b_to_l.inverse();

      if (delta_pos.norm() > m_augmenting_pos_error || delta_ang.norm() > m_augmenting_ang_error) {
        augmented_state_needed = true;
      }
    }
  }

  // Prune old states
  for (int i = m_state.aug_states[0].size() - 1; i >= 0; --i) {
    // Check if any states are too old
    if (
      ((m_current_time - m_state.aug_states[0][i].time) > m_max_track_duration) &&
      (static_cast<unsigned int>(i) < m_state.aug_states[0].size()))
    {
      m_state_size -= g_aug_state_size;
      m_aug_state_size -= g_aug_state_size;
      unsigned int aug_index = m_state.aug_states[0][i].index;

      // Prune old states
      m_state.aug_states[0].erase(m_state.aug_states[0].begin() + i);
      m_cov = RemoveFromMatrix(m_cov, aug_index, aug_index, g_aug_state_size);

      RefreshIndices();
    }
  }

  if (augmented_state_needed) {
    m_augmenting_prev_time = m_current_time;

    AugState aug_state;
    aug_state.time = m_current_time;
    aug_state.frame_id = -1;
    aug_state.pos_b_in_l = m_state.body_state.pos_b_in_l;
    aug_state.ang_b_to_l = m_state.body_state.ang_b_to_l;
    m_state.aug_states[0].push_back(aug_state);

    m_state_size += g_aug_state_size;
    m_aug_state_size += g_aug_state_size;

    RefreshIndices();

    m_cov = AugmentCovariance(m_cov, m_state.aug_states[0].back().index);
  }
}

void EKF::AugmentStateIfNeeded(unsigned int camera_id, int frame_id)
{
  if (m_augmenting_type == AugmentationType::ALL ||
    (m_augmenting_type == AugmentationType::PRIMARY && camera_id == m_primary_camera_id))
  {
    std::stringstream msg;
    msg << "Aug State Frame: " << std::to_string(frame_id);
    m_debug_logger->Log(LogLevel::DEBUG, msg.str());

    AugState aug_state;
    aug_state.time = m_current_time;
    aug_state.frame_id = frame_id;
    aug_state.pos_b_in_l = m_state.body_state.pos_b_in_l;
    aug_state.ang_b_to_l = m_state.body_state.ang_b_to_l;
    m_state.aug_states[camera_id].push_back(aug_state);

    // Limit augmented states to m_max_track_length
    if (m_state.aug_states[camera_id].size() <= m_max_track_length) {
      m_state_size += g_aug_state_size;
      m_aug_state_size += g_aug_state_size;
    } else {
      unsigned int aug_start = m_state.aug_states[camera_id][0].index;

      // Remove first element from state
      m_state.aug_states[camera_id].erase(m_state.aug_states[camera_id].begin());

      // Remove first element from covariance
      m_cov = RemoveFromMatrix(m_cov, aug_start, aug_start, g_aug_state_size);
    }

    RefreshIndices();

    m_cov = AugmentCovariance(m_cov, m_state.aug_states[camera_id].back().index);
  }
}

void EKF::SetBodyProcessNoise(Eigen::VectorXd process_noise)
{
  m_body_process_noise = process_noise;
}

AugState EKF::GetAugState(int camera_id, int frame_id, double time)
{
  AugState aug_state;

  if (m_augmenting_type == AugmentationType::ALL) {
    for (unsigned int i = 0; i < m_state.aug_states[camera_id].size(); ++i) {
      if (m_state.aug_states[camera_id][i].frame_id == frame_id) {
        aug_state = m_state.aug_states[camera_id][i];
      }
    }
  } else {
    unsigned int aug_key;
    if (m_augmenting_type == AugmentationType::PRIMARY) {
      aug_key = m_primary_camera_id;
    } else {
      aug_key = 0;
    }

    double alpha;
    AugState aug_state_0, aug_state_1;

    if (time <= m_state.aug_states[aug_key].back().time) {
      for (unsigned int i = 0; i < m_state.aug_states[aug_key].size() - 1; ++i) {
        if (m_state.aug_states[aug_key][i].time <= time &&
          time <= m_state.aug_states[aug_key][i + 1].time)
        {
          alpha = (time - m_state.aug_states[aug_key][i].time) /
            (m_state.aug_states[aug_key][i + 1].time - m_state.aug_states[aug_key][i].time);

          aug_state_0 = m_state.aug_states[aug_key][i];
          aug_state_1 = m_state.aug_states[aug_key][i + 1];
        }
      }
    } else {
      alpha = (time - m_state.aug_states[aug_key].back().time) /
        (m_current_time - m_state.aug_states[aug_key].back().time);

      aug_state_0 = m_state.aug_states[aug_key].back();
      aug_state_1.pos_b_in_l = m_state.body_state.pos_b_in_l;
      aug_state_1.ang_b_to_l = m_state.body_state.ang_b_to_l;
    }

    Eigen::Vector3d pos_delta = aug_state_1.pos_b_in_l - aug_state_0.pos_b_in_l;

    aug_state.pos_b_in_l = aug_state_0.pos_b_in_l + alpha * pos_delta;
    aug_state.ang_b_to_l = aug_state_0.ang_b_to_l.slerp(alpha, aug_state_1.ang_b_to_l);
    aug_state.time = time;
    aug_state.index = aug_state_0.index;
    aug_state.alpha = alpha;
  }

  return aug_state;
}

void EKF::SetMaxTrackLength(unsigned int max_track_length)
{
  m_max_track_length = max_track_length;
  m_max_track_duration = m_max_frame_period * m_max_track_length;
}

void EKF::SetGpsReference(Eigen::VectorXd pos_l_in_g, double ang_l_to_g)
{
  m_pos_l_in_g = pos_l_in_g;
  m_ang_l_to_g = ang_l_to_g;
  m_is_lla_initialized = true;
}

Eigen::VectorXd EKF::GetReferenceLLA()
{
  if (!m_is_lla_initialized) {
    m_debug_logger->Log(LogLevel::WARN, "LLA is being accessed before initialization!");
  }
  return m_pos_l_in_g;
}

double EKF::GetReferenceAngle()
{
  return m_ang_l_to_g;
}

bool EKF::IsLlaInitialized()
{
  return m_is_lla_initialized;
}

bool EKF::IsGravityInitialized()
{
  return m_is_gravity_initialized;
}

void EKF::InitializeGravity()
{
  m_is_gravity_initialized = true;
}

void EKF::RefreshIndices()
{
  unsigned int current_index{0};
  m_state.body_state.index = current_index;
  current_index += g_body_state_size;

  m_process_noise = Eigen::MatrixXd::Zero(m_state_size, m_state_size);
  m_process_noise.block<g_body_state_size, g_body_state_size>(0, 0) =
    m_body_process_noise.asDiagonal();

  m_imu_state_start = current_index;
  for (auto & imu_iter : m_state.imu_states) {
    imu_iter.second.index = current_index;
    if (imu_iter.second.GetIsExtrinsic()) {
      m_process_noise.block<3, 3>(current_index + 0, current_index + 0) =
        Eigen::Matrix3d::Identity() * imu_iter.second.pos_stability;
      m_process_noise.block<3, 3>(current_index + 3, current_index + 3) =
        Eigen::Matrix3d::Identity() * imu_iter.second.ang_stability;
      imu_iter.second.index_extrinsic = current_index;
      current_index += g_imu_extrinsic_state_size;
    }
    if (imu_iter.second.GetIsIntrinsic()) {
      m_process_noise.block<3, 3>(current_index + 0, current_index + 0) =
        Eigen::Matrix3d::Identity() * imu_iter.second.acc_bias_stability;
      m_process_noise.block<3, 3>(current_index + 3, current_index + 3) =
        Eigen::Matrix3d::Identity() * imu_iter.second.omg_bias_stability;
      imu_iter.second.index_intrinsic = current_index;
      current_index += g_imu_intrinsic_state_size;
    }
  }

  m_gps_state_start = current_index;
  for (auto & gps_iter : m_state.gps_states) {
    gps_iter.second.index = current_index;
    if (gps_iter.second.GetIsExtrinsic()) {
      m_process_noise.block<3, 3>(current_index, current_index) =
        Eigen::Matrix3d::Identity() * gps_iter.second.pos_stability;
      current_index += g_gps_extrinsic_state_size;
    }
  }

  m_cam_state_start = current_index;
  for (auto & cam_iter : m_state.cam_states) {
    m_process_noise.block<3, 3>(current_index + 0, current_index + 0) =
      Eigen::Matrix3d::Identity() * cam_iter.second.pos_stability;
    m_process_noise.block<3, 3>(current_index + 3, current_index + 3) =
      Eigen::Matrix3d::Identity() * cam_iter.second.ang_stability;
    cam_iter.second.index = current_index;
    current_index += g_cam_state_size;
  }

  m_fid_state_start = current_index;
  for (auto & fid_iter : m_state.fid_states) {
    fid_iter.second.index = current_index;
    if (fid_iter.second.GetIsExtrinsic()) {
      m_process_noise.block<3, 3>(current_index + 0, current_index + 0) =
        Eigen::Matrix3d::Identity() * fid_iter.second.pos_stability;
      m_process_noise.block<3, 3>(current_index + 3, current_index + 3) =
        Eigen::Matrix3d::Identity() * fid_iter.second.ang_stability;
      current_index += g_fid_extrinsic_state_size;
    }
  }

  m_aug_state_start = current_index;
  for (auto & aug_iter : m_state.aug_states) {
    unsigned int aug_id = aug_iter.first;
    for (unsigned int i = 0; i < m_state.aug_states[aug_id].size(); ++i) {
      m_state.aug_states[aug_id][i].index = current_index;
      current_index += g_aug_state_size;
    }
  }

  if (m_use_root_covariance) {
    m_process_noise = m_process_noise.cwiseSqrt();
  }
}

void EKF::AttemptGpsInitialization(
  double time,
  Eigen::Vector3d gps_lla)
{
  Eigen::Vector3d gps_ecef = lla_to_ecef(gps_lla);

  m_gps_time_vec.push_back(time);
  m_gps_ecef_vec.push_back(gps_ecef);
  m_gps_xyz_vec.push_back(m_state.body_state.pos_b_in_l);

  if (m_gps_time_vec.size() >= 4) {
    Eigen::Vector3d init_ref_ecef = average_vectors(m_gps_ecef_vec);
    Eigen::Vector3d init_ref_lla = ecef_to_lla(init_ref_ecef);

    std::vector<Eigen::Vector3d> gps_states_enu;
    for (auto gps_ecef : m_gps_ecef_vec) {
      gps_states_enu.push_back(ecef_to_enu(gps_ecef, init_ref_lla));
    }

    double pos_stddev;
    double ang_stddev;
    Eigen::Affine3d transformation;
    bool is_successful = kabsch_2d(
      m_gps_xyz_vec,
      gps_states_enu,
      transformation,
      pos_stddev,
      ang_stddev);

    double max_distance = maximum_distance(gps_states_enu);

    if (((m_gps_init_type == GpsInitType::BASELINE_DIST) &&
      (max_distance > m_gps_init_baseline_dist)) ||
      ((m_gps_init_type == GpsInitType::ERROR_THRESHOLD) && is_successful &&
      (pos_stddev < m_gps_init_pos_thresh) && ang_stddev &&
      (ang_stddev < std::tan(m_gps_init_ang_thresh))))
    {
      Eigen::Vector3d delta_ref_enu = transformation.translation();
      Eigen::Vector3d reference_lla = enu_to_lla(-delta_ref_enu, init_ref_lla);
      double ang_l_to_g = affine_angle(transformation);

      m_debug_logger->Log(LogLevel::INFO, "GPS Updater Initialized");
      SetGpsReference(reference_lla, ang_l_to_g);
    }
  }
}

std::vector<double> EKF::GetGpsTimeVector()
{
  return m_gps_time_vec;
}
std::vector<Eigen::Vector3d> EKF::GetGpsEcefVector()
{
  return m_gps_ecef_vec;
}
std::vector<Eigen::Vector3d> EKF::GetGpsXyzVector()
{
  return m_gps_xyz_vec;
}

unsigned int EKF::GetImuStateStart()
{
  return m_imu_state_start;
}

unsigned int EKF::GetGpsStateStart()
{
  return m_gps_state_start;
}

unsigned int EKF::GetCamStateStart()
{
  return m_cam_state_start;
}

unsigned int EKF::GetAugStateStart()
{
  return m_aug_state_start;
}

unsigned int EKF::GetFidStateStart()
{
  return m_fid_state_start;
}

double EKF::GetCurrentTime()
{
  return m_current_time;
}

double EKF::GetMotionDetectionChiSquared()
{
  return m_motion_detection_chi_squared;
}

double EKF::GetImuNoiseScaleFactor()
{
  return m_imu_noise_scale_factor;
}

bool EKF::GetUseRootCovariance()
{
  return m_use_root_covariance;
}

bool EKF::GetUseFirstEstimateJacobian()
{
  return m_use_first_estimate_jacobian;
}
