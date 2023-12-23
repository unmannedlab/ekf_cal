// Copyright 2023 Jacob Hartzer
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

#include "ekf/update/msckf_updater.hpp"

#include <eigen3/Eigen/Eigen>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <ostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "ekf/constants.hpp"
#include "ekf/ekf.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/types.hpp"
#include "utility/math_helper.hpp"
#include "utility/string_helper.hpp"

MsckfUpdater::MsckfUpdater(
  int cam_id, Intrinsics intrinsics, std::string log_file_directory, bool data_logging_on)
: Updater(cam_id),
  m_data_logger(log_file_directory, "camera_" + std::to_string(cam_id) + ".csv"),
  m_triangulation_logger(log_file_directory, "triangulation_" + std::to_string(cam_id) + ".csv")
{
  std::stringstream msg;
  msg << "time";
  msg << EnumerateHeader("cam_state", g_cam_state_size);
  msg << EnumerateHeader("body_update", g_body_state_size);
  msg << EnumerateHeader("cam_update", g_cam_state_size);
  msg << ",FeatureTracks";
  msg << EnumerateHeader("time", 1);
  msg << std::endl;

  m_data_logger.DefineHeader(msg.str());
  m_data_logger.SetLogging(data_logging_on);

  m_triangulation_logger.DefineHeader("time,feature,x,y,z\n");
  m_triangulation_logger.SetLogging(data_logging_on);

  m_intrinsics = intrinsics;
}

AugmentedState MsckfUpdater::MatchState(int frame_id)
{
  AugmentedState aug_state_match;

  for (auto & aug_state : m_aug_states) {
    if (aug_state.frame_id == frame_id) {
      return aug_state;
    }
  }

  std::stringstream warning_msg;
  warning_msg << "No matching augmented state for frame " << frame_id;
  m_logger->Log(LogLevel::WARN, warning_msg.str());
  return aug_state_match;
}

/// @todo possible move into separate source for re-compilation speed
/// @todo remove class members from function and use reference inputs instead
Eigen::Vector3d MsckfUpdater::TriangulateFeature(std::vector<FeatureTrack> & feature_track)
{
  AugmentedState aug_state_0 = MatchState(feature_track[0].frame_id);

  // 3D Cartesian Triangulation
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  const Eigen::Vector3d position_i0_in_g = aug_state_0.pos_b_in_g;
  const Eigen::Matrix3d rotation_i0_to_g = aug_state_0.ang_b_to_g.toRotationMatrix();
  const Eigen::Vector3d position_c0_in_i0 = aug_state_0.pos_c_in_b;
  const Eigen::Matrix3d rotation_c0_to_i0 = aug_state_0.ang_c_to_b.toRotationMatrix();

  const Eigen::Matrix3d rotation_g_to_i0 = rotation_i0_to_g.transpose();
  const Eigen::Matrix3d rotation_i0_to_c0 = rotation_c0_to_i0.transpose();

  for (unsigned int i = 0; i < feature_track.size(); ++i) {
    AugmentedState aug_state_i = MatchState(feature_track[i].frame_id);

    /// @todo(jhartzer): Need to acknowledge this isn't really an IMU orientation, but a body
    const Eigen::Vector3d position_ii_in_g = aug_state_i.pos_b_in_g;
    const Eigen::Matrix3d rotation_ii_to_g = aug_state_i.ang_b_to_g.toRotationMatrix();
    const Eigen::Vector3d position_ci_in_ii = aug_state_i.pos_c_in_b;
    const Eigen::Matrix3d rotation_ci_to_ii = aug_state_i.ang_c_to_b.toRotationMatrix();

    // Convert current position relative to anchor
    Eigen::Matrix3d rotation_ci_to_c0 =
      rotation_i0_to_c0 * rotation_g_to_i0 * rotation_ii_to_g * rotation_ci_to_ii;
    Eigen::Vector3d position_ci_in_c0 =
      rotation_i0_to_c0 * rotation_g_to_i0 *
      (
      (rotation_ii_to_g * position_ci_in_ii + position_ii_in_g) -
      (rotation_i0_to_g * position_c0_in_i0 + position_i0_in_g));

    // Get the UV coordinate normal
    Eigen::Vector3d b_i;
    b_i(0) = (feature_track[i].key_point.pt.x - (static_cast<double>(m_intrinsics.width) / 2)) /
      (m_intrinsics.f_x / m_intrinsics.pixel_size);
    b_i(1) = (feature_track[i].key_point.pt.y - (static_cast<double>(m_intrinsics.height) / 2)) /
      (m_intrinsics.f_y / m_intrinsics.pixel_size);
    b_i(2) = 1;

    // Rotate and normalize
    b_i = rotation_ci_to_c0 * b_i;
    b_i = b_i / b_i.norm();

    Eigen::Matrix3d b_i_skew = SkewSymmetric(b_i);
    Eigen::Matrix3d A_i = b_i_skew.transpose() * b_i_skew;
    A += A_i;
    b += A_i * position_ci_in_c0;
  }

  // Solve linear triangulation for 3D cartesian estimate of feature position
  Eigen::Vector3d position_f_in_c0 = A.colPivHouseholderQr().solve(b);
  Eigen::Vector3d position_f_in_g =
    rotation_i0_to_g * (rotation_c0_to_i0 * position_f_in_c0 + position_c0_in_i0) +
    position_i0_in_g;

  /// @todo condition check
  /// @todo max and min distance check

  return position_f_in_g;
}

void MsckfUpdater::projection_jacobian(const Eigen::Vector3d & position, Eigen::MatrixXd & jacobian)
{
  // Normalized coordinates in respect to projection function
  jacobian(0, 0) = 1 / position(2);
  jacobian(1, 1) = 1 / position(2);
  jacobian(0, 1) = 0.0;
  jacobian(1, 0) = 0.0;
  jacobian(0, 2) = -position(0) / (position(2) * position(2));
  jacobian(1, 2) = -position(1) / (position(2) * position(2));
}

void MsckfUpdater::distortion_jacobian(
  const Eigen::Vector2d & xy_norm,
  Intrinsics intrinsics,
  Eigen::MatrixXd & H_d)
{
  // Calculate distorted coordinates for radial
  double r = std::sqrt(xy_norm(0) * xy_norm(0) + xy_norm(1) * xy_norm(1));
  double r_2 = r * r;
  double r_4 = r_2 * r_2;

  // Jacobian of distorted pixel to normalized pixel
  H_d = Eigen::MatrixXd::Zero(2, 2);
  double x = xy_norm(0);
  double y = xy_norm(1);
  double x_2 = xy_norm(0) * xy_norm(0);
  double y_2 = xy_norm(1) * xy_norm(1);
  double x_y = xy_norm(0) * xy_norm(1);

  H_d(0, 0) =
    intrinsics.f_x * (
    (1 + intrinsics.k_1 * r_2 + intrinsics.k_2 * r_4) +
    (2 * intrinsics.k_1 * x_2 + 4 * intrinsics.k_2 * x_2 * r_2) +
    (2 * intrinsics.p_1 * y) +
    (2 * intrinsics.p_2 * x) +
    (4 * intrinsics.p_2 * x));

  H_d(0, 1) =
    intrinsics.f_x * (
    (2 * intrinsics.k_1 * x_y) +
    (4 * intrinsics.k_2 * x_y * r_2) +
    (2 * intrinsics.p_1 * x) +
    (2 * intrinsics.p_2 * y));

  H_d(1, 0) =
    intrinsics.f_y * (
    (2 * intrinsics.k_1 * x_y) +
    (4 * intrinsics.k_2 * x_y * r_2) +
    (2 * intrinsics.p_1 * x) +
    (2 * intrinsics.p_2 * y));

  H_d(1, 1) =
    intrinsics.f_y * (
    (1 + intrinsics.k_1 * r_2 + intrinsics.k_2 * r_4) +
    (2 * intrinsics.k_1 * y_2) +
    (4 * intrinsics.k_2 * y_2 * r_2) +
    (2 * intrinsics.p_2 * x) +
    (2 * intrinsics.p_1 * y) +
    (4 * intrinsics.p_1 * y));
}

void MsckfUpdater::UpdateEKF(
  double time,
  int camera_id,
  FeatureTracks feature_tracks,
  double px_error)
{
  m_ekf->ProcessModel(time);
  RefreshStates();
  auto t_start = std::chrono::high_resolution_clock::now();

  m_logger->Log(LogLevel::DEBUG, "Called update_msckf for camera ID: " + std::to_string(camera_id));

  if (feature_tracks.size() == 0) {
    return;
  }

  // Calculate the max possible measurement size
  unsigned int max_meas_size = 0;
  for (unsigned int i = 0; i < feature_tracks.size(); ++i) {
    max_meas_size += 2 * feature_tracks[i].size();
  }

  unsigned int ct_meas = 0;
  unsigned int state_size = m_ekf->GetState().GetStateSize();
  unsigned int cam_state_start = m_ekf->GetCamStateStartIndex(camera_id);

  Eigen::VectorXd res_x = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(max_meas_size, state_size);

  m_logger->Log(LogLevel::DEBUG, "Update track count: " + std::to_string(feature_tracks.size()));

  // MSCKF Update
  for (auto & feature_track : feature_tracks) {
    m_logger->Log(LogLevel::DEBUG, "Feature Track size: " + std::to_string(feature_track.size()));

    // Get triangulated estimate of feature position
    Eigen::Vector3d pos_f_in_g = TriangulateFeature(feature_track);

    /// @todo Additional non-linear optimization

    /// @todo(jhartzer): Get this from input file?
    if (pos_f_in_g.norm() < 1.0) {
      std::stringstream err_msg;
      err_msg << "MSCKF Triangulated Point is too close. r = " << pos_f_in_g.norm();
      m_logger->Log(LogLevel::INFO, err_msg.str());
      continue;
    }

    std::stringstream msg;
    msg << std::setprecision(3) << time;
    msg << "," << std::to_string(feature_track[0].key_point.class_id);
    msg << "," << pos_f_in_g[0];
    msg << "," << pos_f_in_g[1];
    msg << "," << pos_f_in_g[2];
    msg << std::endl;
    m_triangulation_logger.Log(msg.str());

    unsigned int aug_state_size = g_aug_state_size *
      m_ekf->GetCamState(camera_id).augmented_states.size();
    Eigen::VectorXd res_f = Eigen::VectorXd::Zero(2 * feature_track.size());
    Eigen::MatrixXd H_f = Eigen::MatrixXd::Zero(2 * feature_track.size(), 3);
    Eigen::MatrixXd H_c = Eigen::MatrixXd::Zero(
      2 * feature_track.size(), g_cam_state_size + aug_state_size);

    for (unsigned int i = 0; i < feature_track.size(); ++i) {
      AugmentedState aug_state_i = MatchState(feature_track[i].frame_id);

      // Our calibration between the IMU and CAMi frames
      Eigen::Matrix3d rot_ci_to_ii = aug_state_i.ang_c_to_b.toRotationMatrix();
      Eigen::Matrix3d rot_ii_to_g = aug_state_i.ang_b_to_g.toRotationMatrix();
      Eigen::Matrix3d rot_ii_to_ci = rot_ci_to_ii.transpose();
      Eigen::Matrix3d rot_g_to_ci = rot_ii_to_ci * rot_ii_to_g.transpose();

      Eigen::Vector3d pos_ci_in_ii = aug_state_i.pos_c_in_b;
      Eigen::Vector3d pos_ii_in_g = aug_state_i.pos_b_in_g;

      // Project the current feature into the current frame of reference
      Eigen::Vector3d pos_f_in_ii = rot_ii_to_g.transpose() * (pos_f_in_g - pos_ii_in_g);
      Eigen::Vector3d pos_f_in_ci = rot_ci_to_ii.transpose() * (pos_f_in_ii - pos_ci_in_ii);
      Eigen::Vector2d xz_predicted;
      xz_predicted(0) = pos_f_in_ci(0) / pos_f_in_ci(2);
      xz_predicted(1) = pos_f_in_ci(1) / pos_f_in_ci(2);

      // Our residual
      Eigen::Vector2d xz_measured, xz_residual;
      xz_measured(0) = (feature_track[i].key_point.pt.x - m_intrinsics.c_x) / m_intrinsics.f_x;
      xz_measured(1) = (feature_track[i].key_point.pt.y - m_intrinsics.c_y) / m_intrinsics.f_y;
      xz_residual = xz_measured - xz_predicted;
      res_f.segment<2>(2 * i) = xz_residual;

      unsigned int aug_state_start =
        m_ekf->GetAugStateStartIndex(camera_id, feature_track[i].frame_id);

      // Projection Jacobian
      Eigen::MatrixXd H_p(2, 3);
      projection_jacobian(pos_f_in_ci, H_p);

      // Distortion Jacobian
      Eigen::MatrixXd H_d(2, 2);
      distortion_jacobian(xz_measured, m_intrinsics, H_d);

      // Entire feature Jacobian
      H_f.block<2, 3>(2 * i, 0) = H_d * H_p * rot_g_to_ci;

      // Augmented state Jacobian
      Eigen::MatrixXd H_t = Eigen::MatrixXd::Zero(3, 12);
      H_t.block<3, 3>(0, 0) = -rot_g_to_ci;
      H_t.block<3, 3>(0, 3) = rot_ii_to_ci * SkewSymmetric(pos_f_in_ii);
      /// @todo(jhartzer): Enable calibration Jacobian
      // H_t.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
      // H_t.block<3, 3>(0, 9) =
      //   SkewSymmetric(rot_ii_to_ci * rot_ii_to_g.transpose() * (pos_f_in_g - pos_ii_in_g));

      H_c.block<2, 12>(2 * i, aug_state_start - cam_state_start) = H_d * H_p * H_t;
    }
    ApplyLeftNullspace(H_f, H_c, res_f);

    /// @todo Chi^2 distance check

    // Append our Jacobian and residual
    H_x.block(ct_meas, cam_state_start, H_c.rows(), H_c.cols()) = H_c;
    res_x.block(ct_meas, 0, res_f.rows(), 1) = res_f;

    ct_meas += H_c.rows();
  }

  if (ct_meas == 0) {
    return;
  }

  CompressMeasurements(H_x, res_x);

  // Jacobian is ill-formed if either rows or columns post-compression are size 1
  if (res_x.size() == 1) {
    m_logger->Log(LogLevel::INFO, "Compressed MSCKF Jacobian is ill-formed");
    return;
  }

  Eigen::MatrixXd R = px_error * px_error * Eigen::MatrixXd::Identity(res_x.rows(), res_x.rows());

  // Apply Kalman update
  Eigen::MatrixXd S = H_x * m_ekf->GetCov() * H_x.transpose() + R;
  Eigen::MatrixXd K = m_ekf->GetCov() * H_x.transpose() * S.inverse();

  unsigned int imu_states_size = m_ekf->GetImuStateSize();
  unsigned int cam_states_size = state_size - g_body_state_size - imu_states_size;

  Eigen::VectorXd update = K * res_x;
  Eigen::VectorXd body_update = update.segment<g_body_state_size>(0);
  Eigen::VectorXd imu_update = update.segment(g_body_state_size, imu_states_size);
  Eigen::VectorXd cam_update = update.segment(g_body_state_size + imu_states_size, cam_states_size);

  m_ekf->GetState().m_body_state += body_update;
  m_ekf->GetState().m_imu_states += imu_update;
  m_ekf->GetState().m_cam_states += cam_update;

  m_ekf->GetCov() =
    (Eigen::MatrixXd::Identity(state_size, state_size) - K * H_x) * m_ekf->GetCov();

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;
  Eigen::VectorXd cam_state = m_ekf->GetState().m_cam_states[camera_id].ToVector();
  Eigen::VectorXd cam_sub_update = update.segment(cam_state_start, g_cam_state_size);

  msg << time;
  msg << VectorToCommaString(cam_state.segment(0, g_cam_state_size));
  msg << VectorToCommaString(body_update);
  msg << VectorToCommaString(cam_update.segment(0, g_cam_state_size));
  msg << "," << std::to_string(feature_tracks.size());
  msg << "," << t_execution.count();
  msg << std::endl;
  m_data_logger.Log(msg.str());
}

void MsckfUpdater::RefreshStates()
{
  BodyState body_state = m_ekf->GetBodyState();
  m_body_pos = body_state.m_position;
  m_body_vel = body_state.m_velocity;
  m_body_acc = body_state.m_acceleration;
  m_ang_b_to_g = body_state.m_ang_b_to_g;
  m_body_ang_vel = body_state.m_angular_velocity;
  m_body_ang_acc = body_state.m_angular_acceleration;

  CamState cam_state = m_ekf->GetCamState(m_id);
  m_pos_c_in_b = cam_state.pos_c_in_b;
  m_ang_c_to_b = cam_state.ang_c_to_b;
  m_aug_states = cam_state.augmented_states;
}
