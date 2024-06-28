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
#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "utility/math_helper.hpp"
#include "utility/string_helper.hpp"
#include "utility/type_helper.hpp"

MsckfUpdater::MsckfUpdater(
  int cam_id,
  std::string log_file_directory,
  bool data_logging_on,
  double data_log_rate,
  double min_feat_dist,
  std::shared_ptr<DebugLogger> logger
)
: Updater(cam_id, logger),
  m_msckf_logger(log_file_directory, "msckf_" + std::to_string(cam_id) + ".csv"),
  m_triangulation_logger(log_file_directory, "triangulation_" + std::to_string(cam_id) + ".csv")
{
  std::stringstream header;
  header << "time";
  header << EnumerateHeader("cam_pos", 3);
  header << EnumerateHeader("cam_ang_pos", 4);
  header << EnumerateHeader("body_update", g_body_state_size);
  header << EnumerateHeader("cam_update", g_cam_state_size);
  header << EnumerateHeader("cam_cov", g_cam_state_size);
  header << ",FeatureTracks";
  header << EnumerateHeader("duration", 1);

  m_msckf_logger.DefineHeader(header.str());
  m_msckf_logger.SetLogging(data_logging_on);

  m_triangulation_logger.DefineHeader("time,feature,x,y,z");
  m_triangulation_logger.SetLogging(data_logging_on);
  m_triangulation_logger.SetLogRate(data_log_rate);

  m_min_feat_dist = min_feat_dist;
}

Eigen::Vector3d MsckfUpdater::TriangulateFeature(
  std::shared_ptr<EKF> ekf,
  std::vector<FeaturePoint> & feature_track)
{
  AugState aug_state_0 = ekf->GetAugState(m_id, feature_track[0].frame_id);
  CamState cam_state = ekf->m_state.cam_states[m_id];
  Intrinsics intrinsics = ekf->m_state.cam_states[m_id].intrinsics;

  // 3D Cartesian Triangulation
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  const Eigen::Vector3d position_i0_in_l = aug_state_0.pos_b_in_l;
  const Eigen::Matrix3d rotation_i0_to_l = aug_state_0.ang_b_to_l.toRotationMatrix();
  const Eigen::Vector3d position_c0_in_i0 = cam_state.pos_c_in_b;
  const Eigen::Matrix3d rotation_c0_to_i0 = cam_state.ang_c_to_b.toRotationMatrix();

  const Eigen::Matrix3d rotation_l_to_i0 = rotation_i0_to_l.transpose();
  const Eigen::Matrix3d rotation_i0_to_c0 = rotation_c0_to_i0.transpose();

  for (unsigned int i = 0; i < feature_track.size(); ++i) {
    AugState aug_state_i = ekf->GetAugState(m_id, feature_track[i].frame_id);

    const Eigen::Vector3d position_bi_in_l = aug_state_i.pos_b_in_l;
    const Eigen::Matrix3d rotation_bi_to_l = aug_state_i.ang_b_to_l.toRotationMatrix();
    const Eigen::Vector3d position_ci_in_bi = cam_state.pos_c_in_b;
    const Eigen::Matrix3d rotation_ci_to_bi = cam_state.ang_c_to_b.toRotationMatrix();

    // Convert current position relative to anchor
    Eigen::Matrix3d rotation_ci_to_c0 =
      rotation_i0_to_c0 * rotation_l_to_i0 * rotation_bi_to_l * rotation_ci_to_bi;
    Eigen::Vector3d position_ci_in_c0 =
      rotation_i0_to_c0 * rotation_l_to_i0 *
      (
      (rotation_bi_to_l * position_ci_in_bi + position_bi_in_l) -
      (rotation_i0_to_l * position_c0_in_i0 + position_i0_in_l));

    // Get the UV coordinate normal
    Eigen::Vector3d b_i;
    b_i(0) = (feature_track[i].key_point.pt.x - (static_cast<double>(intrinsics.width) / 2)) /
      (intrinsics.f_x / intrinsics.pixel_size);
    b_i(1) = (feature_track[i].key_point.pt.y - (static_cast<double>(intrinsics.height) / 2)) /
      (intrinsics.f_y / intrinsics.pixel_size);
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
  Eigen::Vector3d position_f_in_l =
    rotation_i0_to_l * (rotation_c0_to_i0 * position_f_in_c0 + position_c0_in_i0) +
    position_i0_in_l;

  /// @todo condition check
  /// @todo max and min distance check

  return position_f_in_l;
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
  std::shared_ptr<EKF> ekf,
  double time,
  FeatureTracks feature_tracks,
  double px_error)
{
  ekf->ProcessModel(time);

  auto t_start = std::chrono::high_resolution_clock::now();

  m_logger->Log(LogLevel::DEBUG, "Called MSCKF Update for camera ID: " + std::to_string(m_id));

  if (feature_tracks.size() == 0) {
    return;
  }

  // Calculate the max possible measurement size
  unsigned int max_meas_size = 0;
  for (unsigned int i = 0; i < feature_tracks.size(); ++i) {
    max_meas_size += 2 * feature_tracks[i].size();
  }

  unsigned int ct_meas = 0;
  unsigned int state_size = ekf->GetStateSize();
  unsigned int cam_index = ekf->m_state.cam_states[m_id].index;
  Intrinsics intrinsics = ekf->m_state.cam_states[m_id].intrinsics;

  Eigen::VectorXd res_x = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(max_meas_size, state_size);

  m_logger->Log(LogLevel::DEBUG, "Update track count: " + std::to_string(feature_tracks.size()));

  // MSCKF Update
  for (auto & feature_track : feature_tracks) {
    m_logger->Log(LogLevel::DEBUG, "Feature Track size: " + std::to_string(feature_track.size()));

    // Get triangulated estimate of feature position
    Eigen::Vector3d pos_f_in_l = TriangulateFeature(ekf, feature_track);

    /// @todo Additional non-linear optimization

    if (pos_f_in_l.norm() < m_min_feat_dist) {
      std::stringstream err_msg;
      err_msg << "MSCKF Triangulated Point is too close. r = " << pos_f_in_l.norm();
      m_logger->Log(LogLevel::INFO, err_msg.str());
      continue;
    }

    std::stringstream msg;
    msg << std::setprecision(3) << time;
    msg << "," << std::to_string(feature_track[0].key_point.class_id);
    msg << "," << pos_f_in_l[0];
    msg << "," << pos_f_in_l[1];
    msg << "," << pos_f_in_l[2];
    m_triangulation_logger.RateLimitedLog(msg.str(), time);

    unsigned int aug_state_size = ekf->GetAugStateSize();
    CamState cam_state = ekf->m_state.cam_states[m_id];
    Eigen::VectorXd res_f = Eigen::VectorXd::Zero(2 * feature_track.size());
    Eigen::MatrixXd H_f = Eigen::MatrixXd::Zero(2 * feature_track.size(), 3);
    Eigen::MatrixXd H_c = Eigen::MatrixXd::Zero(
      2 * feature_track.size(), g_cam_state_size + aug_state_size);

    for (unsigned int i = 0; i < feature_track.size(); ++i) {
      AugState aug_state_i = ekf->GetAugState(m_id, feature_track[i].frame_id);

      Eigen::Matrix3d rot_ci_to_bi = cam_state.ang_c_to_b.toRotationMatrix();
      Eigen::Matrix3d rot_bi_to_l = aug_state_i.ang_b_to_l.toRotationMatrix();
      Eigen::Matrix3d rot_bi_to_ci = rot_ci_to_bi.transpose();
      Eigen::Matrix3d rot_l_to_ci = rot_bi_to_ci * rot_bi_to_l.transpose();

      Eigen::Vector3d pos_ci_in_bi = cam_state.pos_c_in_b;
      Eigen::Vector3d pos_bi_in_l = aug_state_i.pos_b_in_l;

      // Project the current feature into the current frame of reference
      Eigen::Vector3d pos_f_in_bi = rot_bi_to_l.transpose() * (pos_f_in_l - pos_bi_in_l);
      Eigen::Vector3d pos_f_in_ci = rot_ci_to_bi.transpose() * (pos_f_in_bi - pos_ci_in_bi);
      Eigen::Vector2d xz_predicted;
      xz_predicted(0) = pos_f_in_ci(0) / pos_f_in_ci(2);
      xz_predicted(1) = pos_f_in_ci(1) / pos_f_in_ci(2);

      Eigen::Vector2d xz_measured, xz_residual;
      xz_measured(0) = (feature_track[i].key_point.pt.x - intrinsics.c_x) / intrinsics.f_x;
      xz_measured(1) = (feature_track[i].key_point.pt.y - intrinsics.c_y) / intrinsics.f_y;
      xz_residual = xz_measured - xz_predicted;
      res_f.segment<2>(2 * i) = xz_residual;

      unsigned int aug_index = ekf->GetAugState(m_id, feature_track[i].frame_id).index;

      // Projection Jacobian
      Eigen::MatrixXd H_p(2, 3);
      projection_jacobian(pos_f_in_ci, H_p);

      // Distortion Jacobian
      Eigen::MatrixXd H_d(2, 2);
      distortion_jacobian(xz_measured, intrinsics, H_d);

      // Entire feature Jacobian
      H_f.block<2, 3>(2 * i, 0) = H_d * H_p * rot_l_to_ci;

      // Augmented state Jacobian
      Eigen::MatrixXd H_t = Eigen::MatrixXd::Zero(3, 12);
      H_t.block<3, 3>(0, 0) = -rot_l_to_ci;
      H_t.block<3, 3>(0, 3) = rot_bi_to_ci * SkewSymmetric(pos_f_in_bi);
      /// @todo(jhartzer): Enable calibration Jacobian
      // H_t.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity(3, 3);
      // H_t.block<3, 3>(0, 9) =
      //   SkewSymmetric(rot_bi_to_ci * rot_bi_to_l.transpose() * (pos_f_in_l - pos_bi_in_l));

      H_c.block<2, 12>(2 * i, aug_index - cam_index) = H_d * H_p * H_t;
    }
    ApplyLeftNullspace(H_f, H_c, res_f);

    /// @todo Chi^2 distance check

    // Append Jacobian and residual
    H_x.block(ct_meas, cam_index, H_c.rows(), H_c.cols()) = H_c;
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
  Eigen::MatrixXd S = H_x * ekf->m_cov * H_x.transpose() + R;
  Eigen::MatrixXd K = ekf->m_cov * H_x.transpose() * S.inverse();

  unsigned int imu_states_size = ekf->GetImuStateSize();
  unsigned int cam_states_size = state_size - g_body_state_size - imu_states_size;

  Eigen::VectorXd update = K * res_x;
  Eigen::VectorXd body_update = update.segment<g_body_state_size>(0);
  Eigen::VectorXd imu_update = update.segment(g_body_state_size, imu_states_size);
  Eigen::VectorXd cam_update = update.segment(g_body_state_size + imu_states_size, cam_states_size);

  ekf->m_state.body_state += body_update;
  ekf->m_state.imu_states += imu_update;
  ekf->m_state.cam_states += cam_update;

  ekf->m_cov =
    (Eigen::MatrixXd::Identity(state_size, state_size) - K * H_x) * ekf->m_cov *
    (Eigen::MatrixXd::Identity(state_size, state_size) - K * H_x).transpose() +
    K * R * K.transpose();

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  Eigen::VectorXd cam_state_vec = ekf->m_state.cam_states[m_id].ToVector();
  Eigen::Vector3d cam_pos = cam_state_vec.segment<3>(0);
  Eigen::Quaterniond cam_ang_pos = RotVecToQuat(cam_state_vec.segment<3>(3));
  Eigen::VectorXd cam_sub_update = update.segment(cam_index, g_cam_state_size);
  Eigen::VectorXd cov_diag = ekf->m_cov.block(
    cam_index, cam_index, g_cam_state_size, g_cam_state_size).diagonal();

  std::stringstream msg;
  msg << time;
  msg << VectorToCommaString(cam_pos);
  msg << QuaternionToCommaString(cam_ang_pos);
  msg << VectorToCommaString(body_update);
  msg << VectorToCommaString(cam_update.segment(0, g_cam_state_size));
  msg << VectorToCommaString(cov_diag);
  msg << "," << std::to_string(feature_tracks.size());
  msg << "," << t_execution.count();
  m_msckf_logger.RateLimitedLog(msg.str(), time);
}
