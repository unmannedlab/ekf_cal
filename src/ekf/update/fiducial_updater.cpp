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

#include "ekf/update/fiducial_updater.hpp"

#include <eigen3/Eigen/Eigen>

#include <algorithm>
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

FiducialUpdater::FiducialUpdater(
  int cam_id,
  std::string log_file_directory,
  bool data_logging_on,
  double data_log_rate,
  std::shared_ptr<DebugLogger> logger
)
: Updater(cam_id, logger),
  m_fiducial_logger(log_file_directory, "fiducial_" + std::to_string(cam_id) + ".csv"),
  m_triangulation_logger(log_file_directory, "triangulation_" + std::to_string(cam_id) + ".csv")
{
  std::stringstream header;
  header << "time";
  header << ",track_size";
  header << EnumerateHeader("cam_pos", 3);
  header << EnumerateHeader("cam_ang_pos", 4);
  header << EnumerateHeader("residual", g_fid_measurement_size);
  header << EnumerateHeader("body_update", g_body_state_size);
  header << EnumerateHeader("cam_update", g_cam_state_size);
  header << EnumerateHeader("cam_cov", g_cam_state_size);
  header << EnumerateHeader("duration", 1);

  m_fiducial_logger.DefineHeader(header.str());
  m_fiducial_logger.SetLogging(data_logging_on);

  m_triangulation_logger.DefineHeader("time,board,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z");
  m_triangulation_logger.SetLogging(data_logging_on);
  m_triangulation_logger.SetLogRate(data_log_rate);
}

void FiducialUpdater::UpdateEKF(
  std::shared_ptr<EKF> ekf, double time,
  BoardTrack board_track, double pos_error, double ang_error)
{
  m_logger->Log(
    LogLevel::DEBUG, "Called Fiducial Update for camera ID: " + std::to_string(m_id));

  if (board_track.size() == 0) {
    return;
  }

  ekf->ProcessModel(time);

  auto t_start = std::chrono::high_resolution_clock::now();

  std::vector<double> pos_weights;
  std::vector<double> ang_weights;
  std::vector<Eigen::Vector3d> pos_f_in_l_vec;
  std::vector<Eigen::Quaterniond> ang_f_to_l_vec;

  /// @todo(jhartzer): Wrap this in a function
  for (auto board_detection : board_track) {
    AugState aug_state_i = ekf->GetAugState(m_id, board_detection.frame_id);

    const Eigen::Vector3d pos_bi_in_g = aug_state_i.pos_b_in_l;
    const Eigen::Matrix3d rot_bi_to_l = aug_state_i.ang_b_to_l.toRotationMatrix();
    const Eigen::Vector3d pos_ci_in_bi = aug_state_i.pos_c_in_b;
    const Eigen::Matrix3d rot_ci_to_bi = aug_state_i.ang_c_to_b.toRotationMatrix();

    Eigen::Vector3d pos_f_in_c;
    CvVectorToEigen(board_detection.t_vec_f_in_c, pos_f_in_c);
    Eigen::Vector3d pos_f_in_l =
      rot_bi_to_l * ((rot_ci_to_bi * pos_f_in_c) + pos_ci_in_bi) + pos_bi_in_g;
    pos_f_in_l_vec.push_back(pos_f_in_l);

    Eigen::Quaterniond ang_f_to_c = RodriguesToQuat(board_detection.r_vec_f_to_c);
    Eigen::Quaterniond ang_f_to_l = aug_state_i.ang_b_to_l * aug_state_i.ang_c_to_b * ang_f_to_c;
    ang_f_to_l_vec.push_back(ang_f_to_l);
    pos_weights.push_back(1.0);
    ang_weights.push_back(1.0);

    std::stringstream data_msg;
    data_msg << std::setprecision(3) << time;
    data_msg << ",0," << pos_f_in_l[0];
    data_msg << "," << pos_f_in_l[1];
    data_msg << "," << pos_f_in_l[2];
    data_msg << "," << ang_f_to_l.w();
    data_msg << "," << ang_f_to_l.x();
    data_msg << "," << ang_f_to_l.y();
    data_msg << "," << ang_f_to_l.z();
    m_triangulation_logger.RateLimitedLog(data_msg.str(), time);
  }

  Eigen::Vector3d pos_f_in_l_est = ekf->m_state.fid_states[m_id].pos_f_in_l;
  Eigen::Quaterniond ang_f_to_l_est = ekf->m_state.fid_states[m_id].ang_f_to_l;

  // Eigen::Vector3d pos_f_in_l_est = average_vectors(pos_f_in_l_vec, pos_weights);
  // Eigen::Quaterniond ang_f_to_l_est = average_quaternions(ang_f_to_l_vec, ang_weights);

  // /// Project fiducial onto 3-sigma error bound
  // Eigen::Vector3d f_pos_delta = pos_f_in_l_est - ekf->m_state.fid_states[m_id].pos_f_in_l;
  // Eigen::Quaterniond f_ang_delta =
  //   ang_f_to_l_est * ekf->m_state.fid_states[m_id].ang_f_to_l.inverse();
  // Eigen::AngleAxisd f_ang_delta_vec{f_ang_delta};
  // if (f_pos_delta.norm() > 3 * pos_error) {
  //   f_pos_delta = (3 * pos_error) / f_pos_delta.norm() * f_pos_delta;
  //   Eigen::Vector3d pos_f_in_l_est_temp = ekf->m_state.fid_states[m_id].pos_f_in_l + f_pos_delta;
  //   pos_f_in_l_est = pos_f_in_l_est_temp;
  // }
  // if (f_ang_delta_vec.angle() > 3 * ang_error) {
  //   f_ang_delta_vec.angle() = 3 * ang_error;
  //   f_ang_delta = Eigen::Quaterniond(f_ang_delta_vec);
  //   Eigen::Quaterniond ang_f_to_l_est_temp =
  //     f_ang_delta * ekf->m_state.fid_states[m_id].ang_f_to_l;
  //   ang_f_to_l_est = ang_f_to_l_est_temp;
  // }

  Eigen::Matrix3d rot_f_to_l_est = ang_f_to_l_est.toRotationMatrix();

  unsigned int max_meas_size = g_fid_measurement_size * board_track.size();
  unsigned int state_size = ekf->GetStateSize();
  unsigned int cam_index = ekf->m_state.cam_states[m_id].index;
  unsigned int aug_state_size = ekf->GetAugStateSize();

  Eigen::VectorXd res_x = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(max_meas_size, state_size);

  Eigen::VectorXd res_f = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd H_f = Eigen::MatrixXd::Zero(max_meas_size, g_fid_extrinsic_state_size);
  Eigen::MatrixXd H_c = Eigen::MatrixXd::Zero(max_meas_size, g_cam_state_size + aug_state_size);

  for (unsigned int i = 0; i < board_track.size(); ++i) {
    AugState aug_state_i = ekf->GetAugState(m_id, board_track[i].frame_id);
    unsigned int aug_index = ekf->GetAugState(m_id, board_track[i].frame_id).index;

    Eigen::Matrix3d rot_ci_to_bi = aug_state_i.ang_c_to_b.toRotationMatrix();
    Eigen::Matrix3d rot_bi_to_l = aug_state_i.ang_b_to_l.toRotationMatrix();
    Eigen::Matrix3d rot_bi_to_ci = rot_ci_to_bi.transpose();
    Eigen::Matrix3d rot_l_to_bi = rot_bi_to_l.transpose();
    Eigen::Matrix3d rot_l_to_ci = rot_bi_to_ci * rot_bi_to_l.transpose();
    Eigen::Quaterniond ang_l_to_ci(rot_l_to_ci);

    Eigen::Vector3d pos_ci_in_bi = aug_state_i.pos_c_in_b;
    Eigen::Vector3d pos_bi_in_g = aug_state_i.pos_b_in_l;

    Eigen::Vector3d pos_predicted, pos_measured, pos_residual;
    Eigen::Quaterniond ang_predicted, ang_measured, ang_residual;

    // Project the current feature into the current frame of reference
    Eigen::Vector3d pos_f_in_bi_est = rot_bi_to_l.transpose() * (pos_f_in_l_est - pos_bi_in_g);
    pos_predicted = rot_ci_to_bi.transpose() * (pos_f_in_bi_est - pos_ci_in_bi);
    ang_predicted = ang_l_to_ci * ang_f_to_l_est;

    CvVectorToEigen(board_track[i].t_vec_f_in_c, pos_measured);
    ang_measured = RodriguesToQuat(board_track[i].r_vec_f_to_c);

    // Residuals for this frame
    pos_residual = pos_measured - pos_predicted;
    ang_residual = ang_predicted * ang_measured.inverse();

    unsigned int meas_row = g_fid_measurement_size * i;
    res_f.segment<3>(meas_row + 0) = pos_residual;
    res_f.segment<3>(meas_row + 3) = QuatToRotVec(ang_residual);

    unsigned int H_c_aug_start = aug_index - cam_index;

    H_c.block<3, 3>(meas_row + 0, H_c_aug_start + 0) = -rot_bi_to_ci * rot_l_to_bi;

    H_c.block<3, 3>(meas_row + 0, H_c_aug_start + 3) = rot_bi_to_ci *
      rot_l_to_bi * SkewSymmetric(pos_f_in_l_est - pos_bi_in_g) *
      quaternion_jacobian_inv(aug_state_i.ang_b_to_l);

    H_c.block<3, 3>(meas_row + 0, H_c_aug_start + 6) = -rot_bi_to_ci;

    H_c.block<3, 3>(meas_row + 0, H_c_aug_start + 9) = rot_bi_to_ci *
      SkewSymmetric(rot_l_to_bi * (pos_f_in_l_est - pos_bi_in_g) - pos_ci_in_bi) *
      quaternion_jacobian_inv(aug_state_i.ang_c_to_b);

    H_c.block<3, 3>(meas_row + 3, H_c_aug_start + 3) =
      rot_bi_to_ci * rot_l_to_bi * quaternion_jacobian_inv(aug_state_i.ang_b_to_l) * rot_f_to_l_est;

    H_c.block<3, 3>(meas_row + 3, H_c_aug_start + 9) =
      rot_bi_to_ci * quaternion_jacobian_inv(aug_state_i.ang_c_to_b) * rot_l_to_bi * rot_f_to_l_est;

    // Feature Jacobian
    H_f.block<3, 3>(meas_row + 0, 0) = rot_ci_to_bi.transpose() * rot_bi_to_l.transpose();
    // H_f.block<3, 3>(meas_row + 0, 0) = ;
  }

  /// @todo Chi^2 distance check

  // Append our Jacobian and residual
  H_x.block(0, cam_index, H_c.rows(), H_c.cols()) = H_c;
  res_x.block(0, 0, res_f.rows(), 1) = res_f;

  if (board_track.size() > 1) {
    CompressMeasurements(H_x, res_x);
  }

  // Jacobian is ill-formed if either rows or columns post-compression are size 1
  if (res_x.size() == 1) {
    m_logger->Log(LogLevel::INFO, "Compressed MSCKF Jacobian is ill-formed");
    return;
  }

  /// @todo(jhartzer): This doesn't account for angular errors. Apply transform to R?
  double position_sigma = 3 * pos_error;  // / std::sqrt(board_track.size());
  Eigen::MatrixXd R = position_sigma * Eigen::MatrixXd::Identity(res_x.rows(), res_x.rows());

  // Apply Kalman update
  Eigen::MatrixXd S = H_x * ekf->m_cov * H_x.transpose() + R;
  Eigen::MatrixXd K = ekf->m_cov * H_x.transpose() * S.inverse();

  unsigned int imu_states_size = ekf->GetImuStateSize();
  unsigned int cam_states_size = state_size - g_body_state_size - imu_states_size;

  Eigen::VectorXd update = K * res_x;
  Eigen::VectorXd cam_state_vec = ekf->m_state.cam_states[m_id].ToVector();
  Eigen::Vector3d cam_pos = cam_state_vec.segment<3>(0);
  Eigen::Quaterniond cam_ang_pos = RotVecToQuat(cam_state_vec.segment<3>(3));
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
  Eigen::VectorXd cov_diag = ekf->m_cov.block(
    cam_index, cam_index, g_cam_state_size, g_cam_state_size).diagonal();

  // Write outputs
  std::stringstream msg;
  msg << time;
  msg << "," << std::to_string(board_track.size());
  msg << VectorToCommaString(cam_pos);
  msg << QuaternionToCommaString(cam_ang_pos);
  msg << VectorToCommaString(res_f.segment<g_fid_measurement_size>(0));
  msg << VectorToCommaString(body_update);
  msg << VectorToCommaString(cam_update.segment(0, g_cam_state_size));
  msg << VectorToCommaString(cov_diag);
  msg << "," << t_execution.count();
  m_fiducial_logger.RateLimitedLog(msg.str(), time);
}
