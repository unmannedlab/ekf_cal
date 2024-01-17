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
#include "utility/type_helper.hpp"

FiducialUpdater::FiducialUpdater(
  int cam_id, std::string log_file_directory, bool data_logging_on)
: Updater(cam_id),
  m_data_logger(log_file_directory, "camera_" + std::to_string(cam_id) + ".csv"),
  m_fiducial_logger(log_file_directory, "fiducial_" + std::to_string(cam_id) + ".csv")
{
  std::stringstream header;
  header << "time";
  header << ",track_size";
  header << EnumerateHeader("cam_state", g_cam_state_size);
  header << EnumerateHeader("body_update", g_body_state_size);
  header << EnumerateHeader("cam_update", g_cam_state_size);
  header << EnumerateHeader("duration", 1);
  header << std::endl;

  m_data_logger.DefineHeader(header.str());
  m_data_logger.SetLogging(data_logging_on);

  m_fiducial_logger.DefineHeader("time,tx,ty,tz,aw,ax,ay,az\n");
  m_fiducial_logger.SetLogging(data_logging_on);
}

void FiducialUpdater::UpdateEKF(
  double time, BoardTrack board_track, Eigen::Vector3d pos_error, Eigen::Vector3d ang_error)
{
  m_logger->Log(
    LogLevel::DEBUG, "Called update_msckf for camera ID: " + std::to_string(m_id));

  if (board_track.size() == 0) {
    return;
  }

  m_ekf->ProcessModel(time);
  RefreshStates();
  auto t_start = std::chrono::high_resolution_clock::now();

  std::vector<double> pos_weights;
  std::vector<double> ang_weights;
  std::vector<Eigen::Vector3d> pos_f_in_g_vec;
  std::vector<Eigen::Quaterniond> ang_f_to_g_vec;

  /// @todo(jhartzer): Wrap this in a function
  for (auto board_detection : board_track) {
    AugmentedState aug_state_i = m_ekf->MatchState(m_id, board_detection.frame_id);

    const Eigen::Vector3d pos_bi_in_g = aug_state_i.pos_b_in_g;
    const Eigen::Matrix3d rot_bi_to_g = aug_state_i.ang_b_to_g.toRotationMatrix();
    const Eigen::Vector3d pos_ci_in_bi = aug_state_i.pos_c_in_b;
    const Eigen::Matrix3d rot_ci_to_bi = aug_state_i.ang_c_to_b.toRotationMatrix();

    Eigen::Vector3d pos_f_in_c;
    CvVectorToEigen(board_detection.t_vec_f_in_c, pos_f_in_c);
    Eigen::Vector3d pos_f_in_g =
      rot_bi_to_g * ((rot_ci_to_bi * pos_f_in_c) + pos_ci_in_bi) + pos_bi_in_g;
    pos_f_in_g_vec.push_back(pos_f_in_g);

    Eigen::Quaterniond ang_f_to_c = RodriguesToQuat(board_detection.r_vec_f_to_c);
    Eigen::Quaterniond ang_f_to_g = aug_state_i.ang_b_to_g * aug_state_i.ang_c_to_b * ang_f_to_c;
    ang_f_to_g_vec.push_back(ang_f_to_g);
    pos_weights.push_back(1.0);
    ang_weights.push_back(1.0);

    std::stringstream data_msg;
    data_msg << std::setprecision(3) << time;
    data_msg << "," << pos_f_in_g[0];
    data_msg << "," << pos_f_in_g[1];
    data_msg << "," << pos_f_in_g[2];
    data_msg << "," << ang_f_to_g.w();
    data_msg << "," << ang_f_to_g.x();
    data_msg << "," << ang_f_to_g.y();
    data_msg << "," << ang_f_to_g.z();
    data_msg << std::endl;
    m_fiducial_logger.Log(data_msg.str());
  }

  // Eigen::Vector3d pos_f_in_g_est = average_vectors(pos_f_in_g_vec, pos_weights);
  // Eigen::Quaterniond ang_f_to_g_est = average_quaternions(ang_f_to_g_vec, ang_weights);
  Eigen::Vector3d pos_f_in_g_est{5.0, 0.0, 0.0};
  Eigen::Quaterniond ang_f_to_g_est{1.0, 0.0, 0.0, 0.0};
  Eigen::Matrix3d rot_f_to_g_est = ang_f_to_g_est.toRotationMatrix();

  unsigned int max_meas_size = 6 * board_track.size();
  unsigned int state_size = m_ekf->GetState().GetStateSize();
  unsigned int cam_state_start = m_ekf->GetCamStateStartIndex(m_id);
  unsigned int aug_state_size = g_aug_state_size * m_ekf->GetCamState(m_id).augmented_states.size();

  Eigen::VectorXd res_x = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(max_meas_size, state_size);

  Eigen::VectorXd res_f = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd H_f = Eigen::MatrixXd::Zero(max_meas_size, 6);
  Eigen::MatrixXd H_c = Eigen::MatrixXd::Zero(max_meas_size, g_cam_state_size + aug_state_size);

  for (unsigned int i = 0; i < board_track.size(); ++i) {
    AugmentedState aug_state_i = m_ekf->MatchState(m_id, board_track[i].frame_id);
    unsigned int aug_state_start = m_ekf->GetAugStateStartIndex(m_id, board_track[i].frame_id);

    Eigen::Matrix3d rot_ci_to_bi = aug_state_i.ang_c_to_b.toRotationMatrix();
    Eigen::Matrix3d rot_bi_to_g = aug_state_i.ang_b_to_g.toRotationMatrix();
    Eigen::Matrix3d rot_bi_to_ci = rot_ci_to_bi.transpose();
    Eigen::Matrix3d rot_g_to_bi = rot_bi_to_g.transpose();
    Eigen::Matrix3d rot_g_to_ci = rot_bi_to_ci * rot_bi_to_g.transpose();
    Eigen::Quaterniond ang_g_to_ci(rot_g_to_ci);

    Eigen::Vector3d pos_ci_in_bi = aug_state_i.pos_c_in_b;
    Eigen::Vector3d pos_bi_in_g = aug_state_i.pos_b_in_g;

    Eigen::Vector3d pos_predicted, pos_measured, pos_residual;
    Eigen::Vector3d ang_predicted, ang_measured, ang_residual;

    // Project the current feature into the current frame of reference
    Eigen::Vector3d pos_f_in_bi = rot_bi_to_g.transpose() * (pos_f_in_g_est - pos_bi_in_g);
    Eigen::Vector3d pos_f_in_ci = rot_ci_to_bi.transpose() * (pos_f_in_bi - pos_ci_in_bi);
    pos_predicted = pos_f_in_ci;
    ang_predicted = QuatToRotVec(ang_g_to_ci * ang_f_to_g_est);

    CvVectorToEigen(board_track[i].t_vec_f_in_c, pos_measured);
    ang_measured = QuatToRotVec(RodriguesToQuat(board_track[i].r_vec_f_to_c));

    // Residuals for this frame
    pos_residual = pos_measured - pos_predicted;
    ang_residual = ang_measured - ang_predicted;

    std::cout << pos_residual << std::endl << std::endl;
    res_f.segment<3>(6 * i + 0) = pos_residual;
    res_f.segment<3>(6 * i + 3) = ang_residual;

    H_c.block<3, 3>(6 * i + 0, aug_state_start - cam_state_start + 0) =
      -rot_bi_to_ci * rot_g_to_ci;

    /// @todo(jhartzer): Fix this
    H_c.block<3, 3>(6 * i + 0, aug_state_start - cam_state_start + 3) = -rot_bi_to_ci *
      rot_g_to_bi * SkewSymmetric(pos_f_in_g_est - pos_bi_in_g);
    // * quaternion_jacobian_inv(aug_state_i.ang_b_to_g);

    H_c.block<3, 3>(6 * i + 3, aug_state_start - cam_state_start + 3) =
      rot_bi_to_ci * quaternion_jacobian_inv(aug_state_i.ang_b_to_g) * rot_f_to_g_est;

    H_c.block<3, 3>(6 * i + 0, aug_state_start - cam_state_start + 6) = -rot_bi_to_ci;

    /// @todo(jhartzer): Fix this
    H_c.block<3, 3>(6 * i + 0, aug_state_start - cam_state_start + 9) = -rot_bi_to_ci *
      SkewSymmetric(rot_g_to_bi * (pos_f_in_g_est - pos_bi_in_g) - pos_ci_in_bi);
    // * quaternion_jacobian_inv(aug_state_i.ang_c_to_b);

    H_c.block<3, 3>(6 * i + 3, aug_state_start - cam_state_start + 9) =
      quaternion_jacobian_inv(aug_state_i.ang_b_to_g) * rot_g_to_bi * rot_f_to_g_est;

    // Feature Jacobian
    H_f.block<3, 3>(6 * i + 0, 0) = rot_bi_to_ci * rot_g_to_bi;

    H_f.block<3, 3>(6 * i + 3, 3) =
      rot_bi_to_ci * rot_g_to_bi * quaternion_jacobian_inv(ang_f_to_g_est);
  }

  ApplyLeftNullspace(H_f, H_c, res_f);

  /// @todo Chi^2 distance check

  // Append our Jacobian and residual
  H_x.block(0, cam_state_start, H_c.rows(), H_c.cols()) = H_c;
  res_x.block(0, 0, res_f.rows(), 1) = res_f;

  CompressMeasurements(H_x, res_x);

  // Jacobian is ill-formed if either rows or columns post-compression are size 1
  if (res_x.size() == 1) {
    m_logger->Log(LogLevel::INFO, "Compressed MSCKF Jacobian is ill-formed");
    return;
  }

  /// @todo(jhartzer): This doesn't account for angular errors
  double position_sigma = pos_error[0] * ang_error[0];
  Eigen::MatrixXd R = position_sigma * Eigen::MatrixXd::Identity(res_x.rows(), res_x.rows());

  // Apply Kalman update
  Eigen::MatrixXd S = H_x * m_ekf->GetCov() * H_x.transpose() + R;
  Eigen::MatrixXd K = m_ekf->GetCov() * H_x.transpose() * S.inverse();

  unsigned int imu_states_size = m_ekf->GetImuStateSize();
  unsigned int cam_states_size = state_size - g_body_state_size - imu_states_size;

  Eigen::VectorXd update = K * res_x;
  Eigen::VectorXd cam_state = m_ekf->GetState().m_cam_states[m_id].ToVector();
  Eigen::VectorXd body_update = update.segment<g_body_state_size>(0);
  Eigen::VectorXd imu_update = update.segment(g_body_state_size, imu_states_size);
  Eigen::VectorXd cam_update = update.segment(g_body_state_size + imu_states_size, cam_states_size);

  m_ekf->GetState().m_body_state += body_update;
  m_ekf->GetState().m_imu_states += imu_update;
  m_ekf->GetState().m_cam_states += cam_update;

  m_ekf->GetCov() = (Eigen::MatrixXd::Identity(state_size, state_size) - K * H_x) * m_ekf->GetCov();

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;
  msg << time;
  msg << "," << std::to_string(board_track.size());
  msg << VectorToCommaString(cam_state.segment(0, g_cam_state_size));
  msg << VectorToCommaString(body_update);
  msg << VectorToCommaString(cam_update.segment(0, g_cam_state_size));
  msg << "," << t_execution.count();
  msg << std::endl;
  m_data_logger.Log(msg.str());
}

void FiducialUpdater::RefreshStates()
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
}
