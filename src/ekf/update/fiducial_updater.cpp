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
  int fiducial_id,
  int camera_id,
  bool is_cam_extrinsic,
  std::string log_file_directory,
  double data_log_rate,
  std::shared_ptr<DebugLogger> logger
)
: Updater(fiducial_id, logger),
  m_is_cam_extrinsic(is_cam_extrinsic),
  m_fiducial_logger(log_file_directory, "fiducial_" + std::to_string(camera_id) + ".csv"),
  m_camera_id(camera_id)
{
  std::stringstream header;
  header << "time,board";
  header << EnumerateHeader("board_pos", 3);
  header << EnumerateHeader("board_ang", 4);
  header << EnumerateHeader("cam_pos", 3);
  header << EnumerateHeader("cam_ang_pos", 4);
  header << EnumerateHeader("residual", g_fid_measurement_size);
  if (m_is_cam_extrinsic) {header << EnumerateHeader("cam_cov", g_cam_extrinsic_state_size);}
  header << EnumerateHeader("duration", 1);

  m_fiducial_logger.DefineHeader(header.str());
  if (data_log_rate) {m_fiducial_logger.EnableLogging();}
}

void FiducialUpdater::UpdateEKF(
  std::shared_ptr<EKF> ekf, double time,
  BoardTrack board_track, double pos_error, double ang_error)
{
  m_logger->Log(
    LogLevel::DEBUG, "Called Fiducial Update for camera ID: " + std::to_string(m_camera_id));

  ekf->PredictModel(time);

  auto t_start = std::chrono::high_resolution_clock::now();

  if (!ekf->GetUseFirstEstimateJacobian() || m_is_first_estimate) {
    m_pos_c_in_b = ekf->m_state.cam_states[m_camera_id].pos_c_in_b;
    m_ang_c_to_b = ekf->m_state.cam_states[m_camera_id].ang_c_to_b;
    m_pos_f_in_l = ekf->m_state.fid_states[m_id].pos_f_in_l;
    m_ang_f_to_l = ekf->m_state.fid_states[m_id].ang_f_to_l;
    m_is_first_estimate = false;
  }

  const Eigen::Vector3d pos_bi_in_l = ekf->m_state.body_state.pos_b_in_l;
  const Eigen::Quaterniond ang_b_to_l = ekf->m_state.body_state.ang_b_to_l;
  const Eigen::Matrix3d rot_bi_to_l = ang_b_to_l.toRotationMatrix();
  const Eigen::Matrix3d rot_c_to_b = m_ang_c_to_b.toRotationMatrix();

  Eigen::Matrix3d rot_f_to_l = m_ang_f_to_l.toRotationMatrix();

  unsigned int max_meas_size = g_fid_measurement_size * board_track.size();
  unsigned int state_size = ekf->GetStateSize();
  unsigned int cam_index = ekf->m_state.cam_states[m_camera_id].index;

  Eigen::VectorXd res = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(max_meas_size, state_size);


  Eigen::Matrix3d rot_bi_to_c = rot_c_to_b.transpose();
  Eigen::Matrix3d rot_l_to_bi = rot_bi_to_l.transpose();
  Eigen::Matrix3d rot_l_to_ci = rot_bi_to_c * rot_l_to_bi;
  Eigen::Matrix3d rot_l_to_c = rot_bi_to_c * rot_l_to_bi;
  Eigen::Quaterniond ang_l_to_ci(rot_l_to_ci);

  Eigen::Vector3d pos_predicted, pos_measured, pos_residual;
  Eigen::Quaterniond ang_predicted, ang_measured, ang_residual;

  // Project the current feature into the current frame of reference
  Eigen::Vector3d pos_f_in_bi = rot_l_to_bi * (m_pos_f_in_l - pos_bi_in_l);
  pos_predicted = rot_bi_to_c * (pos_f_in_bi - m_pos_c_in_b);
  ang_predicted = ang_l_to_ci * m_ang_f_to_l;

  CvVectorToEigen(board_track[0].t_vec_f_in_c, pos_measured);
  ang_measured = RodriguesToQuat(board_track[0].r_vec_f_to_c);

  // Residuals for this frame
  pos_residual = pos_measured - pos_predicted;
  ang_residual = ang_predicted * ang_measured.inverse();

  res.segment<3>(0) = pos_residual;
  res.segment<3>(3) = QuatToRotVec(ang_residual);

  H.block<3, 3>(0, 0) = -rot_l_to_c;

  H.block<3, 3>(0, 9) = rot_l_to_c *
    SkewSymmetric(m_pos_f_in_l - pos_bi_in_l) *
    quaternion_jacobian(ang_b_to_l).transpose();

  H.block<3, 3>(3, 9) = rot_l_to_c *
    quaternion_jacobian(ang_b_to_l).transpose() * rot_f_to_l;

  if (ekf->m_state.cam_states[m_camera_id].GetIsExtrinsic()) {
    H.block<3, 3>(0, cam_index + 0) = -rot_bi_to_c;

    H.block<3, 3>(0, cam_index + 3) = rot_bi_to_c *
      SkewSymmetric(rot_l_to_bi * (m_pos_f_in_l - pos_bi_in_l) - m_pos_c_in_b) *
      quaternion_jacobian(m_ang_c_to_b).transpose();

    H.block<3, 3>(3, cam_index + 3) = rot_bi_to_c *
      quaternion_jacobian(m_ang_c_to_b).transpose() * rot_l_to_bi * rot_f_to_l;
  }

  /// @todo Chi^2 distance check

  // Jacobian is ill-formed if either rows or columns post-compression are size 1
  if (res.size() == 1) {
    m_logger->Log(LogLevel::INFO, "Compressed MSCKF Jacobian is ill-formed");
    return;
  }

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(res.rows(), res.rows());
  R.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * pos_error * pos_error;
  R.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * ang_error * ang_error;

  // Apply Kalman update
  KalmanUpdate(ekf, H, res, R);

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  Eigen::Vector3d cam_pos = ekf->m_state.cam_states[m_camera_id].pos_c_in_b;
  Eigen::Quaterniond cam_ang = ekf->m_state.cam_states[m_camera_id].ang_c_to_b;
  Eigen::VectorXd cov_diag = ekf->m_cov.block(
    cam_index, cam_index, g_cam_extrinsic_state_size, g_cam_extrinsic_state_size).diagonal();
  if (ekf->GetUseRootCovariance()) {
    cov_diag = cov_diag.cwiseProduct(cov_diag);
  }

  // Write outputs
  std::stringstream msg;
  msg << time;
  msg << "," << m_id;
  msg << VectorToCommaString(pos_measured);
  msg << QuaternionToCommaString(ang_measured);
  msg << VectorToCommaString(cam_pos);
  msg << QuaternionToCommaString(cam_ang);
  msg << VectorToCommaString(res);
  if (m_is_cam_extrinsic) {msg << VectorToCommaString(cov_diag);}
  msg << "," << t_execution.count();
  m_fiducial_logger.RateLimitedLog(msg.str(), time);
}
