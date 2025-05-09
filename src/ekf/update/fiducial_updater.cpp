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
  unsigned int fiducial_id,
  unsigned int camera_id,
  bool is_fid_extrinsic,
  bool is_cam_extrinsic,
  const std::string & log_file_directory,
  double data_log_rate,
  std::shared_ptr<DebugLogger> logger
)
: Updater(fiducial_id, logger),
  m_is_fid_extrinsic(is_fid_extrinsic),
  m_is_cam_extrinsic(is_cam_extrinsic),
  m_fiducial_logger(log_file_directory, "fiducial_" + std::to_string(camera_id) + ".csv"),
  m_camera_id(camera_id)
{
  std::stringstream header;
  header << "time,board";
  header << EnumerateHeader("board_meas_pos", 3);
  header << EnumerateHeader("board_meas_ang", 4);
  header << EnumerateHeader("cam_pos", 3);
  header << EnumerateHeader("cam_ang_pos", 4);
  header << EnumerateHeader("residual", g_fid_measurement_size);
  if (m_is_fid_extrinsic) {
    header << EnumerateHeader("fid_pos", 3);
    header << EnumerateHeader("fid_ang", 4);
    header << EnumerateHeader("fid_cov", g_cam_extrinsic_state_size);
  }
  if (m_is_cam_extrinsic) {header << EnumerateHeader("cam_cov", g_cam_extrinsic_state_size);}
  header << EnumerateHeader("duration", 1);

  m_fiducial_logger.DefineHeader(header.str());
  if (data_log_rate != 0.0) {m_fiducial_logger.EnableLogging();}
}

void FiducialUpdater::RefreshStates(EKF & ekf)
{
  if (!ekf.GetUseFirstEstimateJacobian() || m_is_first_estimate) {
    m_pos_c_in_b = ekf.m_state.cam_states[m_camera_id].pos_c_in_b;
    m_ang_c_to_b = ekf.m_state.cam_states[m_camera_id].ang_c_to_b;
    m_pos_f_in_l = ekf.m_state.fid_states[m_id].pos_f_in_l;
    m_ang_f_to_l = ekf.m_state.fid_states[m_id].ang_f_to_l;
    m_is_first_estimate = false;
  }
}

Eigen::MatrixXd FiducialUpdater::GetMeasurementJacobian(EKF & ekf)
{
  RefreshStates(ekf);

  Eigen::Vector3d pos_b_in_l = ekf.m_state.body_state.pos_b_in_l;
  Eigen::Quaterniond ang_b_to_l = ekf.m_state.body_state.ang_b_to_l;
  Eigen::Matrix3d rot_b_to_c = m_ang_c_to_b.toRotationMatrix().transpose();
  Eigen::Matrix3d rot_f_to_l = m_ang_f_to_l.toRotationMatrix();
  Eigen::Matrix3d rot_l_to_b = ang_b_to_l.toRotationMatrix().transpose();
  Eigen::Matrix3d rot_l_to_c = rot_b_to_c * rot_l_to_b;

  unsigned int state_size = ekf.GetStateSize();
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(g_fid_measurement_size, state_size);

  jacobian.block<3, 3>(0, 0) = -rot_l_to_c;

  jacobian.block<3, 3>(0, 9) = rot_l_to_c *
    SkewSymmetric(m_pos_f_in_l - pos_b_in_l) *
    QuaternionJacobian(ang_b_to_l).transpose();

  jacobian.block<3, 3>(3, 9) = -rot_l_to_c * rot_f_to_l *
    QuaternionJacobian(ang_b_to_l).transpose();

  /// @todo Test camera calibration jacobians
  if (m_is_cam_extrinsic) {
    unsigned int cam_index = ekf.m_state.cam_states[m_camera_id].index;
    jacobian.block<3, 3>(0, cam_index + 0) = -rot_b_to_c;

    jacobian.block<3, 3>(0, cam_index + 3) = rot_b_to_c *
      SkewSymmetric(rot_l_to_b * (m_pos_f_in_l - pos_b_in_l) - m_pos_c_in_b) *
      QuaternionJacobian(m_ang_c_to_b).transpose();

    jacobian.block<3, 3>(3, cam_index + 3) = -rot_b_to_c *
      QuaternionJacobian(m_ang_c_to_b).transpose() * rot_l_to_b * rot_f_to_l;
  }

  if (m_is_fid_extrinsic) {
    unsigned int fid_index = ekf.m_state.fid_states[m_id].index;
    jacobian.block<3, 3>(0, fid_index + 0) = Eigen::Matrix3d::Identity();
    jacobian.block<3, 3>(3, fid_index + 3) = Eigen::Matrix3d::Identity();
  }

  return jacobian;
}

Eigen::VectorXd FiducialUpdater::PredictMeasurement(EKF & ekf)
{
  RefreshStates(ekf);
  Eigen::VectorXd predicted_measurement = Eigen::VectorXd::Zero(6);
  Eigen::Quaterniond ang_b_to_l = ekf.m_state.body_state.ang_b_to_l;
  Eigen::Vector3d pos_b_in_l = ekf.m_state.body_state.pos_b_in_l;
  Eigen::Matrix3d rot_b_to_c = m_ang_c_to_b.toRotationMatrix().transpose();
  Eigen::Matrix3d rot_b_to_l = ang_b_to_l.toRotationMatrix();
  Eigen::Matrix3d rot_l_to_b = rot_b_to_l.transpose();
  Eigen::Quaterniond ang_l_to_c(rot_b_to_c * rot_l_to_b);

  Eigen::Vector3d pos_predicted, pos_measured, pos_residual;
  Eigen::Quaterniond ang_predicted, ang_measured, ang_residual;

  // Project the current feature into the current frame of reference
  Eigen::Vector3d pos_f_in_b = rot_l_to_b * (m_pos_f_in_l - pos_b_in_l);
  predicted_measurement.segment<3>(0) = rot_b_to_c * (pos_f_in_b - m_pos_c_in_b);
  predicted_measurement.segment<3>(3) = QuatToRotVec(ang_l_to_c * m_ang_f_to_l);
  return predicted_measurement;
}

void FiducialUpdater::UpdateEKF(
  EKF & ekf,
  const double time,
  const BoardDetection & board_detection)
{
  m_logger->Log(
    LogLevel::DEBUG, "Called Fiducial Update for camera ID: " + std::to_string(m_camera_id));

  double local_time = ekf.CalculateLocalTime(time);
  ekf.PredictModel(local_time);

  auto t_start = std::chrono::high_resolution_clock::now();

  Eigen::VectorXd pred_meas = PredictMeasurement(ekf);

  Eigen::Vector3d pos_measured = board_detection.pos_f_in_c;
  Eigen::Quaterniond ang_measured = board_detection.ang_f_to_c;

  // Residuals for this frame
  Eigen::VectorXd res = Eigen::VectorXd::Zero(g_fid_measurement_size);
  res.segment<3>(0) = pos_measured - pred_meas.segment<3>(0);
  res.segment<3>(3) = QuatToRotVec(RotVecToQuat(pred_meas.segment<3>(3)) * ang_measured.inverse());

  Eigen::MatrixXd jacobian = GetMeasurementJacobian(ekf);

  /// @todo Chi^2 distance check

  Eigen::MatrixXd meas_noise = Eigen::MatrixXd::Zero(res.rows(), res.rows());
  meas_noise.block<3, 3>(0, 0) = board_detection.pos_error.asDiagonal();
  meas_noise.block<3, 3>(3, 3) = board_detection.ang_error.asDiagonal();

  // Apply Kalman update
  KalmanUpdate(ekf, jacobian, res, meas_noise);

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;
  msg << local_time;
  msg << "," << m_id;
  msg << VectorToCommaString(pos_measured);
  msg << QuaternionToCommaString(ang_measured);
  msg << VectorToCommaString(ekf.m_state.cam_states[m_camera_id].pos_c_in_b);
  msg << QuaternionToCommaString(ekf.m_state.cam_states[m_camera_id].ang_c_to_b);
  msg << VectorToCommaString(res);
  if (m_is_fid_extrinsic) {
    msg << VectorToCommaString(ekf.m_state.fid_states[m_id].pos_f_in_l);
    msg << QuaternionToCommaString(ekf.m_state.fid_states[m_id].ang_f_to_l);
    unsigned int fid_index = ekf.m_state.fid_states[m_id].index;
    Eigen::VectorXd cov_diag = ekf.m_cov.block(
      fid_index, fid_index, g_fid_extrinsic_state_size, g_fid_extrinsic_state_size).diagonal();
    if (ekf.GetUseRootCovariance()) {
      cov_diag = cov_diag.cwiseProduct(cov_diag);
    }
    msg << VectorToCommaString(cov_diag);
  }
  if (m_is_cam_extrinsic) {
    unsigned int cam_index = ekf.m_state.cam_states[m_camera_id].index;
    Eigen::VectorXd cov_diag = ekf.m_cov.block(
      cam_index, cam_index, g_cam_extrinsic_state_size, g_cam_extrinsic_state_size).diagonal();
    if (ekf.GetUseRootCovariance()) {
      cov_diag = cov_diag.cwiseProduct(cov_diag);
    }
    msg << VectorToCommaString(cov_diag);
  }
  msg << "," << t_execution.count();
  m_fiducial_logger.RateLimitedLog(msg.str(), local_time);
}
