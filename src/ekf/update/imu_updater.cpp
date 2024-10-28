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

#include "ekf/update/imu_updater.hpp"

#include <eigen3/Eigen/Eigen>

#include <chrono>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include "ekf/constants.hpp"
#include "ekf/ekf.hpp"
#include "ekf/imu_filter.hpp"
#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "utility/math_helper.hpp"
#include "utility/type_helper.hpp"
#include "utility/string_helper.hpp"


ImuUpdater::ImuUpdater(
  unsigned int imu_id,
  bool is_extrinsic,
  bool is_intrinsic,
  std::string log_file_directory,
  bool data_logging_on,
  double data_log_rate,
  std::shared_ptr<DebugLogger> logger
)
: Updater(imu_id, logger),
  m_is_extrinsic(is_extrinsic),
  m_is_intrinsic(is_intrinsic),
  m_data_logger(log_file_directory, "imu_" + std::to_string(imu_id) + ".csv")
{
  std::stringstream header;
  header << "time,stationary,score";
  header << EnumerateHeader("imu_pos", 3);
  header << EnumerateHeader("imu_ang_pos", 4);
  header << EnumerateHeader("imu_acc_bias", 3);
  header << EnumerateHeader("imu_gyr_bias", 3);
  if (m_is_extrinsic) {header << EnumerateHeader("imu_ext_cov", g_imu_extrinsic_state_size);}
  if (m_is_intrinsic) {header << EnumerateHeader("imu_int_cov", g_imu_intrinsic_state_size);}
  header << EnumerateHeader("acc", 3);
  header << EnumerateHeader("omg", 3);
  header << EnumerateHeader("residual", 6);
  header << EnumerateHeader("duration", 1);

  m_data_logger.DefineHeader(header.str());
  m_data_logger.SetLogging(data_logging_on);
  m_data_logger.SetLogRate(data_log_rate);
}

void ImuUpdater::UpdateEKF(
  std::shared_ptr<EKF> ekf,
  double time, Eigen::Vector3d acceleration,
  Eigen::Matrix3d acceleration_covariance, Eigen::Vector3d angular_rate,
  Eigen::Matrix3d angular_rate_covariance)
{
  // Check for zero velocity
  if (ZeroAccelerationUpdate(
      ekf,
      m_id,
      time,
      acceleration,
      acceleration_covariance,
      angular_rate,
      angular_rate_covariance))
  {
    ekf->SetZeroAcceleration(true);
    return;
  } else {
    ekf->SetZeroAcceleration(false);
  }

  auto t_start = std::chrono::high_resolution_clock::now();

  ekf->m_imu_filter.Update(
    acceleration, angular_rate,
    acceleration_covariance,
    angular_rate_covariance,
    ekf->m_state.imu_states[m_id].pos_i_in_b,
    ekf->m_state.imu_states[m_id].ang_i_to_b,
    ekf->m_state.imu_states[m_id].acc_bias,
    ekf->m_state.imu_states[m_id].omg_bias,
    ekf->m_state.body_state.ang_b_to_l);

  ekf->PredictModel(time);

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;

  Eigen::VectorXd resid = Eigen::VectorXd::Zero(6);
  resid.segment<3>(0) = acceleration - ekf->m_imu_filter.GetAcc();
  resid.segment<3>(3) = angular_rate - ekf->m_imu_filter.GetAngVel();

  msg << time;
  msg << ",0," << ekf->GetMotionDetectionChiSquared();
  msg << VectorToCommaString(ekf->m_state.imu_states[m_id].pos_i_in_b);
  msg << QuaternionToCommaString(ekf->m_state.imu_states[m_id].ang_i_to_b);
  msg << VectorToCommaString(ekf->m_state.imu_states[m_id].acc_bias);
  msg << VectorToCommaString(ekf->m_state.imu_states[m_id].omg_bias);
  if (m_is_extrinsic || m_is_intrinsic) {
    unsigned int imu_index = ekf->m_state.imu_states[m_id].index;
    unsigned int imu_size = ekf->m_state.imu_states[m_id].size;
    Eigen::VectorXd cov_diag =
      ekf->m_cov.block(imu_index, imu_index, imu_size, imu_size).diagonal();
    if (ekf->GetUseRootCovariance()) {
      cov_diag = cov_diag.cwiseProduct(cov_diag);
    }
    msg << VectorToCommaString(cov_diag);
  }
  msg << VectorToCommaString(acceleration);
  msg << VectorToCommaString(angular_rate);
  msg << VectorToCommaString(resid);
  msg << "," << t_execution.count();
  m_data_logger.RateLimitedLog(msg.str(), time);

  ekf->LogBodyStateIfNeeded(t_execution.count());
}

Eigen::MatrixXd ImuUpdater::GetMeasurementJacobian(
  std::shared_ptr<EKF> ekf,
  unsigned int imu_id)
{
  unsigned int meas_size = m_is_intrinsic ? 6 : 3;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_size, ekf->GetStateSize());


  Eigen::Quaterniond ang_i_to_b = ekf->m_state.imu_states[imu_id].ang_i_to_b;
  Eigen::Quaterniond ang_b_to_l = ekf->m_state.body_state.ang_b_to_l;

  H.block<3, 3>(0, 6) = -SkewSymmetric(ang_i_to_b.inverse() * ang_b_to_l.inverse() * g_gravity) *
    quaternion_jacobian(ang_b_to_l).transpose();

  if (m_is_extrinsic) {
    unsigned int index_extrinsic = ekf->m_state.imu_states[imu_id].index_extrinsic;
    H.block<3, 3>(0, index_extrinsic) = -SkewSymmetric(
      ang_i_to_b.inverse() * ang_b_to_l.inverse() * g_gravity
    );
  }

  if (m_is_intrinsic) {
    unsigned int index_intrinsic = ekf->m_state.imu_states[imu_id].index_intrinsic;
    H.block<3, 3>(0, index_intrinsic + 0) = -Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, index_intrinsic + 3) = -Eigen::Matrix3d::Identity();
  }

  return H;
}

bool ImuUpdater::ZeroAccelerationUpdate(
  std::shared_ptr<EKF> ekf,
  unsigned int imu_id,
  double time,
  Eigen::Vector3d acceleration,
  Eigen::Matrix3d acceleration_covariance,
  Eigen::Vector3d angular_rate,
  Eigen::Matrix3d angular_rate_covariance)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  if (m_initial_motion_detected) {
    return false;
  }

  unsigned int meas_size = m_is_intrinsic ? 6 : 3;
  Eigen::Quaterniond ang_i_to_b = ekf->m_state.imu_states[imu_id].ang_i_to_b;
  Eigen::Quaterniond ang_b_to_l = ekf->m_state.body_state.ang_b_to_l;

  Eigen::MatrixXd H = GetMeasurementJacobian(ekf, imu_id);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(meas_size, meas_size);
  R.block<3, 3>(0, 0) = acceleration_covariance;
  if (m_is_intrinsic) {
    R.block<3, 3>(3, 3) = angular_rate_covariance;
  }

  Eigen::Vector3d bias_a = ekf->m_state.imu_states[imu_id].acc_bias;
  Eigen::Vector3d bias_g = ekf->m_state.imu_states[imu_id].omg_bias;

  Eigen::VectorXd resid = Eigen::VectorXd::Zero(meas_size);
  resid.segment<3>(0) = -(acceleration - bias_a -
    ang_i_to_b.inverse() *
    ang_b_to_l.inverse() * g_gravity);
  if (m_is_intrinsic) {
    resid.segment<3>(3) = -(angular_rate - bias_g);
  }

  Eigen::MatrixXd score_mat = resid.transpose() *
    (H * ekf->m_cov * H.transpose() + ekf->GetImuNoiseScaleFactor() * R).inverse() * resid;

  double score = std::abs(score_mat(0, 0));
  if (score > ekf->GetMotionDetectionChiSquared() && ekf->IsGravityInitialized()) {
    m_initial_motion_detected = true;
    return false;
  } else if (score < ekf->GetMotionDetectionChiSquared()) {
    ekf->InitializeGravity();
  }

  ekf->m_imu_filter.AngularUpdate(
    angular_rate,
    angular_rate_covariance,
    ekf->m_state.imu_states[m_id].ang_i_to_b,
    ekf->m_state.imu_states[m_id].omg_bias);

  ekf->PredictModel(time);

  // Update Jacobian
  H = GetMeasurementJacobian(ekf, imu_id);

  // Apply Kalman update
  Eigen::Quaterniond ang_b_to_l_pre = ekf->m_state.body_state.ang_b_to_l;
  KalmanUpdate(ekf, H, resid, R);

  /// Prevent unintentional rotation about the vertical axis
  if (m_correct_heading_rotation) {
    Eigen::Vector3d x_axis_body_pre = ang_b_to_l_pre.inverse() * Eigen::Vector3d::UnitX();
    Eigen::Vector3d x_axis_body = ekf->m_state.body_state.ang_b_to_l.inverse() *
      Eigen::Vector3d::UnitX();
    Eigen::Vector3d plane_normal = g_gravity.cross(x_axis_body_pre) / g_gravity.norm();
    double angle = M_PI / 2 -
      std::acos(x_axis_body.dot(plane_normal) / x_axis_body.norm() / plane_normal.norm());
    Eigen::Vector3d rotation_axis = x_axis_body.cross(plane_normal);
    auto correction = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_axis));
    ekf->m_state.body_state.ang_b_to_l = ekf->m_state.body_state.ang_b_to_l * correction;
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;

  msg << time;
  msg << ",1," << score;
  msg << VectorToCommaString(ekf->m_state.imu_states[m_id].pos_i_in_b);
  msg << QuaternionToCommaString(ekf->m_state.imu_states[m_id].ang_i_to_b);
  msg << VectorToCommaString(ekf->m_state.imu_states[m_id].acc_bias);
  msg << VectorToCommaString(ekf->m_state.imu_states[m_id].omg_bias);

  if (m_is_extrinsic) {
    unsigned int index_extrinsic = ekf->m_state.imu_states[imu_id].index_extrinsic;
    Eigen::VectorXd ex_diag = ekf->m_cov.block(index_extrinsic, index_extrinsic, 6, 6).diagonal();
    msg << VectorToCommaString(ex_diag.cwiseProduct(ex_diag));
  }

  if (m_is_intrinsic) {
    unsigned int index_intrinsic = ekf->m_state.imu_states[imu_id].index_intrinsic;
    Eigen::VectorXd in_diag = ekf->m_cov.block(index_intrinsic, index_intrinsic, 6, 6).diagonal();
    msg << VectorToCommaString(in_diag.cwiseProduct(in_diag));
  }

  msg << VectorToCommaString(acceleration);
  msg << VectorToCommaString(angular_rate);
  msg << VectorToCommaString(resid);

  if (!m_is_intrinsic) {
    msg << VectorToCommaString(Eigen::Vector3d::Zero());
  }

  msg << "," << t_execution.count();
  m_data_logger.RateLimitedLog(msg.str(), time);

  ekf->LogBodyStateIfNeeded(t_execution.count());

  return true;
}
