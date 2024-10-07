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

Eigen::VectorXd ImuUpdater::PredictMeasurement(std::shared_ptr<EKF> ekf)
{
  BodyState body_state = ekf->m_state.body_state;
  ImuState imu_state = ekf->m_state.imu_states[m_id];
  Eigen::Vector3d pos_i_in_g = imu_state.pos_i_in_b;
  Eigen::Quaterniond ang_i_to_b = imu_state.ang_i_to_b;
  Eigen::Vector3d acc_bias = imu_state.acc_bias;
  Eigen::Vector3d omg_bias = imu_state.omg_bias;

  Eigen::VectorXd predicted_measurement(6);

  // Transform acceleration to IMU location
  Eigen::Vector3d imu_acc_b = body_state.ang_b_to_l.inverse() *
    (body_state.acc_b_in_l + g_gravity) +
    body_state.ang_acc_b_in_l.cross(pos_i_in_g) +
    body_state.ang_vel_b_in_l.cross((body_state.ang_vel_b_in_l.cross(pos_i_in_g)));

  // Rotate measurements in place
  Eigen::Vector3d imu_acc_i = acc_bias + ang_i_to_b.inverse() * imu_acc_b;
  Eigen::Vector3d imu_omg_i = omg_bias + ang_i_to_b.inverse() * body_state.ang_vel_b_in_l;

  predicted_measurement.segment<3>(0) = imu_acc_i;
  predicted_measurement.segment<3>(3) = imu_omg_i;

  return predicted_measurement;
}

Eigen::MatrixXd ImuUpdater::GetMeasurementJacobian(std::shared_ptr<EKF> ekf)
{
  if (!ekf->GetUseFirstEstimateJacobian() || m_is_first_estimate) {
    m_pos_i_in_g = ekf->m_state.imu_states[m_id].pos_i_in_b;
    m_ang_i_to_b = ekf->m_state.imu_states[m_id].ang_i_to_b;
    m_is_first_estimate = false;
  }

  BodyState body_state = ekf->m_state.body_state;

  unsigned int jacobian_size = 12 + ekf->GetImuStateSize();

  Eigen::MatrixXd measurement_jacobian = Eigen::MatrixXd::Zero(6, jacobian_size);

  // Body Acceleration
  measurement_jacobian.block<3, 3>(0, 0) = m_ang_i_to_b.inverse().toRotationMatrix() *
    body_state.ang_b_to_l.inverse().toRotationMatrix();

  /// @todo: Body Angular Position
  // measurement_jacobian.block<3, 3>(0, 3) = ;

  // Body Angular Velocity
  measurement_jacobian.block<3, 3>(0, 6) = m_ang_i_to_b.inverse().toRotationMatrix() * (
    SkewSymmetric(body_state.ang_vel_b_in_l) *
    SkewSymmetric(m_pos_i_in_g).transpose() +
    SkewSymmetric(body_state.ang_vel_b_in_l.cross(m_pos_i_in_g)).transpose()
  );

  // Body Angular Acceleration
  measurement_jacobian.block<3, 3>(0, 9) = m_ang_i_to_b.inverse().toRotationMatrix() *
    SkewSymmetric(m_pos_i_in_g);

  // Body Angular Velocity
  measurement_jacobian.block<3, 3>(3, 6) = m_ang_i_to_b.inverse().toRotationMatrix() *
    body_state.ang_b_to_l.inverse().toRotationMatrix();

  unsigned int imu_ex_start = ekf->m_state.imu_states[m_id].index - ekf->GetImuStateStart() + 12;
  unsigned int imu_in_start = imu_ex_start;

  if (m_is_extrinsic) {
    // // IMU Positional Offset
    // measurement_jacobian.block<3, 3>(0, imu_ex_start + 0) =
    //   m_ang_i_to_b.inverse() * (
    //   SkewSymmetric(body_state.ang_acc_b_in_l) +
    //   SkewSymmetric(body_state.ang_vel_b_in_l) * SkewSymmetric(body_state.ang_vel_b_in_l)
    //   );

    // // IMU Angular Offset
    // measurement_jacobian.block<3, 3>(0, imu_ex_start + 3) = SkewSymmetric(
    //   m_ang_i_to_b.inverse() * (
    //     body_state.ang_acc_b_in_l.cross(m_pos_i_in_g) +
    //     body_state.ang_vel_b_in_l.cross(body_state.ang_vel_b_in_l.cross(m_pos_i_in_g)) +
    //     body_state.ang_b_to_l.inverse() * (body_state.acc_b_in_l + g_gravity)
    //   )
    // );

    // // IMU Angular Offset
    // measurement_jacobian.block<3, 3>(3, imu_ex_start + 3) = -SkewSymmetric(
    //   m_ang_i_to_b.inverse() * body_state.ang_b_to_l.inverse() * body_state.ang_vel_b_in_l);

    imu_in_start += 6;
  }

  if (m_is_intrinsic) {
    // IMU Accelerometer Bias
    measurement_jacobian.block<3, 3>(0, imu_in_start + 0) = Eigen::MatrixXd::Identity(3, 3);

    // IMU Gyroscope Bias
    measurement_jacobian.block<3, 3>(3, imu_in_start + 3) = Eigen::MatrixXd::Identity(3, 3);
  }

  return measurement_jacobian;
}

void ImuUpdater::UpdateEKF(
  std::shared_ptr<EKF> ekf,
  double time, Eigen::Vector3d acceleration,
  Eigen::Matrix3d acceleration_covariance, Eigen::Vector3d angular_rate,
  Eigen::Matrix3d angular_rate_covariance, bool use_as_predictor)
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
    return;
  } else if (use_as_predictor) {
    ekf->PredictModel(
      time,
      acceleration,
      angular_rate);
    return;
  }

  ekf->ProcessModel(time);

  auto t_start = std::chrono::high_resolution_clock::now();

  Eigen::VectorXd z(acceleration.size() + angular_rate.size());
  z.segment<3>(0) = acceleration;
  z.segment<3>(3) = angular_rate;

  Eigen::VectorXd z_pred = PredictMeasurement(ekf);
  Eigen::VectorXd resid = z - z_pred;

  unsigned int imu_size = ekf->GetImuStateSize();
  unsigned int imu_start = ekf->GetImuStateStart();
  unsigned int sub_size = 12 + imu_size;
  unsigned int imu_update_size {0};
  unsigned int imu_index = ekf->m_state.imu_states[m_id].index;
  if (m_is_extrinsic) {imu_update_size += g_imu_extrinsic_state_size;}
  if (m_is_intrinsic) {imu_update_size += g_imu_intrinsic_state_size;}

  Eigen::MatrixXd H = GetMeasurementJacobian(ekf);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
  R.block<3, 3>(0, 0) = acceleration_covariance;
  R.block<3, 3>(3, 3) = angular_rate_covariance;

  Eigen::MatrixXd sub_cov = Eigen::MatrixXd::Zero(sub_size, sub_size);
  sub_cov.block<12, 12>(0, 0) = ekf->m_cov.block<12, 12>(6, 6);
  sub_cov.block(12, 0, imu_size, 12) = ekf->m_cov.block(imu_start, 0, imu_size, 12);
  sub_cov.block(0, 12, 12, imu_size) = ekf->m_cov.block(0, imu_start, 12, imu_size);
  sub_cov.block(12, 12, imu_size, imu_size) =
    ekf->m_cov.block(imu_start, imu_start, imu_size, imu_size);

  // Apply Kalman update
  Eigen::MatrixXd S, G, K;
  if (ekf->GetUseRootCovariance()) {
    R = R.cwiseSqrt();
    G = QR_r(sub_cov * H.transpose(), R);
    K = (G.inverse() * ((G.transpose()).inverse() * H) * sub_cov.transpose() * sub_cov).transpose();
  } else {
    S = H * sub_cov * H.transpose() + R;
    K = sub_cov * H.transpose() * S.inverse();
  }

  Eigen::VectorXd update = K * resid;
  Eigen::VectorXd imu_update = update.segment(12, sub_size - 12);

  ekf->m_state.body_state.acc_b_in_l += update.segment<3>(0);
  ekf->m_state.body_state.ang_b_to_l =
    ekf->m_state.body_state.ang_b_to_l * RotVecToQuat(update.segment<3>(3));
  ekf->m_state.body_state.ang_vel_b_in_l += update.segment<3>(6);
  ekf->m_state.body_state.ang_acc_b_in_l += update.segment<3>(9);

  ekf->m_state.imu_states += imu_update;

  if (ekf->GetUseRootCovariance()) {
    sub_cov = QR_r(
      sub_cov * (Eigen::MatrixXd::Identity(sub_size, sub_size) - K * H).transpose(),
      R * K.transpose());
  } else {
    sub_cov =
      (Eigen::MatrixXd::Identity(sub_size, sub_size) - K * H) * sub_cov *
      (Eigen::MatrixXd::Identity(sub_size, sub_size) - K * H).transpose() +
      K * R * K.transpose();
  }

  ekf->m_cov.block<12, 12>(6, 6) = sub_cov.block<12, 12>(0, 0);
  ekf->m_cov.block(imu_start, 0, imu_size, 12) = sub_cov.block(12, 0, imu_size, 12);
  ekf->m_cov.block(0, imu_start, 12, imu_size) = sub_cov.block(0, 12, 12, imu_size);
  ekf->m_cov.block(imu_start, imu_start, imu_size, imu_size) =
    sub_cov.block(12, 12, imu_size, imu_size);

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;

  msg << time;
  msg << ",0," << ekf->GetMotionDetectionChiSquared();
  msg << VectorToCommaString(ekf->m_state.imu_states[m_id].pos_i_in_b);
  msg << QuaternionToCommaString(ekf->m_state.imu_states[m_id].ang_i_to_b);
  msg << VectorToCommaString(ekf->m_state.imu_states[m_id].acc_bias);
  msg << VectorToCommaString(ekf->m_state.imu_states[m_id].omg_bias);
  if (imu_update_size) {
    Eigen::VectorXd cov_diag = ekf->m_cov.block(
      imu_index, imu_index, imu_update_size, imu_update_size).diagonal();
    msg << VectorToCommaString(cov_diag);
  }
  msg << VectorToCommaString(acceleration);
  msg << VectorToCommaString(angular_rate);
  msg << VectorToCommaString(resid);
  msg << "," << t_execution.count();
  m_data_logger.RateLimitedLog(msg.str(), time);
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
  /// @todo: Need a cohesive method to handle stationary rotations about the gravity axis
  if (m_initial_motion) {
    return false;
  }

  auto t_start = std::chrono::high_resolution_clock::now();

  unsigned int index_extrinsic = ekf->m_state.imu_states[imu_id].index_extrinsic;
  unsigned int index_intrinsic = ekf->m_state.imu_states[imu_id].index_intrinsic;
  unsigned int meas_size = m_is_intrinsic ? 6 : 3;
  Eigen::Quaterniond ang_i_to_b = ekf->m_state.imu_states[imu_id].ang_i_to_b;
  Eigen::Quaterniond ang_b_to_l = ekf->m_state.body_state.ang_b_to_l;

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_size, ekf->GetStateSize());
  H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
  H.block<3, 3>(0, 9) = -SkewSymmetric(ang_i_to_b.inverse() * ang_b_to_l.inverse() * g_gravity);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(meas_size, meas_size);
  R.block<3, 3>(0, 0) = acceleration_covariance;

  Eigen::Vector3d bias_a = ekf->m_state.imu_states[imu_id].acc_bias;
  Eigen::Vector3d bias_g = ekf->m_state.imu_states[imu_id].omg_bias;

  Eigen::VectorXd resid = Eigen::VectorXd::Zero(meas_size);
  resid.segment<3>(0) = -(acceleration - bias_a -
    ang_i_to_b.inverse() *
    ang_b_to_l.inverse() * g_gravity);

  if (m_is_extrinsic) {
    H.block<3, 3>(0, index_extrinsic) = -SkewSymmetric(
      ang_i_to_b.inverse() * ang_b_to_l.inverse() * g_gravity
    );
  }

  if (m_is_intrinsic) {
    resid.segment<3>(3) = -(angular_rate - bias_g);
    R.block<3, 3>(3, 3) = angular_rate_covariance;
    H.block<3, 3>(0, index_intrinsic + 0) = -Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, index_intrinsic + 3) = -Eigen::Matrix3d::Identity();
  }

  Eigen::MatrixXd score_mat = resid.transpose() *
    (H * ekf->m_cov * H.transpose() + ekf->GetImuNoiseScaleFactor() * R).inverse() * resid;
  double score = score_mat(0, 0);
  if (score > ekf->GetMotionDetectionChiSquared() && ekf->IsGravityInitialized()) {
    m_initial_motion = true;
    return false;
  } else if (score < ekf->GetMotionDetectionChiSquared()) {
    ekf->InitializeGravity();
  }

  ekf->ProcessModel(time);

  // Resize Jacobian if state has changed sizes in processing update (state augmentation)
  if (ekf->GetStateSize() != H.cols()) {
    Eigen::MatrixXd H_temp = H;
    H = Eigen::MatrixXd::Zero(meas_size, ekf->GetStateSize());
    H.block(0, 0, H_temp.rows(), H_temp.cols()) = H_temp;
  }

  // Apply Kalman update
  KalmanUpdate(ekf, H, resid, R);

  /// Prevent unintentional rotation about the vertical axis
  Eigen::Vector3d x_axis_body_pre = ang_b_to_l.inverse() * Eigen::Vector3d::UnitX();
  Eigen::Vector3d x_axis_body = ekf->m_state.body_state.ang_b_to_l.inverse() *
    Eigen::Vector3d::UnitX();
  Eigen::Vector3d plane_normal = g_gravity.cross(x_axis_body_pre) / g_gravity.norm();
  double angle = M_PI / 2 -
    std::acos(x_axis_body.dot(plane_normal) / x_axis_body.norm() / plane_normal.norm());
  Eigen::Vector3d rotation_axis = x_axis_body.cross(plane_normal);
  auto correction = Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_axis));

  ekf->m_state.body_state.ang_b_to_l = ekf->m_state.body_state.ang_b_to_l * correction;
  ekf->m_state.body_state.acc_b_in_l = Eigen::Vector3d::Zero();
  ekf->m_state.body_state.ang_vel_b_in_l = Eigen::Vector3d::Zero();
  ekf->m_state.body_state.ang_acc_b_in_l = Eigen::Vector3d::Zero();

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
    Eigen::VectorXd ex_diag = ekf->m_cov.block(index_extrinsic, index_extrinsic, 6, 6).diagonal();
    msg << VectorToCommaString(ex_diag);
  }

  if (m_is_intrinsic) {
    Eigen::VectorXd in_diag = ekf->m_cov.block(index_intrinsic, index_intrinsic, 6, 6).diagonal();
    msg << VectorToCommaString(in_diag);
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
