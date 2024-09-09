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
  BodyState body_state = ekf->m_state.body_state;
  ImuState imu_state = ekf->m_state.imu_states[m_id];
  Eigen::Vector3d pos_i_in_g = imu_state.pos_i_in_b;
  Eigen::Quaterniond ang_i_to_b = imu_state.ang_i_to_b;

  unsigned int jacobian_size = 12 + ekf->GetImuStateSize();

  Eigen::MatrixXd measurement_jacobian = Eigen::MatrixXd::Zero(6, jacobian_size);

  // Body Acceleration
  measurement_jacobian.block<3, 3>(0, 0) = ang_i_to_b.inverse().toRotationMatrix() *
    body_state.ang_b_to_l.inverse().toRotationMatrix();

  /// @todo: Body Angular Position
  // measurement_jacobian.block<3, 3>(0, 3) = ;

  // Body Angular Velocity
  measurement_jacobian.block<3, 3>(0, 6) = ang_i_to_b.inverse().toRotationMatrix() * (
    SkewSymmetric(body_state.ang_vel_b_in_l) *
    SkewSymmetric(pos_i_in_g).transpose() +
    SkewSymmetric(body_state.ang_vel_b_in_l.cross(pos_i_in_g)).transpose()
  );

  // Body Angular Acceleration
  measurement_jacobian.block<3, 3>(0, 9) = ang_i_to_b.inverse().toRotationMatrix() *
    SkewSymmetric(pos_i_in_g);

  // Body Angular Velocity
  measurement_jacobian.block<3, 3>(3, 6) = ang_i_to_b.inverse().toRotationMatrix() *
    body_state.ang_b_to_l.inverse().toRotationMatrix();

  unsigned int imu_ex_start = ekf->m_state.imu_states[m_id].index - ekf->GetImuStateStart() + 12;
  unsigned int imu_in_start = imu_ex_start;

  if (m_is_extrinsic) {
    // // IMU Positional Offset
    // measurement_jacobian.block<3, 3>(0, imu_ex_start + 0) =
    //   ang_i_to_b.inverse() * (
    //   SkewSymmetric(body_state.ang_acc_b_in_l) +
    //   SkewSymmetric(body_state.ang_vel_b_in_l) * SkewSymmetric(body_state.ang_vel_b_in_l)
    //   );

    // // IMU Angular Offset
    // measurement_jacobian.block<3, 3>(0, imu_ex_start + 3) = SkewSymmetric(
    //   ang_i_to_b.inverse() * (
    //     body_state.ang_acc_b_in_l.cross(pos_i_in_g) +
    //     body_state.ang_vel_b_in_l.cross(body_state.ang_vel_b_in_l.cross(pos_i_in_g)) +
    //     body_state.ang_b_to_l.inverse() * (body_state.acc_b_in_l + g_gravity)
    //   )
    // );

    // // IMU Angular Offset
    // measurement_jacobian.block<3, 3>(3, imu_ex_start + 3) = -SkewSymmetric(
    //   ang_i_to_b.inverse() * body_state.ang_b_to_l.inverse() * body_state.ang_vel_b_in_l);

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
  R.block<3, 3>(0, 0) = acceleration_covariance * 3;
  R.block<3, 3>(3, 3) = angular_rate_covariance * 3;
  MinBoundDiagonal(R, 1e-3);

  Eigen::MatrixXd sub_cov = Eigen::MatrixXd::Zero(sub_size, sub_size);
  sub_cov.block<12, 12>(0, 0) = ekf->m_cov.block<12, 12>(6, 6);
  sub_cov.block(12, 0, imu_size, 12) = ekf->m_cov.block(imu_start, 0, imu_size, 12);
  sub_cov.block(0, 12, 12, imu_size) = ekf->m_cov.block(0, imu_start, 12, imu_size);
  sub_cov.block(12, 12, imu_size, imu_size) =
    ekf->m_cov.block(imu_start, imu_start, imu_size, imu_size);

  Eigen::MatrixXd S = H * sub_cov * H.transpose() + R;
  Eigen::MatrixXd K = sub_cov * H.transpose() * S.inverse();

  Eigen::VectorXd update = K * resid;
  Eigen::VectorXd imu_update = update.segment(12, sub_size - 12);

  ekf->m_state.body_state.acc_b_in_l += update.segment<3>(0);
  ekf->m_state.body_state.ang_b_to_l =
    ekf->m_state.body_state.ang_b_to_l * RotVecToQuat(update.segment<3>(3));
  ekf->m_state.body_state.ang_vel_b_in_l += update.segment<3>(6);
  ekf->m_state.body_state.ang_acc_b_in_l += update.segment<3>(9);

  ekf->m_state.imu_states += imu_update;

  sub_cov =
    (Eigen::MatrixXd::Identity(sub_size, sub_size) - K * H) * sub_cov *
    (Eigen::MatrixXd::Identity(sub_size, sub_size) - K * H).transpose() +
    K * R * K.transpose();

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
  return false;
  ekf->ProcessModel(time);

  unsigned int sub_size{3}, meas_size{3};
  unsigned int sub_ex_index{0}, sub_in_index{0};
  unsigned int ex_index{0}, in_index{0};

  auto t_start = std::chrono::high_resolution_clock::now();

  if (ekf->m_state.imu_states[imu_id].GetIsExtrinsic()) {
    ex_index = ekf->m_state.imu_states[imu_id].index;
    in_index += 3;
    sub_ex_index = 3;
    sub_size += 3;
  }

  if (ekf->m_state.imu_states[imu_id].GetIsIntrinsic()) {
    in_index += ekf->m_state.imu_states[imu_id].index;
    sub_in_index = sub_ex_index + 3;
    sub_size += 6;
    meas_size += 3;
  }

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_size, sub_size);
  H.block<3, 3>(0, 0) = -ekf->m_state.imu_states[imu_id].ang_i_to_b.inverse().toRotationMatrix() *
    SkewSymmetric(ekf->m_state.body_state.ang_b_to_l.inverse() * g_gravity);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(meas_size, meas_size);
  R.block<3, 3>(0, 0) = acceleration_covariance;

  Eigen::Vector3d bias_a = ekf->m_state.imu_states[imu_id].acc_bias;
  Eigen::Vector3d bias_g = ekf->m_state.imu_states[imu_id].omg_bias;

  Eigen::VectorXd z = Eigen::VectorXd::Zero(meas_size);
  z.segment<3>(0) = -(acceleration - bias_a -
    ekf->m_state.imu_states[imu_id].ang_i_to_b.inverse() *
    ekf->m_state.body_state.ang_b_to_l.inverse() * g_gravity);

  Eigen::MatrixXd sub_cov = Eigen::MatrixXd::Zero(sub_size, sub_size);
  sub_cov.block<3, 3>(0, 0) = ekf->m_cov.block<3, 3>(6, 6);

  if (ekf->m_state.imu_states[imu_id].GetIsExtrinsic()) {
    H.block<3, 3>(0, 0) = -SkewSymmetric(
      ekf->m_state.imu_states[imu_id].ang_i_to_b.inverse() *
      ekf->m_state.body_state.ang_b_to_l.inverse() * g_gravity
    );
    sub_cov.block(sub_ex_index, sub_ex_index, 3, 3) = ekf->m_cov.block(ex_index, ex_index, 3, 3);
    sub_cov.block(0, sub_ex_index, 3, 3) = ekf->m_cov.block(0, ex_index, 3, 3);
    sub_cov.block(sub_ex_index, 0, 3, 3) = ekf->m_cov.block(ex_index, 0, 3, 3);
  }

  if (ekf->m_state.imu_states[imu_id].GetIsIntrinsic()) {
    z.segment<3>(3) = -(angular_rate - bias_g);
    R.block<3, 3>(3, 3) = angular_rate_covariance;
    H.block<3, 3>(0, sub_in_index + 0) = -Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, sub_in_index + 3) = -Eigen::Matrix3d::Identity();
    sub_cov.block(sub_in_index, sub_in_index, 6, 6) = ekf->m_cov.block(in_index, in_index, 6, 6);
    sub_cov.block(0, sub_in_index, 6, 6) = ekf->m_cov.block(0, in_index, 6, 6);
    sub_cov.block(sub_in_index, 0, 6, 6) = ekf->m_cov.block(in_index, 0, 6, 6);
  }

  MinBoundDiagonal(R, 1e-2);

  Eigen::MatrixXd score_mat = z.transpose() *
    (H * sub_cov * H.transpose() + ekf->GetImuNoiseScaleFactor() * R).inverse() * z;
  double score = score_mat(0, 0);
  if (score > ekf->GetMotionDetectionChiSquared() && ekf->IsGravityInitialized()) {
    return false;
  } else if (score < ekf->GetMotionDetectionChiSquared()) {
    ekf->InitializeGravity();
  }

  // Apply Kalman update
  Eigen::MatrixXd S = H * sub_cov * H.transpose() + R;
  Eigen::MatrixXd K = sub_cov * H.transpose() * S.inverse();

  Eigen::VectorXd update = K * z;
  ekf->m_state.body_state.acc_b_in_l = Eigen::Vector3d::Zero();
  ekf->m_state.body_state.ang_vel_b_in_l = Eigen::Vector3d::Zero();
  ekf->m_state.body_state.ang_acc_b_in_l = Eigen::Vector3d::Zero();
  ekf->m_state.body_state.ang_b_to_l =
    ekf->m_state.body_state.ang_b_to_l * RotVecToQuat(update.segment<3>(0));

  sub_cov =
    (Eigen::MatrixXd::Identity(sub_size, sub_size) - K * H) * sub_cov *
    (Eigen::MatrixXd::Identity(sub_size, sub_size) - K * H).transpose() +
    K * R * K.transpose();

  MinBoundDiagonal(sub_cov, 1e-2, 0, 3);

  ekf->m_cov.block<3, 3>(6, 6) = sub_cov.block<3, 3>(0, 0);

  if (ekf->m_state.imu_states[imu_id].GetIsExtrinsic()) {
    ekf->m_cov.block(ex_index, ex_index, 3, 3) = sub_cov.block(sub_ex_index, sub_ex_index, 3, 3);
    ekf->m_cov.block(0, ex_index, 3, 3) = sub_cov.block(0, sub_ex_index, 3, 3);
    ekf->m_cov.block(ex_index, 0, 3, 3) = sub_cov.block(sub_ex_index, 0, 3, 3);
  }

  if (ekf->m_state.imu_states[imu_id].GetIsIntrinsic()) {
    ekf->m_cov.block(in_index, in_index, 6, 6) = sub_cov.block(sub_in_index, sub_in_index, 6, 6);
    ekf->m_cov.block(0, in_index, 6, 6) = sub_cov.block(0, sub_in_index, 6, 6);
    ekf->m_cov.block(in_index, 0, 6, 6) = sub_cov.block(sub_in_index, 0, 6, 6);
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

  if (ekf->m_state.imu_states[imu_id].GetIsExtrinsic()) {
    Eigen::VectorXd ex_diag = ekf->m_cov.block(ex_index, ex_index, 6, 6).diagonal();
    msg << VectorToCommaString(ex_diag);
  }

  if (ekf->m_state.imu_states[imu_id].GetIsIntrinsic()) {
    Eigen::VectorXd in_diag = ekf->m_cov.block(in_index, in_index, 6, 6).diagonal();
    msg << VectorToCommaString(in_diag);
  }

  msg << VectorToCommaString(acceleration);
  msg << VectorToCommaString(angular_rate);
  msg << VectorToCommaString(z);

  if (!ekf->m_state.imu_states[imu_id].GetIsIntrinsic()) {
    msg << VectorToCommaString(Eigen::Vector3d::Zero());
  }

  msg << "," << t_execution.count();
  m_data_logger.RateLimitedLog(msg.str(), time);

  ekf->LogBodyStateIfNeeded(t_execution.count());

  return true;
}
