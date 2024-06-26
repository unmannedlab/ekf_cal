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
  header << "time";
  header << EnumerateHeader("imu_pos", 3);
  header << EnumerateHeader("imu_ang_pos", 4);
  header << EnumerateHeader("imu_acc_bias", 3);
  header << EnumerateHeader("imu_gyr_bias", 3);
  if (m_is_extrinsic) {header << EnumerateHeader("imu_ext_cov", g_imu_extrinsic_state_size);}
  if (m_is_intrinsic) {header << EnumerateHeader("imu_int_cov", g_imu_intrinsic_state_size);}
  header << EnumerateHeader("acc", 3);
  header << EnumerateHeader("omg", 3);
  header << EnumerateHeader("residual", 6);
  header << EnumerateHeader("body_update", g_body_state_size);
  if (m_is_extrinsic) {header << EnumerateHeader("imu_ext_update", g_imu_extrinsic_state_size);}
  if (m_is_intrinsic) {header << EnumerateHeader("imu_int_update", g_imu_intrinsic_state_size);}

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
  Eigen::Vector3d acc_bias = imu_state.acc_bias;
  Eigen::Vector3d omg_bias = imu_state.omg_bias;

  unsigned int jacobian_size = g_body_state_size;

  if (m_is_extrinsic) {jacobian_size += g_imu_extrinsic_state_size;}
  if (m_is_intrinsic) {jacobian_size += g_imu_intrinsic_state_size;}

  Eigen::MatrixXd measurement_jacobian = Eigen::MatrixXd::Zero(6, jacobian_size);

  // Body Acceleration
  measurement_jacobian.block<3, 3>(0, 6) = ang_i_to_b.inverse().toRotationMatrix() *
    body_state.ang_b_to_l.inverse().toRotationMatrix();

  // Body Angular Velocity
  measurement_jacobian.block<3, 3>(0, 12) = ang_i_to_b.inverse().toRotationMatrix() * (
    SkewSymmetric(body_state.ang_vel_b_in_l) *
    SkewSymmetric(pos_i_in_g).transpose() +
    SkewSymmetric(body_state.ang_vel_b_in_l.cross(pos_i_in_g)).transpose()
  );

  // Body Angular Acceleration
  measurement_jacobian.block<3, 3>(0, 15) = ang_i_to_b.inverse().toRotationMatrix() *
    SkewSymmetric(pos_i_in_g);

  // IMU Body Angular Velocity
  measurement_jacobian.block<3, 3>(3, 12) = ang_i_to_b.inverse().toRotationMatrix() *
    body_state.ang_b_to_l.inverse().toRotationMatrix();

  // IMU Positional Offset
  unsigned int extrinsic_offset{0};
  if (m_is_extrinsic) {
    // IMU Positional Offset
    measurement_jacobian.block<3, 3>(0, g_body_state_size) =
      ang_i_to_b.inverse().toRotationMatrix() * (
      SkewSymmetric(body_state.ang_acc_b_in_l) +
      SkewSymmetric(body_state.ang_vel_b_in_l) * SkewSymmetric(body_state.ang_vel_b_in_l)
      );

    // IMU Angular Offset
    measurement_jacobian.block<3, 3>(0, g_body_state_size + 3) = SkewSymmetric(
      ang_i_to_b.inverse().toRotationMatrix() * (
        body_state.ang_acc_b_in_l.cross(pos_i_in_g) +
        body_state.ang_vel_b_in_l.cross(body_state.ang_vel_b_in_l.cross(pos_i_in_g)) +
        body_state.ang_b_to_l.inverse().toRotationMatrix() * (
          body_state.acc_b_in_l + g_gravity
        )
      )
    );

    // IMU Angular Offset
    measurement_jacobian.block<3, 3>(3, g_body_state_size + 3) = SkewSymmetric(
      ang_i_to_b.inverse().toRotationMatrix() *
      body_state.ang_b_to_l.inverse().toRotationMatrix() *
      body_state.ang_vel_b_in_l
    );
    extrinsic_offset = 3;
  }

  if (m_is_intrinsic) {
    // IMU Accelerometer Bias
    measurement_jacobian.block<3, 3>(0, g_body_state_size + extrinsic_offset + 0) =
      Eigen::MatrixXd::Identity(3, 3);

    // IMU Gyroscope Bias
    measurement_jacobian.block<3, 3>(3, g_body_state_size + extrinsic_offset + 3) =
      Eigen::MatrixXd::Identity(3, 3);
  }

  return measurement_jacobian;
}

void ImuUpdater::UpdateEKF(
  std::shared_ptr<EKF> ekf,
  double time, Eigen::Vector3d acceleration,
  Eigen::Matrix3d acceleration_covariance, Eigen::Vector3d angular_rate,
  Eigen::Matrix3d angular_rate_covariance, bool use_as_predictor)
{
  if (use_as_predictor) {
    ekf->PredictModel(
      time,
      acceleration,
      angular_rate);
    return;
  }

  ekf->ProcessModel(time);

  BodyState body_state = ekf->m_state.body_state;

  auto t_start = std::chrono::high_resolution_clock::now();

  Eigen::VectorXd z(acceleration.size() + angular_rate.size());
  z.segment<3>(0) = acceleration;
  z.segment<3>(3) = angular_rate;

  Eigen::VectorXd z_pred = PredictMeasurement(ekf);
  Eigen::VectorXd resid = z - z_pred;

  unsigned int imu_index = ekf->m_state.imu_states[m_id].index;

  unsigned int update_size = g_body_state_size + ekf->GetImuStateSize();
  unsigned int imu_update_size {0};
  if (m_is_extrinsic) {imu_update_size += g_imu_extrinsic_state_size;}
  if (m_is_intrinsic) {imu_update_size += g_imu_intrinsic_state_size;}

  Eigen::MatrixXd subH = GetMeasurementJacobian(ekf);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, update_size);

  H.block<6, g_body_state_size>(0, 0) = subH.block<6, g_body_state_size>(0, 0);

  if (imu_update_size) {
    H.block(0, imu_index, 6, imu_update_size) =
      subH.block(0, g_body_state_size, 6, imu_update_size);
  }

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
  R.block<3, 3>(0, 0) = acceleration_covariance * 3;
  R.block<3, 3>(3, 3) = angular_rate_covariance * 3;

  Eigen::MatrixXd S = H * ekf->m_cov.block(0, 0, update_size, update_size) * H.transpose() + R;
  Eigen::MatrixXd K =
    ekf->m_cov.block(0, 0, update_size, update_size) * H.transpose() * S.inverse();

  Eigen::VectorXd update = K * resid;
  Eigen::VectorXd body_update = update.segment<g_body_state_size>(0);
  Eigen::VectorXd imu_update = update.segment(g_body_state_size, update_size - g_body_state_size);

  ekf->m_state.body_state += body_update;
  ekf->m_state.imu_states += imu_update;

  ekf->m_cov.block(0, 0, update_size, update_size) =
    (Eigen::MatrixXd::Identity(update_size, update_size) - K * H) *
    ekf->m_cov.block(0, 0, update_size, update_size) *
    (Eigen::MatrixXd::Identity(update_size, update_size) - K * H).transpose() +
    K * R * K.transpose();

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;

  msg << time;
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
  msg << VectorToCommaString(body_update);
  if (imu_update_size) {
    Eigen::VectorXd imu_sub_update = update.segment(imu_index, imu_update_size);
    msg << VectorToCommaString(imu_sub_update);
  }
  msg << "," << t_execution.count();
  m_data_logger.RateLimitedLog(msg.str(), time);
}
