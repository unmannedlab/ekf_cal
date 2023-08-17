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
#include <unistd.h>

#include <string>
#include <sstream>

#include "ekf/types.hpp"
#include "ekf/constants.hpp"
#include "infrastructure/debug_logger.hpp"
#include "utility/math_helper.hpp"
#include "utility/type_helper.hpp"
#include "utility/string_helper.hpp"


/// @todo Should combine IMU with file name for multiple IMU logs
ImuUpdater::ImuUpdater(unsigned int imu_id, std::string log_file_directory, bool data_logging_on)
: Updater(imu_id), m_data_logger(log_file_directory, "imu_" + std::to_string(imu_id) + ".csv")
{
  std::stringstream header;
  header << "time";
  header << EnumerateHeader("imu_pos", 3);
  header << EnumerateHeader("imu_ang", 4);
  header << EnumerateHeader("imu_acc_bias", 3);
  header << EnumerateHeader("imu_omg_bias", 3);
  header << EnumerateHeader("imu_cov", g_imu_state_size);
  header << EnumerateHeader("acc", 3);
  header << EnumerateHeader("omg", 3);
  header << EnumerateHeader("residual", 6);
  header << EnumerateHeader("body_update", g_body_state_size);
  header << EnumerateHeader("imu_update", g_imu_state_size);
  header << EnumerateHeader("time", 1);
  header << std::endl;

  m_data_logger.DefineHeader(header.str());
  m_data_logger.SetLogging(data_logging_on);
}

Eigen::VectorXd ImuUpdater::PredictMeasurement()
{
  Eigen::VectorXd predicted_measurement(6);
  // Transform acceleration to IMU location
  Eigen::Vector3d imuAcc = m_body_ang_pos.inverse() * (m_body_acc + g_gravity) +
    m_body_ang_acc.cross(m_pos_offset) +
    m_body_ang_vel.cross((m_body_ang_vel.cross(m_pos_offset)));

  // Rotate measurements in place
  Eigen::Vector3d imu_acc_rot = m_ang_offset * imuAcc;
  Eigen::Vector3d imu_omg_rot = m_ang_offset * m_body_ang_vel;

  // Add bias
  imu_acc_rot += m_acc_bias;
  imu_omg_rot += m_omg_bias;

  predicted_measurement.segment<3>(0) = imu_acc_rot;
  predicted_measurement.segment<3>(3) = imu_omg_rot;

  return predicted_measurement;
}

Eigen::MatrixXd ImuUpdater::GetMeasurementJacobian(bool isBaseSensor, bool isIntrinsic)
{
  Eigen::MatrixXd measurement_jacobian =
    Eigen::MatrixXd::Zero(6, g_body_state_size + g_imu_state_size);

  // Body Acceleration
  measurement_jacobian.block<3, 3>(0, 6) = m_ang_offset.toRotationMatrix() *
    m_body_ang_pos.inverse().toRotationMatrix();

  // Body Orientation
  measurement_jacobian.block<3, 3>(0, 9) = -m_ang_offset.toRotationMatrix() *
    SkewSymmetric(m_body_ang_pos.inverse().toRotationMatrix() * g_gravity);

  // Body Angular Velocity
  Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(3, 3);
  temp(0, 0) = m_pos_offset(1) * m_body_ang_vel(1) + 1 * m_pos_offset(2) * m_body_ang_vel(2);
  temp(0, 1) = m_pos_offset(1) * m_body_ang_vel(0) - 2 * m_pos_offset(0) * m_body_ang_vel(1);
  temp(0, 2) = m_pos_offset(2) * m_body_ang_vel(0) - 2 * m_pos_offset(0) * m_body_ang_vel(2);
  temp(1, 0) = m_pos_offset(0) * m_body_ang_vel(1) - 2 * m_pos_offset(1) * m_body_ang_vel(0);
  temp(1, 1) = m_pos_offset(0) * m_body_ang_vel(0) + 1 * m_pos_offset(2) * m_body_ang_vel(2);
  temp(1, 2) = m_pos_offset(2) * m_body_ang_vel(1) - 2 * m_pos_offset(1) * m_body_ang_vel(2);
  temp(2, 0) = m_pos_offset(0) * m_body_ang_vel(2) - 2 * m_pos_offset(2) * m_body_ang_vel(0);
  temp(2, 1) = m_pos_offset(1) * m_body_ang_vel(2) - 2 * m_pos_offset(2) * m_body_ang_vel(1);
  temp(2, 2) = m_pos_offset(0) * m_body_ang_vel(0) + 1 * m_pos_offset(1) * m_body_ang_vel(1);
  measurement_jacobian.block<3, 3>(0, 12) = m_ang_offset * temp;

  // Body Angular Acceleration
  measurement_jacobian.block<3, 3>(0, 15) = m_ang_offset * SkewSymmetric(
    m_pos_offset);

  // IMU Body Angular Velocity
  measurement_jacobian.block<3, 3>(3, 12) = m_ang_offset.toRotationMatrix();

  // IMU Positional Offset
  if (!isBaseSensor) {
    temp.setZero();
    temp(0, 0) = -(m_body_ang_vel(1) * m_body_ang_vel(1)) - (m_body_ang_vel(2) * m_body_ang_vel(2));
    temp(0, 1) = m_body_ang_vel(0) * m_body_ang_vel(1);
    temp(0, 2) = m_body_ang_vel(0) * m_body_ang_vel(2);
    temp(1, 0) = m_body_ang_vel(0) * m_body_ang_vel(1);
    temp(1, 1) = -(m_body_ang_vel(0) * m_body_ang_vel(0)) - (m_body_ang_vel(2) * m_body_ang_vel(2));
    temp(1, 2) = m_body_ang_vel(1) * m_body_ang_vel(2);
    temp(2, 0) = m_body_ang_vel(0) * m_body_ang_vel(2);
    temp(2, 1) = m_body_ang_vel(1) * m_body_ang_vel(2);
    temp(2, 2) = -(m_body_ang_vel(0) * m_body_ang_vel(0)) - (m_body_ang_vel(1) * m_body_ang_vel(1));
    measurement_jacobian.block<3, 3>(0, g_body_state_size) =
      m_ang_offset * SkewSymmetric(m_body_ang_acc) + temp;

    // IMU Angular Offset
    Eigen::Vector3d imu_acc = m_body_acc +
      m_body_ang_acc.cross(m_pos_offset) +
      m_body_ang_vel.cross(m_body_ang_vel.cross(m_pos_offset));

    measurement_jacobian.block<3, 3>(0, g_body_state_size + 3) =
      -(m_ang_offset * SkewSymmetric(imu_acc));

    // IMU Angular Offset
    measurement_jacobian.block<3, 3>(3, g_body_state_size + 3) =
      -(m_ang_offset * SkewSymmetric(m_body_ang_vel));
  }

  if (isIntrinsic) {
    // IMU Accelerometer Bias
    measurement_jacobian.block<3, 3>(0, g_body_state_size + 6) = Eigen::MatrixXd::Identity(3, 3);

    // IMU Gyroscope Bias
    measurement_jacobian.block<3, 3>(3, g_body_state_size + 9) = Eigen::MatrixXd::Identity(3, 3);
  }

  return measurement_jacobian;
}

void ImuUpdater::RefreshStates()
{
  BodyState body_state = m_ekf->GetBodyState();
  m_body_pos = body_state.m_position;
  m_body_vel = body_state.m_velocity;
  m_body_acc = body_state.m_acceleration;
  m_body_ang_pos = body_state.m_orientation;
  m_body_ang_vel = body_state.m_angular_velocity;
  m_body_ang_acc = body_state.m_angular_acceleration;

  ImuState imu_state = m_ekf->GetImuState(m_id);
  m_pos_offset = imu_state.position;
  m_ang_offset = imu_state.orientation;
  m_acc_bias = imu_state.acc_bias;
  m_omg_bias = imu_state.omg_bias;
}


void ImuUpdater::UpdateEKF(
  double time, Eigen::Vector3d acceleration,
  Eigen::Matrix3d accelerationCovariance, Eigen::Vector3d angularRate,
  Eigen::Matrix3d angularRateCovariance, bool isBaseSensor, bool isIntrinsic, bool useAsPredictor)
{
  if (useAsPredictor) {
    m_ekf->PredictModel(
      time,
      acceleration,
      accelerationCovariance,
      angularRate,
      angularRateCovariance);
    return;
  }

  m_ekf->ProcessModel(time);
  RefreshStates();
  auto t_start = std::chrono::high_resolution_clock::now();

  Eigen::VectorXd z(acceleration.size() + angularRate.size());
  z.segment<3>(0) = acceleration;
  z.segment<3>(3) = angularRate;

  Eigen::VectorXd z_pred = PredictMeasurement();
  Eigen::VectorXd resid = z - z_pred;
  std::stringstream msg0;
  msg0 << "IMU resid: " << resid.transpose() << std::endl;
  m_logger->Log(LogLevel::DEBUG, msg0.str());

  unsigned int imu_state_start = m_ekf->GetImuStateStartIndex(m_id);

  unsigned int updateSize = g_body_state_size + g_imu_state_size * m_ekf->GetImuCount();

  Eigen::MatrixXd subH = GetMeasurementJacobian(isBaseSensor, isIntrinsic);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, updateSize);

  H.block<6, g_body_state_size>(0, 0) = subH.block<6, g_body_state_size>(0, 0);

  H.block<6, g_imu_state_size>(0, imu_state_start) =
    subH.block<6, g_imu_state_size>(0, g_body_state_size);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
  R.block<3, 3>(0, 0) = MinBoundDiagonal(accelerationCovariance, 1e-3);
  R.block<3, 3>(3, 3) = MinBoundDiagonal(angularRateCovariance, 1e-2);

  Eigen::MatrixXd S = H * m_ekf->GetCov().block(0, 0, updateSize, updateSize) * H.transpose() + R;
  Eigen::MatrixXd K =
    m_ekf->GetCov().block(0, 0, updateSize, updateSize) * H.transpose() * S.inverse();

  Eigen::VectorXd update = K * resid;
  Eigen::VectorXd body_update = update.segment<g_body_state_size>(0);
  Eigen::VectorXd imu_update = update.segment(g_body_state_size, updateSize - g_body_state_size);

  m_ekf->GetState().m_body_state += body_update;
  m_ekf->GetState().m_imu_states += imu_update;
  m_ekf->GetCov().block(0, 0, updateSize, updateSize) =
    (Eigen::MatrixXd::Identity(updateSize, updateSize) - K * H) *
    m_ekf->GetCov().block(0, 0, updateSize, updateSize);

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;
  Eigen::VectorXd imu_sub_update = update.segment(imu_state_start, g_imu_state_size);
  Eigen::VectorXd cov_diag = m_ekf->GetCov().block<g_imu_state_size, g_imu_state_size>(
    imu_state_start, imu_state_start).diagonal();

  msg << time;
  msg << VectorToCommaString(m_ekf->GetState().m_imu_states[m_id].position);
  msg << QuaternionToCommaString(m_ekf->GetState().m_imu_states[m_id].orientation);
  msg << VectorToCommaString(m_ekf->GetState().m_imu_states[m_id].acc_bias);
  msg << VectorToCommaString(m_ekf->GetState().m_imu_states[m_id].omg_bias);
  msg << VectorToCommaString(cov_diag);
  msg << VectorToCommaString(acceleration);
  msg << VectorToCommaString(angularRate);
  msg << VectorToCommaString(resid);
  msg << VectorToCommaString(body_update);
  msg << VectorToCommaString(imu_sub_update);
  msg << "," << t_execution.count();
  msg << std::endl;
  m_data_logger.Log(msg.str());
}
