// Copyright 2022 Jacob Hartzer
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

#include "sensors/imu.hpp"

#include <eigen3/Eigen/Eigen>

#include <memory>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/imu_message.hpp"
#include "sensors/sensor.hpp"
#include "utility/math_helper.hpp"


IMU::IMU(IMU::Parameters params)
: Sensor(params.name), m_imu_updater(m_id, params.is_extrinsic, params.is_intrinsic,
    params.output_directory, params.data_logging_on, params.data_log_rate)
{
  m_is_extrinsic = params.is_extrinsic;
  m_is_intrinsic = params.is_intrinsic;
  m_use_for_prediction = params.use_for_prediction;
  m_rate = params.rate;

  ImuState imu_state;
  imu_state.is_extrinsic = params.is_extrinsic;
  imu_state.is_intrinsic = params.is_intrinsic;
  imu_state.pos_stability = params.pos_stability;
  imu_state.ang_stability = params.ang_stability;
  imu_state.acc_bias_stability = params.acc_bias_stability;
  imu_state.omg_bias_stability = params.omg_bias_stability;
  imu_state.pos_i_in_b = params.pos_i_in_b;
  imu_state.ang_i_to_b = params.ang_i_to_b;
  imu_state.acc_bias = params.acc_bias;
  imu_state.omg_bias = params.omg_bias;

  Eigen::MatrixXd cov;
  if (imu_state.is_extrinsic && imu_state.is_intrinsic) {
    Eigen::VectorXd variance = MinBoundVector(params.variance, 1e-6).segment<12>(0);
    cov = variance.asDiagonal();
  } else if (imu_state.is_extrinsic || imu_state.is_intrinsic) {
    Eigen::VectorXd variance = MinBoundVector(params.variance, 1e-6).segment<6>(0);
    cov = variance.asDiagonal();
  }

  m_ekf->RegisterIMU(m_id, imu_state, cov);
}

void IMU::Callback(std::shared_ptr<ImuMessage> imu_message)
{
  m_logger->Log(
    LogLevel::DEBUG,
    "IMU \"" + m_name + "\" callback at time " + std::to_string(imu_message->m_time));
  m_imu_updater.UpdateEKF(
    imu_message->m_time,
    imu_message->m_acceleration,
    imu_message->m_acceleration_covariance,
    imu_message->m_angular_rate,
    imu_message->m_angular_rate_covariance,
    m_use_for_prediction);
  m_logger->Log(LogLevel::DEBUG, "IMU \"" + m_name + "\" callback complete");
}
