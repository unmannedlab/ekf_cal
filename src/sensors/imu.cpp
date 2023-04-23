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

#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/sensor.hpp"
#include "utility/math_helper.hpp"
#include "utility/type_helper.hpp"


IMU::IMU(IMU::Parameters params)
: Sensor(params.name), m_imu_updater(m_id, params.output_directory, params.data_logging_on)
{
  m_is_base_sensor = params.base_sensor;
  m_is_intrinsic = params.intrinsic;
  m_rate = params.rate;

  ImuState imu_state;
  imu_state.position = params.pos_offset;
  imu_state.orientation = params.ang_offset;
  imu_state.acc_bias = params.acc_bias;
  imu_state.omg_bias = params.omg_bias;

  Eigen::MatrixXd cov = MinBoundVector(params.variance, 1e-6).asDiagonal();

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
    m_is_base_sensor, m_is_intrinsic);
  m_logger->Log(LogLevel::DEBUG, "IMU \"" + m_name + "\" callback complete");
}
