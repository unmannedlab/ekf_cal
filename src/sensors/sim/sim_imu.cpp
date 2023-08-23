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

#include "sensors/sim/sim_imu.hpp"

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <memory>
#include <cmath>

#include "ekf/constants.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/sim/sim_imu_message.hpp"
#include "utility/math_helper.hpp"
#include "utility/sim/sim_rng.hpp"

SimIMU::SimIMU(SimIMU::Parameters params, std::shared_ptr<TruthEngine> truthEngine)
: IMU(params.imu_params)
{
  m_time_bias = params.time_bias;
  m_time_skew = params.time_skew;
  m_time_error = std::max(params.time_error, 1e-9);
  m_acc_bias = params.acc_bias;
  m_acc_error = MinBoundVector(params.acc_error, 1e-9);
  m_omg_bias = params.omg_bias;
  m_omg_error = MinBoundVector(params.omg_error, 1e-9);
  m_pos_offset = params.pos_offset;
  m_ang_offset = params.ang_offset;
  m_no_errors = params.no_errors;
  m_truth = truthEngine;
}

std::vector<std::shared_ptr<SimImuMessage>> SimIMU::GenerateMessages(double max_time)
{
  unsigned int num_measurements =
    static_cast<int>(std::floor(max_time * m_rate / (1 + m_time_skew)));

  m_logger->Log(
    LogLevel::INFO, "Generating " + std::to_string(num_measurements) + " IMU measurements");

  double time_init = m_no_errors ? 0 : m_rng.UniRand(0.0, 1.0 / m_rate);

  std::vector<std::shared_ptr<SimImuMessage>> messages;
  for (unsigned int i = 0; i < num_measurements; ++i) {
    auto sim_imu_msg = std::make_shared<SimImuMessage>();
    double measurementTime = (1.0 + m_time_skew) / m_rate * static_cast<double>(i) + time_init;
    sim_imu_msg->m_time = measurementTime;
    sim_imu_msg->m_sensor_id = m_id;
    sim_imu_msg->m_sensor_type = SensorType::IMU;

    Eigen::Vector3d body_acc_g = m_truth->GetBodyAcceleration(measurementTime);
    Eigen::Quaterniond body_b_to_g = m_truth->GetBodyAngularPosition(measurementTime);
    Eigen::Vector3d body_ang_vel_g = m_truth->GetBodyAngularRate(measurementTime);
    Eigen::Vector3d body_ang_acc_g = m_truth->GetBodyAngularAcceleration(measurementTime);

    // Transform acceleration to IMU location
    Eigen::Vector3d imu_acc_b = body_b_to_g.inverse() * (body_acc_g + g_gravity) +
      body_ang_acc_g.cross(m_pos_offset) +
      body_ang_vel_g.cross((body_ang_vel_g.cross(m_pos_offset)));

    // Rotate measurements in place
    Eigen::Vector3d imu_acc_i = m_ang_offset.inverse() * imu_acc_b;
    Eigen::Vector3d imu_omg_i = m_ang_offset.inverse() * body_ang_vel_g;

    sim_imu_msg->m_acceleration = imu_acc_i;
    sim_imu_msg->m_angular_rate = imu_omg_i;

    if (!m_no_errors) {
      sim_imu_msg->m_time += m_rng.NormRand(m_time_bias, m_time_error);
      sim_imu_msg->m_acceleration[0] += m_rng.NormRand(m_acc_bias[0], m_acc_error[0]);
      sim_imu_msg->m_acceleration[1] += m_rng.NormRand(m_acc_bias[1], m_acc_error[1]);
      sim_imu_msg->m_acceleration[2] += m_rng.NormRand(m_acc_bias[2], m_acc_error[2]);
      sim_imu_msg->m_angular_rate[0] += m_rng.NormRand(m_omg_bias[0], m_omg_error[0]);
      sim_imu_msg->m_angular_rate[1] += m_rng.NormRand(m_omg_bias[1], m_omg_error[1]);
      sim_imu_msg->m_angular_rate[2] += m_rng.NormRand(m_omg_bias[2], m_omg_error[2]);
    }

    Eigen::Vector3d accSigmas(m_acc_error[0], m_acc_error[1], m_acc_error[2]);
    Eigen::Vector3d omgSigmas(m_omg_error[0], m_omg_error[1], m_omg_error[2]);

    sim_imu_msg->m_acceleration_covariance = accSigmas.asDiagonal() * 10.0;
    sim_imu_msg->m_angular_rate_covariance = omgSigmas.asDiagonal() * 10.0;
    messages.push_back(sim_imu_msg);
  }
  return messages;
}
