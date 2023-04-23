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
#include "utility/sim/sim_rng.hpp"
#include "utility/math_helper.hpp"

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
  m_truth = truthEngine;
}

std::vector<std::shared_ptr<SimImuMessage>> SimIMU::GenerateMessages(double maxTime)
{
  double num_measurements = static_cast<int>(std::round(maxTime * m_rate / (1 + m_time_skew)));
  std::vector<std::shared_ptr<SimImuMessage>> messages;
  m_logger->Log(LogLevel::INFO, "Generating " + std::to_string(num_measurements) + " measurements");

  for (unsigned int i = 0; i < num_measurements; ++i) {
    auto sim_imu_msg = std::make_shared<SimImuMessage>();
    double measurementTime = (1.0 + m_time_skew) / m_rate * static_cast<double>(i);
    sim_imu_msg->m_time = measurementTime + m_rng.NormRand(m_time_bias, m_time_error);
    sim_imu_msg->m_sensor_id = m_id;
    sim_imu_msg->m_sensor_type = SensorType::IMU;

    Eigen::Vector3d body_acc = m_truth->GetBodyAcceleration(measurementTime);
    Eigen::Quaterniond body_ang_pos = m_truth->GetBodyAngularPosition(measurementTime);
    Eigen::Vector3d body_ang_vel = m_truth->GetBodyAngularRate(measurementTime);
    Eigen::Vector3d body_ang_acc = m_truth->GetBodyAngularAcceleration(measurementTime);

    // Transform acceleration to IMU location
    Eigen::Vector3d imuAcc = body_acc +
      body_ang_acc.cross(m_pos_offset) +
      body_ang_vel.cross((body_ang_vel.cross(m_pos_offset)));

    // Rotate measurements in place
    Eigen::Vector3d imu_acc_rot = m_ang_offset * imuAcc;
    Eigen::Vector3d imu_omg_rot = m_ang_offset * body_ang_vel;

    imu_acc_rot += body_ang_pos * m_ang_offset * GRAVITY;

    sim_imu_msg->m_acceleration = imu_acc_rot;
    sim_imu_msg->m_acceleration[0] += m_rng.NormRand(m_acc_bias[0], m_acc_error[0]);
    sim_imu_msg->m_acceleration[1] += m_rng.NormRand(m_acc_bias[1], m_acc_error[1]);
    sim_imu_msg->m_acceleration[2] += m_rng.NormRand(m_acc_bias[2], m_acc_error[2]);

    sim_imu_msg->m_angular_rate = imu_omg_rot;
    sim_imu_msg->m_angular_rate[0] += m_rng.NormRand(m_omg_bias[0], m_omg_error[0]);
    sim_imu_msg->m_angular_rate[1] += m_rng.NormRand(m_omg_bias[1], m_omg_error[1]);
    sim_imu_msg->m_angular_rate[2] += m_rng.NormRand(m_omg_bias[2], m_omg_error[2]);

    Eigen::Vector3d accSigmas(m_acc_error[0], m_acc_error[1], m_acc_error[2]);
    Eigen::Vector3d omgSigmas(m_omg_error[0], m_omg_error[1], m_omg_error[2]);

    sim_imu_msg->m_acceleration_covariance = accSigmas.asDiagonal();
    sim_imu_msg->m_angular_rate_covariance = omgSigmas.asDiagonal();
    messages.push_back(sim_imu_msg);
  }
  return messages;
}
