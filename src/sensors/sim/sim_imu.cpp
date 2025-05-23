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
#include <cmath>
#include <memory>
#include <string>

#include "ekf/constants.hpp"
#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/sim/sim_imu_message.hpp"
#include "utility/sim/sim_rng.hpp"
#include "utility/type_helper.hpp"

SimIMU::SimIMU(SimIMU::Parameters params, std::shared_ptr<TruthEngine> truth_engine)
: IMU(params.imu_params), SimSensor(params)
{
  m_acc_error = params.acc_error;
  m_omg_error = params.omg_error;
  m_acc_bias_error = params.acc_bias_error;
  m_omg_bias_error = params.omg_bias_error;
  m_pos_error = params.pos_error;
  m_ang_error = params.ang_error;
  m_truth = truth_engine;

  // Set true IMU values
  Eigen::Vector3d pos_i_in_b_true;
  Eigen::Quaterniond ang_i_to_b_true;
  Eigen::Vector3d acc_bias_true;
  Eigen::Vector3d omg_bias_true;
  if (m_no_errors) {
    pos_i_in_b_true = params.imu_params.pos_i_in_b;
    ang_i_to_b_true = params.imu_params.ang_i_to_b;
    acc_bias_true = params.imu_params.acc_bias;
    omg_bias_true = params.imu_params.omg_bias;
  } else {
    pos_i_in_b_true = SimRNG::VecNormRand(params.imu_params.pos_i_in_b, params.pos_error);
    ang_i_to_b_true = SimRNG::QuatNormRand(params.imu_params.ang_i_to_b, params.ang_error);
    acc_bias_true = SimRNG::VecNormRand(params.imu_params.acc_bias, params.acc_bias_error);
    omg_bias_true = SimRNG::VecNormRand(params.imu_params.omg_bias, params.omg_bias_error);
  }

  m_truth->SetImuPosition(m_id, pos_i_in_b_true);
  m_truth->SetImuAngularPosition(m_id, ang_i_to_b_true);
  m_truth->SetImuAccelerometerBias(m_id, acc_bias_true);
  m_truth->SetImuGyroscopeBias(m_id, omg_bias_true);
}

std::vector<std::shared_ptr<SimImuMessage>> SimIMU::GenerateMessages() const
{
  std::vector<double> measurement_times = GenerateMeasurementTimes(m_rate);

  m_logger->Log(
    LogLevel::INFO, "Generating " + std::to_string(measurement_times.size()) + " IMU measurements");

  std::vector<std::shared_ptr<SimImuMessage>> messages;
  for (auto measurement_time : measurement_times) {
    auto sim_imu_msg = std::make_shared<SimImuMessage>();
    sim_imu_msg->time = measurement_time;
    sim_imu_msg->sensor_id = m_id;
    sim_imu_msg->sensor_type = SensorType::IMU;

    Eigen::Vector3d body_acc_l = m_truth->GetBodyAcceleration(measurement_time);
    Eigen::Quaterniond body_b_to_l = m_truth->GetBodyAngularPosition(measurement_time);
    Eigen::Vector3d body_ang_vel_l = m_truth->GetBodyAngularRate(measurement_time);
    Eigen::Vector3d body_ang_acc_l = m_truth->GetBodyAngularAcceleration(measurement_time);
    Eigen::Vector3d pos_i_in_b_true = m_truth->GetImuPosition(m_id);
    Eigen::Quaterniond ang_i_to_b_true = m_truth->GetImuAngularPosition(m_id);
    Eigen::Vector3d acc_bias_true = m_truth->GetImuAccelerometerBias(m_id);
    Eigen::Vector3d gyr_bias_true = m_truth->GetImuGyroscopeBias(m_id);

    // Transform acceleration to IMU location
    Eigen::Vector3d imu_acc_b = body_b_to_l.inverse() * (body_acc_l + g_gravity) +
      body_ang_acc_l.cross(pos_i_in_b_true) +
      body_ang_vel_l.cross((body_ang_vel_l.cross(pos_i_in_b_true)));

    // Rotate measurements in place
    Eigen::Vector3d imu_acc_i = ang_i_to_b_true.inverse() * imu_acc_b;
    Eigen::Vector3d imu_omg_i = ang_i_to_b_true.inverse() * body_ang_vel_l;

    sim_imu_msg->acceleration = imu_acc_i;
    sim_imu_msg->angular_rate = imu_omg_i;

    if (!m_no_errors) {
      sim_imu_msg->acceleration += SimRNG::VecNormRand(acc_bias_true, m_acc_error);
      sim_imu_msg->angular_rate += SimRNG::VecNormRand(gyr_bias_true, m_omg_error);
    }

    sim_imu_msg->acceleration_covariance = m_acc_error.cwiseProduct(m_acc_error).asDiagonal();
    sim_imu_msg->angular_rate_covariance = m_omg_error.cwiseProduct(m_omg_error).asDiagonal();

    messages.push_back(sim_imu_msg);
  }
  return messages;
}
