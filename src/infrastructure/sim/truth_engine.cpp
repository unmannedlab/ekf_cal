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

#include "infrastructure/sim/truth_engine.hpp"

#include <eigen3/Eigen/Eigen>
#include <string>

#include "infrastructure/debug_logger.hpp"

Eigen::Vector3d TruthEngine::GetBodyPosition(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Vector3d::Zero(3);
}

Eigen::Vector3d TruthEngine::GetBodyVelocity(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Vector3d::Zero(3);
}

Eigen::Vector3d TruthEngine::GetBodyAcceleration(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Vector3d::Zero(3);
}

Eigen::Quaterniond TruthEngine::GetBodyAngularPosition(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Quaterniond::Identity();
}

Eigen::Vector3d TruthEngine::GetBodyAngularRate(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Vector3d::Zero(3);
}

Eigen::Vector3d TruthEngine::GetBodyAngularAcceleration(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Vector3d::Zero(3);
}

Eigen::Vector3d TruthEngine::GetImuPosition(unsigned int sensor_id)
{
  return m_imu_pos[sensor_id];
}

Eigen::Quaterniond TruthEngine::GetImuAngularPosition(unsigned int sensor_id)
{
  return m_imu_ang_pos[sensor_id];
}

Eigen::Vector3d TruthEngine::GetImuAccelerometerBias(unsigned int sensor_id)
{
  return m_imu_acc_bias[sensor_id];
}

Eigen::Vector3d TruthEngine::GetImuGyroscopeBias(unsigned int sensor_id)
{
  return m_imu_gyro_bias[sensor_id];
}

Eigen::Vector3d TruthEngine::GetCameraPosition(unsigned int sensor_id)
{
  return m_cam_pos[sensor_id];
}

Eigen::Quaterniond TruthEngine::GetCameraAngularPosition(unsigned int sensor_id)
{
  return m_cam_ang_pos[sensor_id];
}

void TruthEngine::SetImuPosition(unsigned int sensor_id, Eigen::Vector3d imu_pos)
{
  m_imu_pos[sensor_id] = imu_pos;
}

void TruthEngine::SetImuAngularPosition(unsigned int sensor_id, Eigen::Quaterniond imu_ang_pos)
{
  m_imu_ang_pos[sensor_id] = imu_ang_pos;
}

void TruthEngine::SetImuAccelerometerBias(unsigned int sensor_id, Eigen::Vector3d imu_acc_bias)
{
  m_imu_acc_bias[sensor_id] = imu_acc_bias;
}

void TruthEngine::SetImuGyroscopeBias(unsigned int sensor_id, Eigen::Vector3d imu_gyro_bias)
{
  m_imu_gyro_bias[sensor_id] = imu_gyro_bias;
}

void TruthEngine::SetCameraPosition(unsigned int sensor_id, Eigen::Vector3d cam_pos)
{
  m_cam_pos[sensor_id] = cam_pos;
}

void TruthEngine::SetCameraAngularPosition(unsigned int sensor_id, Eigen::Quaterniond cam_ang_pos)
{
  m_cam_ang_pos[sensor_id] = cam_ang_pos;
}

TruthEngine::~TruthEngine() {}
