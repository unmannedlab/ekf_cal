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

#ifndef INFRASTRUCTURE__SIM__TRUTH_ENGINE_HPP_
#define INFRASTRUCTURE__SIM__TRUTH_ENGINE_HPP_

#include <eigen3/Eigen/Eigen>

#include "infrastructure/debug_logger.hpp"

///
/// @class TruthEngine
/// @brief Truth for simulation
/// @todo Add initialization time to start of sim
///
class TruthEngine
{
public:
  TruthEngine() {}

  virtual ~TruthEngine() = 0;

  ///
  /// @brief True body position getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyPosition(double time) = 0;

  ///
  /// @brief True body velocity getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyVelocity(double time) = 0;

  ///
  /// @brief True body acceleration getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyAcceleration(double time) = 0;

  ///
  /// @brief True body orientation quaternion getter
  /// @param time Simulation time
  ///
  virtual Eigen::Quaterniond GetBodyAngularPosition(double time) = 0;

  ///
  /// @brief True body angular rate getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyAngularRate(double time) = 0;

  ///
  /// @brief True body angular acceleration getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyAngularAcceleration(double time) = 0;

  ///
  /// @brief True sensor position getter
  /// @param time Simulation time
  /// @param sensor_id IMU ID
  ///
  Eigen::Vector3d GetImuPosition(unsigned int sensor_id);

  ///
  /// @brief True sensor orientation getter
  /// @param time Simulation time
  /// @param sensor_id IMU ID
  ///
  Eigen::Quaterniond GetImuAngularPosition(unsigned int sensor_id);

  ///
  /// @brief True sensor orientation getter
  /// @param time Simulation time
  /// @param sensor_id IMU ID
  ///
  Eigen::Vector3d GetImuAccelerometerBias(unsigned int sensor_id);

  ///
  /// @brief True sensor orientation getter
  /// @param time Simulation time
  /// @param sensor_id IMU ID
  ///
  Eigen::Vector3d GetImuGyroscopeBias(unsigned int sensor_id);

  ///
  /// @brief True sensor position getter
  /// @param time Simulation time
  /// @param sensor_id Camera ID
  ///
  Eigen::Vector3d GetCameraPosition(unsigned int sensor_id);

  ///
  /// @brief True sensor orientation getter
  /// @param time Simulation time
  /// @param sensor_id Camera ID
  ///
  Eigen::Quaterniond GetCameraAngularPosition(unsigned int sensor_id);

  ///
  /// @brief True sensor position setter
  /// @param sensor_id sensor ID
  /// @param imu_pos sensor parameter
  ///
  void SetImuPosition(unsigned int sensor_id, Eigen::Vector3d imu_pos);

  ///
  /// @brief True sensor orientation setter
  /// @param sensor_id sensor ID
  /// @param imu_ang_pos sensor parameter
  ///
  void SetImuAngularPosition(unsigned int sensor_id, Eigen::Quaterniond imu_ang_pos);

  ///
  /// @brief True sensor orientation setter
  /// @param sensor_id sensor ID
  /// @param imu_acc_bias sensor parameter
  ///
  void SetImuAccelerometerBias(unsigned int sensor_id, Eigen::Vector3d imu_acc_bias);

  ///
  /// @brief True sensor orientation setter
  /// @param sensor_id sensor ID
  /// @param imu_gyro_bias sensor parameter
  ///
  void SetImuGyroscopeBias(unsigned int sensor_id, Eigen::Vector3d imu_gyro_bias);

  ///
  /// @brief True sensor position setter
  /// @param sensor_id sensor ID
  /// @param cam_pos sensor parameter
  ///
  void SetCameraPosition(unsigned int sensor_id, Eigen::Vector3d cam_pos);

  ///
  /// @brief True sensor orientation setter
  /// @param sensor_id sensor ID
  /// @param cam_ang_pos sensor parameter
  ///
  void SetCameraAngularPosition(unsigned int sensor_id, Eigen::Quaterniond cam_ang_pos);

protected:
  std::map<unsigned int, Eigen::Vector3d> m_imu_pos;
  std::map<unsigned int, Eigen::Quaterniond> m_imu_ang_pos;
  std::map<unsigned int, Eigen::Vector3d> m_imu_acc_bias;
  std::map<unsigned int, Eigen::Vector3d> m_imu_gyro_bias;
  std::map<unsigned int, Eigen::Vector3d> m_cam_pos;
  std::map<unsigned int, Eigen::Quaterniond> m_cam_ang_pos;

private:
  DebugLogger * m_logger = DebugLogger::GetInstance();  ///< @brief Logger singleton
};

#endif  // INFRASTRUCTURE__SIM__TRUTH_ENGINE_HPP_
