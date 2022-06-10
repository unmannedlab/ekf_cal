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

#ifndef EKF__EKF_HPP_
#define EKF__EKF_HPP_

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <vector>
#include <unordered_map>
#include <iostream>

#include "ekf/sensors/Camera.hpp"
#include "ekf/sensors/Imu.hpp"
#include "ekf/sensors/Lidar.hpp"
#include "ekf/sensors/Sensor.hpp"

///
/// @class EKF
/// @brief Calibration EKF class
///
class EKF
{
public:
  ///
  /// @brief EKF class constructor
  ///
  EKF();

  ///
  /// @brief Callback method for IMU messages
  ///
  void ImuCallback(
    unsigned int id, double time,
    Eigen::Vector3d acceleration,
    Eigen::Matrix3d accelerationCovariance,
    Eigen::Vector3d angularRate,
    Eigen::Matrix3d angularRateCovariance);

  void CameraCallback(unsigned int id, double time);

  void LidarCallback(unsigned int id, double time);

  ///
  /// @brief Registers new sensor to calibration EKF
  /// @tparam T Sensor time to register
  /// @param params Sensor parameters to use in registration
  ///
  unsigned int RegisterSensor(typename Imu::Params params);
  unsigned int RegisterSensor(typename Camera::Params params);
  unsigned int RegisterSensor(typename Lidar::Params params);

private:
  void Predict(double currentTime);
  Eigen::MatrixXd GetStateTransition(double dT);
  Eigen::MatrixXd GetProcessInput();
  Eigen::MatrixXd GetProcessNoise();


  unsigned int ExtendState(unsigned int sensorStateSize);
  unsigned int m_stateSize{18U};
  Eigen::VectorXd m_state = Eigen::VectorXd::Zero(18U);
  Eigen::MatrixXd m_cov = Eigen::MatrixXd::Zero(18U, 18U);
  std::map<int, std::shared_ptr<Imu>> m_mapImu{};
  std::map<int, std::shared_ptr<Camera>> m_mapCamera{};
  std::map<int, std::shared_ptr<Lidar>> m_mapLidar{};
  double m_currentTime {0};
};

#endif  // EKF__EKF_HPP_
