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

#ifndef SENSORS__IMU_HPP_
#define SENSORS__IMU_HPP_

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>

#include "ekf/update/ImuUpdater.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "sensors/Sensor.hpp"


///
/// @class ImuMessage
/// @brief Data class for IMU messages
///
class ImuMessage : public SensorMessage
{
public:
  ImuMessage() {}
  Eigen::Vector3d acceleration;            ///< @brief IMU acceleration
  Eigen::Vector3d angularRate;             ///< @brief IMU angular rate
  Eigen::Matrix3d accelerationCovariance;  ///< @brief IMU acceleration covariance
  Eigen::Matrix3d angularRateCovariance;   ///< @brief IMU angular rate covariance
};


///
/// @class IMU
/// @brief IMU Sensor Class
/// @todo Add parameter input/defaults for covariance
/// @todo Bias Stability and Noise process inputs for IMUs
///
class IMU : public Sensor
{
public:
  ///
  /// @brief IMU initialization parameters structure
  /// @todo remove topic from parameters
  ///
  typedef struct Parameters
  {
    std::string name {"Name"};                      ///< @brief IMU name
    std::string topic {"Topic"};                    ///< @brief IMU topic
    bool baseSensor{true};                          ///< @brief IMU base sensor flag
    bool intrinsic{false};                          ///< @brief IMU intrinsic calibration
    double rate{1.0};                               ///< @brief IMU update rate
    Eigen::Vector3d posOffset {0, 0, 0};            ///< @brief IMU position offset vector
    Eigen::Quaterniond angOffset {1, 0, 0, 0};      ///< @brief IMU angular offset quaternion
    Eigen::Vector3d accBias {0, 0, 0};              ///< @brief IMU accelerometer bias vector
    Eigen::Vector3d omgBias {0, 0, 0};              ///< @brief IMU gyroscope bias vector
    double accBiasStability {0};                    ///< @brief IMU accelerometer bias stability
    double omgBiasStability {0};                    ///< @brief IMU gyroscope bias stability
    std::string outputDirectory {""};               ///< @brief IMU data logging directory
    bool dataLoggingOn {false};                     ///< @brief IMU data logging flag
    /// @brief Initial state variance
    Eigen::VectorXd variance {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
  } Parameters;

  ///
  /// @brief IMU class constructor
  /// @param params IMU sensor parameters
  ///
  explicit IMU(IMU::Parameters params);

  ///
  /// @brief Callback method for IMU measurements
  /// @param imuMessage IMU measurement message
  ///
  void callback(std::shared_ptr<ImuMessage> imuMessage);

  static constexpr unsigned int IMU_STATE_SIZE {12};

private:
  bool m_isBaseSensor;
  bool m_isIntrinsic;

  ImuUpdater m_imuUpdater;
};

#endif  // SENSORS__IMU_HPP_
