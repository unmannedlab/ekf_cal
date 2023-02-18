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

#include <string>

#include "sensors/Sensor.hpp"
#include "infrastructure/Logger.hpp"

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
  typedef struct Params
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
    /// @brief Initial state variance
    Eigen::VectorXd variance {{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
  } Params;

  ///
  /// @brief IMU class constructor
  /// @param params IMU sensor parameters
  ///
  explicit IMU(IMU::Params params);

  ///
  /// @brief Predict measurement method
  /// @return Predicted measurement vector
  /// @todo Add gravity to prediction
  ///
  Eigen::VectorXd predictMeasurement();

  ///
  /// @brief Measurement Jacobian method
  /// @return Measurement Jacobian matrix
  ///
  Eigen::MatrixXd getMeasurementJacobian();

  ///
  /// @brief Accelerometer bias stability getter method
  /// @return Accelerometer bias stability
  ///
  double getAccBiasStability();

  ///
  /// @brief Gyroscope bias stability getter method
  /// @return Gyroscope bias stability
  ///
  double getOmgBiasStability();

  ///
  /// @brief Callback method for IMU measurements
  /// @param time Measurement time
  /// @param acceleration Measured acceleration
  /// @param accelerationCovariance Estimated acceleration error
  /// @param angularRate Measured angular rate
  /// @param angularRateCovariance Estimated angular rate error
  ///
  void callback(
    double time, Eigen::Vector3d acceleration, Eigen::Matrix3d accelerationCovariance,
    Eigen::Vector3d angularRate, Eigen::Matrix3d angularRateCovariance);


  static const Eigen::Vector3d GRAVITY;

private:
  bool m_isBaseSensor;
  bool m_isIntrinsic;
  Eigen::Vector3d m_posOffset;
  Eigen::Quaterniond m_angOffset;
  Eigen::Vector3d m_accBias;
  Eigen::Vector3d m_omgBias;
  double m_accBiasStability;
  double m_omgBiasStability;
};

#endif  // SENSORS__IMU_HPP_
