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

#ifndef SENSORS__SENSOR_HPP_
#define SENSORS__SENSOR_HPP_

#include <eigen3/Eigen/Eigen>

#include <string>
#include <vector>

#include "ekf/EKF.hpp"


///
/// @class Sensor
/// @brief Pure base sensor class
///
class Sensor
{
public:
  ///
  /// @brief Sensor class constructor
  /// @param name Sensor name
  ///
  explicit Sensor(std::string name);

  ///
  /// @brief Predict measurement method
  /// @return Predicted measurement vector
  ///
  virtual Eigen::VectorXd predictMeasurement() = 0;

  ///
  /// @brief Measurement Jacobian method
  /// @return Measurement Jacobian matrix
  ///
  virtual Eigen::MatrixXd getMeasurementJacobian() = 0;

  ///
  /// @brief Sensor position offset getter method
  /// @return Position offset vector
  ///
  Eigen::Vector3d getPosOffset();

  ///
  /// @brief Sensor angular offset getter method
  /// @return Angular offset quaternion
  ///
  Eigen::Quaterniond getAngOffset();

  ///
  /// @brief Sensor name getter
  /// @return Sensor name
  ///
  std::string getName();

  ///
  /// @brief Sensor ID getter method
  /// @return Sensor ID
  ///
  unsigned int getId();

  ///
  /// @brief Sensor state start index getter method
  /// @return Sensor state start index
  ///
  unsigned int getStateStartIndex();

  ///
  /// @brief Sensor state size getter method
  /// @return Sensor state size
  ///
  unsigned int getStateSize();

protected:
  ///
  /// @brief Protected state setter method
  /// @param state EKF state
  ///
  virtual void setState() = 0;

  Eigen::Vector3d m_posOffset{0.0, 0.0, 0.0};          ///< @brief Sensor position offset vector
  Eigen::Quaterniond m_angOffset{0.0, 0.0, 0.0, 0.0};  ///< @brief Sensor angular offset quaternion

  unsigned int m_stateStartIndex{0};  ///< @brief Sensor state start index
  unsigned int m_stateSize{0};        ///< @brief Sensor state size

  static Eigen::Vector3d m_bodyPos;         ///< @brief Body position vector
  static Eigen::Vector3d m_bodyVel;         ///< @brief Body velocity vector
  static Eigen::Vector3d m_bodyAcc;         ///< @brief Body acceleration vector
  static Eigen::Quaterniond m_bodyAngPos;   ///< @brief Body angular position quaternion
  static Eigen::Vector3d m_bodyAngVel;      ///< @brief Body angular velocity vector
  static Eigen::Vector3d m_bodyAngAcc;      ///< @brief Body angular acceleration vector

  double m_rate;                      ///< @brief Sensor measurement rate
  unsigned int m_id;                  ///< @brief Sensor id
  std::string m_name;                 ///< @brief Sensor name

  EKF * m_ekf = EKF::getInstance();           ///< @brief EKF singleton
  Logger * m_logger = Logger::getInstance();  ///< @brief Logger singleton

private:
  static unsigned int _idCount;
};

#endif  // SENSORS__SENSOR_HPP_
