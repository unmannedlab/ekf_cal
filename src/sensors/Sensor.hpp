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

#include <memory>
#include <string>

#include "ekf/EKF.hpp"
#include "infrastructure/DebugLogger.hpp"


enum class SensorType
{
  IMU,
  Camera,
  Tracker
};

///
/// @class SensorMessage
/// @brief Sensor message base class
///
class SensorMessage
{
public:
  /// @brief SensorMessage constructor
  SensorMessage() {}

  /// @brief Associated sensor ID of measurement
  unsigned int sensorID;

  /// @brief Associated sensor type of measurement
  SensorType sensorType;

  /// @brief Measurement time
  double time;
};

///
/// @class Sensor
/// @brief Pure base sensor class
/// @todo Time offset filter
/// @todo Function for checking callback rate
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
  /// @brief Sensor ID getter method
  /// @return Sensor ID
  ///
  unsigned int getId();

  ///
  /// @brief Sensor name getter method
  /// @return Sensor name
  ///
  std::string getName();

  ///
  /// @brief
  /// @param sensorMessage
  ///
  void callback(SensorMessage sensorMessage);

protected:
  Eigen::Vector3d m_posOffset{0.0, 0.0, 0.0};          ///< @brief Sensor position offset vector
  Eigen::Quaterniond m_angOffset{0.0, 0.0, 0.0, 0.0};  ///< @brief Sensor angular offset quaternion

  double m_rate;                      ///< @brief Sensor measurement rate
  unsigned int m_id;                  ///< @brief Sensor id
  std::string m_name;                 ///< @brief Sensor name

  EKF * m_ekf = EKF::getInstance();           ///< @brief EKF singleton
  DebugLogger * m_logger = DebugLogger::getInstance();  ///< @brief Logger singleton

private:
  static unsigned int _sensorCount;
};


bool messageCompare(std::shared_ptr<SensorMessage> a, std::shared_ptr<SensorMessage> b);

#endif  // SENSORS__SENSOR_HPP_
