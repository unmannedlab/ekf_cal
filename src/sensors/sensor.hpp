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

#include "ekf/ekf.hpp"
#include "sensors/sensor_message.hpp"
#include "infrastructure/debug_logger.hpp"


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
  unsigned int GetId();

  ///
  /// @brief Sensor name getter method
  /// @return Sensor name
  ///
  std::string GetName();

  ///
  /// @brief
  /// @param sensor_message
  ///
  void Callback(SensorMessage sensor_message);

protected:
  double m_rate;                      ///< @brief Sensor measurement rate
  unsigned int m_id;                  ///< @brief Sensor id
  std::string m_name;                 ///< @brief Sensor name

  EKF * m_ekf = EKF::GetInstance();           ///< @brief EKF singleton
  DebugLogger * m_logger = DebugLogger::GetInstance();  ///< @brief Logger singleton

private:
  static unsigned int m_sensor_count;
};


bool MessageCompare(std::shared_ptr<SensorMessage> a, std::shared_ptr<SensorMessage> b);

#endif  // SENSORS__SENSOR_HPP_
