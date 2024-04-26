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

#include <memory>
#include <string>

#include "ekf/ekf.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/sensor_message.hpp"

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
  /// @brief Sensor parameter structure
  ///
  typedef struct Parameters
  {
    std::string name {""};                ///< @brief Name
    std::string topic {""};               ///< @brief Topic
    double rate{1.0};                     ///< @brief Update rate
    bool data_logging_on {false};         ///< @brief Data logging flag
    double data_log_rate {0.0};           ///< @brief Data logging rate
    std::string output_directory {""};    ///< @brief Data logging directory
    std::shared_ptr<DebugLogger> logger;  ///< @brief Debug logger
    std::shared_ptr<EKF> ekf;             ///< @brief EKF to update
  } Parameters;

  ///
  /// @brief Sensor class constructor
  /// @param name Sensor name
  /// @param logger Debug logger pointer
  ///
  explicit Sensor(std::string name, std::shared_ptr<DebugLogger> logger);

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
  /// @brief Sensor callback function
  /// @param sensor_message callback message
  ///
  void Callback(SensorMessage sensor_message);

protected:
  double m_rate;                          ///< @brief Sensor measurement rate
  unsigned int m_id;                      ///< @brief Sensor id
  std::string m_name;                     ///< @brief Sensor name
  std::shared_ptr<DebugLogger> m_logger;  ///< @brief Debug logger
  bool m_is_initialized{false};           ///< @brief Sensor initialization flag

private:
  static unsigned int m_sensor_count;     ///< @brief Static sensor count
};


bool MessageCompare(std::shared_ptr<SensorMessage> a, std::shared_ptr<SensorMessage> b);

#endif  // SENSORS__SENSOR_HPP_
