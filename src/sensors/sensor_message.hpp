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

#ifndef SENSORS__SENSOR_MESSAGE_HPP_
#define SENSORS__SENSOR_MESSAGE_HPP_

#include "sensors/types.hpp"

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
  unsigned int sensor_id;

  /// @brief Associated sensor type of measurement
  SensorType sensor_type;

  /// @brief Measurement time
  double time;
};

#endif  // SENSORS__SENSOR_MESSAGE_HPP_
