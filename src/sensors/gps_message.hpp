// Copyright 2023 Jacob Hartzer
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


#ifndef SENSORS__GPS_MESSAGE_HPP_
#define SENSORS__GPS_MESSAGE_HPP_

#include <eigen3/Eigen/Eigen>

#include "sensors/sensor_message.hpp"

///
/// @class GpsMessage
/// @brief Data class for GPS messages
///
class GpsMessage : public SensorMessage
{
public:
  GpsMessage() {}
  double m_latitude;   ///< @brief GPS latitude
  double m_longitude;  ///< @brief GPS longitude
  double m_altitude;   ///< @brief GPS altitude
};

#endif  // SENSORS__GPS_MESSAGE_HPP_
