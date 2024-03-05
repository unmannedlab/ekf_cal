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

#ifndef SENSORS__ROS__ROS_GPS_MESSAGE_HPP_
#define SENSORS__ROS__ROS_GPS_MESSAGE_HPP_

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "sensors/gps_message.hpp"


///
/// @class RosGpsMessage
/// @brief Ros GPS message class
///
class RosGpsMessage : public GpsMessage
{
public:
  ///
  /// @brief RosGpsMessage constructor
  /// @param msg Ros NavSatFix message pointer
  ///
  explicit RosGpsMessage(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};

#endif  // SENSORS__ROS__ROS_GPS_MESSAGE_HPP_
