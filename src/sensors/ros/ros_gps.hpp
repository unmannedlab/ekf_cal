// Copyright 2024 Jacob Hartzer
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

#ifndef SENSORS__ROS__ROS_GPS_HPP_
#define SENSORS__ROS__ROS_GPS_HPP_

#include <memory>

#include "sensors/gps.hpp"
#include "sensors/ros/ros_gps_message.hpp"


///
/// @class RosGPS
/// @brief GPS Sensor Class
///
class RosGPS : public GPS
{
public:
  using GPS::GPS;

  ///
  /// @brief RosGPS callback method
  /// @param ros_gps_message ROS GPS message
  ///
  void Callback(const RosGpsMessage & ros_gps_message);
};

#endif  // SENSORS__ROS__ROS_GPS_HPP_
