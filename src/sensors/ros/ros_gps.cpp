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

#include <memory>

#include "sensors/ros/ros_gps.hpp"
#include "sensors/ros/ros_gps_message.hpp"

void RosGPS::Callback(RosGpsMessage & ros_gps_message)
{
  GPS::Callback(ros_gps_message);

  m_logger->Log(LogLevel::DEBUG, "Image publish ROS");
}
