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

#include "sensors/ros/ros_imu.hpp"
#include "sensors/ros/ros_imu_message.hpp"

void RosIMU::Callback(std::shared_ptr<RosImuMessage> ros_imu_message)
{
  auto imu_message = std::make_shared<ImuMessage>();
  imu_message->m_time = ros_imu_message->m_time;
  imu_message->m_sensor_id = ros_imu_message->m_sensor_id;
  imu_message->m_sensor_type = ros_imu_message->m_sensor_type;
  IMU::Callback(imu_message);

  m_logger->Log(LogLevel::DEBUG, "Image publish ROS");
}
