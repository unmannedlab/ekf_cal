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

#include "sensors/sensor.hpp"

#include <memory>
#include <string>

#include "sensors/sensor_message.hpp"

// Initialize static variable
unsigned int Sensor::m_sensor_count = 0;

Sensor::Sensor(Parameters params)
: m_id(++m_sensor_count)
{
  m_name = params.name;
  m_logger = params.logger;
}

unsigned int Sensor::GetId()
{
  return m_id;
}

std::string Sensor::GetName()
{
  return m_name;
}

bool MessageCompare(std::shared_ptr<SensorMessage> a, std::shared_ptr<SensorMessage> b)
{
  return a->time < b->time;
}

void Sensor::Callback(SensorMessage sensor_message)
{
  std::stringstream msg;
  msg << "Base Sensor callback invoked at time " << sensor_message.time;
  msg << " for sensor " << sensor_message.sensor_id;
  msg << " of type " << static_cast<unsigned int>(sensor_message.sensor_type);
  m_logger->Log(LogLevel::INFO, msg.str());
  m_logger->Log(LogLevel::INFO, "Base Sensor callback invoked");
}
