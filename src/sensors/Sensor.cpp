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

#include "sensors/Sensor.hpp"

#include <memory>
#include <string>

#include "utility/TypeHelper.hpp"

// Initialize static variable
unsigned int Sensor::m_sensor_count = 0;

Sensor::Sensor(std::string name)
: m_id(++m_sensor_count), m_name(name) {}

unsigned int Sensor::GetId()
{
  return m_id;
}

std::string Sensor::GetName()
{
  return m_name;
}

void Sensor::Callback(SensorMessage sensor_message)
{
  m_logger->Log(
    LogLevel::ERROR,
    "Callback on Base Sensor Called at Time" + std::to_string(sensor_message.m_time));
}

bool MessageCompare(std::shared_ptr<SensorMessage> a, std::shared_ptr<SensorMessage> b)
{
  return a->m_time < b->m_time;
}
