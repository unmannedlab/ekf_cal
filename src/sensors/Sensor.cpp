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
unsigned int Sensor::_idCount = 0;

Sensor::Sensor(std::string name)
: m_id(++_idCount), m_name(name) {}

unsigned int Sensor::getId()
{
  return m_id;
}

std::string Sensor::getName()
{
  return m_name;
}

void Sensor::callback(SensorMessage sensorMessage)
{
  m_logger->log(
    LogLevel::ERROR,
    "Callback on Base Sensor Called at Time" + std::to_string(sensorMessage.time));
}

bool messageCompare(std::shared_ptr<SensorMessage> a, std::shared_ptr<SensorMessage> b)
{
  return a->time < b->time;
}
