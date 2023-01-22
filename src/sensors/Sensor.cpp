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

#include <string>

#include "utility/TypeHelper.hpp"

// Initialize static variables
unsigned int Sensor::_idCount = 0;
Eigen::Vector3d Sensor::m_bodyPos(0.0, 0.0, 0.0);
Eigen::Vector3d Sensor::m_bodyVel(0.0, 0.0, 0.0);
Eigen::Vector3d Sensor::m_bodyAcc(0.0, 0.0, 0.0);
Eigen::Quaterniond Sensor::m_bodyAngPos(0.0, 0.0, 0.0, 0.0);
Eigen::Vector3d Sensor::m_bodyAngVel(0.0, 0.0, 0.0);
Eigen::Vector3d Sensor::m_bodyAngAcc(0.0, 0.0, 0.0);


Sensor::Sensor(std::string name)
: m_id(++_idCount), m_name(name) {}

std::string Sensor::getName()
{
  return m_name;
}

unsigned int Sensor::getId()
{
  return m_id;
}

unsigned int Sensor::getStateStartIndex()
{
  return m_stateStartIndex;
}

unsigned int Sensor::getStateSize()
{
  return m_stateSize;
}

Eigen::Vector3d Sensor::getPosOffset()
{
  return m_posOffset;
}

Eigen::Quaterniond Sensor::getAngOffset()
{
  return m_angOffset;
}
