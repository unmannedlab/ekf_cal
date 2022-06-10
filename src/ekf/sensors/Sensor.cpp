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

#include "ekf/sensors/Sensor.hpp"

#include <string>

unsigned int Sensor::_idCount = 0;

Sensor::Sensor(std::string name)
: m_id(++_idCount), m_name(name) {}

std::string Sensor::GetName()
{
  return m_name;
}

unsigned int Sensor::GetId()
{
  return m_id;
}

unsigned int Sensor::GetStateStartIndex()
{
  return m_stateStartIndex;
}

unsigned int Sensor::GetStateSize()
{
  return m_stateSize;
}

void Sensor::SetStateStartIndex(unsigned int stateStartIndex)
{
  m_stateStartIndex = stateStartIndex;
}

Eigen::Vector3d Sensor::GetPosOffset()
{
  return m_posOffset;
}

void Sensor::SetPosOffset(Eigen::Vector3d posOffset)
{
  m_posOffset = posOffset;
}

Eigen::Quaterniond Sensor::GetQuatOffset()
{
  return m_quatOffset;
}

void Sensor::SetQuatOffset(Eigen::Quaterniond quatOffset)
{
  m_quatOffset = quatOffset;
}

Eigen::VectorXd Sensor::PredictMeasurement()
{
  Eigen::VectorXd predictedMeasurement(m_stateSize);
  return predictedMeasurement;
}

Eigen::MatrixXd Sensor::GetMeasurementJacobian()
{
  Eigen::MatrixXd measurementJacobian(m_stateSize, m_stateSize);
  return measurementJacobian;
}

Eigen::MatrixXd Sensor::GetMeasurementCovariance()
{
  Eigen::MatrixXd measurementCovariance(m_stateSize, m_stateSize);
  return measurementCovariance;
}
