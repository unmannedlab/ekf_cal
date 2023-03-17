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

#include "infrastructure/sim/TruthEngineCyclic.hpp"

#include <eigen3/Eigen/Eigen>
#include <math.h>

Eigen::Vector3d TruthEngineCyclic::GetBodyPosition(double time)
{
  Eigen::Vector3d position;
  position[0] = std::sin(m_posCycleFrequency[0] * time / 2 / M_PI);
  position[1] = std::sin(m_posCycleFrequency[1] * time / 2 / M_PI);
  position[2] = std::sin(m_posCycleFrequency[2] * time / 2 / M_PI);
  return position;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyVelocity(double time)
{
  Eigen::Vector3d velocity;
  velocity[0] = m_posCycleFrequency[0] * std::cos(m_posCycleFrequency[0] * time / 2 / M_PI);
  velocity[1] = m_posCycleFrequency[1] * std::cos(m_posCycleFrequency[1] * time / 2 / M_PI);
  velocity[2] = m_posCycleFrequency[2] * std::cos(m_posCycleFrequency[2] * time / 2 / M_PI);
  Eigen::Quaterniond angPos = GetBodyAngularPosition(time);
  return angPos * velocity;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAcceleration(double time)
{
  Eigen::Vector3d acceleration;
  acceleration[0] = -m_posCycleFrequency[0] * m_posCycleFrequency[0] *
    std::sin(m_posCycleFrequency[0] * time / 2 / M_PI);
  acceleration[1] = -m_posCycleFrequency[1] * m_posCycleFrequency[1] *
    std::sin(m_posCycleFrequency[1] * time / 2 / M_PI);
  acceleration[2] = -m_posCycleFrequency[2] * m_posCycleFrequency[2] *
    std::sin(m_posCycleFrequency[2] * time / 2 / M_PI);
  Eigen::Quaterniond angPos = GetBodyAngularPosition(time);
  return angPos * acceleration;
}

Eigen::Quaterniond TruthEngineCyclic::GetBodyAngularPosition(double time)
{
  double a = std::sin(m_angCycleFrequency[0] * time / 2 / M_PI);
  double b = std::sin(m_angCycleFrequency[1] * time / 2 / M_PI);
  double g = std::sin(m_angCycleFrequency[2] * time / 2 / M_PI);

  Eigen::Quaterniond angularPosition =
    Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(g, Eigen::Vector3d::UnitZ());

  return angularPosition;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAngularRate(double time)
{
  Eigen::Vector3d angularRate;
  angularRate[0] = m_angCycleFrequency[0] * std::cos(m_angCycleFrequency[0] * time / 2 / M_PI);
  angularRate[1] = m_angCycleFrequency[1] * std::cos(m_angCycleFrequency[1] * time / 2 / M_PI);
  angularRate[2] = m_angCycleFrequency[2] * std::cos(m_angCycleFrequency[2] * time / 2 / M_PI);
  return angularRate;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAngularAcceleration(double time)
{
  Eigen::Vector3d angularAcceleration;
  angularAcceleration[0] = -m_angCycleFrequency[0] * m_angCycleFrequency[0] * std::sin(
    m_angCycleFrequency[0] * time / 2 / M_PI);
  angularAcceleration[1] = -m_angCycleFrequency[1] * m_angCycleFrequency[1] * std::sin(
    m_angCycleFrequency[1] * time / 2 / M_PI);
  angularAcceleration[2] = -m_angCycleFrequency[2] * m_angCycleFrequency[2] * std::sin(
    m_angCycleFrequency[2] * time / 2 / M_PI);
  return angularAcceleration;
}

void TruthEngineCyclic::SetBodyPosCycleFrequency(Eigen::Vector3d period)
{
  m_posCycleFrequency = period;
}

void TruthEngineCyclic::SetBodyAngCycleFrequency(Eigen::Vector3d period)
{
  m_angCycleFrequency = period;
}
