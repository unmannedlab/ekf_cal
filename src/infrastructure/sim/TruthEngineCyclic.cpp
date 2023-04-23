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
  position[0] = std::sin(m_pos_cycle_frequency[0] * time / 2 / M_PI);
  position[1] = std::sin(m_pos_cycle_frequency[1] * time / 2 / M_PI);
  position[2] = std::sin(m_pos_cycle_frequency[2] * time / 2 / M_PI);
  return position;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyVelocity(double time)
{
  Eigen::Vector3d velocity;
  velocity[0] = m_pos_cycle_frequency[0] * std::cos(m_pos_cycle_frequency[0] * time / 2 / M_PI);
  velocity[1] = m_pos_cycle_frequency[1] * std::cos(m_pos_cycle_frequency[1] * time / 2 / M_PI);
  velocity[2] = m_pos_cycle_frequency[2] * std::cos(m_pos_cycle_frequency[2] * time / 2 / M_PI);
  Eigen::Quaterniond ang_pos = GetBodyAngularPosition(time);
  return ang_pos * velocity;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAcceleration(double time)
{
  Eigen::Vector3d acceleration;
  acceleration[0] = -m_pos_cycle_frequency[0] * m_pos_cycle_frequency[0] *
    std::sin(m_pos_cycle_frequency[0] * time / 2 / M_PI);
  acceleration[1] = -m_pos_cycle_frequency[1] * m_pos_cycle_frequency[1] *
    std::sin(m_pos_cycle_frequency[1] * time / 2 / M_PI);
  acceleration[2] = -m_pos_cycle_frequency[2] * m_pos_cycle_frequency[2] *
    std::sin(m_pos_cycle_frequency[2] * time / 2 / M_PI);
  Eigen::Quaterniond ang_pos = GetBodyAngularPosition(time);
  return ang_pos * acceleration;
}

Eigen::Quaterniond TruthEngineCyclic::GetBodyAngularPosition(double time)
{
  double a = std::sin(m_ang_cycle_frequency[0] * time / 2 / M_PI);
  double b = std::sin(m_ang_cycle_frequency[1] * time / 2 / M_PI);
  double g = std::sin(m_ang_cycle_frequency[2] * time / 2 / M_PI);

  Eigen::Quaterniond angular_position =
    Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(g, Eigen::Vector3d::UnitZ());

  return angular_position;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAngularRate(double time)
{
  Eigen::Vector3d angular_rate;
  angular_rate[0] = m_ang_cycle_frequency[0] * std::cos(m_ang_cycle_frequency[0] * time / 2 / M_PI);
  angular_rate[1] = m_ang_cycle_frequency[1] * std::cos(m_ang_cycle_frequency[1] * time / 2 / M_PI);
  angular_rate[2] = m_ang_cycle_frequency[2] * std::cos(m_ang_cycle_frequency[2] * time / 2 / M_PI);
  return angular_rate;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAngularAcceleration(double time)
{
  Eigen::Vector3d angularAcceleration;
  angularAcceleration[0] = -m_ang_cycle_frequency[0] * m_ang_cycle_frequency[0] * std::sin(
    m_ang_cycle_frequency[0] * time / 2 / M_PI);
  angularAcceleration[1] = -m_ang_cycle_frequency[1] * m_ang_cycle_frequency[1] * std::sin(
    m_ang_cycle_frequency[1] * time / 2 / M_PI);
  angularAcceleration[2] = -m_ang_cycle_frequency[2] * m_ang_cycle_frequency[2] * std::sin(
    m_ang_cycle_frequency[2] * time / 2 / M_PI);
  return angularAcceleration;
}

void TruthEngineCyclic::SetBodyPosCycleFrequency(Eigen::Vector3d period)
{
  m_pos_cycle_frequency = period;
}

void TruthEngineCyclic::SetBodyAngCycleFrequency(Eigen::Vector3d period)
{
  m_ang_cycle_frequency = period;
}
