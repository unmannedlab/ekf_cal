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

#include "infrastructure/sim/truth_engine_cyclic.hpp"

#include <eigen3/Eigen/Eigen>
#include <math.h>

#include <cmath>


TruthEngineCyclic::TruthEngineCyclic(
  Eigen::Vector3d pos_frequency,
  Eigen::Vector3d ang_frequency,
  Eigen::Vector3d pos_offset)
{
  m_pos_cycle_frequency = pos_frequency;
  m_ang_cycle_frequency = ang_frequency;
  m_pos_offset = pos_offset;
  m_ang_amplitude = 0.1;
  m_pos_amplitude = 1.0;
}


Eigen::Vector3d TruthEngineCyclic::GetBodyPosition(double time)
{
  Eigen::Vector3d position;
  position[0] = m_pos_amplitude * (1 - std::cos(m_pos_cycle_frequency[0] * 2 * M_PI * time)) +
    m_pos_offset[0];
  position[1] = m_pos_amplitude * (1 - std::cos(m_pos_cycle_frequency[1] * 2 * M_PI * time)) +
    m_pos_offset[1];
  position[2] = m_pos_amplitude * (1 - std::cos(m_pos_cycle_frequency[2] * 2 * M_PI * time)) +
    m_pos_offset[2];
  return position;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyVelocity(double time)
{
  Eigen::Vector3d velocity;
  velocity[0] = m_pos_amplitude * m_pos_cycle_frequency[0] * 2 * M_PI * std::sin(
    m_pos_cycle_frequency[0] * 2 * M_PI * time);
  velocity[1] = m_pos_amplitude * m_pos_cycle_frequency[1] * 2 * M_PI * std::sin(
    m_pos_cycle_frequency[1] * 2 * M_PI * time);
  velocity[2] = m_pos_amplitude * m_pos_cycle_frequency[2] * 2 * M_PI * std::sin(
    m_pos_cycle_frequency[2] * 2 * M_PI * time);
  Eigen::Quaterniond ang_pos = GetBodyAngularPosition(time);
  return ang_pos * velocity;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAcceleration(double time)
{
  Eigen::Vector3d acceleration;
  acceleration[0] = m_pos_amplitude * 4 * M_PI * M_PI * m_pos_cycle_frequency[0] *
    m_pos_cycle_frequency[0] * std::cos(m_pos_cycle_frequency[0] * 2 * M_PI * time);
  acceleration[1] = m_pos_amplitude * 4 * M_PI * M_PI * m_pos_cycle_frequency[1] *
    m_pos_cycle_frequency[1] * std::cos(m_pos_cycle_frequency[1] * 2 * M_PI * time);
  acceleration[2] = m_pos_amplitude * 4 * M_PI * M_PI * m_pos_cycle_frequency[2] *
    m_pos_cycle_frequency[2] * std::cos(m_pos_cycle_frequency[2] * 2 * M_PI * time);
  Eigen::Quaterniond ang_pos = GetBodyAngularPosition(time);
  return ang_pos * acceleration;
}

Eigen::Quaterniond TruthEngineCyclic::GetBodyAngularPosition(double time)
{
  double a = m_ang_amplitude * (1 - std::cos(m_ang_cycle_frequency[0] * 2 * M_PI * time));
  double b = m_ang_amplitude * (1 - std::cos(m_ang_cycle_frequency[1] * 2 * M_PI * time));
  double g = m_ang_amplitude * (1 - std::cos(m_ang_cycle_frequency[2] * 2 * M_PI * time));

  Eigen::Quaterniond angular_position =
    Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(g, Eigen::Vector3d::UnitZ());

  return angular_position;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAngularRate(double time)
{
  Eigen::Vector3d angular_rate;
  angular_rate[0] = m_ang_amplitude * m_ang_cycle_frequency[0] * 2 * M_PI * std::sin(
    m_ang_cycle_frequency[0] * 2 * M_PI * time);
  angular_rate[1] = m_ang_amplitude * m_ang_cycle_frequency[1] * 2 * M_PI * std::sin(
    m_ang_cycle_frequency[1] * 2 * M_PI * time);
  angular_rate[2] = m_ang_amplitude * m_ang_cycle_frequency[2] * 2 * M_PI * std::sin(
    m_ang_cycle_frequency[2] * 2 * M_PI * time);
  return angular_rate;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAngularAcceleration(double time)
{
  Eigen::Vector3d angularAcceleration;
  angularAcceleration[0] = m_ang_amplitude * 4 * M_PI * M_PI * m_ang_cycle_frequency[0] *
    m_ang_cycle_frequency[0] * std::cos(m_ang_cycle_frequency[0] * 2 * M_PI * time);
  angularAcceleration[1] = m_ang_amplitude * 4 * M_PI * M_PI * m_ang_cycle_frequency[1] *
    m_ang_cycle_frequency[1] * std::cos(m_ang_cycle_frequency[1] * 2 * M_PI * time);
  angularAcceleration[2] = m_ang_amplitude * 4 * M_PI * M_PI * m_ang_cycle_frequency[2] *
    m_ang_cycle_frequency[2] * std::cos(m_ang_cycle_frequency[2] * 2 * M_PI * time);
  return angularAcceleration;
}

void TruthEngineCyclic::SetBodyPosCycleFrequency(Eigen::Vector3d frequency)
{
  m_pos_cycle_frequency = frequency;
}

void TruthEngineCyclic::SetBodyAngCycleFrequency(Eigen::Vector3d frequency)
{
  m_ang_cycle_frequency = frequency;
}
