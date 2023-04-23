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

#include "infrastructure/sim/TruthEngine.hpp"

#include <eigen3/Eigen/Eigen>
#include <math.h>

#include "infrastructure/DebugLogger.hpp"

Eigen::Vector3d TruthEngine::GetBodyPosition(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Vector3d::Zero(3);
}

Eigen::Vector3d TruthEngine::GetBodyVelocity(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Vector3d::Zero(3);
}

Eigen::Vector3d TruthEngine::GetBodyAcceleration(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Vector3d::Zero(3);
}

Eigen::Quaterniond TruthEngine::GetBodyAngularPosition(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Quaterniond::Identity();
}

Eigen::Vector3d TruthEngine::GetBodyAngularRate(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Vector3d::Zero(3);
}

Eigen::Vector3d TruthEngine::GetBodyAngularAcceleration(double time)
{
  m_logger->Log(LogLevel::WARN, "Base Truth Engine called at t=" + std::to_string(time));
  return Eigen::Vector3d::Zero(3);
}
