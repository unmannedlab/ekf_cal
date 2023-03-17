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

#include <random>
#include <eigen3/Eigen/Eigen>
#include <math.h>

Eigen::Vector3d TruthEngine::GetBodyPosition(double time)
{
  return Eigen::Vector3d::Zero(3);
}

Eigen::Vector3d TruthEngine::GetBodyVelocity(double time)
{
  return Eigen::Vector3d::Zero(3);
}

Eigen::Vector3d TruthEngine::GetBodyAcceleration(double time)
{
  return Eigen::Vector3d::Zero(3);
}

Eigen::Quaterniond TruthEngine::GetBodyAngularPosition(double time)
{
  return Eigen::Quaterniond::Identity();
}

Eigen::Vector3d TruthEngine::GetBodyAngularRate(double time)
{
  return Eigen::Vector3d::Zero(3);
}

Eigen::Vector3d TruthEngine::GetBodyAngularAcceleration(double time)
{
  return Eigen::Vector3d::Zero(3);
}
