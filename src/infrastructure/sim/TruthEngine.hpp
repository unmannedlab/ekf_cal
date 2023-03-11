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

#ifndef INFRASTRUCTURE__SIM__TRUTHENGINE_HPP_
#define INFRASTRUCTURE__SIM__TRUTHENGINE_HPP_

#include <eigen3/Eigen/Eigen>

///
/// @class TruthEngine
/// @brief Truth for low fidelity simulations
///
class TruthEngine
{
public:
  TruthEngine() {}

  Eigen::Vector3d GetBodyPosition();
  Eigen::Vector3d GetBodyVelocity();
  Eigen::Vector3d GetBodyAcceleration();

  Eigen::Quaterniond GetBodyAngularPosition();
  Eigen::Vector3d GetBodyAngularRate();
  Eigen::Vector3d GetBodyAngularAcceleration();
};

#endif  // INFRASTRUCTURE__SIM__TRUTHENGINE_HPP_
