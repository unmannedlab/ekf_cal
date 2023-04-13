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

#include "infrastructure/DebugLogger.hpp"

///
/// @class TruthEngine
/// @brief Truth for simulation
/// @todo Add initialization time to start of sim
///
class TruthEngine
{
public:
  TruthEngine() {}

  ///
  /// @brief True body position getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyPosition(double time) = 0;

  ///
  /// @brief True body velocity getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyVelocity(double time) = 0;

  ///
  /// @brief True body acceleration getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyAcceleration(double time) = 0;

  ///
  /// @brief True body orientation quaternion getter
  /// @param time Simulation time
  ///
  virtual Eigen::Quaterniond GetBodyAngularPosition(double time) = 0;

  ///
  /// @brief True body angular rate getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyAngularRate(double time) = 0;

  ///
  /// @brief True body angular acceleration getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyAngularAcceleration(double time) = 0;

private:
  DebugLogger * m_logger = DebugLogger::getInstance();  ///< @brief Logger singleton
};

#endif  // INFRASTRUCTURE__SIM__TRUTHENGINE_HPP_
