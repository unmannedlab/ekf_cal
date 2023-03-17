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

#ifndef INFRASTRUCTURE__SIM__TRUTHENGINESPLINE_HPP_
#define INFRASTRUCTURE__SIM__TRUTHENGINESPLINE_HPP_

#include <eigen3/Eigen/Eigen>
#include <eigen3/unsupported/Eigen/Splines>

#include <vector>

#include "infrastructure/sim/TruthEngine.hpp"

///
/// @class TruthEngineSpline
/// @brief Truth for simulation
/// @todo Add initialization time to start of sim
/// @todo Add csv reading option to set control points
///
class TruthEngineSpline : public TruthEngine
{
public:
  TruthEngineSpline();

  ///
  /// @brief True body position getter
  /// @param time Simulation time
  ///
  Eigen::Vector3d GetBodyPosition(double time);

  ///
  /// @brief True body velocity getter
  /// @param time Simulation time
  ///
  Eigen::Vector3d GetBodyVelocity(double time);

  ///
  /// @brief True body acceleration getter
  /// @param time Simulation time
  ///
  Eigen::Vector3d GetBodyAcceleration(double time);

  ///
  /// @brief True body orientation quaternion getter
  /// @param time Simulation time
  ///
  Eigen::Quaterniond GetBodyAngularPosition(double time);

  ///
  /// @brief True body angular rate getter
  /// @param time Simulation time
  ///
  Eigen::Vector3d GetBodyAngularRate(double time);

  ///
  /// @brief True body angular acceleration getter
  /// @param time Simulation time
  ///
  Eigen::Vector3d GetBodyAngularAcceleration(double time);

  ///
  /// @brief Setter function for spline control points
  /// @param points Control Points
  ///
  void SetControlPoints(
    std::vector<Eigen::Vector3d> positions,
    std::vector<Eigen::Vector3d> angles);

private:
  bool IsTimeInvalid(double time);

  double m_timeMax {0.0};
  Eigen::Spline3d m_posSpline;
  Eigen::Spline3d m_angSpline;
};

#endif  // INFRASTRUCTURE__SIM__TRUTHENGINESPLINE_HPP_
