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

#ifndef INFRASTRUCTURE__SIM__TRUTH_ENGINE_SPLINE_HPP_
#define INFRASTRUCTURE__SIM__TRUTH_ENGINE_SPLINE_HPP_

#include <eigen3/Eigen/Eigen>
#include <eigen3/unsupported/Eigen/Splines>

#include <memory>
#include <vector>

#include "infrastructure/sim/truth_engine.hpp"

///
/// @class TruthEngineSpline
/// @brief Truth for simulation
///
class TruthEngineSpline : public TruthEngine
{
public:
  ///
  /// @brief Spline-based truth engine
  /// @param positions Position control points
  /// @param angles Angular position control points
  /// @param stationary_time Time to be stationary before beginning motion
  /// @param max_time Maximum simulation time
  /// @param logger Debug logger pointer
  ///
  TruthEngineSpline(
    std::vector<std::vector<double>> positions,
    std::vector<std::vector<double>> angles,
    double stationary_time,
    double max_time,
    std::shared_ptr<DebugLogger> logger
  );

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

private:
  bool IsTimeInvalid(double time);
  bool IsSplineInvalid(Eigen::Spline3d spline);

  double m_stationary_time {0.0};
  Eigen::Spline3d m_pos_spline;
  Eigen::Spline3d m_ang_spline;
};

#endif  // INFRASTRUCTURE__SIM__TRUTH_ENGINE_SPLINE_HPP_
