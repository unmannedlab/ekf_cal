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

#ifndef INFRASTRUCTURE__SIM__TRUTH_ENGINE_CYCLIC_HPP_
#define INFRASTRUCTURE__SIM__TRUTH_ENGINE_CYCLIC_HPP_

#include <eigen3/Eigen/Eigen>

#include <memory>

#include "infrastructure/sim/truth_engine.hpp"

///
/// @class TruthEngineCyclic
/// @brief Truth for simulation
///
class TruthEngineCyclic : public TruthEngine
{
public:
  ///
  /// @brief Setter for body cycle frequencies
  /// @param pos_frequency Vector of position frequencies
  /// @param ang_frequency Vector of angular frequencies
  /// @param pos_offset Initial position offset in the local frame
  /// @param ang_offset Initial angular offset to the local frame rpy
  /// @param pos_amplitude Amplitude of position cycle
  /// @param ang_amplitude Amplitude of angular cycle
  /// @param stationary_time Time to be stationary before beginning motion
  /// @param max_time Maximum simulation time
  /// @param logger Debug logger pointer
  ///
  TruthEngineCyclic(
    Eigen::Vector3d pos_frequency,
    Eigen::Vector3d ang_frequency,
    Eigen::Vector3d pos_offset,
    Eigen::Vector3d ang_offset,
    double pos_amplitude,
    double ang_amplitude,
    double stationary_time,
    double max_time,
    std::shared_ptr<DebugLogger> logger
  );

  ///
  /// @brief Setter for body position cycle frequency
  /// @param frequency Vector of frequencies to use
  ///
  void SetBodyPosCycleFrequency(const Eigen::Vector3d & frequency);

  ///
  /// @brief Setter for body euler angle cycle frequency
  /// @param frequency Vector of frequencies to use
  ///
  void SetBodyAngCycleFrequency(const Eigen::Vector3d & frequency);

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
  /// @brief True body Roll-Pitch-Yaw vector getter
  /// @param time Simulation time
  ///
  Eigen::Vector3d GetBodyRollPitchYaw(double time);

  ///
  /// @brief True body orientation quaternion getter
  /// @param time Simulation time
  ///
  Eigen::Quaterniond GetBodyAngularPosition(double time);

  ///
  /// @brief Calculate transformation matrix to convert RPY derivatives into final frame
  /// @param rpy_vector Roll-Pitch-Yaw vector
  ///
  Eigen::Matrix3d EulerDerivativeTransform(const Eigen::Vector3d & rpy_vector);

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
  Eigen::Vector3d m_pos_frequency {0.0, 0.0, 0.0};
  Eigen::Vector3d m_ang_frequency {0.0, 0.0, 0.0};
  Eigen::Vector3d m_pos_offset {0.0, 0.0, 0.0};
  Eigen::Vector3d m_ang_offset {0.0, 0.0, 0.0};
  double m_ang_amplitude = 0.1;
  double m_pos_amplitude = 1.0;
  double m_stationary_time {0.1};
};

#endif  // INFRASTRUCTURE__SIM__TRUTH_ENGINE_CYCLIC_HPP_
