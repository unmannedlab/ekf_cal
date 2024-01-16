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

#include <cmath>

#include "utility/type_helper.hpp"


TruthEngineCyclic::TruthEngineCyclic(
  Eigen::Vector3d pos_frequency,
  Eigen::Vector3d ang_frequency,
  Eigen::Vector3d pos_offset,
  Eigen::Vector3d ang_offset,
  double pos_amplitude,
  double ang_amplitude,
  double stationary_time)
{
  m_pos_frequency = pos_frequency;
  m_ang_frequency = ang_frequency;
  m_pos_offset = pos_offset;
  m_ang_offset = ang_offset;
  m_ang_amplitude = ang_amplitude;
  m_pos_amplitude = pos_amplitude;
  m_stationary_time = stationary_time;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyPosition(double time)
{
  double relative_time = time - m_stationary_time;
  if (relative_time < 0.0) {
    return Eigen::Vector3d::Zero(3);
  } else {
    Eigen::Vector3d position;
    position[0] = m_pos_amplitude / 2.0 *
      (1 - std::cos(m_pos_frequency[0] * 2 * M_PI * relative_time));
    position[1] = m_pos_amplitude / 2.0 *
      (1 - std::cos(m_pos_frequency[1] * 2 * M_PI * relative_time));
    position[2] = m_pos_amplitude / 2.0 *
      (1 - std::cos(m_pos_frequency[2] * 2 * M_PI * relative_time));
    position += m_pos_offset;
    return position;
  }
}

Eigen::Vector3d TruthEngineCyclic::GetBodyVelocity(double time)
{
  double relative_time = time - m_stationary_time;
  if (relative_time < 0.0) {
    return Eigen::Vector3d::Zero(3);
  } else {
    Eigen::Vector3d velocity;
    velocity[0] = m_pos_amplitude * m_pos_frequency[0] *
      M_PI * std::sin(m_pos_frequency[0] * 2 * M_PI * relative_time);
    velocity[1] = m_pos_amplitude * m_pos_frequency[1] *
      M_PI * std::sin(m_pos_frequency[1] * 2 * M_PI * relative_time);
    velocity[2] = m_pos_amplitude * m_pos_frequency[2] *
      M_PI * std::sin(m_pos_frequency[2] * 2 * M_PI * relative_time);
    return velocity;
  }
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAcceleration(double time)
{
  double relative_time = time - m_stationary_time;
  if (relative_time < 0.0) {
    return Eigen::Vector3d::Zero(3);
  } else {
    Eigen::Vector3d acceleration;
    acceleration[0] = m_pos_amplitude * 2 * M_PI * M_PI * m_pos_frequency[0] *
      m_pos_frequency[0] * std::cos(m_pos_frequency[0] * 2 * M_PI * relative_time);
    acceleration[1] = m_pos_amplitude * 2 * M_PI * M_PI * m_pos_frequency[1] *
      m_pos_frequency[1] * std::cos(m_pos_frequency[1] * 2 * M_PI * relative_time);
    acceleration[2] = m_pos_amplitude * 2 * M_PI * M_PI * m_pos_frequency[2] *
      m_pos_frequency[2] * std::cos(m_pos_frequency[2] * 2 * M_PI * relative_time);
    return acceleration;
  }
}

Eigen::Vector3d TruthEngineCyclic::GetBodyRollPitchYaw(double time)
{
  double relative_time = time - m_stationary_time;
  if (relative_time < 0.0) {
    return Eigen::Vector3d::Zero(3);
  } else {
    Eigen::Vector3d rpy_vector;
    rpy_vector[0] = m_ang_amplitude / 2.0 *
      (1 - std::cos(m_ang_frequency[0] * 2 * M_PI * relative_time));
    rpy_vector[1] = m_ang_amplitude / 2.0 *
      (1 - std::cos(m_ang_frequency[1] * 2 * M_PI * relative_time));
    rpy_vector[2] = m_ang_amplitude / 2.0 *
      (1 - std::cos(m_ang_frequency[2] * 2 * M_PI * relative_time));
    rpy_vector += m_ang_offset;
    return rpy_vector;
  }
}

Eigen::Quaterniond TruthEngineCyclic::GetBodyAngularPosition(double time)
{
  Eigen::Vector3d rpy_vector = GetBodyRollPitchYaw(time);

  Eigen::Quaterniond angular_position = EigVecToQuat(rpy_vector);

  return angular_position;
}

Eigen::Matrix3d TruthEngineCyclic::EulerDerivativeTransform(Eigen::Vector3d rpy_vector)
{
  Eigen::Matrix3d transform_matrix;
  transform_matrix.setZero();
  double roll = rpy_vector[0];
  double pitch = rpy_vector[1];
  transform_matrix(0, 0) = 1.0;
  transform_matrix(0, 2) = -std::sin(pitch);
  transform_matrix(1, 1) = std::cos(roll);
  transform_matrix(1, 2) = std::sin(roll) * std::cos(pitch);
  transform_matrix(2, 1) = -std::sin(roll);
  transform_matrix(2, 2) = std::cos(roll) * std::cos(pitch);
  return transform_matrix;
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAngularRate(double time)
{
  double relative_time = time - m_stationary_time;
  if (relative_time < 0.0) {
    return Eigen::Vector3d::Zero(3);
  } else {
    Eigen::Vector3d d_rpy;
    d_rpy[0] = m_ang_amplitude * m_ang_frequency[0] * M_PI *
      std::sin(m_ang_frequency[0] * 2 * M_PI * relative_time);
    d_rpy[1] = m_ang_amplitude * m_ang_frequency[1] * M_PI *
      std::sin(m_ang_frequency[1] * 2 * M_PI * relative_time);
    d_rpy[2] = m_ang_amplitude * m_ang_frequency[2] * M_PI *
      std::sin(m_ang_frequency[2] * 2 * M_PI * relative_time);

    Eigen::Vector3d rpy = GetBodyRollPitchYaw(time);
    Eigen::Matrix3d transform = EulerDerivativeTransform(rpy);
    Eigen::Vector3d angular_rate = transform * d_rpy;
    return angular_rate;
  }
}

Eigen::Vector3d TruthEngineCyclic::GetBodyAngularAcceleration(double time)
{
  double relative_time = time - m_stationary_time;
  if (relative_time < 0.0) {
    return Eigen::Vector3d::Zero(3);
  } else {
    Eigen::Vector3d dd_rpy;
    dd_rpy[0] = m_ang_amplitude * 2 * M_PI * M_PI * m_ang_frequency[0] *
      m_ang_frequency[0] * std::cos(m_ang_frequency[0] * 2 * M_PI * relative_time);
    dd_rpy[1] = m_ang_amplitude * 2 * M_PI * M_PI * m_ang_frequency[1] *
      m_ang_frequency[1] * std::cos(m_ang_frequency[1] * 2 * M_PI * relative_time);
    dd_rpy[2] = m_ang_amplitude * 2 * M_PI * M_PI * m_ang_frequency[2] *
      m_ang_frequency[2] * std::cos(m_ang_frequency[2] * 2 * M_PI * relative_time);

    Eigen::Vector3d rpy = GetBodyRollPitchYaw(time);
    Eigen::Matrix3d transform = EulerDerivativeTransform(rpy);
    Eigen::Vector3d angularAcceleration = transform * dd_rpy;
    return angularAcceleration;
  }
}

void TruthEngineCyclic::SetBodyPosCycleFrequency(Eigen::Vector3d frequency)
{
  m_pos_frequency = frequency;
}

void TruthEngineCyclic::SetBodyAngCycleFrequency(Eigen::Vector3d frequency)
{
  m_ang_frequency = frequency;
}
