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

#include "infrastructure/sim/truth_engine_spline.hpp"

#include <eigen3/unsupported/Eigen/Splines>

#include <vector>

Eigen::Vector3d TruthEngineSpline::GetBodyPosition(double time)
{
  return m_pos_spline(time);
}

Eigen::Vector3d TruthEngineSpline::GetBodyVelocity(double time)
{
  if (IsTimeInvalid(time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    double u = time / m_time_max;
    Eigen::Vector3d velocity = m_pos_spline.derivatives(u, 1) / m_time_max;
    return velocity;
  }
}

Eigen::Vector3d TruthEngineSpline::GetBodyAcceleration(double time)
{
  if (IsTimeInvalid(time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    double u = time / m_time_max;
    Eigen::Vector3d acceleration = m_pos_spline.derivatives(u, 2) / m_time_max;
    return acceleration;
  }
}

Eigen::Quaterniond TruthEngineSpline::GetBodyAngularPosition(double time)
{
  Eigen::Vector3d eulAng = m_ang_spline(time);

  Eigen::Quaterniond angularPosition =
    Eigen::AngleAxisd(eulAng(0), Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(eulAng(1), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(eulAng(2), Eigen::Vector3d::UnitZ());

  return angularPosition;
}

Eigen::Vector3d TruthEngineSpline::GetBodyAngularRate(double time)
{
  if (IsTimeInvalid(time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    double u = time / m_time_max;
    Eigen::Vector3d velocity = m_ang_spline.derivatives(u, 1) / m_time_max;
    return velocity;
  }
}

Eigen::Vector3d TruthEngineSpline::GetBodyAngularAcceleration(double time)
{
  if (IsTimeInvalid(time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    double u = time / m_time_max;
    Eigen::Vector3d acceleration = m_ang_spline.derivatives(u, 2) / m_time_max;
    return acceleration;
  }
}

TruthEngineSpline::TruthEngineSpline(
  std::vector<Eigen::Vector3d> positions,
  std::vector<Eigen::Vector3d> angles)
{
  Eigen::MatrixXd pos_mat(3, positions.size());
  int row_index;
  row_index = 0;
  for (auto const & pos : positions) {
    pos_mat.col(row_index) << pos[0], pos[1], pos[2];
    row_index++;
  }

  Eigen::MatrixXd ang_mat(3, angles.size());
  row_index = 0;
  for (auto const & ang : angles) {
    pos_mat.col(row_index) << ang[0], ang[1], ang[2];
    row_index++;
  }

  m_pos_spline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(pos_mat, 2);
  m_ang_spline = Eigen::SplineFitting<Eigen::Spline3d>::Interpolate(ang_mat, 2);
  m_time_max = static_cast<double>(positions.size());
}

bool TruthEngineSpline::IsTimeInvalid(double time)
{
  if (time < 0.0 || time > m_time_max) {
    return false;
  } else {
    return true;
  }
}
