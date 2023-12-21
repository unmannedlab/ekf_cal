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

#include "utility/type_helper.hpp"

Eigen::Vector3d TruthEngineSpline::GetBodyPosition(double time)
{
  if (IsTimeInvalid(time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    return m_pos_spline(time / m_time_max);
  }
}

Eigen::Vector3d TruthEngineSpline::GetBodyVelocity(double time)
{
  if (IsTimeInvalid(time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    unsigned int order {1};
    double u = time / m_time_max;
    Eigen::Vector3d velocity;
    velocity = m_pos_spline.derivatives(u, order).block<3, 1>(0, order) / m_time_max;
    return velocity;
  }
}

Eigen::Vector3d TruthEngineSpline::GetBodyAcceleration(double time)
{
  if (IsTimeInvalid(time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    unsigned int order {2};
    double u = time / m_time_max;
    Eigen::Vector3d acceleration;
    acceleration =
      m_pos_spline.derivatives(u, order).block<3, 1>(0, order) / m_time_max / m_time_max;
    return acceleration;
  }
}

Eigen::Quaterniond TruthEngineSpline::GetBodyAngularPosition(double time)
{
  if (IsTimeInvalid(time)) {
    return Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};
  } else {
    double u = time / m_time_max;
    Eigen::Vector3d euler_angles = m_ang_spline(u);

    Eigen::Quaterniond angular_position = EigVecToQuat(euler_angles);

    return angular_position;
  }
}

Eigen::Vector3d TruthEngineSpline::GetBodyAngularRate(double time)
{
  if (IsTimeInvalid(time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    unsigned int order {1};
    double u = time / m_time_max;
    Eigen::Vector3d velocity;
    velocity = m_ang_spline.derivatives(u, order).block<3, 1>(0, order) / m_time_max;
    return velocity;
  }
}

Eigen::Vector3d TruthEngineSpline::GetBodyAngularAcceleration(double time)
{
  if (IsTimeInvalid(time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    unsigned int order {2};
    double u = time / m_time_max;
    Eigen::Vector3d acceleration;
    acceleration =
      m_ang_spline.derivatives(u, order).block<3, 1>(0, order) / m_time_max / m_time_max;
    return acceleration;
  }
}

/// @todo Check sizes before use
TruthEngineSpline::TruthEngineSpline(
  double delta_time,
  std::vector<std::vector<double>> positions,
  std::vector<std::vector<double>> angles)
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
    ang_mat.col(row_index) << ang[0], ang[1], ang[2];
    row_index++;
  }

  Eigen::MatrixXd derivatives(3, 1);
  Eigen::VectorXd indices(1);
  derivatives.setZero();
  indices.setZero();

  m_pos_spline = Eigen::SplineFitting<Eigen::Spline3d>::InterpolateWithDerivatives(
    pos_mat, derivatives, indices, 2);
  m_ang_spline = Eigen::SplineFitting<Eigen::Spline3d>::InterpolateWithDerivatives(
    ang_mat, derivatives, indices, 2);
  m_delta_time = delta_time;
  m_time_max = (positions.size() - 1) * delta_time;
}

bool TruthEngineSpline::IsTimeInvalid(double time)
{
  if (time < 0.0 || time > m_time_max) {
    return true;
  } else {
    return false;
  }
}
