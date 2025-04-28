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

#include <algorithm>
#include <memory>
#include <vector>

#include "utility/sim/sim_rng.hpp"
#include "utility/type_helper.hpp"

Eigen::Vector3d TruthEngineSpline::GetBodyPosition(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time) || IsSplineInvalid(m_pos_spline)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    return m_pos_spline(relative_time / (m_max_time - m_stationary_time));
  }
}

Eigen::Vector3d TruthEngineSpline::GetBodyVelocity(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time) || IsSplineInvalid(m_pos_spline)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    unsigned int order {1};
    double u = relative_time / (m_max_time - m_stationary_time);
    Eigen::Vector3d velocity;
    velocity =
      m_pos_spline.derivatives(u, order).block<3, 1>(0, order) / (m_max_time - m_stationary_time);
    return velocity;
  }
}

Eigen::Vector3d TruthEngineSpline::GetBodyAcceleration(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time) || IsSplineInvalid(m_pos_spline)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    unsigned int order {2};
    double u = relative_time / (m_max_time - m_stationary_time);
    Eigen::Vector3d acceleration;
    acceleration =
      m_pos_spline.derivatives(u, order).block<3, 1>(0, order) /
      (m_max_time - m_stationary_time) / (m_max_time - m_stationary_time);
    return acceleration;
  }
}

Eigen::Quaterniond TruthEngineSpline::GetBodyAngularPosition(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time) || IsSplineInvalid(m_ang_spline)) {
    return Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};
  } else {
    double u = relative_time / (m_max_time - m_stationary_time);
    Eigen::Vector3d euler_angles = m_ang_spline(u);

    Eigen::Quaterniond angular_position = EigVecToQuat(euler_angles);

    return angular_position;
  }
}

Eigen::Vector3d TruthEngineSpline::GetBodyAngularRate(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time) || IsSplineInvalid(m_ang_spline)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    unsigned int order {1};
    double u = relative_time / (m_max_time - m_stationary_time);
    Eigen::Vector3d velocity;
    velocity =
      m_ang_spline.derivatives(u, order).block<3, 1>(0, order) / (m_max_time - m_stationary_time);
    return velocity;
  }
}

Eigen::Vector3d TruthEngineSpline::GetBodyAngularAcceleration(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time) || IsSplineInvalid(m_ang_spline)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    unsigned int order {2};
    double u = relative_time / (m_max_time - m_stationary_time);
    Eigen::Vector3d acceleration;
    acceleration =
      m_ang_spline.derivatives(u, order).block<3, 1>(0, order) /
      (m_max_time - m_stationary_time) / (m_max_time - m_stationary_time);
    return acceleration;
  }
}

TruthEngineSpline::TruthEngineSpline(
  std::vector<double> poses,
  std::vector<double> angles,
  std::vector<double> position_errors,
  std::vector<double> angle_errors,
  double stationary_time,
  double max_time,
  std::shared_ptr<DebugLogger> logger,
  SimRNG rng
)
: TruthEngine(max_time, logger)
{
  unsigned int spline_size =
    static_cast<unsigned int>( std::floor(
      std::min(static_cast<double>(poses.size()), static_cast<double>(angles.size())) / 3.0));

  Eigen::Vector3d pos_errors = StdToEigVec(position_errors);
  Eigen::Vector3d ang_errors = StdToEigVec(angle_errors);

  Eigen::MatrixXd pos_mat(3, spline_size);
  Eigen::MatrixXd ang_mat(3, spline_size);
  for (unsigned int index = 0; index < spline_size; ++index) {
    Eigen::Vector3d pos {poses[3 * index], poses[3 * index + 1], poses[3 * index + 2]};
    Eigen::Vector3d ang {angles[3 * index], angles[3 * index + 1], angles[3 * index + 2]};
    if (index != 0) {
      pos = rng.VecNormRand(pos, pos_errors);
      ang = rng.VecNormRand(ang, ang_errors);
    }
    pos_mat.col(index) << pos[0], pos[1], pos[2];
    ang_mat.col(index) << ang[0], ang[1], ang[2];
  }

  Eigen::MatrixXd derivatives(3, 1);
  Eigen::VectorXd indices(1);
  derivatives.setZero();
  indices.setZero();

  m_pos_spline = Eigen::SplineFitting<Eigen::Spline3d>::InterpolateWithDerivatives(
    pos_mat, derivatives, indices, 2);
  m_ang_spline = Eigen::SplineFitting<Eigen::Spline3d>::InterpolateWithDerivatives(
    ang_mat, derivatives, indices, 2);
  m_stationary_time = stationary_time;
}

bool TruthEngineSpline::IsTimeInvalid(double time)
{
  if (time < 0.0 || time > m_max_time) {
    return true;
  } else {
    return false;
  }
}

bool TruthEngineSpline::IsSplineInvalid(const Eigen::Spline3d & spline)
{
  return !(*spline.ctrls().data() == *spline.ctrls().data());
}
