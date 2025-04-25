// Copyright 2024 Jacob Hartzer
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

#include "infrastructure/sim/truth_engine_smoother.hpp"

#include <algorithm>
#include <memory>
#include <vector>

#include "utility/sim/sim_rng.hpp"
#include "utility/type_helper.hpp"


Eigen::Vector3d TruthEngineSmoother::GetInterpolatedValue(
  double time,
  const std::vector<Eigen::Vector3d> & values
) const
{
  double whole, alpha;
  alpha = std::modf(time * m_rate, &whole);
  auto index = static_cast<unsigned int>(whole);
  return values[index] * (1 - alpha) + values[index + 1] * alpha;
}

Eigen::Vector3d TruthEngineSmoother::GetBodyPosition(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time) ) {
    return Eigen::Vector3d::Zero(3);
  } else {
    return GetInterpolatedValue(time, m_pos);
  }
}

Eigen::Vector3d TruthEngineSmoother::GetBodyVelocity(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time) ) {
    return Eigen::Vector3d::Zero(3);
  } else {
    return GetInterpolatedValue(time, m_vel);
  }
}

Eigen::Vector3d TruthEngineSmoother::GetBodyAcceleration(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time) ) {
    return Eigen::Vector3d::Zero(3);
  } else {
    return GetInterpolatedValue(time, m_acc);
  }
}

Eigen::Quaterniond TruthEngineSmoother::GetBodyAngularPosition(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time)) {
    return Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};
  } else {
    Eigen::Vector3d euler_angles = GetInterpolatedValue(time, m_ang);
    Eigen::Quaterniond angular_position = EigVecToQuat(euler_angles);
    return angular_position;
  }
}

Eigen::Vector3d TruthEngineSmoother::GetBodyAngularRate(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    return GetInterpolatedValue(time, m_ang_vel);
  }
}

Eigen::Vector3d TruthEngineSmoother::GetBodyAngularAcceleration(double time)
{
  double relative_time = time - m_stationary_time;
  if (IsTimeInvalid(relative_time)) {
    return Eigen::Vector3d::Zero(3);
  } else {
    return GetInterpolatedValue(time, m_ang_acc);
  }
}


std::vector<Eigen::Vector3d> InterpolateVectors(
  std::vector<double> m_time,
  std::vector<double> times,
  std::vector<Eigen::Vector3d> & vectors
)
{
  std::vector<Eigen::Vector3d> out_vectors;
  unsigned int j {0};
  for (unsigned int i = 0; i < m_time.size(); ++i) {
    Eigen::Vector3d vec {0.0, 0.0, 0.0};
    if ((m_time[i] > times[0]) && (m_time[i] < times.back())) {
      while (j < times.size() && ((m_time[i] < times[j]) || m_time[i] > times[j + 1])) {
        ++j;
      }
      if (j >= times.size()) {
        break;
      }
      double alpha = (times[j + 1] - m_time[i]) / (times[j + 1] - times[j]);
      vec = vectors[j] * alpha + vectors[j + 1] * (1 - alpha);
    }
    out_vectors.push_back(vec);
  }

  return out_vectors;
}


std::vector<Eigen::Vector3d> Derivative(
  std::vector<double> times,
  std::vector<Eigen::Vector3d> & values)
{
  std::vector<Eigen::Vector3d> derivatives;
  for (unsigned int i = 0; i < values.size() - 1; ++i) {
    derivatives.push_back((values[i + 1] - values[i]) / (times[i + 1] - times[i]));
  }
  derivatives.push_back(derivatives.back());
  return derivatives;
}

std::vector<Eigen::Vector3d> SlidingWindowFilter(
  std::vector<Eigen::Vector3d> & data,
  unsigned int buffer_size)
{
  std::vector<Eigen::Vector3d> smooth_vectors;
  unsigned int width = 2 * buffer_size + 1;

  for (unsigned int i = 0; i < buffer_size; ++i) {
    data.insert(data.begin(), Eigen::Vector3d::Zero());
    data.insert(data.end(), Eigen::Vector3d::Zero());
  }

  for (unsigned int i = 0; i < data.size() - 2 * buffer_size; ++i) {
    Eigen::Vector3d mean_vector = Eigen::Vector3d::Zero();
    for (unsigned int j = 0; j < width; ++j) {
      mean_vector += data[i + j];
    }
    smooth_vectors.push_back(mean_vector / width);
  }

  return smooth_vectors;
}

double MaxAcceleration(std::vector<double> times, std::vector<Eigen::Vector3d> & data)
{
  double max_acc {0};

  for (unsigned int i = 1; i < data.size(); ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      double deriv = (data[i + 1][j] - 2 * data[i][j] + data[i - 1][j]) /
        std::pow((times[i + 1] - times[i - 1]) / 2, 2);
      max_acc = std::max(max_acc, deriv);
    }
  }

  return max_acc;
}

TruthEngineSmoother::TruthEngineSmoother(
  std::vector<double> times,
  std::vector<double> poses,
  std::vector<double> angles,
  std::vector<double> position_errors,
  std::vector<double> angle_errors,
  double stationary_time,
  double max_time,
  double rate,
  std::shared_ptr<DebugLogger> logger,
  SimRNG rng
)
: TruthEngine(max_time, logger), m_rate(rate)
{
  unsigned int spline_size =
    static_cast<unsigned int>( std::floor(
      std::min(static_cast<double>(poses.size()), static_cast<double>(angles.size())) / 3.0));

  Eigen::Vector3d pos_errors = StdToEigVec(position_errors);
  Eigen::Vector3d ang_errors = StdToEigVec(angle_errors);

  std::vector<Eigen::Vector3d> position_vectors;
  std::vector<Eigen::Vector3d> angle_vectors;

  for (unsigned int i = 0; i < spline_size; ++i) {
    Eigen::Vector3d pos {poses[3 * i], poses[3 * i + 1], poses[3 * i + 2]};
    Eigen::Vector3d ang {angles[3 * i], angles[3 * i + 1], angles[3 * i + 2]};

    if (i) {
      pos = rng.VecNormRand(pos, pos_errors);
      ang = rng.VecNormRand(ang, ang_errors);
    }

    position_vectors.push_back(pos);
    angle_vectors.push_back(ang);
  }

  auto n_points = static_cast<unsigned int>(std::ceil(m_max_time * m_rate));
  for (unsigned int i = 0; i < n_points; ++i) {
    m_time.push_back(static_cast<double>(i) / m_rate);
  }

  m_pos = InterpolateVectors(m_time, times, position_vectors);
  m_ang = InterpolateVectors(m_time, times, angle_vectors);

  m_vel = Derivative(m_time, m_pos);
  m_acc = Derivative(m_time, m_vel);

  m_ang_vel = Derivative(m_time, m_ang);
  m_ang_acc = Derivative(m_time, m_ang_vel);

  m_stationary_time = stationary_time;
}

bool TruthEngineSmoother::IsTimeInvalid(double time) const
{
  if (time < 0.0 || time > m_max_time) {
    return true;
  } else {
    return false;
  }
}
