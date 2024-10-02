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

#include "utility/sim/sim_rng.hpp"

#include "utility/type_helper.hpp"

std::mt19937_64 SimRNG::m_generator;

void SimRNG::SetSeed(double seed)
{
  m_generator = std::mt19937_64(seed);
}

double SimRNG::NormRand(double mean, double std_dev)
{
  std::normal_distribution<double> normal_distribution(mean, std_dev);
  return normal_distribution(m_generator);
}

double SimRNG::UniRand(double min, double max)
{
  std::uniform_real_distribution<double> uniform_distribution(min, max);
  return uniform_distribution(m_generator);
}

Eigen::Vector3d SimRNG::VecNormRand(Eigen::Vector3d mean, Eigen::Vector3d std_dev)
{
  Eigen::Vector3d out_vec;
  out_vec[0] = NormRand(mean[0], std_dev[0]);
  out_vec[1] = NormRand(mean[1], std_dev[1]);
  out_vec[2] = NormRand(mean[2], std_dev[2]);
  return out_vec;
}

Eigen::Quaterniond SimRNG::QuatNormRand(Eigen::Quaterniond mean, Eigen::Vector3d std_dev)
{
  Eigen::Quaterniond out_quat;
  Eigen::Vector3d ang_error_rpy;
  ang_error_rpy(0) = NormRand(0.0, std_dev[0]);
  ang_error_rpy(1) = NormRand(0.0, std_dev[1]);
  ang_error_rpy(2) = NormRand(0.0, std_dev[2]);
  out_quat = RotVecToQuat(ang_error_rpy) * mean;
  return out_quat;
}
