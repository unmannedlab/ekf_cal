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
