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

#include "utility/sim/SimRNG.hpp"

std::mt19937_64 SimRNG::m_generator;

void SimRNG::SetSeed(double seed)
{
  m_generator = std::mt19937_64(seed);
}

double SimRNG::NormRand(double mean, double stdDev)
{
  std::normal_distribution<double> normDist(mean, stdDev);
  return normDist(m_generator);
}
