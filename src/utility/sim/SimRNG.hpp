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

#ifndef UTILITY__SIM_SIMRNG_HPP_
#define UTILITY__SIM_SIMRNG_HPP_

#include <random>

class SimRNG
{
public:
  SimRNG() {}

  void SetSeed(double seed);

  double NormRand(double mean, double stdDev);

private:
  static std::mt19937_64 m_generator;
};

#endif  // UTILITY__SIM_SIMRNG_HPP_
