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

#ifndef UTILITY__SIM__SIM_RNG_HPP_
#define UTILITY__SIM__SIM_RNG_HPP_

#include <random>

///
/// @class SimRNG
/// @brief Simulation Random Number Generator
///
class SimRNG
{
public:
  SimRNG() {}

  ///
  /// @brief Function to set seed for random number generator
  /// @param seed Seed to use for random number generation
  ///
  void SetSeed(double seed);

  ///
  /// @brief Function to return normal random number
  /// @param mean Mean of normal distribution
  /// @param std_dev Standard deviation of normal distribution
  ///
  double NormRand(double mean, double std_dev);

  ///
  /// @brief Uniform random number generator
  /// @param min Minimum value for uniform distribution
  /// @param max Maximum value for uniform distribution
  ///
  double UniRand(double min, double max);

private:
  static std::mt19937_64 m_generator;
};

#endif  // UTILITY__SIM__SIM_RNG_HPP_
