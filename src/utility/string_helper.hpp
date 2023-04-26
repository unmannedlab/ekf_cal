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

#ifndef UTILITY__STRING_HELPER_HPP_
#define UTILITY__STRING_HELPER_HPP_

#include <eigen3/Eigen/Eigen>

#include <string>
#include <sstream>


///
/// @brief Creates comma-separated enumerated list
/// @param name Base name for the header
/// @param size Header count
/// @return Comma-separated, enumerated header string
///
inline std::string EnumerateHeader(std::string name, unsigned int size)
{
  std::stringstream stream;
  for (unsigned int i = 0; i < size; ++i) {
    stream << "," << name << "_" << std::to_string(i);
  }
  return stream.str();
}

///
/// @brief Create comma-separated string from vector
/// @param vec Input vector
/// @return Comma-separated string vector
///
inline std::string VectorToCommaString(Eigen::VectorXd vec)
{
  std::stringstream stream;
  for (unsigned int i = 0; i < vec.size(); ++i) {
    stream << "," << vec[i];
  }
  return stream.str();
}

#endif  // UTILITY__STRING_HELPER_HPP_
