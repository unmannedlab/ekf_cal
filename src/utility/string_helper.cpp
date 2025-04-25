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

#include "utility/string_helper.hpp"

#include <eigen3/Eigen/Eigen>

#include <iomanip>
#include <sstream>
#include <string>

std::string EnumerateHeader(const std::string & name, const unsigned int size)
{
  std::stringstream stream;
  for (unsigned int i = 0; i < size; ++i) {
    stream << "," << name << "_" << std::to_string(i);
  }
  return stream.str();
}

std::string VectorToCommaString(const Eigen::VectorXd & vec)
{
  std::stringstream stream;
  for (unsigned int i = 0; i < vec.size(); ++i) {
    stream << "," << vec[i];
  }
  return stream.str();
}

std::string VectorToCommaString(const Eigen::VectorXd & vec, const int precision)
{
  std::stringstream stream;
  for (unsigned int i = 0; i < vec.size(); ++i) {
    stream << "," << std::setprecision(precision) << vec[i];
  }
  return stream.str();
}

std::string QuaternionToCommaString(const Eigen::Quaterniond & quat)
{
  std::stringstream stream;
  stream << "," << quat.w();
  stream << "," << quat.x();
  stream << "," << quat.y();
  stream << "," << quat.z();
  return stream.str();
}
