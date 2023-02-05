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

#ifndef UTILITY__TYPEHELPER_HPP_
#define UTILITY__TYPEHELPER_HPP_

#include <eigen3/Eigen/Eigen>

#include <vector>

#include "utility/Constants.hpp"
#include "infrastructure/Logger.hpp"

///
/// @brief Converts std::vector into Eigen Quaternion
/// @param in Input std::vector
/// @return Output Eigen Vector3
///
inline Eigen::VectorXd stdToEigVec(std::vector<double> const & in)
{
  Eigen::VectorXd out(in.size());
  for (unsigned int i = 0; i < in.size(); ++i) {
    out(i) = in[i];
  }
  return out;
}

///
/// @brief Converts std::vector into Eigen Quaternion
/// @param in Input std::vector
/// @return Output Eigen Quaternion
///
inline Eigen::Quaterniond stdToEigQuat(std::vector<double> const & in)
{
  if (in.size() == 4U) {
    Eigen::Quaterniond quat{in[0U], in[1U], in[2U], in[3U]};
    quat.normalize();
    return quat;
  } else {
    Logger::getInstance()->log(LogLevel::WARN, "Vector incorrect size for Eigen conversion");
    return Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};
  }
}

inline Eigen::Quaterniond rotVecToQuat(Eigen::Vector3d rotVec)
{
  double angle = rotVec.norm();
  if (angle > 1e-9) {
    Eigen::Vector3d axis = rotVec / rotVec.norm();
    Eigen::AngleAxisd angAxis{angle, axis};
    return Eigen::Quaterniond(angAxis);
  } else {
    return Eigen::Quaterniond{1, 0, 0, 0};
  }
}

inline Eigen::Vector3d quatToRotVec(Eigen::Quaterniond quat)
{
  Eigen::AngleAxisd angAxis{quat};
  Eigen::Vector3d rotVec = angAxis.axis() * angAxis.angle();
  return rotVec;
}

#endif  // UTILITY__TYPEHELPER_HPP_
