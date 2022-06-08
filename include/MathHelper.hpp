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

#ifndef MATHHELPER_HPP_
#define MATHHELPER_HPP_

#include <eigen3/Eigen/Eigen>

namespace MathHelper
{
///
/// @brief Produces a cross product matrix
/// @param inMat Input vector with which to find the left hand size cross
/// product matrix
/// @return Cross product matrix
///
static Eigen::Matrix<double, 3U, 3U> CrossMatrix(Eigen::Vector3d const & inMat)
{
  Eigen::Matrix<double, 3U, 3U> outMat = Eigen::Matrix<double, 3U, 3U>::Zero();

  outMat(0U, 1U) = -inMat[2U];
  outMat(0U, 2U) = inMat[1U];
  outMat(1U, 2U) = -inMat[0U];
  outMat(1U, 0U) = inMat[2U];
  outMat(2U, 0U) = -inMat[1U];
  outMat(2U, 1U) = inMat[0U];

  return outMat;
}

}  // namespace MathHelper

#endif  // MATHHELPER_HPP_
