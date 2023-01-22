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

#ifndef UTILITY__MATHHELPER_HPP_
#define UTILITY__MATHHELPER_HPP_

#include <eigen3/Eigen/Eigen>

namespace MathHelper
{
///
/// @brief Produces a cross product matrix
/// @param inVec Input vector with which to find the left hand size cross
/// product matrix
/// @return Cross product matrix
///
inline Eigen::Matrix3d skewSymmetric(Eigen::Vector3d inVec)
{
  Eigen::Matrix3d outMat = Eigen::Matrix3d::Zero();

  outMat(0U, 1U) = -inVec(2U);
  outMat(0U, 2U) = inVec(1U);
  outMat(1U, 2U) = -inVec(0U);
  outMat(1U, 0U) = inVec(2U);
  outMat(2U, 0U) = -inVec(1U);
  outMat(2U, 1U) = inVec(0U);

  return outMat;
}

///
/// @brief Bound matrix diagonal by a minimum value
/// @param inMat Input matrix to be bound
/// @param minBound Bounding value
/// @return
///
inline Eigen::Matrix3d minBoundDiagonal(Eigen::Matrix3d inMat, double minBound)
{
  Eigen::Matrix3d outMat = inMat;
  for (unsigned int i = 0; i < 3; ++i) {
    if (outMat(i, i) < minBound) {
      outMat(i, i) = minBound;
    }
  }

  return outMat;
}

///
/// @brief Bound vector by a minimum value
/// @param inVec Input vector to be bound
/// @param minBound Bounding value
/// @return
///
inline Eigen::VectorXd minBoundVector(Eigen::VectorXd inVec, double minBound)
{
  Eigen::VectorXd outVec = inVec;
  for (unsigned int i = 0; i < outVec.size(); ++i) {
    if (outVec(i) < minBound) {
      outVec(i) = minBound;
    }
  }

  return outVec;
}

}  // namespace MathHelper

#endif  // UTILITY__MATHHELPER_HPP_
