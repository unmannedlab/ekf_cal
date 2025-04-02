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

#ifndef UTILITY__CUSTOM_ASSERTIONS_HPP_
#define UTILITY__CUSTOM_ASSERTIONS_HPP_

#include <eigen3/Eigen/Eigen>
#include <gtest/gtest.h>
#include <math.h>

static testing::AssertionResult EXPECT_EIGEN_NEAR(
  const Eigen::MatrixXd & mat1, const Eigen::MatrixXd & mat2, double precision)
{
  for (unsigned int i = 0; i < mat1.rows(); ++i) {
    for (unsigned int j = 0; j < mat1.cols(); ++j) {
      if (abs(mat1(i, j) - mat2(i, j)) > precision) {
        return ::testing::AssertionFailure() << "mat1[" << i << "," << j <<
               "] (" << mat1(i, j) << ") != mat2[" << i << "," << j <<
               "] (" << mat2(i, j) << ") Diff:" << abs(mat1(i, j) - mat2(i, j));
      }
    }
  }
  return testing::AssertionSuccess();
}

static testing::AssertionResult EXPECT_EIGEN_NEAR(
  const Eigen::Quaterniond & quat1,
  const Eigen::Quaterniond & quat2,
  double precision)
{
  Eigen::Quaterniond delta_quat = quat1.inverse() * quat2;
  double angle = atan2(delta_quat.vec().norm(), delta_quat.w());
  if (angle > precision) {
    return testing::AssertionFailure()
           << "quat1 (" << quat1 << ") != quat2 (" << quat2 << ") Diff:" << angle;
  } else {
    return testing::AssertionSuccess();
  }
}

#endif  // UTILITY__CUSTOM_ASSERTIONS_HPP_
