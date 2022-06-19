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

#ifndef CUSTOMASSERTIONS_HPP_
#define CUSTOMASSERTIONS_HPP_

#include <eigen3/Eigen/Eigen>
#include <gtest/gtest.h>

namespace CustomAssertions
{
///
/// @brief Function to expect near for two Eigen matrices
/// @todo Should be a macro or otherwise show where the error occurred
///
static void EXPECT_EIGEN_NEAR(Eigen::MatrixXd mat1, Eigen::MatrixXd mat2, double precision)
{
  ASSERT_EQ(mat1.rows(), mat2.rows());
  ASSERT_EQ(mat1.cols(), mat2.cols());

  for (int i = 0; i < mat1.rows(); ++i) {
    for (int j = 0; j < mat1.cols(); ++j) {
      EXPECT_NEAR(mat1(i, j), mat2(i, j), precision) << "(" << i << "," << j << ")";
    }
  }
}

}  // namespace CustomAssertions

#endif  // CUSTOMASSERTIONS_HPP_
