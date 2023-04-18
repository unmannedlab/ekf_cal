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

#include <eigen3/Eigen/Eigen>

#include <gtest/gtest.h>

#include "utility/MathHelper.hpp"
#include "utility/test/CustomAssertions.hpp"

TEST(test_MathHelper, SkewSymmetric) {
  Eigen::Vector3d testVec(1.0, 2.0, 3.0);
  Eigen::Matrix3d outMat = skewSymmetric(testVec);
  EXPECT_EQ(outMat(0, 0), 0.0);
  EXPECT_EQ(outMat(0, 1), -testVec(2));
  EXPECT_EQ(outMat(0, 2), testVec(1));
  EXPECT_EQ(outMat(1, 2), -testVec(0));
  EXPECT_EQ(outMat(1, 1), 0.0);
  EXPECT_EQ(outMat(1, 0), testVec(2));
  EXPECT_EQ(outMat(2, 0), -testVec(1));
  EXPECT_EQ(outMat(2, 1), testVec(0));
  EXPECT_EQ(outMat(2, 2), 0.0);
}

TEST(test_MathHelper, minBoundDiagonal)
{
  Eigen::MatrixXd mat2 = Eigen::MatrixXd::Ones(2, 2);
  mat2 = minBoundDiagonal(mat2, 1);
  EXPECT_EQ(mat2, Eigen::MatrixXd::Ones(2, 2));

  Eigen::MatrixXd mat3 = Eigen::MatrixXd::Zero(3, 3);
  mat3 = minBoundDiagonal(mat3, 1);
  EXPECT_EQ(mat3, Eigen::MatrixXd::Identity(3, 3));

  Eigen::MatrixXd mat4 = Eigen::MatrixXd::Zero(4, 4);
  mat4 = minBoundDiagonal(mat4, 1);
  EXPECT_EQ(mat4, Eigen::MatrixXd::Identity(4, 4));
}

TEST(test_MathHelper, minBoundVector)
{
  Eigen::VectorXd vec2 = Eigen::VectorXd::Ones(2);
  vec2 = minBoundVector(vec2, 1);
  EXPECT_EQ(vec2, Eigen::VectorXd::Ones(2));

  Eigen::VectorXd vec3 = Eigen::VectorXd::Zero(3);
  vec3 = minBoundVector(vec3, 1);
  EXPECT_EQ(vec3, Eigen::VectorXd::Ones(3));

  Eigen::VectorXd vec4 = Eigen::VectorXd::Zero(4);
  vec4 = minBoundVector(vec4, 1);
  EXPECT_EQ(vec4, Eigen::VectorXd::Ones(4));
}
