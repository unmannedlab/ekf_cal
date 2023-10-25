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

#include <gtest/gtest.h>

#include <eigen3/Eigen/Eigen>
#include <memory>

#include "utility/math_helper.hpp"

TEST(test_MathHelper, SkewSymmetric) {
  Eigen::Vector3d testVec(1.0, 2.0, 3.0);
  Eigen::Matrix3d outMat = SkewSymmetric(testVec);
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

TEST(test_MathHelper, MinBoundDiagonal)
{
  Eigen::MatrixXd mat2 = Eigen::MatrixXd::Ones(2, 2);
  mat2 = MinBoundDiagonal(mat2, 1);
  EXPECT_EQ(mat2, Eigen::MatrixXd::Ones(2, 2));

  Eigen::MatrixXd mat3 = Eigen::MatrixXd::Zero(3, 3);
  mat3 = MinBoundDiagonal(mat3, 1);
  EXPECT_EQ(mat3, Eigen::MatrixXd::Identity(3, 3));

  Eigen::MatrixXd mat4 = Eigen::MatrixXd::Zero(4, 4);
  mat4 = MinBoundDiagonal(mat4, 1);
  EXPECT_EQ(mat4, Eigen::MatrixXd::Identity(4, 4));
}

TEST(test_MathHelper, MinBoundVector)
{
  Eigen::VectorXd vec2 = Eigen::VectorXd::Ones(2);
  vec2 = MinBoundVector(vec2, 1);
  EXPECT_EQ(vec2, Eigen::VectorXd::Ones(2));

  Eigen::VectorXd vec3 = Eigen::VectorXd::Zero(3);
  vec3 = MinBoundVector(vec3, 1);
  EXPECT_EQ(vec3, Eigen::VectorXd::Ones(3));

  Eigen::VectorXd vec4 = Eigen::VectorXd::Zero(4);
  vec4 = MinBoundVector(vec4, 1);
  EXPECT_EQ(vec4, Eigen::VectorXd::Ones(4));
}

TEST(test_MathHelper, RemoveFromMatrix)
{
  // Remove middle
  Eigen::MatrixXd matrix_in(4, 4);
  Eigen::MatrixXd matrix_out(4, 4);

  matrix_in << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  matrix_out = RemoveFromMatrix(matrix_in, 1, 1, 2);
  EXPECT_EQ(matrix_out(0, 0), 1);
  EXPECT_EQ(matrix_out(0, 1), 4);
  EXPECT_EQ(matrix_out(1, 0), 13);
  EXPECT_EQ(matrix_out(1, 1), 16);

  // Remove top-left
  matrix_in << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  matrix_out = RemoveFromMatrix(matrix_in, 0, 0, 2);
  EXPECT_EQ(matrix_out(0, 0), 11);
  EXPECT_EQ(matrix_out(0, 1), 12);
  EXPECT_EQ(matrix_out(1, 0), 15);
  EXPECT_EQ(matrix_out(1, 1), 16);

  // Remove top-right
  matrix_in << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  matrix_out = RemoveFromMatrix(matrix_in, 0, 2, 2);
  EXPECT_EQ(matrix_out(0, 0), 9);
  EXPECT_EQ(matrix_out(0, 1), 10);
  EXPECT_EQ(matrix_out(1, 0), 13);
  EXPECT_EQ(matrix_out(1, 1), 14);

  // Remove bottom-left
  matrix_in << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  matrix_out = RemoveFromMatrix(matrix_in, 2, 0, 2);
  EXPECT_EQ(matrix_out(0, 0), 3);
  EXPECT_EQ(matrix_out(0, 1), 4);
  EXPECT_EQ(matrix_out(1, 0), 7);
  EXPECT_EQ(matrix_out(1, 1), 8);

  // Remove bottom-right
  matrix_in << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
  matrix_out = RemoveFromMatrix(matrix_in, 2, 2, 2);
  EXPECT_EQ(matrix_out(0, 0), 1);
  EXPECT_EQ(matrix_out(0, 1), 2);
  EXPECT_EQ(matrix_out(1, 0), 5);
  EXPECT_EQ(matrix_out(1, 1), 6);
}

TEST(test_MathHelper, ApplyLeftNullspace) {
  Eigen::MatrixXd H_f(2, 2);
  H_f << 1, 0, 0, 1;
  Eigen::MatrixXd H_x(5, 5);
  H_x <<
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1;
  Eigen::VectorXd res(5);
  res << 1, 2, 3, 4, 5;
  ApplyLeftNullspace(H_f, H_x, res);

  EXPECT_EQ(H_x.rows(), 3U);
  EXPECT_EQ(H_x.cols(), 5U);
  EXPECT_EQ(res.size(), 3U);
  EXPECT_EQ(res(0), 3);
  EXPECT_EQ(res(1), 4);
  EXPECT_EQ(res(2), 5);
}

TEST(test_MathHelper, CompressMeasurements) {
  Eigen::MatrixXd jacobian1(4, 2);
  Eigen::VectorXd residual1(4);
  jacobian1 << 1, 0, 1, 0, 0, 1, 0, 1;
  residual1 << 1, 1, 1, 1;
  CompressMeasurements(jacobian1, residual1);

  EXPECT_EQ(jacobian1.rows(), 2U);
  EXPECT_EQ(jacobian1.cols(), 2U);
  EXPECT_EQ(residual1.size(), 2U);
  EXPECT_EQ(jacobian1(0, 1), 0.0);
  EXPECT_EQ(jacobian1(1, 0), 0.0);
  EXPECT_NEAR(jacobian1(0, 0), 1.414214, 1e-6);
  EXPECT_NEAR(jacobian1(1, 1), 1.414214, 1e-6);
  EXPECT_NEAR(residual1(0), 1.414214, 1e-6);
  EXPECT_NEAR(residual1(1), 1.414214, 1e-6);

  Eigen::MatrixXd jacobian2(4, 2);
  Eigen::VectorXd residual2(4);
  jacobian2 << 1, 0, 1, 0, 1, 0, 1, 0;
  residual2 << 1, 1, 1, 1;
  CompressMeasurements(jacobian2, residual2);

  EXPECT_EQ(jacobian2.rows(), 1U);
  EXPECT_EQ(jacobian2.cols(), 2U);
  EXPECT_EQ(residual2.size(), 1U);
  EXPECT_NEAR(jacobian2(0, 0), 2.0, 1e-6);
  EXPECT_NEAR(jacobian2(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(residual2(0), 2.0, 1e-6);
}
