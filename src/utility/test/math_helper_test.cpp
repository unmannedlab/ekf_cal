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

#include "utility/custom_assertions.hpp"
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

TEST(test_MathHelper, MinBoundMatrix)
{
  Eigen::Matrix3d zeros = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d ones = Eigen::Matrix3d::Ones();
  Eigen::Matrix3d out = MinBoundMatrix(zeros, 1.0);

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(out, ones, 1e-6));
}

TEST(test_MathHelper, MaxBoundMatrix)
{
  Eigen::Matrix3d zeros = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d ones = Eigen::Matrix3d::Ones();
  Eigen::Matrix3d out = MaxBoundMatrix(ones, 0.0);

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(out, zeros, 1e-6));
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

TEST(test_MathHelper, average_quaternions) {
  std::vector<Eigen::Quaterniond> quaternions_1;
  quaternions_1.push_back(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  quaternions_1.push_back(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  quaternions_1.push_back(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  quaternions_1.push_back(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  quaternions_1.push_back(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  std::vector<double> weights_1;
  weights_1.push_back(1.0);
  weights_1.push_back(1.0);
  weights_1.push_back(1.0);
  weights_1.push_back(1.0);
  weights_1.push_back(1.0);

  Eigen::Quaterniond average_quaternion_1 = average_quaternions(quaternions_1, weights_1);
  EXPECT_EQ(average_quaternion_1.w(), 1.0);
  EXPECT_EQ(average_quaternion_1.x(), 0.0);
  EXPECT_EQ(average_quaternion_1.y(), 0.0);
  EXPECT_EQ(average_quaternion_1.z(), 0.0);

  std::vector<Eigen::Quaterniond> quaternions_2;
  quaternions_2.push_back(Eigen::Quaterniond(0.9990482, 0.0436194, 0, 0));
  quaternions_2.push_back(Eigen::Quaterniond(0.9990482, -0.0436194, 0, 0));
  quaternions_2.push_back(Eigen::Quaterniond(0.9990482, 0, 0.0436194, 0));
  quaternions_2.push_back(Eigen::Quaterniond(0.9990482, 0, -0.0436194, 0));
  quaternions_2.push_back(Eigen::Quaterniond(0.9990482, 0, 0, 0.0436194));
  quaternions_2.push_back(Eigen::Quaterniond(0.9990482, 0, 0, -0.0436194));

  std::vector<double> weights_2;
  weights_2.push_back(1.0);
  weights_2.push_back(1.0);
  weights_2.push_back(1.0);
  weights_2.push_back(1.0);
  weights_2.push_back(1.0);
  weights_2.push_back(1.0);

  Eigen::Quaterniond average_quaternion_2 = average_quaternions(quaternions_2, weights_2);
  EXPECT_EQ(average_quaternion_2.w(), 1.0);
  EXPECT_EQ(average_quaternion_2.x(), 0.0);
  EXPECT_EQ(average_quaternion_2.y(), 0.0);
  EXPECT_EQ(average_quaternion_2.z(), 0.0);

  std::vector<Eigen::Quaterniond> quaternions_3;
  quaternions_3.push_back(Eigen::Quaterniond(0.9914449, 0.1305262, 0.0, 0.0));
  quaternions_3.push_back(Eigen::Quaterniond(0.9238795, 0.3826834, 0.0, 0.0));
  quaternions_3.push_back(Eigen::Quaterniond(0.7372773, 0.6755902, 0.0, 0.0));
  quaternions_3.push_back(Eigen::Quaterniond(0.6755902, 0.7372773, 0.0, 0.0));
  quaternions_3.push_back(Eigen::Quaterniond(0.6755902, -0.7372773, 0.0, 0.0));
  quaternions_3.push_back(Eigen::Quaterniond(0.7372773, -0.6755902, 0.0, 0.0));
  quaternions_3.push_back(Eigen::Quaterniond(0.9238795, -0.3826834, 0.0, 0.0));
  quaternions_3.push_back(Eigen::Quaterniond(0.9914449, -0.1305262, 0.0, 0.0));

  std::vector<double> weights_3;
  weights_3.push_back(1.0);
  weights_3.push_back(1.0);
  weights_3.push_back(1.0);
  weights_3.push_back(1.0);
  weights_3.push_back(1.0);
  weights_3.push_back(1.0);
  weights_3.push_back(1.0);
  weights_3.push_back(1.0);

  Eigen::Quaterniond average_quaternion_3 = average_quaternions(quaternions_3, weights_3);
  EXPECT_EQ(average_quaternion_3.w(), 1.0);
  EXPECT_EQ(average_quaternion_3.x(), 0.0);
  EXPECT_EQ(average_quaternion_3.y(), 0.0);
  EXPECT_EQ(average_quaternion_3.z(), 0.0);
}

TEST(test_MathHelper, average_vectors) {
  std::vector<Eigen::Vector3d> vectors_1;
  vectors_1.push_back(Eigen::Vector3d(9.0, 0.0, 0.0));
  vectors_1.push_back(Eigen::Vector3d(0.0, 6.0, 0.0));
  vectors_1.push_back(Eigen::Vector3d(0.0, 0.0, 3.0));

  std::vector<double> weights_1;
  weights_1.push_back(1.0);
  weights_1.push_back(1.0);
  weights_1.push_back(1.0);

  Eigen::Vector3d average_vector_1 = average_vectors(vectors_1, weights_1);
  EXPECT_EQ(average_vector_1[0], 3.0);
  EXPECT_EQ(average_vector_1[1], 2.0);
  EXPECT_EQ(average_vector_1[2], 1.0);

  std::vector<Eigen::Vector3d> vectors_2;
  vectors_2.push_back(Eigen::Vector3d(6.0, 0.0, 0.0));
  vectors_2.push_back(Eigen::Vector3d(0.0, 6.0, 0.0));
  vectors_2.push_back(Eigen::Vector3d(0.0, 0.0, 6.0));

  std::vector<double> weights_2;
  weights_2.push_back(1.0);
  weights_2.push_back(2.0);
  weights_2.push_back(3.0);

  Eigen::Vector3d average_vector_2 = average_vectors(vectors_2, weights_2);
  EXPECT_EQ(average_vector_2[0], 1.0);
  EXPECT_EQ(average_vector_2[1], 2.0);
  EXPECT_EQ(average_vector_2[2], 3.0);
}

TEST(test_MathHelper, quaternion_jacobian) {
  Eigen::Quaterniond quat{1, 0, 0, 0};
  Eigen::Matrix3d jac = quaternion_jacobian(quat);
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(jac, Eigen::Matrix3d::Identity(), 1e-6));
}

TEST(test_MathHelper, quaternion_jacobian_inv) {
  Eigen::Quaterniond quat{1, 0, 0, 0};
  Eigen::Matrix3d jac = quaternion_jacobian_inv(quat);
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(jac, Eigen::Matrix3d::Identity(), 1e-6));
}

TEST(test_MathHelper, kabsch_2d) {
  std::vector<Eigen::Vector3d> points_src;
  points_src.push_back(Eigen::Vector3d{0, 0, 0});
  points_src.push_back(Eigen::Vector3d{1, 1, 1});
  points_src.push_back(Eigen::Vector3d{0, 1, 0});
  points_src.push_back(Eigen::Vector3d{1, 0, 1});
  points_src.push_back(Eigen::Vector3d{1, 0, 0});
  points_src.push_back(Eigen::Vector3d{0, 1, 0});
  points_src.push_back(Eigen::Vector3d{0, 0, 1});

  std::vector<Eigen::Vector3d> points_tgt;
  points_tgt.push_back(Eigen::Vector3d{1.0000, 2.0000, 3.0000});
  points_tgt.push_back(Eigen::Vector3d{0.6340, 3.3660, 4.0000});
  points_tgt.push_back(Eigen::Vector3d{0.1340, 2.5000, 3.0000});
  points_tgt.push_back(Eigen::Vector3d{1.5000, 2.8660, 4.0000});
  points_tgt.push_back(Eigen::Vector3d{1.5000, 2.8660, 3.0000});
  points_tgt.push_back(Eigen::Vector3d{0.1340, 2.5000, 3.0000});
  points_tgt.push_back(Eigen::Vector3d{1.0000, 2.0000, 4.0000});

  Eigen::Affine3d transform;
  std::vector<Eigen::Vector3d> projection_errors;

  EXPECT_TRUE(kabsch_2d(points_tgt, points_src, transform, projection_errors));

  Eigen::Vector3d translation = transform.translation();
  Eigen::Matrix3d rotation = transform.linear();

  EXPECT_NEAR(translation(0), 1.0, 1e-3);
  EXPECT_NEAR(translation(1), 2.0, 1e-3);
  EXPECT_NEAR(translation(2), 3.0, 1e-3);

  EXPECT_NEAR(rotation(0, 0), 0.5, 1e-3);
  EXPECT_NEAR(rotation(1, 1), 0.5, 1e-3);
  EXPECT_NEAR(rotation(0, 1), -0.8660, 1e-3);
  EXPECT_NEAR(rotation(1, 0), 0.8660, 1e-3);
  EXPECT_NEAR(rotation(2, 2), 1.0, 1e-3);

  EXPECT_NEAR(affine_angle(transform), M_PI / 3.0, 1e-3);
}

TEST(test_MathHelper, maximum_distance) {
  std::vector<Eigen::Vector3d> eigen_points;
  eigen_points.push_back(Eigen::Vector3d{0, 0, 0});
  eigen_points.push_back(Eigen::Vector3d{1, 1, 0});
  eigen_points.push_back(Eigen::Vector3d{1, 0, 0});
  eigen_points.push_back(Eigen::Vector3d{1, -1, 0});
  eigen_points.push_back(Eigen::Vector3d{4, 0, 0});
  double max_dist = maximum_distance(eigen_points);

  EXPECT_EQ(max_dist, 4.0);
}
