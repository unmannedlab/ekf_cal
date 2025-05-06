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

TEST(test_MathHelper, MinBoundVector)
{
  Eigen::VectorXd vec2 = Eigen::VectorXd::Ones(2);
  MinBoundVector(vec2, 1);
  EXPECT_EQ(vec2, Eigen::VectorXd::Ones(2));

  Eigen::VectorXd vec3 = Eigen::VectorXd::Zero(3);
  MinBoundVector(vec3, 1);
  EXPECT_EQ(vec3, Eigen::VectorXd::Ones(3));

  Eigen::VectorXd vec4 = Eigen::VectorXd::Zero(4);
  MinBoundVector(vec4, 1);
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
  Eigen::MatrixXd H_f(6, 3);
  H_f <<
    1, 0, -1,
    0, 1, -1,
    1, 0, -2,
    0, 1, -3,
    1, 0, -4,
    0, 1, -5;
  Eigen::MatrixXd H_x(6, 5);
  H_x <<
    1, 0, 0, 0, 0,
    0, 1, 0, 0, 0,
    0, 0, 1, 0, 0,
    0, 0, 0, 1, 0,
    0, 0, 0, 0, 1,
    1, 1, 1, 1, 1;
  Eigen::VectorXd res(6);
  res << 1, 2, 3, 4, 5, 6;

  ApplyLeftNullspace(H_f, H_x, res);

  EXPECT_EQ(res.size(), 3);
  EXPECT_EQ(H_x.rows(), 3);
  EXPECT_EQ(H_x.cols(), 5);
}

TEST(test_MathHelper, CompressMeasurements) {
  Eigen::MatrixXd jacobian1(4, 2);
  Eigen::VectorXd residual1(4);
  jacobian1 << 1, 0, 1, 0, 0, 1, 0, 1;
  residual1 << 1, 1, 1, 1;
  CompressMeasurements(jacobian1, residual1);

  EXPECT_EQ(jacobian1.rows(), 2);
  EXPECT_EQ(jacobian1.cols(), 2);
  EXPECT_EQ(residual1.size(), 2);
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

  EXPECT_EQ(jacobian2.rows(), 1);
  EXPECT_EQ(jacobian2.cols(), 2);
  EXPECT_EQ(residual2.size(), 1);
  EXPECT_NEAR(jacobian2(0, 0), 2.0, 1e-6);
  EXPECT_NEAR(jacobian2(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(residual2(0), 2.0, 1e-6);
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

TEST(test_MathHelper, QuaternionJacobian) {
  Eigen::Quaterniond quat{1, 0, 0, 0};
  Eigen::Matrix3d jac = QuaternionJacobian(quat);
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(jac, Eigen::Matrix3d::Identity(3, 3), 1e-6));
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
  double pos_stddev;
  double ang_stddev;

  EXPECT_TRUE(kabsch_2d(points_tgt, points_src, transform, pos_stddev, ang_stddev));

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

TEST(test_MathHelper, InsertInMatrix) {
  Eigen::MatrixXd in_mat(2, 2);
  in_mat << 1, 2, 3, 4;

  Eigen::MatrixXd sub_mat(2, 2);
  sub_mat << 1, 2, 3, 4;

  in_mat = InsertInMatrix(sub_mat, in_mat, 1, 1);

  EXPECT_EQ(in_mat(0, 0), 1);
  EXPECT_EQ(in_mat(0, 3), 2);
  EXPECT_EQ(in_mat(3, 0), 3);
  EXPECT_EQ(in_mat(3, 3), 4);

  EXPECT_EQ(in_mat(1, 1), 1);
  EXPECT_EQ(in_mat(1, 2), 2);
  EXPECT_EQ(in_mat(2, 1), 3);
  EXPECT_EQ(in_mat(2, 2), 4);
}

TEST(test_MathHelper, matrix2d_from_vectors3d) {
  std::vector<Eigen::Vector3d> input_vectors;
  input_vectors.push_back(Eigen::Vector3d(0, 0, 0));
  input_vectors.push_back(Eigen::Vector3d(1, 1, 1));
  input_vectors.push_back(Eigen::Vector3d(2, 2, 2));
  input_vectors.push_back(Eigen::Vector3d(3, 3, 3));

  Eigen::MatrixXd out_mat = matrix2d_from_vectors3d(input_vectors);

  EXPECT_EQ(out_mat.rows(), 2);
  EXPECT_EQ(out_mat.cols(), 4);
  EXPECT_EQ(out_mat(0, 0), 0);
  EXPECT_EQ(out_mat(0, 1), 1);
  EXPECT_EQ(out_mat(0, 2), 2);
  EXPECT_EQ(out_mat(0, 3), 3);
}
