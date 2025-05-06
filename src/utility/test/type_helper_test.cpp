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
#include <math.h>

#include <vector>

#include <opencv2/opencv.hpp>

#include "utility/type_helper.hpp"


TEST(test_TypeHelper, StdToEigVec) {
  std::vector<double> vec2 {1.0, 2.0};
  Eigen::VectorXd out2 = StdToEigVec(vec2);
  EXPECT_EQ(vec2.size(), static_cast<unsigned int>(out2.size()));
  EXPECT_EQ(vec2[0], out2(0));
  EXPECT_EQ(vec2[1], out2(1));

  std::vector<double> vec3 {1.0, 2.0, 3.0};
  Eigen::Vector3d out3 = StdToEigVec(vec3);
  EXPECT_EQ(vec3.size(), static_cast<unsigned int>(out3.size()));
  EXPECT_EQ(vec3[0], out3.x());
  EXPECT_EQ(vec3[1], out3.y());
  EXPECT_EQ(vec3[2], out3.z());
}

TEST(test_TypeHelper, StdToEigQuat) {
  std::vector<double> std_vec {1.0, 2.0, 3.0, 4.0};
  double norm = sqrt(
    std_vec[0] * std_vec[0] + std_vec[1] * std_vec[1] + std_vec[2] * std_vec[2] +
    std_vec[3] * std_vec[3]);
  std_vec[0] = std_vec[0] / norm;
  std_vec[1] = std_vec[1] / norm;
  std_vec[2] = std_vec[2] / norm;
  std_vec[3] = std_vec[3] / norm;

  Eigen::Quaterniond eig_vec = StdToEigQuat(std_vec);
  EXPECT_NEAR(eig_vec.w(), std_vec[0], 1e-6);
  EXPECT_NEAR(eig_vec.x(), std_vec[1], 1e-6);
  EXPECT_NEAR(eig_vec.y(), std_vec[2], 1e-6);
  EXPECT_NEAR(eig_vec.z(), std_vec[3], 1e-6);
}

/// @todo replace with round numbers
TEST(test_TypeHelper, RotVecToQuat) {
  Eigen::Vector3d vec0 {0.0, 0.0, 0.0};
  Eigen::Quaterniond quat0 = RotVecToQuat(vec0);
  EXPECT_NEAR(quat0.w(), 1.0000000, 1e-6);
  EXPECT_NEAR(quat0.x(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat0.y(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat0.z(), 0.0000000, 1e-6);

  Eigen::Vector3d vec1 {1.0, 0.0, 0.0};
  Eigen::Quaterniond quat1 = RotVecToQuat(vec1);
  EXPECT_NEAR(quat1.w(), 0.8775826, 1e-6);
  EXPECT_NEAR(quat1.x(), 0.4794255, 1e-6);
  EXPECT_NEAR(quat1.y(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat1.z(), 0.0000000, 1e-6);

  Eigen::Vector3d vec2 {0.0, 1.0, 0.0};
  Eigen::Quaterniond quat2 = RotVecToQuat(vec2);
  EXPECT_NEAR(quat2.w(), 0.8775826, 1e-6);
  EXPECT_NEAR(quat2.x(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat2.y(), 0.4794255, 1e-6);
  EXPECT_NEAR(quat2.z(), 0.0000000, 1e-6);

  Eigen::Vector3d vec3 {0.0, 0.0, 1.0};
  Eigen::Quaterniond quat3 = RotVecToQuat(vec3);
  EXPECT_NEAR(quat3.w(), 0.8775826, 1e-6);
  EXPECT_NEAR(quat3.x(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat3.y(), 0.0000000, 1e-6);
  EXPECT_NEAR(quat3.z(), 0.4794255, 1e-6);

  Eigen::Vector3d vec4 {1.0, 2.0, 3.0};
  Eigen::Quaterniond quat4 = RotVecToQuat(vec4);
  EXPECT_NEAR(quat4.w(), -0.2955511, 1e-6);
  EXPECT_NEAR(quat4.x(), 0.2553219, 1e-6);
  EXPECT_NEAR(quat4.y(), 0.5106437, 1e-6);
  EXPECT_NEAR(quat4.z(), 0.7659656, 1e-6);

  Eigen::Vector3d vec5 {-4.0, 5.0, -6.0};
  Eigen::Quaterniond quat5 = RotVecToQuat(vec5);
  EXPECT_NEAR(quat5.w(), -0.3192205, 1e-6);
  EXPECT_NEAR(quat5.x(), 0.4319929, 1e-6);
  EXPECT_NEAR(quat5.y(), -0.5399911, 1e-6);
  EXPECT_NEAR(quat5.z(), 0.6479893, 1e-6);
}

TEST(test_TypeHelper, EigenMatrixToCv) {
  Eigen::Matrix3d matrix_eigen;
  matrix_eigen << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  cv::Mat matrix_cv(3, 3, cv::DataType<double>::type);
  EigenMatrixToCv(matrix_eigen, matrix_cv);
  EXPECT_EQ(matrix_cv.at<double>(0, 0), matrix_eigen(0, 0));
  EXPECT_EQ(matrix_cv.at<double>(0, 1), matrix_eigen(0, 1));
  EXPECT_EQ(matrix_cv.at<double>(0, 2), matrix_eigen(0, 2));
  EXPECT_EQ(matrix_cv.at<double>(1, 0), matrix_eigen(1, 0));
  EXPECT_EQ(matrix_cv.at<double>(1, 1), matrix_eigen(1, 1));
  EXPECT_EQ(matrix_cv.at<double>(1, 2), matrix_eigen(1, 2));
  EXPECT_EQ(matrix_cv.at<double>(2, 0), matrix_eigen(2, 0));
  EXPECT_EQ(matrix_cv.at<double>(2, 1), matrix_eigen(2, 1));
  EXPECT_EQ(matrix_cv.at<double>(2, 2), matrix_eigen(2, 2));
}

TEST(test_TypeHelper, CvMatrixToEigen) {
  cv::Mat matrix_cv = (cv::Mat_<double>(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);
  Eigen::Matrix3d matrix_eigen;
  CvMatrixToEigen(matrix_cv, matrix_eigen);
  EXPECT_EQ(matrix_eigen(0, 0), matrix_cv.at<double>(0, 0));
  EXPECT_EQ(matrix_eigen(0, 1), matrix_cv.at<double>(0, 1));
  EXPECT_EQ(matrix_eigen(0, 2), matrix_cv.at<double>(0, 2));
  EXPECT_EQ(matrix_eigen(1, 0), matrix_cv.at<double>(1, 0));
  EXPECT_EQ(matrix_eigen(1, 1), matrix_cv.at<double>(1, 1));
  EXPECT_EQ(matrix_eigen(1, 2), matrix_cv.at<double>(1, 2));
  EXPECT_EQ(matrix_eigen(2, 0), matrix_cv.at<double>(2, 0));
  EXPECT_EQ(matrix_eigen(2, 1), matrix_cv.at<double>(2, 1));
  EXPECT_EQ(matrix_eigen(2, 2), matrix_cv.at<double>(2, 2));
}

TEST(test_TypeHelper, QuatToRodrigues) {
  Eigen::Quaterniond quat{1, 0, 0, 0};
  cv::Vec3d vec = QuatToRodrigues(quat);
  EXPECT_EQ(vec(0), 0);
  EXPECT_EQ(vec(1), 0);
  EXPECT_EQ(vec(2), 0);
}

TEST(test_TypeHelper, RodriguesToQuat) {
  cv::Vec3d rodrigues_vector(0, 0, 0);
  Eigen::Quaterniond quat = RodriguesToQuat(rodrigues_vector);
  EXPECT_EQ(quat.w(), 1);
  EXPECT_EQ(quat.x(), 0);
  EXPECT_EQ(quat.y(), 0);
  EXPECT_EQ(quat.z(), 0);
}

TEST(test_TypeHelper, CvVectorToEigen) {
  cv::Vec3d vector_cv(1, 2, 3);
  Eigen::Vector3d vector_eigen = CvVectorToEigen(vector_cv);
  EXPECT_EQ(vector_eigen(0), vector_cv(0));
  EXPECT_EQ(vector_eigen(1), vector_cv(1));
  EXPECT_EQ(vector_eigen(2), vector_cv(2));
}
