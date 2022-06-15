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
#include <iostream>
#include <vector>

#include "TypeHelper.hpp"


TEST(test_TypeHelper, StdToEigVec_2) {
  std::vector<double> in {1.0, 2.0};
  Eigen::VectorXd out = TypeHelper::StdToEigVec(in);
  unsigned int size_1 = out.size();
  unsigned int size_2 = in.size();
  EXPECT_EQ(size_1, size_2);
  EXPECT_EQ(out(0), in[0]);
  EXPECT_EQ(out(1), in[1]);
}

TEST(test_TypeHelper, StdToEigVec_3) {
  std::vector<double> in {1.0, 2.0, 3.0};
  Eigen::Vector3d out = TypeHelper::StdToEigVec(in);
  unsigned int size_1 = out.size();
  unsigned int size_2 = in.size();
  EXPECT_EQ(size_1, size_2);
  EXPECT_EQ(out.x(), in[0]);
  EXPECT_EQ(out.y(), in[1]);
  EXPECT_EQ(out.z(), in[2]);
}

TEST(test_TypeHelper, StdToEigQuat) {
  std::vector<double> in {1.0, 2.0, 3.0, 4.0};
  double norm = sqrt(in[0] * in[0] + in[1] * in[1] + in[2] * in[2] + in[3] * in[3]);
  in[0] = in[0] / norm;
  in[1] = in[1] / norm;
  in[2] = in[2] / norm;
  in[3] = in[3] / norm;

  Eigen::Quaterniond out = TypeHelper::StdToEigQuat(in);
  EXPECT_NEAR(out.w(), in[0], 1e-6);
  EXPECT_NEAR(out.x(), in[1], 1e-6);
  EXPECT_NEAR(out.y(), in[2], 1e-6);
  EXPECT_NEAR(out.z(), in[3], 1e-6);
}

TEST(test_TypeHelper, StdToEigQuat_err) {
  std::vector<double> in {1.0, 2.0, 3.0};

  Eigen::Quaterniond out = TypeHelper::StdToEigQuat(in);
  EXPECT_EQ(out.w(), 1.0);
  EXPECT_EQ(out.x(), 0.0);
  EXPECT_EQ(out.y(), 0.0);
  EXPECT_EQ(out.z(), 0.0);
}

TEST(test_TypeHelper, RosHeaderToTime) {
  std_msgs::msg::Header in;
  in.stamp.set__nanosec(987654321);
  in.stamp.set__sec(123456789);
  double out = TypeHelper::RosHeaderToTime(in);
  EXPECT_EQ(out, 123456789.987654321);
}

TEST(test_TypeHelper, RosVecToEigen) {
  geometry_msgs::msg::Vector3 in;
  in.set__x(1.0);
  in.set__y(2.0);
  in.set__z(3.0);
  Eigen::Vector3d out = TypeHelper::RosToEigen(in);
  EXPECT_EQ(out(0), in.x);
  EXPECT_EQ(out(1), in.y);
  EXPECT_EQ(out(2), in.z);
}

TEST(test_TypeHelper, RosMatToEigen) {
  std::array<double, 9> in{1, 2, 3, 4, 5, 6, 7, 8, 9};
  Eigen::Matrix3d out = TypeHelper::RosToEigen(in);
  EXPECT_EQ(out(0, 0), in[0]);
  EXPECT_EQ(out(0, 1), in[1]);
  EXPECT_EQ(out(0, 2), in[2]);
  EXPECT_EQ(out(1, 0), in[3]);
  EXPECT_EQ(out(1, 1), in[4]);
  EXPECT_EQ(out(1, 2), in[5]);
  EXPECT_EQ(out(2, 0), in[6]);
  EXPECT_EQ(out(2, 1), in[7]);
  EXPECT_EQ(out(2, 2), in[8]);

  EXPECT_TRUE(true);
}
