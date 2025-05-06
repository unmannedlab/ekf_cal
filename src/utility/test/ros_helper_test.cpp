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

#include <array>
#include <vector>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "utility/ros_helper.hpp"

TEST(test_TypeHelper, RosHeaderToTime) {
  std_msgs::msg::Header ros_header;
  ros_header.stamp.nanosec = 987654321;
  ros_header.stamp.sec = 123456789;
  double out = RosHeaderToTime(ros_header);
  EXPECT_EQ(out, 123456789.987654321);
}

TEST(test_TypeHelper, RosVecToEigen) {
  geometry_msgs::msg::Vector3 ros_vec;
  ros_vec.x = 1.0;
  ros_vec.y = 2.0;
  ros_vec.z = 3.0;
  Eigen::Vector3d eig_vec = RosToEigen(ros_vec);
  EXPECT_EQ(eig_vec(0), ros_vec.x);
  EXPECT_EQ(eig_vec(1), ros_vec.y);
  EXPECT_EQ(eig_vec(2), ros_vec.z);
}

TEST(test_TypeHelper, RosMatToEigen) {
  std::array<double, 9> ros_mat{1, 2, 3, 4, 5, 6, 7, 8, 9};
  Eigen::Matrix3d eig_mat = RosToEigen(ros_mat);
  EXPECT_EQ(eig_mat(0, 0), ros_mat[0]);
  EXPECT_EQ(eig_mat(0, 1), ros_mat[1]);
  EXPECT_EQ(eig_mat(0, 2), ros_mat[2]);
  EXPECT_EQ(eig_mat(1, 0), ros_mat[3]);
  EXPECT_EQ(eig_mat(1, 1), ros_mat[4]);
  EXPECT_EQ(eig_mat(1, 2), ros_mat[5]);
  EXPECT_EQ(eig_mat(2, 0), ros_mat[6]);
  EXPECT_EQ(eig_mat(2, 1), ros_mat[7]);
  EXPECT_EQ(eig_mat(2, 2), ros_mat[8]);
}
