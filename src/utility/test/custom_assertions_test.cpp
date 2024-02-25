// Copyright 2024 Jacob Hartzer
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

#include "utility/custom_assertions.hpp"

TEST(test_custom_assertions, expect_near) {
  Eigen::Vector2d vec1{0, 0};
  Eigen::Vector2d vec2{1, 1};

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(vec1, vec1, 1e-6));
  EXPECT_FALSE(EXPECT_EIGEN_NEAR(vec1, vec2, 1e-6));

  Eigen::Vector3d vec3{0, 0, 0};
  Eigen::Vector3d vec4{1, 1, 1};

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(vec3, vec3, 1e-6));
  EXPECT_FALSE(EXPECT_EIGEN_NEAR(vec3, vec4, 1e-6));

  Eigen::Quaterniond quat1{1, 0, 0, 0};
  Eigen::Quaterniond quat2{0.5, 0.5, 0.5, 0.5};

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(quat1, quat1, 1e-6));
  EXPECT_FALSE(EXPECT_EIGEN_NEAR(quat1, quat2, 1e-6));
}
