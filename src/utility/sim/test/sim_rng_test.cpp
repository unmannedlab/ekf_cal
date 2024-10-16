// Copyright 2023 Jacob Hartzer
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
#include "utility/sim/sim_rng.hpp"


TEST(test_SimRNG, RNG) {
  SimRNG rng;
  rng.SetSeed(123456.789);

  double norm_rand = rng.NormRand(0, 1);
  double uni_rand = rng.UniRand(0, 1);

  EXPECT_NEAR(norm_rand, 0.536670, 1e-6);
  EXPECT_NEAR(uni_rand, 0.876675, 1e-6);

  Eigen::Vector3d vec_mean{0, 0, 0};
  Eigen::Vector3d vec_std_dev{1, 1, 1};
  Eigen::Quaterniond quat_mean{1, 0, 0, 0};
  Eigen::Vector3d quat_std_dev{0.1, 0.1, 0.1};

  Eigen::Vector3d rand_vec = rng.VecNormRand(vec_mean, vec_std_dev);
  Eigen::Quaterniond rand_quat = rng.QuatNormRand(quat_mean, quat_std_dev);

  EXPECT_TRUE(
    EXPECT_EIGEN_NEAR(
      rand_vec, Eigen::Vector3d{-1.021055, -1.901610, 1.722162}, 1e-6));
  EXPECT_TRUE(
    EXPECT_EIGEN_NEAR(
      rand_quat, Eigen::Quaterniond{0.99607, 0.0106369, -0.0584558, -0.0656766}, 1e-6));
}
