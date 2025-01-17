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

#include "infrastructure/sim/truth_engine_smoother.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "utility/custom_assertions.hpp"
#include "utility/sim/sim_rng.hpp"
#include "utility/type_helper.hpp"


// TEST(test_TruthEngineSmoother, Constructor) {
//   double stationary_time {0.0};
//   double max_time {1.0};
//   std::vector<double> times{0, 1, 2, 3};
//   std::vector<double> positions;
//   std::vector<double> angles;

//   for (unsigned int i = 0; i < 4 * 3; ++i) {
//     positions.push_back(0.0);
//     angles.push_back(0.0);
//   }

//   auto pos_errs = std::vector<double>{0.0, 0.0, 0.0};
//   auto ang_errs = std::vector<double>{0.0, 0.0, 0.0};

//   auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

//   SimRNG rng;

//   TruthEngineSmoother truth_engine_smoother(
//     times, positions, angles, pos_errs, ang_errs, stationary_time, max_time, 100.0, logger, rng);
// }


// TEST(test_TruthEngineSmoother, Constant_Velocity) {
//   double stationary_time {0.0};
//   double max_time {3.0};
//   std::vector<double> times {0, 1, 2, 3, 4, 5};
//   std::vector<double> positions {0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0};
//   std::vector<double> angles {0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3};

//   auto pos_errs = std::vector<double>{0.0, 0.0, 0.0};
//   auto ang_errs = std::vector<double>{0.0, 0.0, 0.0};

//   auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

//   SimRNG rng;

//   TruthEngineSmoother truth_engine_spline(
//     times, positions, angles, pos_errs, ang_errs, stationary_time, max_time, 100.0, logger, rng);

//   Eigen::Vector3d pos_n = truth_engine_spline.GetBodyPosition(-1.0);
//   Eigen::Vector3d pos_0 = truth_engine_spline.GetBodyPosition(0.0);
//   Eigen::Vector3d pos_1 = truth_engine_spline.GetBodyPosition(1.0);
//   Eigen::Vector3d pos_2 = truth_engine_spline.GetBodyPosition(2.0);
//   Eigen::Vector3d pos_3 = truth_engine_spline.GetBodyPosition(3.0);
//   Eigen::Vector3d pos_4 = truth_engine_spline.GetBodyPosition(4.0);

//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(pos_n, Eigen::Vector3d{0, 0, 0}, 1e-6));
//   EXPECT_NEAR(pos_0[0], positions[0], 1e-6);
//   EXPECT_NEAR(pos_1[0], positions[3], 1e-6);
//   EXPECT_NEAR(pos_2[0], positions[6], 1e-6);
//   EXPECT_NEAR(pos_3[0], positions[9], 1e-6);
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(pos_4, Eigen::Vector3d{0, 0, 0}, 1e-6));

//   Eigen::Vector3d vel_n = truth_engine_spline.GetBodyVelocity(-1.0);
//   Eigen::Vector3d vel_0 = truth_engine_spline.GetBodyVelocity(0.0);
//   Eigen::Vector3d vel_1 = truth_engine_spline.GetBodyVelocity(1.0);
//   Eigen::Vector3d vel_2 = truth_engine_spline.GetBodyVelocity(2.0);
//   Eigen::Vector3d vel_3 = truth_engine_spline.GetBodyVelocity(3.0);
//   Eigen::Vector3d vel_4 = truth_engine_spline.GetBodyVelocity(4.0);

//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(vel_n, Eigen::Vector3d{0, 0, 0}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(vel_0, Eigen::Vector3d{0, 0, 0}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(vel_1, Eigen::Vector3d{1.1724137, 1.1724137, 1.1724137}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(vel_2, Eigen::Vector3d{0.9655172, 0.9655172, 0.9655172}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(vel_3, Eigen::Vector3d{1.0344827, 1.0344827, 1.0344827}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(vel_4, Eigen::Vector3d{0, 0, 0}, 1e-6));

//   Eigen::Vector3d acc_n = truth_engine_spline.GetBodyAcceleration(-1.0);
//   Eigen::Vector3d acc_0 = truth_engine_spline.GetBodyAcceleration(0.0);
//   Eigen::Vector3d acc_1 = truth_engine_spline.GetBodyAcceleration(1.0);
//   Eigen::Vector3d acc_2 = truth_engine_spline.GetBodyAcceleration(2.0);
//   Eigen::Vector3d acc_3 = truth_engine_spline.GetBodyAcceleration(3.0);
//   Eigen::Vector3d acc_4 = truth_engine_spline.GetBodyAcceleration(4.0);

//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(acc_n, Eigen::Vector3d{0, 0, 0}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(acc_0, Eigen::Vector3d{2.8275862, 2.8275862, 2.8275862}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(acc_1, Eigen::Vector3d{-0.4827586, -0.4827586, -0.4827586}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(acc_2, Eigen::Vector3d{0.0689655, 0.0689655, 0.0689655}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(acc_3, Eigen::Vector3d{0.0689655, 0.0689655, 0.0689655}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(acc_4, Eigen::Vector3d{0, 0, 0}, 1e-6));

//   Eigen::Quaterniond ang_n = truth_engine_spline.GetBodyAngularPosition(-1.0);
//   Eigen::Quaterniond ang_0 = truth_engine_spline.GetBodyAngularPosition(0.0);
//   Eigen::Quaterniond ang_1 = truth_engine_spline.GetBodyAngularPosition(1.0);
//   Eigen::Quaterniond ang_2 = truth_engine_spline.GetBodyAngularPosition(2.0);
//   Eigen::Quaterniond ang_3 = truth_engine_spline.GetBodyAngularPosition(3.0);
//   Eigen::Quaterniond ang_4 = truth_engine_spline.GetBodyAngularPosition(4.0);

//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_n, EigVecToQuat(Eigen::Vector3d{0, 0, 0}), 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_0, EigVecToQuat(Eigen::Vector3d{0.0, 0.0, 0.0}), 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_1, EigVecToQuat(Eigen::Vector3d{0.1, 0.1, 0.1}), 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_2, EigVecToQuat(Eigen::Vector3d{0.2, 0.2, 0.2}), 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_3, EigVecToQuat(Eigen::Vector3d{0.3, 0.3, 0.3}), 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_4, EigVecToQuat(Eigen::Vector3d{0, 0, 0}), 1e-6));

//   Eigen::Vector3d ang_vel_n = truth_engine_spline.GetBodyAngularRate(-1.0);
//   Eigen::Vector3d ang_vel_0 = truth_engine_spline.GetBodyAngularRate(0.0);
//   Eigen::Vector3d ang_vel_1 = truth_engine_spline.GetBodyAngularRate(1.0);
//   Eigen::Vector3d ang_vel_2 = truth_engine_spline.GetBodyAngularRate(2.0);
//   Eigen::Vector3d ang_vel_3 = truth_engine_spline.GetBodyAngularRate(3.0);
//   Eigen::Vector3d ang_vel_4 = truth_engine_spline.GetBodyAngularRate(4.0);

//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_vel_n, Eigen::Vector3d{0.0, 0.0, 0.0}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_vel_0, Eigen::Vector3d{0.0, 0.0, 0.0}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_vel_1, Eigen::Vector3d{0.1172413, 0.1172413, 0.1172413}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_vel_2, Eigen::Vector3d{0.0965517, 0.0965517, 0.0965517}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_vel_3, Eigen::Vector3d{0.1034482, 0.1034482, 0.1034482}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_vel_4, Eigen::Vector3d{0.0, 0.0, 0.0}, 1e-6));

//   Eigen::Vector3d alpha_n = truth_engine_spline.GetBodyAngularAcceleration(-1.0);
//   Eigen::Vector3d alpha_0 = truth_engine_spline.GetBodyAngularAcceleration(0.0);
//   Eigen::Vector3d alpha_1 = truth_engine_spline.GetBodyAngularAcceleration(1.0);
//   Eigen::Vector3d alpha_2 = truth_engine_spline.GetBodyAngularAcceleration(2.0);
//   Eigen::Vector3d alpha_3 = truth_engine_spline.GetBodyAngularAcceleration(3.0);
//   Eigen::Vector3d alpha_4 = truth_engine_spline.GetBodyAngularAcceleration(4.0);

//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(alpha_n, Eigen::Vector3d{0, 0, 0}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(alpha_0, Eigen::Vector3d{0.2827586, 0.2827586, 0.2827586}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(alpha_1, Eigen::Vector3d{-0.048276, -0.048276, -0.048276}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(alpha_2, Eigen::Vector3d{0.0068965, 0.0068965, 0.0068965}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(alpha_3, Eigen::Vector3d{0.0068965, 0.0068965, 0.0068965}, 1e-6));
//   EXPECT_TRUE(EXPECT_EIGEN_NEAR(alpha_4, Eigen::Vector3d{0, 0, 0}, 1e-6));
// }

TEST(test_TruthEngineSmoother, Oscillating) {
  double stationary_time {0.0};
  double max_time {6.0};
  std::vector<double> times {0, 1, 2, 3, 4, 5, 6};
  std::vector<double> positions{
    0, 0, 0,
    1, 0, 0,
    1, 1, 1,
    1, 0, 1,
    0, 0, 1,
    0, 0, 0,
    0, 0, 0};
  std::vector<double> angles{
    0.0, 0.0, 0.0,
    0.1, 0.0, 0.0,
    0.1, 0.1, 0.1,
    0.1, 0.0, 0.1,
    0.0, 0.0, 0.1,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0};

  auto pos_errs = std::vector<double>{0.0, 0.0, 0.0};
  auto ang_errs = std::vector<double>{0.0, 0.0, 0.0};

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

  SimRNG rng;

  TruthEngineSmoother truth_engine_spline(
    times, positions, angles, pos_errs, ang_errs, stationary_time, max_time, 10.0, logger, rng);

  for (unsigned int i = 0; i < 70; ++i) {
    double time = static_cast<double>(i) / 10.0;
    auto pos = truth_engine_spline.GetBodyPosition(time);
    std::cout << time << " " << pos.transpose() << std::endl;
  }
}
