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

#include <vector>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"


TEST(test_EKF, get_counts) {
  auto debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(debug_logger, 10.0, false, "");
  EXPECT_EQ(ekf->GetImuCount(), 0U);
  EXPECT_EQ(ekf->GetCamCount(), 0U);

  ImuState imu_state;
  imu_state.is_intrinsic = true;
  imu_state.is_extrinsic = false;
  Eigen::MatrixXd imu_covariance(12, 12);
  ekf->RegisterIMU(0, imu_state, imu_covariance);

  AugmentedState aug_state_1;
  aug_state_1.frame_id = 0;

  AugmentedState aug_state_2;
  aug_state_2.frame_id = 1;

  CamState cam_state;
  cam_state.augmented_states.push_back(aug_state_1);
  cam_state.augmented_states.push_back(aug_state_2);
  Eigen::MatrixXd cam_covariance(12, 12);
  ekf->RegisterCamera(1, cam_state, cam_covariance);

  EXPECT_EQ(ekf->GetImuCount(), 1U);
  EXPECT_EQ(ekf->GetCamCount(), 1U);

  EXPECT_EQ(ekf->GetImuStateStartIndex(0), 18U);
  EXPECT_EQ(ekf->GetCamStateStartIndex(1), 24U);
  EXPECT_EQ(ekf->GetAugStateStartIndex(1, 0), 30U);
  EXPECT_EQ(ekf->GetAugStateStartIndex(1, 1), 42U);
}
