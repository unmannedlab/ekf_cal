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
#include "utility/custom_assertions.hpp"


TEST(test_EKF, get_counts) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  EXPECT_EQ(ekf->GetImuCount(), 0);
  EXPECT_EQ(ekf->GetCamCount(), 0);

  ImuState imu_state;
  imu_state.set_is_intrinsic(true);
  imu_state.set_is_extrinsic(false);
  Eigen::MatrixXd imu_covariance(6, 6);
  ekf->RegisterIMU(0, imu_state, imu_covariance);

  CamState cam_state;
  Eigen::MatrixXd cam_covariance(6, 6);
  ekf->RegisterCamera(1, cam_state, cam_covariance);
  ekf->AugmentStateIfNeeded(1, 0);
  ekf->AugmentStateIfNeeded(1, 1);

  EXPECT_EQ(ekf->GetImuCount(), 1);
  EXPECT_EQ(ekf->GetCamCount(), 1);

  EXPECT_EQ(ekf->m_state.imu_states[0].index, 18);
  EXPECT_EQ(ekf->m_state.cam_states[1].index, 24);
  EXPECT_EQ(ekf->GetAugState(1, 0).index, 30);
  EXPECT_EQ(ekf->GetAugState(1, 1).index, 36);
}

TEST(test_EKF, duplicate_sensors) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  EXPECT_EQ(ekf->GetImuCount(), 0);
  EXPECT_EQ(ekf->GetCamCount(), 0);

  ImuState imu_state;
  imu_state.set_is_intrinsic(true);
  imu_state.set_is_extrinsic(false);
  Eigen::MatrixXd imu_covariance(6, 6);
  ekf->RegisterIMU(0, imu_state, imu_covariance);
  ekf->RegisterIMU(0, imu_state, imu_covariance);

  GpsState gps_state;
  gps_state.set_is_extrinsic(true);
  Eigen::Matrix3d gps_cov(3, 3);
  ekf->RegisterGPS(1, gps_state, gps_cov);
  ekf->RegisterGPS(1, gps_state, gps_cov);

  AugState aug_state_1;
  aug_state_1.frame_id = 0;

  AugState aug_state_2;
  aug_state_2.frame_id = 1;

  CamState cam_state;
  Eigen::MatrixXd cam_covariance(12, 12);
  ekf->RegisterCamera(2, cam_state, cam_covariance);
  ekf->RegisterCamera(2, cam_state, cam_covariance);

  ekf->AugmentStateIfNeeded(2, 0);
  ekf->AugmentStateIfNeeded(2, 1);

  EXPECT_EQ(ekf->GetImuCount(), 1);
  EXPECT_EQ(ekf->GetGpsCount(), 1);
  EXPECT_EQ(ekf->GetCamCount(), 1);

  EXPECT_EQ(ekf->m_state.imu_states[0].index, 18);
  EXPECT_EQ(ekf->m_state.gps_states[1].index, 24);
  EXPECT_EQ(ekf->m_state.cam_states[2].index, 27);
  EXPECT_EQ(ekf->GetAugState(2, 0).index, 33);
  EXPECT_EQ(ekf->GetAugState(2, 1).index, 39);
}

TEST(test_EKF, SetProcessNoise) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  Eigen::VectorXd process_noise = Eigen::VectorXd::Ones(18);
  ekf->SetProcessNoise(process_noise);
}

TEST(test_EKF, MatchState) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  AugState aug_state = ekf->GetAugState(0, 0);

  Eigen::Quaterniond zero_quat {1, 0, 0, 0};
  Eigen::Vector3d zero_vec {0, 0, 0};

  EXPECT_EQ(aug_state.frame_id, -1);
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(aug_state.ang_b_to_l, zero_quat, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(aug_state.pos_b_in_l, zero_vec, 1e-6));

  EXPECT_EQ(ekf->GetAugState(0, 0).index, -1);
}
