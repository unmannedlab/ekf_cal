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
  imu_state.SetIsIntrinsic(true);
  imu_state.SetIsExtrinsic(false);
  Eigen::MatrixXd imu_covariance(6, 6);
  ekf->RegisterIMU(0, imu_state, imu_covariance);

  CamState cam_state;
  cam_state.SetIsExtrinsic(true);
  Eigen::MatrixXd cam_covariance(6, 6);
  ekf->RegisterCamera(1, cam_state, cam_covariance);
  ekf->AugmentStateIfNeeded(1, 0);
  ekf->AugmentStateIfNeeded(1, 1);

  EXPECT_EQ(ekf->GetImuCount(), 1);
  EXPECT_EQ(ekf->GetCamCount(), 1);

  EXPECT_EQ(ekf->m_state.imu_states[0].index, 18);
  EXPECT_EQ(ekf->m_state.cam_states[1].index, 24);
  EXPECT_EQ(ekf->GetAugState(1, 0, 0).index, 30);
  EXPECT_EQ(ekf->GetAugState(1, 1, 0).index, 36);
}

TEST(test_EKF, duplicate_sensors) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  EXPECT_EQ(ekf->GetImuCount(), 0);
  EXPECT_EQ(ekf->GetCamCount(), 0);

  ImuState imu_state;
  imu_state.SetIsIntrinsic(true);
  imu_state.SetIsExtrinsic(false);
  Eigen::MatrixXd imu_covariance(g_imu_intrinsic_state_size, g_imu_intrinsic_state_size);
  ekf->RegisterIMU(0, imu_state, imu_covariance);
  ekf->RegisterIMU(0, imu_state, imu_covariance);

  GpsState gps_state;
  gps_state.SetIsExtrinsic(true);
  Eigen::MatrixXd gps_cov(g_gps_extrinsic_state_size, g_gps_extrinsic_state_size);
  ekf->RegisterGPS(1, gps_state, gps_cov);
  ekf->RegisterGPS(1, gps_state, gps_cov);

  CamState cam_state;
  cam_state.SetIsExtrinsic(true);
  Eigen::MatrixXd cam_covariance(g_cam_extrinsic_state_size, g_cam_extrinsic_state_size);
  ekf->RegisterCamera(2, cam_state, cam_covariance);
  ekf->RegisterCamera(2, cam_state, cam_covariance);

  FidState fid_state;
  fid_state.SetIsExtrinsic(true);
  fid_state.id = 3;
  Eigen::MatrixXd fid_cov(g_fid_extrinsic_state_size, g_fid_extrinsic_state_size);
  ekf->RegisterFiducial(fid_state, fid_cov);
  ekf->RegisterFiducial(fid_state, fid_cov);

  ekf->AugmentStateIfNeeded(2, 0);
  ekf->AugmentStateIfNeeded(2, 1);

  EXPECT_EQ(ekf->GetImuCount(), 1);
  EXPECT_EQ(ekf->GetGpsCount(), 1);
  EXPECT_EQ(ekf->GetCamCount(), 1);

  EXPECT_EQ(ekf->m_state.imu_states[0].index, 18);
  EXPECT_EQ(ekf->m_state.gps_states[1].index, 24);
  EXPECT_EQ(ekf->m_state.cam_states[2].index, 27);
  EXPECT_EQ(ekf->GetAugState(2, 0, 0).index, 39);
  EXPECT_EQ(ekf->GetAugState(2, 1, 0).index, 45);

  EXPECT_EQ(ekf->GetImuStateStart(), 18);
  EXPECT_EQ(ekf->GetGpsStateStart(), 24);
  EXPECT_EQ(ekf->GetCamStateStart(), 27);
  EXPECT_EQ(ekf->GetFidStateStart(), 33);
  EXPECT_EQ(ekf->GetAugStateStart(), 39);
}

TEST(test_EKF, SetBodyProcessNoise) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  Eigen::VectorXd process_noise = Eigen::VectorXd::Ones(18);
  ekf->SetBodyProcessNoise(process_noise);
}

TEST(test_EKF, MatchState) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  AugState aug_state = ekf->GetAugState(0, 0, 0);

  Eigen::Quaterniond zero_quat {1, 0, 0, 0};
  Eigen::Vector3d zero_vec {0, 0, 0};

  EXPECT_EQ(aug_state.frame_id, 0);
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(aug_state.ang_b_to_l, zero_quat, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(aug_state.pos_b_in_l, zero_vec, 1e-6));

  EXPECT_EQ(ekf->GetAugState(0, 0, 0).index, 0);
}

TEST(test_EKF, SetMaxTrackLength) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  ekf->SetMaxTrackLength(20);
}

TEST(test_EKF, SetGpsReference) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  Eigen::Vector3d pos_e_in_g {0, 0, 0};
  double ang_l_to_e {0};

  ekf->SetGpsReference(pos_e_in_g, ang_l_to_e);
  EXPECT_TRUE(ekf->IsLlaInitialized());
}

TEST(test_EKF, AugmentCovariance) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  Eigen::VectorXd in_vec(12);
  in_vec << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
  Eigen::MatrixXd in_cov = in_vec.asDiagonal();

  Eigen::MatrixXd out_cov = ekf->AugmentCovariance(in_cov, 12);

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(out_cov.block<12, 12>(0, 0), in_cov, 1e-6));

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(out_cov.block<3, 3>(12, 12), in_cov.block<3, 3>(0, 0), 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(out_cov.block<3, 3>(15, 15), in_cov.block<3, 3>(9, 9), 1e-6));

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(out_cov.block<3, 3>(0, 12), in_cov.block<3, 3>(0, 0), 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(out_cov.block<3, 3>(9, 15), in_cov.block<3, 3>(9, 9), 1e-6));
}
