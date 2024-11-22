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

#include <string>
#include <iostream>

#include "ekf/constants.hpp"
#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "ekf/update/imu_updater.hpp"

TEST(test_imu_updater, update) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  double time_init = 0.0;
  BodyState body_state;
  body_state.vel_b_in_l = Eigen::Vector3d::Ones();
  ekf->Initialize(time_init, body_state);

  unsigned int imu_id{0};
  std::string log_file_directory{""};

  ImuState imu_state_1, imu_state_2, imu_state_3;
  imu_state_1.SetIsIntrinsic(true);
  imu_state_1.SetIsExtrinsic(true);
  Eigen::MatrixXd imu_covariance_1 = Eigen::MatrixXd::Identity(12, 12);
  imu_state_2.SetIsIntrinsic(true);
  imu_state_2.SetIsExtrinsic(false);
  Eigen::MatrixXd imu_covariance_2 = Eigen::MatrixXd::Identity(6, 6);
  imu_state_3.SetIsIntrinsic(false);
  imu_state_3.SetIsExtrinsic(true);
  Eigen::MatrixXd imu_covariance_3 = Eigen::MatrixXd::Identity(6, 6);
  ekf->RegisterIMU(0, imu_state_1, imu_covariance_1);
  ekf->RegisterIMU(1, imu_state_2, imu_covariance_2);
  ekf->RegisterIMU(2, imu_state_3, imu_covariance_3);

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  ImuUpdater imu_updater(imu_id, true, true, log_file_directory, 0.0, logger);

  Eigen::Vector3d acceleration = g_gravity;
  Eigen::Matrix3d acceleration_cov = Eigen::Matrix3d::Identity() * 1e-3;
  Eigen::Vector3d angular_rate = Eigen::Vector3d::Zero();
  Eigen::Matrix3d angular_rate_cov = Eigen::Matrix3d::Identity() * 1e-3;

  State state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 0, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 0, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 0, 1e-2);

  double time = time_init + 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 1, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 1, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 1, 1e-2);

  time += 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 2, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 2, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 2, 1e-2);

  imu_updater.UpdateEKF(
    ekf, time - 1, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 2, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 2, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 2, 1e-2);

  time += 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 3, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 3, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 3, 1e-2);

  EXPECT_NEAR(state.body_state.vel_b_in_l[0], 1, 1e-2);
  EXPECT_NEAR(state.body_state.vel_b_in_l[1], 1, 1e-2);
  EXPECT_NEAR(state.body_state.vel_b_in_l[2], 1, 1e-2);
}

TEST(test_imu_updater, imu_prediction_update) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  double time_init = 0.0;
  BodyState body_state;
  body_state.vel_b_in_l = Eigen::Vector3d::Ones();
  ekf->Initialize(time_init, body_state);

  unsigned int imu_id{0};
  std::string log_file_directory{""};

  ImuState imu_state;
  imu_state.SetIsExtrinsic(true);
  imu_state.SetIsIntrinsic(true);
  Eigen::MatrixXd imu_cov = Eigen::MatrixXd::Zero(12, 12);
  ekf->RegisterIMU(imu_id, imu_state, imu_cov);

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  ImuUpdater imu_updater(imu_id, true, true, log_file_directory, 0.0, logger);

  Eigen::Vector3d acceleration = g_gravity;
  Eigen::Matrix3d acceleration_cov = Eigen::Matrix3d::Identity() * 1e-3;
  Eigen::Vector3d angular_rate = Eigen::Vector3d::Zero();
  Eigen::Matrix3d angular_rate_cov = Eigen::Matrix3d::Identity() * 1e-3;

  State state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 0);

  double time = time_init + 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 1);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 1);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 1);

  time += 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 2);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 2);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 2);

  imu_updater.UpdateEKF(
    ekf, time - 1, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 2);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 2);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 2);

  time += 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 3);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 3);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 3);

  EXPECT_EQ(state.body_state.vel_b_in_l[0], 1);
  EXPECT_EQ(state.body_state.vel_b_in_l[1], 1);
  EXPECT_EQ(state.body_state.vel_b_in_l[2], 1);

  time += 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 4);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 4);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 4);

  EXPECT_EQ(state.body_state.vel_b_in_l[0], 1);
  EXPECT_EQ(state.body_state.vel_b_in_l[1], 1);
  EXPECT_EQ(state.body_state.vel_b_in_l[2], 1);
}

TEST(test_imu_updater, non_initialized_time) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  unsigned int imu_id{0};
  std::string log_file_directory{""};

  ImuState imu_state;
  imu_state.SetIsExtrinsic(true);
  imu_state.SetIsIntrinsic(true);
  Eigen::MatrixXd imu_cov = Eigen::MatrixXd::Zero(12, 12);
  ekf->RegisterIMU(imu_id, imu_state, imu_cov);

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  ImuUpdater imu_updater(imu_id, true, true, log_file_directory, 0.0, logger);

  Eigen::Vector3d acceleration = g_gravity;
  Eigen::Matrix3d acceleration_cov = Eigen::Matrix3d::Identity() * 1e-3;
  Eigen::Vector3d angular_rate = Eigen::Vector3d::Zero();
  Eigen::Matrix3d angular_rate_cov = Eigen::Matrix3d::Identity() * 1e-3;

  State state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 0);

  double time = 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 0);
}
