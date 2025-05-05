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
#include "utility/custom_assertions.hpp"

TEST(test_imu_updater, update) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  EKF ekf(ekf_params);

  double time_init = 0.0;
  BodyState body_state;
  body_state.vel_b_in_l = Eigen::Vector3d::Ones();
  ekf.Initialize(time_init, body_state);

  unsigned int imu_id{0};

  ImuState imu_state_1;
  ImuState imu_state_2;
  ImuState imu_state_3;
  imu_state_1.SetIsIntrinsic(true);
  imu_state_1.SetIsExtrinsic(true);
  Eigen::MatrixXd imu_covariance_1 = Eigen::MatrixXd::Identity(12, 12);
  imu_state_2.SetIsIntrinsic(true);
  imu_state_2.SetIsExtrinsic(false);
  Eigen::MatrixXd imu_covariance_2 = Eigen::MatrixXd::Identity(6, 6);
  imu_state_3.SetIsIntrinsic(false);
  imu_state_3.SetIsExtrinsic(true);
  Eigen::MatrixXd imu_covariance_3 = Eigen::MatrixXd::Identity(6, 6);
  ekf.RegisterIMU(0, imu_state_1, imu_covariance_1);
  ekf.RegisterIMU(1, imu_state_2, imu_covariance_2);
  ekf.RegisterIMU(2, imu_state_3, imu_covariance_3);

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  ImuUpdater imu_updater(imu_id, true, true, "", 0.0, logger);

  Eigen::Vector3d acceleration = g_gravity;
  Eigen::Matrix3d acceleration_cov = Eigen::Matrix3d::Identity() * 1e-3;
  Eigen::Vector3d angular_rate = Eigen::Vector3d::Zero();
  Eigen::Matrix3d angular_rate_cov = Eigen::Matrix3d::Identity() * 1e-3;

  State state = ekf.m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 0, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 0, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 0, 1e-2);

  double time = time_init + 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf.m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 1, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 1, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 1, 1e-2);

  time += 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf.m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 2, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 2, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 2, 1e-2);

  imu_updater.UpdateEKF(
    ekf, time - 1, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf.m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 2, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 2, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 2, 1e-2);

  time += 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf.m_state;
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
  EKF ekf(ekf_params);

  double time_init = 0.0;
  BodyState body_state;
  body_state.vel_b_in_l = Eigen::Vector3d::Ones();
  ekf.Initialize(time_init, body_state);

  unsigned int imu_id{0};

  ImuState imu_state;
  imu_state.SetIsExtrinsic(true);
  imu_state.SetIsIntrinsic(true);
  Eigen::MatrixXd imu_cov = Eigen::MatrixXd::Zero(12, 12);
  ekf.RegisterIMU(imu_id, imu_state, imu_cov);

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  ImuUpdater imu_updater(imu_id, true, true, "", 0.0, logger);

  Eigen::Vector3d acceleration = g_gravity;
  Eigen::Matrix3d acceleration_cov = Eigen::Matrix3d::Identity() * 1e-3;
  Eigen::Vector3d angular_rate = Eigen::Vector3d::Zero();
  Eigen::Matrix3d angular_rate_cov = Eigen::Matrix3d::Identity() * 1e-3;

  State state = ekf.m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 0);

  double time = time_init + 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf.m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 1);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 1);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 1);

  time += 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf.m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 2);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 2);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 2);

  imu_updater.UpdateEKF(
    ekf, time - 1, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf.m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 2);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 2);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 2);

  time += 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf.m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 3);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 3);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 3);

  EXPECT_EQ(state.body_state.vel_b_in_l[0], 1);
  EXPECT_EQ(state.body_state.vel_b_in_l[1], 1);
  EXPECT_EQ(state.body_state.vel_b_in_l[2], 1);

  time += 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf.m_state;
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
  EKF ekf(ekf_params);

  unsigned int imu_id{0};

  ImuState imu_state;
  imu_state.SetIsExtrinsic(true);
  imu_state.SetIsIntrinsic(true);
  Eigen::MatrixXd imu_cov = Eigen::MatrixXd::Zero(12, 12);
  ekf.RegisterIMU(imu_id, imu_state, imu_cov);

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  ImuUpdater imu_updater(imu_id, true, true, "", 0.0, logger);

  Eigen::Vector3d acceleration = g_gravity;
  Eigen::Matrix3d acceleration_cov = Eigen::Matrix3d::Identity() * 1e-3;
  Eigen::Vector3d angular_rate = Eigen::Vector3d::Zero();
  Eigen::Matrix3d angular_rate_cov = Eigen::Matrix3d::Identity() * 1e-3;

  State state = ekf.m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 0);

  double time = 1;
  imu_updater.UpdateEKF(
    ekf, time, acceleration, acceleration_cov, angular_rate, angular_rate_cov);

  state = ekf.m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 0);
}

TEST(test_imu_updater, jacobian) {
  unsigned int imu_id{0};
  bool is_extrinsic{true};
  bool is_intrinsic{true};
  ImuState imu_state;
  imu_state.pos_i_in_b = Eigen::Vector3d{1, 2, 3};
  imu_state.SetIsExtrinsic(is_extrinsic);
  imu_state.SetIsIntrinsic(is_intrinsic);

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

  auto debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = debug_logger;
  EKF ekf(ekf_params);
  ekf.RegisterIMU(imu_id, imu_state, covariance);

  auto imu_updater =
    ImuUpdater(imu_id, is_extrinsic, is_intrinsic, "log_file_directory", 0.0, debug_logger);

  Eigen::VectorXd base_state = ekf.m_state.ToVector();
  Eigen::MatrixXd jac_analytical = imu_updater.GetMeasurementJacobian(ekf);
  Eigen::VectorXd base_meas = imu_updater.PredictMeasurement(ekf);

  double delta = 1.0e-6;
  unsigned int jac_size = base_state.size();
  Eigen::MatrixXd jac_numerical = Eigen::MatrixXd::Zero(6, jac_size);
  for (unsigned int i = 0; i < jac_size; ++i) {
    Eigen::VectorXd delta_vec = base_state;
    delta_vec[i] += delta;
    ekf.m_state.SetState(delta_vec);
    jac_numerical.block<6, 1>(0, i) = (imu_updater.PredictMeasurement(ekf) - base_meas) / delta;
  }

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(jac_analytical, jac_numerical, 1e-3));
}
