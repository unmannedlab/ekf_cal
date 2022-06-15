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

#include "ekf/EKF.hpp"

///
/// @todo Write test with varying covariance in sensors
///
TEST(test_EKF, register_imu) {
  EKF ekf;

  Imu::Params imu_params_base;
  imu_params_base.baseSensor = true;
  imu_params_base.intrinsic = false;

  Imu::Params imu_params_extrinsic;
  imu_params_extrinsic.baseSensor = false;
  imu_params_extrinsic.intrinsic = false;
  imu_params_extrinsic.variance = Eigen::VectorXd::Ones(6);

  Imu::Params imu_params_intrinsic;
  imu_params_intrinsic.baseSensor = false;
  imu_params_intrinsic.intrinsic = true;
  imu_params_intrinsic.variance = Eigen::VectorXd::Ones(12);

  EXPECT_EQ(ekf.GetStateSize(), 18U);
  EXPECT_EQ(ekf.GetState(), Eigen::VectorXd::Zero(18U));
  EXPECT_EQ(ekf.GetCov(), Eigen::MatrixXd::Identity(18U, 18U));

  unsigned int id_1 = ekf.RegisterSensor(imu_params_base);

  EXPECT_EQ(ekf.GetStateSize(), 18U);
  EXPECT_EQ(ekf.GetState(), Eigen::VectorXd::Zero(18U));
  EXPECT_EQ(ekf.GetCov(), Eigen::MatrixXd::Identity(18U, 18U));

  unsigned int id_2 = ekf.RegisterSensor(imu_params_extrinsic);

  EXPECT_EQ(ekf.GetStateSize(), 24U);
  EXPECT_EQ(ekf.GetState(), Eigen::VectorXd::Zero(24U));
  EXPECT_EQ(ekf.GetCov(), Eigen::MatrixXd::Identity(24U, 24U));

  unsigned int id_3 = ekf.RegisterSensor(imu_params_intrinsic);

  EXPECT_EQ(ekf.GetStateSize(), 36U);
  EXPECT_EQ(ekf.GetState(), Eigen::VectorXd::Zero(36U));
  EXPECT_EQ(ekf.GetCov(), Eigen::MatrixXd::Identity(36U, 36U));

  EXPECT_EQ(id_1 + 1U, id_2);
  EXPECT_EQ(id_2 + 1U, id_3);
}
