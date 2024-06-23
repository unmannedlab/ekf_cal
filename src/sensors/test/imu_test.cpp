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

#include <memory>

#include "sensors/imu_message.hpp"
#include "sensors/imu.hpp"


TEST(test_IMU, Constructor) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  IMU::Parameters imu_params1;
  imu_params1.ekf = ekf;
  imu_params1.logger = ekf_params.debug_logger;
  imu_params1.is_intrinsic = false;
  imu_params1.is_extrinsic = false;
  IMU imu1(imu_params1);

  IMU::Parameters imu_params2;
  imu_params2.ekf = ekf;
  imu_params2.logger = ekf_params.debug_logger;
  imu_params2.is_intrinsic = true;
  imu_params2.is_extrinsic = false;
  IMU imu2(imu_params2);

  IMU::Parameters imu_params3;
  imu_params3.ekf = ekf;
  imu_params3.logger = ekf_params.debug_logger;
  imu_params3.is_intrinsic = false;
  imu_params3.is_extrinsic = true;
  IMU imu3(imu_params3);

  IMU::Parameters imu_params4;
  imu_params4.ekf = ekf;
  imu_params4.logger = ekf_params.debug_logger;
  imu_params4.is_intrinsic = true;
  imu_params4.is_extrinsic = true;
  IMU imu4(imu_params4);
}

TEST(test_IMU, ID) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  IMU::Parameters imu_params1;
  imu_params1.ekf = ekf;
  imu_params1.logger = ekf_params.debug_logger;

  IMU imu1(imu_params1);

  IMU::Parameters imu_params2;
  imu_params2.ekf = ekf;
  imu_params2.logger = ekf_params.debug_logger;

  IMU imu2(imu_params2);

  unsigned int id_one = imu1.GetId();
  unsigned int id_two = id_one + 1;

  EXPECT_EQ(imu1.GetId(), id_one);
  EXPECT_EQ(imu2.GetId(), id_two);
}

TEST(test_IMU, IMU_message) {
  ImuMessage imu_message;
}
