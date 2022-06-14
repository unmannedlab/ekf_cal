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

#include "ekf/sensors/Imu.hpp"


TEST(test_Imu, Name) {
  Imu::Params params;
  params.name = "IMU_Name";
  Imu imu(params);
  EXPECT_EQ(imu.GetName(), params.name);
}


TEST(test_Imu, Id) {
  Imu::Params params1;
  Imu::Params params2;

  params1.name = "Imu_1";
  params2.name = "Imu_2";

  Imu imu1(params1);
  Imu imu2(params2);

  unsigned int id_one = imu1.GetId();
  unsigned int id_two = id_one + 1;

  EXPECT_EQ(imu1.GetId(), id_one);
  EXPECT_EQ(imu2.GetId(), id_two);
}


TEST(test_Imu, StateStartIndex) {
  Imu::Params params;
  params.name = "StateStartIndex";
  Imu imu(params);

  unsigned int index1 = 1U;
  unsigned int index2 = 2U;

  imu.SetStateStartIndex(index1);
  EXPECT_EQ(imu.GetStateStartIndex(), index1);

  imu.SetStateStartIndex(index2);
  EXPECT_EQ(imu.GetStateStartIndex(), index2);
}

TEST(test_Imu, StateSize) {
  Imu::Params params;
  params.name = "StateSize";
  Imu imu(params);
  EXPECT_EQ(imu.GetStateSize(), 6U);
}

TEST(test_Imu, PosOffset) {
  Imu::Params params;
  params.name = "PosOffset";
  Imu imu(params);

  Eigen::Vector3d posOff1 = {1.2, -3.4, 5.6};
  Eigen::Vector3d posOff2 = {-9.8, 7.6, -5.4};

  imu.SetPosOffset(posOff1);
  EXPECT_EQ(imu.GetPosOffset(), posOff1);

  imu.SetPosOffset(posOff2);
  EXPECT_EQ(imu.GetPosOffset(), posOff2);
}

TEST(test_Imu, AngOffset) {
  Imu::Params params;
  params.name = "PosOffset";
  Imu imu(params);

  Eigen::Quaterniond angOff1 = {1.2, -3.4, 5.6, -7.8};
  Eigen::Quaterniond angOff2 = {-9.8, 7.6, -5.4, 3.2};

  imu.SetAngOffset(angOff1);
  EXPECT_EQ(imu.GetAngOffset(), angOff1);

  imu.SetAngOffset(angOff2);
  EXPECT_EQ(imu.GetAngOffset(), angOff2);
}
