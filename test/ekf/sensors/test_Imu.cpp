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

  Imu imu1(params1);
  Imu imu2(params2);

  unsigned int id_one = imu1.GetId();
  unsigned int id_two = id_one + 1;

  EXPECT_EQ(imu1.GetId(), id_one);
  EXPECT_EQ(imu2.GetId(), id_two);
}


TEST(test_Imu, StateStartIndex) {
  Imu::Params params;
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

  params.baseSensor = true;
  params.intrinsic = false;

  Imu imu1(params);
  EXPECT_EQ(imu1.GetStateSize(), 0U);

  params.baseSensor = true;
  params.intrinsic = true;
  Imu imu2(params);
  EXPECT_EQ(imu2.GetStateSize(), 6U);

  params.baseSensor = false;
  params.intrinsic = false;
  Imu imu3(params);
  EXPECT_EQ(imu3.GetStateSize(), 6U);

  params.baseSensor = false;
  params.intrinsic = true;
  Imu imu4(params);
  EXPECT_EQ(imu4.GetStateSize(), 12U);
}

TEST(test_Imu, PosOffset) {
  Imu::Params params;
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
  Imu imu(params);

  Eigen::Quaterniond angOff1 = {1.2, -3.4, 5.6, -7.8};
  Eigen::Quaterniond angOff2 = {-9.8, 7.6, -5.4, 3.2};

  imu.SetAngOffset(angOff1);
  EXPECT_EQ(imu.GetAngOffset(), angOff1);

  imu.SetAngOffset(angOff2);
  EXPECT_EQ(imu.GetAngOffset(), angOff2);
}

TEST(test_Imu, GetAccBiasStability)
{
  Imu::Params params;
  params.accBiasStability = 1.234;
  Imu imu(params);

  EXPECT_EQ(imu.GetAccBiasStability(), params.accBiasStability);
}

TEST(test_Imu, GetOmgBiasStability)
{
  Imu::Params params;
  params.omgBiasStability = 4.567;
  Imu imu(params);

  EXPECT_EQ(imu.GetOmgBiasStability(), params.omgBiasStability);
}

TEST(test_Imu, IsBaseSensor)
{
  Imu::Params params1;
  Imu::Params params2;

  params1.baseSensor = true;
  params2.baseSensor = false;

  Imu imu1(params1);
  Imu imu2(params2);

  EXPECT_TRUE(imu1.IsBaseSensor());
  EXPECT_FALSE(imu2.IsBaseSensor());
}

TEST(test_Imu, IsIntrinsic)
{
  Imu::Params params1;
  Imu::Params params2;

  params1.intrinsic = true;
  params2.intrinsic = false;

  Imu imu1(params1);
  Imu imu2(params2);

  EXPECT_TRUE(imu1.IsIntrinsic());
  EXPECT_FALSE(imu2.IsIntrinsic());
}

TEST(test_Imu, Cov)
{
  Imu::Params params;
  params.baseSensor = false;
  params.intrinsic = false;
  Imu imu(params);

  EXPECT_EQ(imu.GetStateSize(), 6U);

  Eigen::MatrixXd cov(6, 6);
  cov.setIdentity();

  imu.SetCov(cov);

  EXPECT_EQ(imu.GetCov(), cov);
}
