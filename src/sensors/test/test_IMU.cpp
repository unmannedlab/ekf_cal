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

#include "sensors/IMU.hpp"

#include "utility/TypeHelper.hpp"
#include "utility/test/CustomAssertions.hpp"

TEST(test_IMU, Constructor) {
  IMU::Parameters params;
  IMU imu(params);
}

TEST(test_IMU, Id) {
  IMU::Parameters params1;
  IMU::Parameters params2;

  IMU imu1(params1);
  IMU imu2(params2);

  unsigned int id_one = imu1.getId();
  unsigned int id_two = id_one + 1;

  EXPECT_EQ(imu1.getId(), id_one);
  EXPECT_EQ(imu2.getId(), id_two);
}
