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

#include "ekf/sensors/Sensor.hpp"


TEST(test_Sensor, Name) {
  std::string name {"Sensor_Name"};
  Sensor sensor(name);
  ASSERT_EQ(sensor.GetName(), name);
}


TEST(test_Sensor, Id) {
  Sensor sensor1("Sensor_1");
  Sensor sensor2("Sensor_2");
  unsigned int zero {0U};
  unsigned int one {1U};
  ASSERT_EQ(sensor1.GetId(), zero);
  ASSERT_EQ(sensor2.GetId(), one);
}


TEST(test_Sensor, StateStartIndex) {
  Sensor sensor("StateStartIndex");
  unsigned int index1 = 1;
  unsigned int index2 = 1;
  sensor.SetStateStartIndex(index1);
  ASSERT_EQ(sensor.GetStateStartIndex(), index1);

  sensor.SetStateStartIndex(index2);
  ASSERT_EQ(sensor.GetStateStartIndex(), index2);
}

TEST(test_Sensor, StateSize) {
  Sensor sensor("StateSize");
  ASSERT_EQ(sensor.GetStateSize(), 6);
}

TEST(test_Sensor, PosOffset) {
  Sensor sensor("PosOffset");
  Eigen::Vector3d posOff1 = {1.2, -3.4, 5.6};
  Eigen::Vector3d posOff2 = {-9.8, 7.6, -5.4};
  sensor.SetPosOffset(posOff1);
  ASSERT_EQ(sensor.GetPosOffset(), posOff1);

  sensor.SetPosOffset(posOff2);
  ASSERT_EQ(sensor.GetPosOffset(), posOff2);
}

TEST(test_Sensor, AngOffset) {
  Sensor sensor("PosOffset");
  Eigen::Quaterniond angOff1 = {1.2, -3.4, 5.6, -7.8};
  Eigen::Quaterniond angOff2 = {-9.8, 7.6, -5.4, 3.2};
  sensor.SetAngOffset(angOff1);
  ASSERT_EQ(sensor.GetAngOffset(), angOff1);

  sensor.SetAngOffset(angOff2);
  ASSERT_EQ(sensor.GetAngOffset(), angOff2);
}

// TEST(test_Sensor, ) PredictMeasurement
// TEST(test_Sensor, ) GetMeasurementJacobian
// TEST(test_Sensor, ) SetState
