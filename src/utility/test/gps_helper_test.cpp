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

#include "utility/gps_helper.hpp"


TEST(test_GpsHelper, lla_to_ecef) {
  Eigen::Vector3d lla_1 {0.0, 0.0, 0.0};
  Eigen::Vector3d ecef_1 = lla_to_ecef(lla_1);

  EXPECT_NEAR(ecef_1(0), 6378137.0, 1e-2);
  EXPECT_NEAR(ecef_1(1), 0.0, 1e-2);
  EXPECT_NEAR(ecef_1(2), 0.0, 1e-2);

  Eigen::Vector3d lla_2 {90.0, 0.0, 0.0};
  Eigen::Vector3d ecef_2 = lla_to_ecef(lla_2);

  EXPECT_NEAR(ecef_2(0), 0.0, 1e-2);
  EXPECT_NEAR(ecef_2(1), 0.0, 1e-2);
  EXPECT_NEAR(ecef_2(2), 6356752.314, 1e-2);

  Eigen::Vector3d lla_3 {0.0, 90.0, 0.0};
  Eigen::Vector3d ecef_3 = lla_to_ecef(lla_3);

  EXPECT_NEAR(ecef_3(0), 0.0, 1e-2);
  EXPECT_NEAR(ecef_3(1), 6378137.0, 1e-2);
  EXPECT_NEAR(ecef_3(2), 0.0, 1e-2);

  Eigen::Vector3d lla_4 {30.619723587178996, -96.34081358686792, 0.0};
  Eigen::Vector3d ecef_4 = lla_to_ecef(lla_4);

  EXPECT_NEAR(ecef_4(0), -606724.231, 1e-2);
  EXPECT_NEAR(ecef_4(1), -5459978.172, 1e-2);
  EXPECT_NEAR(ecef_4(2), 3229683.720, 1e-2);
}

TEST(test_GpsHelper, ecef_to_enu) {
  Eigen::Vector3d lla_1 {0.0, 0.0, 0.0};
  Eigen::Vector3d ecef_1a {6378137.0, 0.0, 0.0};
  Eigen::Vector3d ecef_1b {6378237.0, 0.0, 0.0};
  Eigen::Vector3d enu_1a = ecef_to_enu(ecef_1a, lla_1);
  Eigen::Vector3d enu_1b = ecef_to_enu(ecef_1b, lla_1);

  EXPECT_NEAR(enu_1a(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_1a(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_1a(2), 0.0, 1e-2);
  EXPECT_NEAR(enu_1b(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_1b(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_1b(2), 100.0, 1e-2);

  Eigen::Vector3d lla_2 {0.0, 90.0, 0.0};
  Eigen::Vector3d ecef_2a {0.0, 6378137.0, 0.0};
  Eigen::Vector3d ecef_2b {0.0, 6378237.0, 0.0};
  Eigen::Vector3d enu_2a = ecef_to_enu(ecef_2a, lla_2);
  Eigen::Vector3d enu_2b = ecef_to_enu(ecef_2b, lla_2);

  EXPECT_NEAR(enu_2a(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_2a(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_2a(2), 0.0, 1e-2);
  EXPECT_NEAR(enu_2b(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_2b(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_2b(2), 100.0, 1e-2);

  Eigen::Vector3d lla_3 {90.0, 0.0, 0.0};
  Eigen::Vector3d ecef_3a {0.0, 0.0, 6356752.31};
  Eigen::Vector3d ecef_3b {0.0, 0.0, 6356852.31};
  Eigen::Vector3d enu_3a = ecef_to_enu(ecef_3a, lla_3);
  Eigen::Vector3d enu_3b = ecef_to_enu(ecef_3b, lla_3);

  EXPECT_NEAR(enu_3a(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_3a(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_3a(2), 0.0, 1e-2);
  EXPECT_NEAR(enu_3b(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_3b(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_3b(2), 100.0, 1e-2);
}

TEST(test_GpsHelper, lla_to_enu) {
  Eigen::Vector3d ref_1 {0.0, 0.0, 0.0};
  Eigen::Vector3d lla_1a {0.0, 0.0, 0.0};
  Eigen::Vector3d lla_1b {0.001, 0.0, 0.0};
  Eigen::Vector3d lla_1c {0.0, 0.001, 0.0};
  Eigen::Vector3d lla_1d {0.0, 0.0, 100.0};
  Eigen::Vector3d lla_1e {0.001, 0.001, 100.0};
  Eigen::Vector3d enu_1a = lla_to_enu(lla_1a, ref_1);
  Eigen::Vector3d enu_1b = lla_to_enu(lla_1b, ref_1);
  Eigen::Vector3d enu_1c = lla_to_enu(lla_1c, ref_1);
  Eigen::Vector3d enu_1d = lla_to_enu(lla_1d, ref_1);
  Eigen::Vector3d enu_1e = lla_to_enu(lla_1e, ref_1);

  EXPECT_NEAR(enu_1a(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_1a(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_1a(2), 0.0, 1e-2);

  EXPECT_NEAR(enu_1b(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_1b(1), 110.57, 1e-2);
  EXPECT_NEAR(enu_1b(2), 0.0, 1e-2);

  EXPECT_NEAR(enu_1c(0), 111.32, 1e-2);
  EXPECT_NEAR(enu_1c(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_1c(2), 0.0, 1e-2);

  EXPECT_NEAR(enu_1d(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_1d(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_1d(2), 100.0, 1e-2);

  EXPECT_NEAR(enu_1e(0), 111.32, 1e-2);
  EXPECT_NEAR(enu_1e(1), 110.57, 1e-2);
  EXPECT_NEAR(enu_1e(2), 100.0, 1e-2);

  Eigen::Vector3d ref_2  {0.0, 90.0, 0.0};
  Eigen::Vector3d lla_2a {0.0, 90.0, 0.0};
  Eigen::Vector3d lla_2b {0.0, 90.001, 0.0};
  Eigen::Vector3d lla_2c {0.001, 90.0, 0.0};
  Eigen::Vector3d lla_2d {0.0, 90.0, 100.0};
  Eigen::Vector3d lla_2e {0.001, 90.001, 100.0};
  Eigen::Vector3d enu_2a = lla_to_enu(lla_2a, ref_2);
  Eigen::Vector3d enu_2b = lla_to_enu(lla_2b, ref_2);
  Eigen::Vector3d enu_2c = lla_to_enu(lla_2c, ref_2);
  Eigen::Vector3d enu_2d = lla_to_enu(lla_2d, ref_2);
  Eigen::Vector3d enu_2e = lla_to_enu(lla_2e, ref_2);

  EXPECT_NEAR(enu_2a(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_2a(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_2a(2), 0.0, 1e-2);

  EXPECT_NEAR(enu_2b(0), 111.32, 1e-2);
  EXPECT_NEAR(enu_2b(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_2b(2), 0.0, 1e-2);

  EXPECT_NEAR(enu_2c(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_2c(1), 110.57, 1e-2);
  EXPECT_NEAR(enu_2c(2), 0.0, 1e-2);

  EXPECT_NEAR(enu_2d(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_2d(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_2d(2), 100.0, 1e-2);

  EXPECT_NEAR(enu_2e(0), 111.32, 1e-2);
  EXPECT_NEAR(enu_2e(1), 110.57, 1e-2);
  EXPECT_NEAR(enu_2e(2), 100.0, 1e-2);

  Eigen::Vector3d ref_3 {0.0, 0.0, 100.0};
  Eigen::Vector3d lla_3a {0.0, 0.0, 100.0};
  Eigen::Vector3d lla_3b {0.001, 0.0, 100.0};
  Eigen::Vector3d lla_3c {0.0, 0.001, 100.0};
  Eigen::Vector3d lla_3d {0.0, 0.0, 200.0};
  Eigen::Vector3d lla_3e {0.001, 0.001, 200.0};
  Eigen::Vector3d enu_3a = lla_to_enu(lla_3a, ref_3);
  Eigen::Vector3d enu_3b = lla_to_enu(lla_3b, ref_3);
  Eigen::Vector3d enu_3c = lla_to_enu(lla_3c, ref_3);
  Eigen::Vector3d enu_3d = lla_to_enu(lla_3d, ref_3);
  Eigen::Vector3d enu_3e = lla_to_enu(lla_3e, ref_3);

  EXPECT_NEAR(enu_3a(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_3a(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_3a(2), 0.0, 1e-2);

  EXPECT_NEAR(enu_3b(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_3b(1), 110.58, 1e-2);
  EXPECT_NEAR(enu_3b(2), 0.0, 1e-2);

  EXPECT_NEAR(enu_3c(0), 111.32, 1e-2);
  EXPECT_NEAR(enu_3c(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_3c(2), 0.0, 1e-2);

  EXPECT_NEAR(enu_3d(0), 0.0, 1e-2);
  EXPECT_NEAR(enu_3d(1), 0.0, 1e-2);
  EXPECT_NEAR(enu_3d(2), 100.0, 1e-2);

  EXPECT_NEAR(enu_3e(0), 111.32, 1e-2);
  EXPECT_NEAR(enu_3e(1), 110.58, 1e-2);
  EXPECT_NEAR(enu_3e(2), 100.0, 1e-2);
}

TEST(test_GpsHelper, enu_to_ecef) {
  Eigen::Vector3d ref_1 {0.0, 0.0, 0.0};
  Eigen::Vector3d enu_1a {0.0, 0.0, 0.0};
  Eigen::Vector3d enu_1b {100.0, 0.0, 0.0};
  Eigen::Vector3d enu_1c {0.0, 100.0, 0.0};
  Eigen::Vector3d enu_1d {0.0, 0.0, 100.0};
  Eigen::Vector3d enu_1e {100.0, 100.0, 100.0};
  Eigen::Vector3d ecef_1a = enu_to_ecef(enu_1a, ref_1);
  Eigen::Vector3d ecef_1b = enu_to_ecef(enu_1b, ref_1);
  Eigen::Vector3d ecef_1c = enu_to_ecef(enu_1c, ref_1);
  Eigen::Vector3d ecef_1d = enu_to_ecef(enu_1d, ref_1);
  Eigen::Vector3d ecef_1e = enu_to_ecef(enu_1e, ref_1);

  EXPECT_NEAR(ecef_1a(0), 6378137.0, 1e-2);
  EXPECT_NEAR(ecef_1a(1), 0.0, 1e-2);
  EXPECT_NEAR(ecef_1a(2), 0.0, 1e-2);

  EXPECT_NEAR(ecef_1b(0), 6378137.0, 1e-2);
  EXPECT_NEAR(ecef_1b(1), 100.0, 1e-2);
  EXPECT_NEAR(ecef_1b(2), 0.0, 1e-2);

  EXPECT_NEAR(ecef_1c(0), 6378137.0, 1e-2);
  EXPECT_NEAR(ecef_1c(1), 0.0, 1e-2);
  EXPECT_NEAR(ecef_1c(2), 100.0, 1e-2);

  EXPECT_NEAR(ecef_1d(0), 6378237.0, 1e-2);
  EXPECT_NEAR(ecef_1d(1), 0.0, 1e-2);
  EXPECT_NEAR(ecef_1d(2), 0.0, 1e-2);

  EXPECT_NEAR(ecef_1e(0), 6378237.0, 1e-2);
  EXPECT_NEAR(ecef_1e(1), 100.0, 1e-2);
  EXPECT_NEAR(ecef_1e(2), 100.0, 1e-2);

  Eigen::Vector3d ref_2  {0.0, 90.0, 0.0};
  Eigen::Vector3d enu_2a {0.0, 0.0, 0.0};
  Eigen::Vector3d enu_2b {100.0, 0.0, 0.0};
  Eigen::Vector3d enu_2c {0.0, 100.0, 0.0};
  Eigen::Vector3d enu_2d {0.0, 0.0, 100.0};
  Eigen::Vector3d enu_2e {100.0, 100.0, 100.0};
  Eigen::Vector3d ecef_2a = enu_to_ecef(enu_2a, ref_2);
  Eigen::Vector3d ecef_2b = enu_to_ecef(enu_2b, ref_2);
  Eigen::Vector3d ecef_2c = enu_to_ecef(enu_2c, ref_2);
  Eigen::Vector3d ecef_2d = enu_to_ecef(enu_2d, ref_2);
  Eigen::Vector3d ecef_2e = enu_to_ecef(enu_2e, ref_2);

  EXPECT_NEAR(ecef_2a(0), 0.0, 1e-2);
  EXPECT_NEAR(ecef_2a(1), 6378137.0, 1e-2);
  EXPECT_NEAR(ecef_2a(2), 0.0, 1e-2);

  EXPECT_NEAR(ecef_2b(0), -100.0, 1e-2);
  EXPECT_NEAR(ecef_2b(1), 6378137.0, 1e-2);
  EXPECT_NEAR(ecef_2b(2), 0.0, 1e-2);

  EXPECT_NEAR(ecef_2c(0), 0.0, 1e-2);
  EXPECT_NEAR(ecef_2c(1), 6378137.0, 1e-2);
  EXPECT_NEAR(ecef_2c(2), 100.0, 1e-2);

  EXPECT_NEAR(ecef_2d(0), 0.0, 1e-2);
  EXPECT_NEAR(ecef_2d(1), 6378237.0, 1e-2);
  EXPECT_NEAR(ecef_2d(2), 0.0, 1e-2);

  EXPECT_NEAR(ecef_2e(0), -100.0, 1e-2);
  EXPECT_NEAR(ecef_2e(1), 6378237.0, 1e-2);
  EXPECT_NEAR(ecef_2e(2), 100.0, 1e-2);

  Eigen::Vector3d ref_3 {0.0, 0.0, 100.0};
  Eigen::Vector3d enu_3a {0.0, 0.0, 0.0};
  Eigen::Vector3d enu_3b {100.0, 0.0, 0.0};
  Eigen::Vector3d enu_3c {0.0, 100.0, 0.0};
  Eigen::Vector3d enu_3d {0.0, 0.0, 100.0};
  Eigen::Vector3d enu_3e {100.0, 100.0, 100.0};
  Eigen::Vector3d ecef_3a = enu_to_ecef(enu_3a, ref_3);
  Eigen::Vector3d ecef_3b = enu_to_ecef(enu_3b, ref_3);
  Eigen::Vector3d ecef_3c = enu_to_ecef(enu_3c, ref_3);
  Eigen::Vector3d ecef_3d = enu_to_ecef(enu_3d, ref_3);
  Eigen::Vector3d ecef_3e = enu_to_ecef(enu_3e, ref_3);

  EXPECT_NEAR(ecef_3a(0), 6378237.0, 1e-2);
  EXPECT_NEAR(ecef_3a(1), 0.0, 1e-2);
  EXPECT_NEAR(ecef_3a(2), 0.0, 1e-2);

  EXPECT_NEAR(ecef_3b(0), 6378237.0, 1e-2);
  EXPECT_NEAR(ecef_3b(1), 100.0, 1e-2);
  EXPECT_NEAR(ecef_3b(2), 0.0, 1e-2);

  EXPECT_NEAR(ecef_3c(0), 6378237.0, 1e-2);
  EXPECT_NEAR(ecef_3c(1), 0.0, 1e-2);
  EXPECT_NEAR(ecef_3c(2), 100.0, 1e-2);

  EXPECT_NEAR(ecef_3d(0), 6378337.0, 1e-2);
  EXPECT_NEAR(ecef_3d(1), 0.0, 1e-2);
  EXPECT_NEAR(ecef_3d(2), 0.0, 1e-2);

  EXPECT_NEAR(ecef_3e(0), 6378337.0, 1e-2);
  EXPECT_NEAR(ecef_3e(1), 100.0, 1e-2);
  EXPECT_NEAR(ecef_3e(2), 100.0, 1e-2);
}

TEST(test_GpsHelper, ecef_to_lla) {
  Eigen::Vector3d ecef_1 {6378137.0, 0.0, 0.0};
  Eigen::Vector3d lla_1 = ecef_to_lla(ecef_1);

  EXPECT_NEAR(lla_1(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_1(1), 0.0, 1e-2);
  EXPECT_NEAR(lla_1(2), 0.0, 1e-1);

  Eigen::Vector3d ecef_2 {0.0, 0.0, 6356752.314};
  Eigen::Vector3d lla_2 = ecef_to_lla(ecef_2);

  EXPECT_NEAR(lla_2(0), 90.0, 1e-2);
  EXPECT_NEAR(lla_2(1), 0.0, 1e-2);
  EXPECT_NEAR(lla_2(2), 0.0, 1e-1);

  Eigen::Vector3d ecef_3 {0.0, 6378137.0, 0.0};
  Eigen::Vector3d lla_3 = ecef_to_lla(ecef_3);

  EXPECT_NEAR(lla_3(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_3(1), 90.0, 1e-2);
  EXPECT_NEAR(lla_3(2), 0.0, 1e-1);

  Eigen::Vector3d ecef_4 {-606724.231, -5459978.172, 3229683.720};
  Eigen::Vector3d lla_4 = ecef_to_lla(ecef_4);

  EXPECT_NEAR(lla_4(0), 30.619723587178996, 1e-2);
  EXPECT_NEAR(lla_4(1), -96.34081358686792, 1e-2);
  EXPECT_NEAR(lla_4(2), 0.0, 1e-1);
}

TEST(test_GpsHelper, enu_to_lla) {
  Eigen::Vector3d ref_1 {0.0, 0.0, 0.0};
  Eigen::Vector3d enu_1a = {0.0, 0.0, 0.0};
  Eigen::Vector3d enu_1b = {0.0, 110.57, 0.0};
  Eigen::Vector3d enu_1c = {111.32, 0.0, 0.0};
  Eigen::Vector3d enu_1d = {0.0, 0.0, 100.0};
  Eigen::Vector3d enu_1e = {111.32, 110.57, 100.0};
  Eigen::Vector3d lla_1a = enu_to_lla(enu_1a, ref_1);
  Eigen::Vector3d lla_1b = enu_to_lla(enu_1b, ref_1);
  Eigen::Vector3d lla_1c = enu_to_lla(enu_1c, ref_1);
  Eigen::Vector3d lla_1d = enu_to_lla(enu_1d, ref_1);
  Eigen::Vector3d lla_1e = enu_to_lla(enu_1e, ref_1);

  EXPECT_NEAR(lla_1a(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_1a(1), 0.0, 1e-2);
  EXPECT_NEAR(lla_1a(2), 0.0, 1e-1);

  EXPECT_NEAR(lla_1b(0), 0.001, 1e-2);
  EXPECT_NEAR(lla_1b(1), 0.0, 1e-2);
  EXPECT_NEAR(lla_1b(2), 0.0, 1e-1);

  EXPECT_NEAR(lla_1c(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_1c(1), 0.001, 1e-2);
  EXPECT_NEAR(lla_1c(2), 0.0, 1e-1);

  EXPECT_NEAR(lla_1d(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_1d(1), 0.0, 1e-2);
  EXPECT_NEAR(lla_1d(2), 100.0, 1e-1);

  EXPECT_NEAR(lla_1e(0), 0.001, 1e-2);
  EXPECT_NEAR(lla_1e(1), 0.001, 1e-2);
  EXPECT_NEAR(lla_1e(2), 100.0, 1e-1);

  Eigen::Vector3d ref_2  {0.0, 90.0, 0.0};
  Eigen::Vector3d enu_2a = {0.0, 0.0, 0.0};
  Eigen::Vector3d enu_2b = {111.32, 0.0, 0.0};
  Eigen::Vector3d enu_2c = {0.0, 110.57, 0.0};
  Eigen::Vector3d enu_2d = {0.0, 0.0, 100.0};
  Eigen::Vector3d enu_2e = {111.32, 110.57, 100.0};
  Eigen::Vector3d lla_2a = enu_to_lla(enu_2a, ref_2);
  Eigen::Vector3d lla_2b = enu_to_lla(enu_2b, ref_2);
  Eigen::Vector3d lla_2c = enu_to_lla(enu_2c, ref_2);
  Eigen::Vector3d lla_2d = enu_to_lla(enu_2d, ref_2);
  Eigen::Vector3d lla_2e = enu_to_lla(enu_2e, ref_2);

  EXPECT_NEAR(lla_2a(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_2a(1), 90.0, 1e-2);
  EXPECT_NEAR(lla_2a(2), 0.0, 1e-1);

  EXPECT_NEAR(lla_2b(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_2b(1), 90.001, 1e-2);
  EXPECT_NEAR(lla_2b(2), 0.0, 1e-1);

  EXPECT_NEAR(lla_2c(0), 0.001, 1e-2);
  EXPECT_NEAR(lla_2c(1), 90.0, 1e-2);
  EXPECT_NEAR(lla_2c(2), 0.0, 1e-1);

  EXPECT_NEAR(lla_2d(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_2d(1), 90.0, 1e-2);
  EXPECT_NEAR(lla_2d(2), 100.0, 1e-1);

  EXPECT_NEAR(lla_2e(0), 0.001, 1e-2);
  EXPECT_NEAR(lla_2e(1), 90.001, 1e-2);
  EXPECT_NEAR(lla_2e(2), 100.0, 1e-1);

  Eigen::Vector3d ref_3 {0.0, 0.0, 100.0};
  Eigen::Vector3d enu_3a = {0.0, 0.0, 0.0};
  Eigen::Vector3d enu_3b = {0.0, 110.58, 0.0};
  Eigen::Vector3d enu_3c = {111.32, 0.0, 0.0};
  Eigen::Vector3d enu_3d = {0.0, 0.0, 100.0};
  Eigen::Vector3d enu_3e = {111.32, 110.58, 100.0};
  Eigen::Vector3d lla_3a = enu_to_lla(enu_3a, ref_3);
  Eigen::Vector3d lla_3b = enu_to_lla(enu_3b, ref_3);
  Eigen::Vector3d lla_3c = enu_to_lla(enu_3c, ref_3);
  Eigen::Vector3d lla_3d = enu_to_lla(enu_3d, ref_3);
  Eigen::Vector3d lla_3e = enu_to_lla(enu_3e, ref_3);

  EXPECT_NEAR(lla_3a(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_3a(1), 0.0, 1e-2);
  EXPECT_NEAR(lla_3a(2), 100.0, 1e-1);

  EXPECT_NEAR(lla_3b(0), 0.001, 1e-2);
  EXPECT_NEAR(lla_3b(1), 0.0, 1e-2);
  EXPECT_NEAR(lla_3b(2), 100.0, 1e-1);

  EXPECT_NEAR(lla_3c(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_3c(1), 0.001, 1e-2);
  EXPECT_NEAR(lla_3c(2), 100.0, 1e-1);

  EXPECT_NEAR(lla_3d(0), 0.0, 1e-2);
  EXPECT_NEAR(lla_3d(1), 0.0, 1e-2);
  EXPECT_NEAR(lla_3d(2), 200.0, 1e-1);

  EXPECT_NEAR(lla_3e(0), 0.001, 1e-2);
  EXPECT_NEAR(lla_3e(1), 0.001, 1e-2);
  EXPECT_NEAR(lla_3e(2), 200.0, 1e-1);
}
