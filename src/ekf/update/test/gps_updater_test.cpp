// Copyright 2024 Jacob Hartzer
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
#include "ekf/update/gps_updater.hpp"
#include "utility/gps_helper.hpp"

TEST(test_gps_updater, update) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  double time_init = 0.0;
  BodyState body_state;
  body_state.vel_b_in_l = Eigen::Vector3d::Ones();
  ekf->Initialize(time_init, body_state);

  unsigned int gps_id{0};
  std::string log_file_dir{""};
  bool data_logging_on {true};

  GpsState gps_state;
  Eigen::Matrix3d gps_cov = Eigen::Matrix3d::Zero(3, 3);
  ekf->RegisterGPS(gps_id, gps_state, gps_cov);

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  GpsInitType init_type = GpsInitType::ERROR_THRESHOLD;
  GpsUpdater gps_updater(
    gps_id, init_type, 1.0, 1.0, 1.0, false, log_file_dir, data_logging_on, 0.0, logger);

  State state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 0);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 0);

  double time = time_init + 1;
  Eigen::Vector3d ref_lla{0, 0, 0};
  Eigen::Vector3d antenna_enu{1, 1, 1};
  Eigen::Vector3d gps_lla = enu_to_lla(antenna_enu, ref_lla);
  gps_updater.UpdateEKF(ekf, time, gps_lla);

  state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 1);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 1);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 1);

  time += 1;
  antenna_enu = Eigen::Vector3d{2, 2, 2};
  gps_lla = enu_to_lla(antenna_enu, ref_lla);
  gps_updater.UpdateEKF(ekf, time, gps_lla);

  state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 2);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 2);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 2);

  time += 1;
  antenna_enu = Eigen::Vector3d{3, 3, 3};
  gps_lla = enu_to_lla(antenna_enu, ref_lla);
  gps_updater.UpdateEKF(ekf, time, gps_lla);

  state = ekf->m_state;
  EXPECT_EQ(state.body_state.pos_b_in_l[0], 3);
  EXPECT_EQ(state.body_state.pos_b_in_l[1], 3);
  EXPECT_EQ(state.body_state.pos_b_in_l[2], 3);

  time += 1;
  antenna_enu = Eigen::Vector3d{4, 4, 4};
  gps_lla = enu_to_lla(antenna_enu, ref_lla);
  gps_updater.UpdateEKF(ekf, time, gps_lla);

  state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 4, 1e-3);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 4, 1e-3);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 4, 1e-3);

  time += 1;
  antenna_enu = Eigen::Vector3d{5, 5, 5};
  gps_lla = enu_to_lla(antenna_enu, ref_lla);
  gps_updater.UpdateEKF(ekf, time, gps_lla);

  state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 5, 1e-3);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 5, 1e-3);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 5, 1e-3);

  time += 1;
  antenna_enu = Eigen::Vector3d{6, 6, 6};
  gps_lla = enu_to_lla(antenna_enu, ref_lla);
  gps_updater.UpdateEKF(ekf, time, gps_lla);

  state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 6, 1e-3);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 6, 1e-3);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 6, 1e-3);
}
