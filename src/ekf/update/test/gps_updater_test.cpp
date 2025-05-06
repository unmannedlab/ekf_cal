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
#include "utility/custom_assertions.hpp"
#include "utility/gps_helper.hpp"
#include "utility/type_helper.hpp"

TEST(test_gps_updater, update) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  double time_init = 0.0;
  BodyState body_state;
  body_state.vel_b_in_l = Eigen::Vector3d::Ones();
  ekf->Initialize(time_init, body_state);

  unsigned int gps_id{0};
  bool is_extrinsic{false};

  GpsState gps_state;
  gps_state.SetIsExtrinsic(is_extrinsic);
  Eigen::Matrix3d gps_cov = Eigen::Matrix3d::Zero(3, 3);
  ekf->RegisterGPS(gps_id, gps_state, gps_cov);

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  GpsUpdater gps_updater(gps_id, is_extrinsic, "", 0.0, logger);
  Eigen::Matrix3d pos_cov = Eigen::Matrix3d::Identity() * 1e-9;

  State state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 0, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 0, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 0, 1e-2);

  double time = time_init + 1;
  Eigen::Vector3d ref_lla{0, 0, 0};
  Eigen::Vector3d antenna_enu{1, 1, 1};
  Eigen::Vector3d gps_lla = enu_to_lla(antenna_enu, ref_lla);
  gps_updater.UpdateEKF(*ekf, time, gps_lla, pos_cov);

  state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 1.0, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 1.0, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 1.0, 1e-2);

  time += 1;
  antenna_enu = Eigen::Vector3d{2, 2, 2};
  gps_lla = enu_to_lla(antenna_enu, ref_lla);
  gps_updater.UpdateEKF(*ekf, time, gps_lla, pos_cov);

  state = ekf->m_state;
  EXPECT_NEAR(state.body_state.pos_b_in_l[0], 1.5, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[1], 1.5, 1e-2);
  EXPECT_NEAR(state.body_state.pos_b_in_l[2], 1.5, 1e-2);
}

TEST(test_gps_updater, jacobian) {
  unsigned int gps_id{0};
  bool is_extrinsic{true};
  GpsState gps_state;
  gps_state.pos_a_in_b = Eigen::Vector3d{1, 2, 3};
  gps_state.SetIsExtrinsic(is_extrinsic);

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

  auto debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = debug_logger;
  EKF ekf(ekf_params);
  ekf.RegisterGPS(gps_id, gps_state, covariance);

  auto gps_updater = GpsUpdater(gps_id, is_extrinsic, "log_file_directory", 0.0, debug_logger);

  Eigen::VectorXd base_state = ekf.m_state.ToVector();
  Eigen::MatrixXd jac_analytical = gps_updater.GetMeasurementJacobian(ekf);
  Eigen::Vector3d base_meas = gps_updater.PredictMeasurement(ekf);

  double delta = 1.0e-6;
  unsigned int jac_size = base_state.size();
  Eigen::MatrixXd jac_numerical = Eigen::MatrixXd::Zero(3, jac_size);
  for (unsigned int i = 0; i < jac_size; ++i) {
    Eigen::VectorXd delta_vec = base_state;
    delta_vec[i] += delta;
    ekf.m_state.SetState(delta_vec);
    jac_numerical.block<3, 1>(0, i) = (gps_updater.PredictMeasurement(ekf) - base_meas) / delta;
  }

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(jac_analytical, jac_numerical, 1e-3));
}
