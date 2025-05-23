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

#include "sensors/sim/sim_gps.hpp"
#include "sensors/gps.hpp"
#include "infrastructure/sim/truth_engine_cyclic.hpp"
#include "utility/gps_helper.hpp"

TEST(test_SimIMU, Constructor) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  GPS::Parameters gps_params;
  gps_params.name = "GPS_1";
  gps_params.topic = "GPS_1";
  gps_params.rate = 5.0;
  gps_params.pos_a_in_b = Eigen::Vector3d{0, 0, 0};
  gps_params.variance = Eigen::Vector3d{5.0, 5.0, 5.0};
  gps_params.ekf = ekf;
  gps_params.logger = ekf_params.debug_logger;

  SimGPS::Parameters sim_gps_params;
  sim_gps_params.lla_error = Eigen::Vector3d{5.0, 5.0, 5.0};
  sim_gps_params.pos_e_in_g_err = Eigen::Vector3d{0.0, 0.0, 0.0};
  sim_gps_params.ang_l_to_e_err = 0.0;
  sim_gps_params.gps_params = gps_params;

  Eigen::Vector3d pos_frequency{1, 2, 3};
  Eigen::Vector3d ang_frequency{1, 2, 3};
  Eigen::Vector3d pos_offset{0, 0, 0};
  Eigen::Vector3d ang_offset{0, 0, 0};
  double pos_amplitude{1.0};
  double ang_amplitude{0.1};
  double stationary_time{1.0};
  double max_time{1.0};

  auto truth_engine_cyclic = std::make_shared<TruthEngineCyclic>(
    pos_frequency,
    ang_frequency,
    pos_offset,
    ang_offset,
    pos_amplitude,
    ang_amplitude,
    stationary_time,
    max_time,
    ekf_params.debug_logger
  );

  auto truth_engine = std::static_pointer_cast<TruthEngine>(truth_engine_cyclic);
  SimGPS sim_gps(sim_gps_params, truth_engine);
  truth_engine->SetGpsPosition(sim_gps.GetId(), Eigen::Vector3d{0, 0, 0});
  truth_engine->SetLocalPosition(Eigen::Vector3d{0, 0, 0});
  truth_engine->SetLocalHeading(0.0);

  SimRNG::SetSeed(1);
  auto gps_msgs = sim_gps.GenerateMessages();

  EXPECT_NEAR(gps_msgs[1]->time - gps_msgs[0]->time, 0.2, 1e-3);
  EXPECT_NEAR(gps_msgs[2]->time - gps_msgs[1]->time, 0.2, 1e-3);
  EXPECT_NEAR(gps_msgs[3]->time - gps_msgs[2]->time, 0.2, 1e-3);
  EXPECT_NEAR(gps_msgs[4]->time - gps_msgs[3]->time, 0.2, 1e-3);

  Eigen::Vector3d lla_ref = Eigen::Vector3d::Zero();

  Eigen::Vector3d enu_0 = lla_to_enu(gps_msgs[0]->gps_lla, lla_ref);
  Eigen::Vector3d enu_1 = lla_to_enu(gps_msgs[1]->gps_lla, lla_ref);
  Eigen::Vector3d enu_2 = lla_to_enu(gps_msgs[2]->gps_lla, lla_ref);
  Eigen::Vector3d enu_3 = lla_to_enu(gps_msgs[3]->gps_lla, lla_ref);
  Eigen::Vector3d enu_4 = lla_to_enu(gps_msgs[4]->gps_lla, lla_ref);

  EXPECT_NEAR(enu_0[0], -2.069, 1e-3);
  EXPECT_NEAR(enu_0[1], -4.190, 1e-3);
  EXPECT_NEAR(enu_0[2], -0.588, 1e-3);

  EXPECT_NEAR(enu_1[0], -4.642, 1e-3);
  EXPECT_NEAR(enu_1[1], -1.370, 1e-3);
  EXPECT_NEAR(enu_1[2], -5.420, 1e-3);

  EXPECT_NEAR(enu_2[0], 0.973, 1e-3);
  EXPECT_NEAR(enu_2[1], 5.834, 1e-3);
  EXPECT_NEAR(enu_2[2], 3.286, 1e-3);

  EXPECT_NEAR(enu_3[0], -6.216, 1e-3);
  EXPECT_NEAR(enu_3[1], 4.574, 1e-3);
  EXPECT_NEAR(enu_3[2], -1.977, 1e-3);

  EXPECT_NEAR(enu_4[0], 3.058, 1e-3);
  EXPECT_NEAR(enu_4[1], 0.735, 1e-3);
  EXPECT_NEAR(enu_4[2], 0.027, 1e-3);
}
