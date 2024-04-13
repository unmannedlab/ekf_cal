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
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(logger, 0.0, false, "");

  GPS::Parameters gps_params;
  gps_params.name = "GPS_1";
  gps_params.topic = "GPS_1";
  gps_params.rate = 5.0;
  gps_params.pos_a_in_b = Eigen::Vector3d{0, 0, 0};
  gps_params.variance = Eigen::Vector3d{5.0, 5.0, 5.0};
  gps_params.ekf = ekf;
  gps_params.logger = logger;

  SimGPS::Parameters sim_gps_params;
  sim_gps_params.pos_a_in_b = Eigen::Vector3d{0.0, 0.0, 0.0};
  sim_gps_params.gps_error = Eigen::Vector3d{5.0, 5.0, 5.0};
  sim_gps_params.pos_l_in_g = Eigen::Vector3d{0.0, 0.0, 0.0};
  sim_gps_params.ang_l_to_g = 0.0;
  sim_gps_params.gps_params = gps_params;

  Eigen::Vector3d pos_frequency{1, 2, 3};
  Eigen::Vector3d ang_frequency{1, 2, 3};
  Eigen::Vector3d pos_offset{0, 0, 0};
  Eigen::Vector3d ang_offset{0, 0, 0};
  double pos_amplitude{1.0};
  double ang_amplitude{0.1};
  double stationary_time{1.0};

  auto truth_engine_cyclic = std::make_shared<TruthEngineCyclic>(
    pos_frequency,
    ang_frequency,
    pos_offset,
    ang_offset,
    pos_amplitude,
    ang_amplitude,
    stationary_time,
    logger
  );
  auto truth_engine = std::static_pointer_cast<TruthEngine>(truth_engine_cyclic);

  SimGPS sim_gps(sim_gps_params, truth_engine);
  SimRNG rng;
  rng.SetSeed(0.0);
  auto gps_msgs = sim_gps.GenerateMessages(rng, 1.0);

  EXPECT_NEAR(gps_msgs[1]->m_time - gps_msgs[0]->m_time, 0.2, 1e-3);
  EXPECT_NEAR(gps_msgs[2]->m_time - gps_msgs[1]->m_time, 0.2, 1e-3);
  EXPECT_NEAR(gps_msgs[3]->m_time - gps_msgs[2]->m_time, 0.2, 1e-3);
  EXPECT_NEAR(gps_msgs[4]->m_time - gps_msgs[3]->m_time, 0.2, 1e-3);


  Eigen::Vector3d lla_ref, lla_0, lla_1, lla_2, lla_3, lla_4;
  lla_ref[0] = 0.0;
  lla_ref[1] = 0.0;
  lla_ref[2] = 0.0;

  lla_0[0] = gps_msgs[0]->m_latitude;
  lla_0[1] = gps_msgs[0]->m_longitude;
  lla_0[2] = gps_msgs[0]->m_altitude;

  lla_1[0] = gps_msgs[1]->m_latitude;
  lla_1[1] = gps_msgs[1]->m_longitude;
  lla_1[2] = gps_msgs[1]->m_altitude;

  lla_2[0] = gps_msgs[2]->m_latitude;
  lla_2[1] = gps_msgs[2]->m_longitude;
  lla_2[2] = gps_msgs[2]->m_altitude;

  lla_3[0] = gps_msgs[3]->m_latitude;
  lla_3[1] = gps_msgs[3]->m_longitude;
  lla_3[2] = gps_msgs[3]->m_altitude;

  lla_4[0] = gps_msgs[4]->m_latitude;
  lla_4[1] = gps_msgs[4]->m_longitude;
  lla_4[2] = gps_msgs[4]->m_altitude;

  Eigen::Vector3d enu_0 = lla_to_enu(lla_0, lla_ref);
  Eigen::Vector3d enu_1 = lla_to_enu(lla_1, lla_ref);
  Eigen::Vector3d enu_2 = lla_to_enu(lla_2, lla_ref);
  Eigen::Vector3d enu_3 = lla_to_enu(lla_3, lla_ref);
  Eigen::Vector3d enu_4 = lla_to_enu(lla_4, lla_ref);

  EXPECT_NEAR(enu_0[0], 4.951, 1e-3);
  EXPECT_NEAR(enu_0[1], 0.800, 1e-3);
  EXPECT_NEAR(enu_0[2], 5.812, 1e-3);

  EXPECT_NEAR(enu_1[0], -0.809, 1e-3);
  EXPECT_NEAR(enu_1[1], 2.797, 1e-3);
  EXPECT_NEAR(enu_1[2], -6.221, 1e-3);

  EXPECT_NEAR(enu_2[0], 7.125, 1e-3);
  EXPECT_NEAR(enu_2[1], 2.682, 1e-3);
  EXPECT_NEAR(enu_2[2], -3.747, 1e-3);

  EXPECT_NEAR(enu_3[0], 7.956, 1e-3);
  EXPECT_NEAR(enu_3[1], -0.508, 1e-3);
  EXPECT_NEAR(enu_3[2], 0.208, 1e-3);

  EXPECT_NEAR(enu_4[0], -4.668, 1e-3);
  EXPECT_NEAR(enu_4[1], -0.527, 1e-3);
  EXPECT_NEAR(enu_4[2], 5.694, 1e-3);
}
