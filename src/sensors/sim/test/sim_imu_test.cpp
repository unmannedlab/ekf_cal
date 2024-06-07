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

#include "infrastructure/sim/truth_engine_cyclic.hpp"
#include "sensors/imu.hpp"
#include "sensors/sim/sim_imu.hpp"

TEST(test_SimIMU, Constructor) {
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(logger, 0.0, false, "");
  Eigen::Vector3d pos_frequency{1, 2, 3};
  Eigen::Vector3d ang_frequency{4, 5, 6};
  Eigen::Vector3d pos_offset{1, 2, 3};
  Eigen::Vector3d ang_offset{0.1, 0.2, 0.3};
  double pos_amplitude = 1.0;
  double ang_amplitude = 0.1;
  double stationary_time{0.0};
  double max_time{1.0};

  auto truthEngine = std::make_shared<TruthEngineCyclic>(
    pos_frequency,
    ang_frequency,
    pos_offset,
    ang_offset,
    pos_amplitude,
    ang_amplitude,
    stationary_time,
    max_time,
    logger
  );

  IMU::Parameters imu_params;
  imu_params.rate = 100.0;
  imu_params.ekf = ekf;
  imu_params.logger = logger;

  SimIMU::Parameters sim_imu_params;
  sim_imu_params.imu_params = imu_params;

  SimIMU sim_imu(sim_imu_params, truthEngine);
  SimRNG rng;
  rng.SetSeed(1.0);
  std::vector<std::shared_ptr<SimImuMessage>> imu_messages = sim_imu.GenerateMessages();
}
