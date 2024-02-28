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
  Eigen::Vector3d pos_frequency{1, 2, 3};
  Eigen::Vector3d ang_frequency{4, 5, 6};
  Eigen::Vector3d pos_offset{1, 2, 3};
  Eigen::Vector3d ang_offset{0.1, 0.2, 0.3};
  double pos_amplitude = 1.0;
  double ang_amplitude = 0.1;
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

  auto truthEngine = std::make_shared<TruthEngineCyclic>(
    pos_frequency,
    ang_frequency,
    pos_offset,
    ang_offset,
    pos_amplitude,
    ang_amplitude,
    0.0,
    logger
  );

  IMU::Parameters imu_params;
  imu_params.rate = 100.0;

  SimIMU::Parameters sim_imu_params;
  sim_imu_params.imu_params = imu_params;

  SimIMU sim_imu(sim_imu_params, truthEngine);

  /// @todo(jhartzer): Verify the content of the messages
  std::vector<std::shared_ptr<SimImuMessage>> imu_messages = sim_imu.GenerateMessages(1.0);
}
