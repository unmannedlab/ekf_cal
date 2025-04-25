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

#include <gtest/gtest.h>

#include "trackers/fiducial_tracker.hpp"
#include "trackers/sim/sim_fiducial_tracker.hpp"
#include "infrastructure/sim/truth_engine_cyclic.hpp"

TEST(test_fiducial_tracker, constructor) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(ekf_params);

  Eigen::Vector3d pos_frequency {1, 2, 3};
  Eigen::Vector3d ang_frequency {1, 2, 3};
  Eigen::Vector3d pos_offset {0, 0, 0};
  Eigen::Vector3d ang_offset {0, 0, 0};
  double pos_amplitude {1.0};
  double ang_amplitude {0.1};
  double stationary_time {1.0};
  double max_time {1.0};

  auto truth_engine = std::make_shared<TruthEngineCyclic>(
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

  FiducialTracker::Parameters fiducial_params;
  fiducial_params.ekf = ekf;
  fiducial_params.is_extrinsic = true;
  fiducial_params.pos_f_in_l = Eigen::Vector3d{5, 0, 0};
  fiducial_params.ang_f_to_l = Eigen::Quaterniond{1, 0, 0, 0};
  SimFiducialTracker::Parameters sim_params;
  sim_params.fiducial_params = fiducial_params;
  SimFiducialTracker sim_fiducial_tracker(sim_params, truth_engine);
}
