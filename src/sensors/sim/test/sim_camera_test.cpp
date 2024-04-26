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
#include "sensors/camera.hpp"
#include "sensors/sim/sim_camera.hpp"
#include "trackers/feature_tracker.hpp"
#include "trackers/fiducial_tracker.hpp"
#include "trackers/sim/sim_feature_tracker.hpp"
#include "trackers/sim/sim_fiducial_tracker.hpp"

TEST(test_SimCamera, feature_track) {
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(logger, 0.0, false, "");
  Eigen::Vector3d pos_frequency{1, 2, 3};
  Eigen::Vector3d ang_frequency{4, 5, 6};
  Eigen::Vector3d pos_offset{1, 2, 3};
  Eigen::Vector3d ang_offset{0.1, 0.2, 0.3};
  double stationary_time{0.0};
  double max_time{1.0};
  double pos_amplitude = 1.0;
  double ang_amplitude = 0.1;

  auto truth_engine = std::make_shared<TruthEngineCyclic>(
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

  SimRNG rng;
  rng.SetSeed(1.0);
  truth_engine->GenerateFeatures(1000, 10, rng);

  Intrinsics intrinsics;
  intrinsics.F = 1.0;
  intrinsics.c_x = 320.0;
  intrinsics.c_y = 240.0;
  intrinsics.k_1 = 0.0;
  intrinsics.k_2 = 0.0;
  intrinsics.p_1 = 0.0;
  intrinsics.p_2 = 0.0;
  intrinsics.pixel_size = 1.0e-2;

  Camera::Parameters cam_params;
  cam_params.ekf = ekf;
  cam_params.logger = logger;
  cam_params.rate = 20.0;
  cam_params.intrinsics = intrinsics;
  cam_params.ang_c_to_b = Eigen::Quaterniond{-0.5, 0.5, -0.5, 0.5};

  SimCamera::Parameters sim_camera_params;
  sim_camera_params.cam_params = cam_params;
  sim_camera_params.no_errors = true;

  SimCamera sim_camera(sim_camera_params, truth_engine);

  FeatureTracker::Parameters feature_params;
  feature_params.ekf = ekf;
  feature_params.logger = logger;
  feature_params.camera_id = sim_camera.GetId();
  SimFeatureTracker::Parameters sim_feature_params;
  sim_feature_params.tracker_params = feature_params;
  auto feature_tracker = std::make_shared<SimFeatureTracker>(sim_feature_params, truth_engine);
  sim_camera.AddTracker(feature_tracker);

  std::vector<std::shared_ptr<SimCameraMessage>> cam_messages = sim_camera.GenerateMessages(rng);

  for (unsigned int i = 0; i < 5; ++i) {
    sim_camera.Callback(cam_messages[i]);
  }
}

TEST(test_SimCamera, fiducial_track) {
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

  auto truth_engine = std::make_shared<TruthEngineCyclic>(
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

  SimRNG rng;
  rng.SetSeed(1.0);
  truth_engine->GenerateFeatures(1000, 10, rng);

  Intrinsics intrinsics;
  intrinsics.F = 1.0;
  intrinsics.c_x = 320.0;
  intrinsics.c_y = 240.0;
  intrinsics.k_1 = 0.0;
  intrinsics.k_2 = 0.0;
  intrinsics.p_1 = 0.0;
  intrinsics.p_2 = 0.0;
  intrinsics.pixel_size = 1.0e-2;

  Camera::Parameters cam_params;
  cam_params.ekf = ekf;
  cam_params.logger = logger;
  cam_params.rate = 10.0;
  cam_params.intrinsics = intrinsics;
  cam_params.ang_c_to_b = Eigen::Quaterniond{-0.5, 0.5, -0.5, 0.5};

  SimCamera::Parameters sim_camera_params;
  sim_camera_params.cam_params = cam_params;

  SimCamera sim_camera(sim_camera_params, truth_engine);

  FiducialTracker::Parameters fiducial_params;
  fiducial_params.ekf = ekf;
  fiducial_params.logger = logger;
  SimFiducialTracker::Parameters sim_fiducial_params;
  sim_fiducial_params.fiducial_params = fiducial_params;
  auto fiducial_tracker = std::make_shared<SimFiducialTracker>(sim_fiducial_params, truth_engine);
  sim_camera.AddFiducial(fiducial_tracker);

  std::vector<std::shared_ptr<SimCameraMessage>> cam_messages =
    sim_camera.GenerateMessages(rng);

  for (auto cam_message : cam_messages) {
    sim_camera.Callback(cam_message);
  }
}
