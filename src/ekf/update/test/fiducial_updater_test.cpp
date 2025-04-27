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

#include "ekf/update/fiducial_updater.hpp"

TEST(test_fiducial_updater, constructor) {
  unsigned int fid_id {0};
  unsigned int cam_id {1};
  Eigen::Vector3d fiducial_pos{0.0, 0.0, 0.0};
  Eigen::Quaterniond fiducial_ang{1.0, 0.0, 0.0, 0.0};
  const std::string & log_file_directory("");
  bool is_cam_extrinsic{false};
  double data_log_rate{1.0};
  auto debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = debug_logger;

  FiducialUpdater fiducial_updater(
    fid_id,
    cam_id,
    is_cam_extrinsic,
    log_file_directory,
    data_log_rate,
    debug_logger);

  EKF ekf(ekf_params);

  CamState cam_state;
  cam_state.SetIsExtrinsic(is_cam_extrinsic);
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(6, 6) * 1e-3;
  ekf.RegisterCamera(cam_id, cam_state, covariance);

  BoardDetection board_detection;
  board_detection.frame_id = 0;
  board_detection.pos_f_in_c = Eigen::Vector3d{5, 0, 0};
  board_detection.ang_f_to_c = Eigen::Quaterniond{1, 0, 0, 0};
  board_detection.pos_error = Eigen::Vector3d{0.1, 0.1, 0.1};
  board_detection.ang_error = Eigen::Vector3d{0.1, 0.1, 0.1};

  fiducial_updater.UpdateEKF(ekf, 0.0, board_detection);
}
