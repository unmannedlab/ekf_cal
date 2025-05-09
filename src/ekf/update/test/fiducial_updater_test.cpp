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
#include "utility/custom_assertions.hpp"

TEST(test_fiducial_updater, constructor) {
  unsigned int fid_id {0};
  unsigned int cam_id {1};
  Eigen::Vector3d fiducial_pos{0.0, 0.0, 0.0};
  Eigen::Quaterniond fiducial_ang{1.0, 0.0, 0.0, 0.0};
  const std::string & log_file_directory("");
  bool is_fid_extrinsic{false};
  bool is_cam_extrinsic{false};
  double data_log_rate{1.0};
  auto debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = debug_logger;

  FiducialUpdater fiducial_updater(
    fid_id,
    cam_id,
    is_fid_extrinsic,
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

TEST(test_fiducial_updater, jacobian) {
  auto debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = debug_logger;
  EKF ekf(ekf_params);

  unsigned int cam_id{0};
  bool is_cam_extrinsic{true};
  CamState cam_state;
  cam_state.SetIsExtrinsic(is_cam_extrinsic);
  Eigen::MatrixXd cam_cov = Eigen::MatrixXd::Identity(6, 6);
  ekf.RegisterCamera(cam_id, cam_state, cam_cov);

  unsigned int fid_id{1};
  bool is_fid_extrinsic{true};
  FidState fid_state;
  fid_state.id = fid_id;
  fid_state.pos_f_in_l = Eigen::Vector3d{2, 3, 5};
  fid_state.ang_f_to_l = Eigen::Quaterniond{1, 0, 0, 0};
  fid_state.SetIsExtrinsic(is_cam_extrinsic);
  Eigen::MatrixXd fid_cov = Eigen::MatrixXd::Identity(6, 6);
  ekf.RegisterFiducial(fid_state, fid_cov);

  auto fid_updater =
    FiducialUpdater(
    fid_id, cam_id, is_cam_extrinsic, is_fid_extrinsic, "log_file_directory", 0.0, debug_logger);

  Eigen::VectorXd base_state = ekf.m_state.ToVector();
  Eigen::MatrixXd jac_analytical = fid_updater.GetMeasurementJacobian(ekf);
  Eigen::VectorXd base_meas = fid_updater.PredictMeasurement(ekf);
  double delta = 1.0e-6;
  unsigned int jac_size = base_state.size();
  Eigen::MatrixXd jac_numerical = Eigen::MatrixXd::Zero(6, jac_size);

  for (unsigned int i = 0; i < jac_size; ++i) {
    Eigen::VectorXd delta_vec = base_state;
    delta_vec[i] = delta_vec[i] + delta;

    ekf.m_state.SetState(delta_vec);
    Eigen::VectorXd curr = fid_updater.PredictMeasurement(ekf);
    Eigen::VectorXd diff = curr - base_meas;
    jac_numerical.block<6, 1>(0, i) = (diff) / delta;
  }

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(jac_analytical, jac_numerical, 1e-3));
}
