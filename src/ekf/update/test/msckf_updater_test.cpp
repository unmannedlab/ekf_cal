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

#include <string>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "ekf/update/msckf_updater.hpp"
#include "utility/custom_assertions.hpp"


TEST(test_msckf_updater, projection_jacobian) {
  Eigen::Vector3d base_pos {1, 2, 3};
  Eigen::MatrixXd jac_analytic = Eigen::MatrixXd::Zero(2, 3);
  MsckfUpdater::ProjectionJacobian(base_pos, jac_analytic);

  double delta = 1.0e-6;
  Eigen::MatrixXd jac_numerical = Eigen::MatrixXd::Zero(2, 3);
  Eigen::Vector2d base_meas = MsckfUpdater::Project(base_pos);
  for (unsigned int i = 0; i < 3; ++i) {
    Eigen::Vector3d delta_pos = base_pos;
    delta_pos[i] += delta;
    jac_numerical.block<2, 1>(0, i) = (MsckfUpdater::Project(delta_pos) - base_meas) / delta;
  }

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(jac_analytic, jac_numerical, 1e-3));
}


TEST(test_msckf_updater, distortion_jacobian) {
  Eigen::Vector2d xy_norm {1, 2};
  Intrinsics intrinsics;
  Eigen::MatrixXd jac_analytic = Eigen::MatrixXd::Zero(2, 2);
  MsckfUpdater::DistortionJacobian(xy_norm, intrinsics, jac_analytic);

  double delta = 1.0e-6;
  Eigen::MatrixXd jac_numerical = Eigen::MatrixXd::Zero(2, 2);
  Eigen::Vector2d base_meas = MsckfUpdater::Distort(xy_norm, intrinsics);
  for (unsigned int i = 0; i < 2; ++i) {
    Eigen::Vector2d delta_pos = xy_norm;
    delta_pos[i] += delta;
    jac_numerical.block<2, 1>(0, i) =
      (MsckfUpdater::Distort(delta_pos, intrinsics) - base_meas) / delta;
  }

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(jac_analytic, jac_numerical, 1e-3));
}


TEST(test_msckf_updater, update) {
  EKF::Parameters ekf_params;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  EKF ekf(ekf_params);
  BodyState body_state;
  body_state.vel_b_in_l = Eigen::Vector3d{0, 5, 0};
  ekf.Initialize(0.0, body_state);

  unsigned int cam_id{1};

  CamState cam_state;
  Eigen::MatrixXd cam_cov = Eigen::MatrixXd::Zero(6, 6);
  ekf.RegisterCamera(cam_id, cam_state, cam_cov);

  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto msckf_updater = MsckfUpdater(1, false, "", 0.0, 1.0, logger);

  double time {0.3};
  cv::KeyPoint point_1;
  cv::KeyPoint point_2;
  cv::KeyPoint point_3;
  point_1.pt.x = 320;
  point_1.pt.y = 240;
  point_2.pt.x = 220;
  point_2.pt.y = 240;
  point_3.pt.x = 120;
  point_3.pt.y = 240;

  FeaturePoint feature_point_1;
  FeaturePoint feature_point_2;
  FeaturePoint feature_point_3;
  feature_point_1.frame_id = 1;
  feature_point_1.key_point = point_1;
  feature_point_2.frame_id = 2;
  feature_point_2.key_point = point_2;
  feature_point_3.frame_id = 3;
  feature_point_3.key_point = point_3;

  FeatureTrack feature_points;
  feature_points.track.push_back(feature_point_1);
  feature_points.track.push_back(feature_point_2);
  feature_points.track.push_back(feature_point_3);

  FeatureTracks feature_tracks;
  feature_tracks.push_back(feature_points);

  ekf.PredictModel(0.1);
  ekf.AugmentStateIfNeeded(cam_id, feature_point_1.frame_id);
  ekf.PredictModel(0.2);
  ekf.AugmentStateIfNeeded(cam_id, feature_point_2.frame_id);
  ekf.PredictModel(0.3);
  ekf.AugmentStateIfNeeded(cam_id, feature_point_3.frame_id);

  msckf_updater.UpdateEKF(ekf, time, feature_tracks, 1e-3);
}
