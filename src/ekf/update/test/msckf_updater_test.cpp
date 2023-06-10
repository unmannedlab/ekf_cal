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

#include "ekf/ekf.hpp"
#include "ekf/update/msckf_updater.hpp"
#include "ekf/constants.hpp"

TEST(test_msckf_updater, TriangulateFeature) {
  EKF * ekf = EKF::GetInstance();

  double time_init = 0.0;
  BodyState body_state;
  body_state.m_velocity = Eigen::Vector3d::Ones();
  ekf->Initialize(time_init, body_state);

  unsigned int cam_id{1};
  std::string log_file_directory{""};
  bool data_logging_on {true};

  CamState cam_state;
  Eigen::MatrixXd cam_cov = Eigen::MatrixXd::Zero(6, 6);
  ekf->RegisterCamera(cam_id, cam_state, cam_cov);

  MsckfUpdater msckf_updater(cam_id, log_file_directory, data_logging_on);

  // msckf_updater.TriangulateFeature()
}
