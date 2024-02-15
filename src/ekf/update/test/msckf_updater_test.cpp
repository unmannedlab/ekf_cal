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
#include "sensors/types.hpp"

///
/// @class test_msckf_updater
/// @brief test harness for msckf_updater
///
class test_msckf_updater : public ::testing::Test
{
protected:
  ///
  /// @brief Setup method for msckf_updater test harness
  ///
  void SetUp() override
  {
    EKF * ekf = EKF::GetInstance();

    double time_init = 0.0;
    BodyState body_state;
    body_state.m_velocity = Eigen::Vector3d::Ones();
    ekf->Initialize(time_init, body_state);

    unsigned int cam_id{1};
    Intrinsics intrinsics;
    std::string log_file_directory{""};
    bool data_logging_on {true};

    CamState cam_state;
    Eigen::MatrixXd cam_cov = Eigen::MatrixXd::Zero(6, 6);
    ekf->RegisterCamera(cam_id, cam_state, cam_cov);

    msckf_updater = MsckfUpdater(cam_id, intrinsics, log_file_directory, data_logging_on, 0.0, 1.0);
  }

  /// @brief msckf_updater class for testing
  MsckfUpdater msckf_updater{0, Intrinsics(), "", false, 0.0, 1.0};
};

TEST_F(test_msckf_updater, projection_jacobian) {
  Eigen::Vector3d position{2, 3, 4};
  Eigen::MatrixXd jacobian(2, 3);
  msckf_updater.projection_jacobian(position, jacobian);
  EXPECT_EQ(jacobian(0, 0), 1.0 / 4.0);
  EXPECT_EQ(jacobian(0, 1), 0);
  EXPECT_EQ(jacobian(0, 2), -2.0 / 4.0 / 4.0);
  EXPECT_EQ(jacobian(1, 0), 0);
  EXPECT_EQ(jacobian(1, 1), 1.0 / 4.0);
  EXPECT_EQ(jacobian(1, 2), -3.0 / 4.0 / 4.0);
}

TEST_F(test_msckf_updater, distortion_jacobian) {
  Eigen::Vector2d uv_norm;
  uv_norm << 1, 2;
  Intrinsics intrinsics;
  intrinsics.f_x = 1;
  intrinsics.f_y = 1;
  intrinsics.k_1 = 0.0;
  intrinsics.k_2 = 0.0;
  intrinsics.p_1 = 0.0;
  intrinsics.p_2 = 0.0;

  Eigen::MatrixXd jacobian;

  msckf_updater.distortion_jacobian(uv_norm, intrinsics, jacobian);

  EXPECT_EQ(jacobian(0, 0), 1);
  EXPECT_EQ(jacobian(0, 1), 0);
  EXPECT_EQ(jacobian(1, 0), 0);
  EXPECT_EQ(jacobian(1, 1), 1);
}
