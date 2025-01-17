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

#include <eigen3/Eigen/Eigen>

#include "infrastructure/sim/truth_engine_cyclic.hpp"
#include "utility/custom_assertions.hpp"
#include "utility/sim/sim_rng.hpp"
#include "utility/type_helper.hpp"


TEST(test_TruthEngine, InheritedFunctions) {
  Eigen::Vector3d pos_frequency{1, 2, 3};
  Eigen::Vector3d ang_frequency{4, 5, 6};
  Eigen::Vector3d pos_offset{1, 2, 3};
  Eigen::Vector3d ang_offset{0.1, 0.2, 0.3};
  double pos_amplitude = 1.0;
  double ang_amplitude = 0.1;
  double stationary_time{0.0};
  double max_time{1.0};
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

  TruthEngineCyclic truth_engine_cyclic(
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

  Eigen::Vector3d imu_pos {1, 2, 3};
  Eigen::Quaterniond imu_ang_pos{-0.5, 0.5, -0.5, 0.5};
  Eigen::Vector3d imu_acc_bias{1, 2, 3};
  Eigen::Vector3d imu_gyro_bias{1, 2, 3};
  Eigen::Vector3d cam_pos {1, 2, 3};
  Eigen::Quaterniond cam_ang_pos{-0.5, 0.5, -0.5, 0.5};

  truth_engine_cyclic.SetImuPosition(1, imu_pos);
  truth_engine_cyclic.SetImuAngularPosition(1, imu_ang_pos);
  truth_engine_cyclic.SetImuAccelerometerBias(1, imu_acc_bias);
  truth_engine_cyclic.SetImuGyroscopeBias(1, imu_gyro_bias);
  truth_engine_cyclic.SetCameraPosition(2, cam_pos);
  truth_engine_cyclic.SetCameraAngularPosition(2, cam_ang_pos);

  Eigen::Vector3d imu_pos_out = truth_engine_cyclic.GetImuPosition(1);
  Eigen::Quaterniond imu_ang_pos_out = truth_engine_cyclic.GetImuAngularPosition(1);
  Eigen::Vector3d imu_acc_bias_out = truth_engine_cyclic.GetImuAccelerometerBias(1);
  Eigen::Vector3d imu_gyro_bias_out = truth_engine_cyclic.GetImuGyroscopeBias(1);
  Eigen::Vector3d cam_pos_out = truth_engine_cyclic.GetCameraPosition(2);
  Eigen::Quaterniond cam_ang_pos_out = truth_engine_cyclic.GetCameraAngularPosition(2);

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(imu_pos, imu_pos_out, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(imu_ang_pos, imu_ang_pos_out, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(imu_acc_bias, imu_acc_bias_out, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(imu_gyro_bias, imu_gyro_bias_out, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(cam_pos, cam_pos_out, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(cam_ang_pos, cam_ang_pos_out, 1e-6));

  Eigen::Vector3d board_pos {1, 2, 3};
  Eigen::Quaterniond board_ang{-0.5, 0.5, -0.5, 0.5};

  truth_engine_cyclic.SetBoardPosition(3, board_pos);
  truth_engine_cyclic.SetBoardOrientation(3, board_ang);

  Eigen::Vector3d board_pos_out = truth_engine_cyclic.GetBoardPosition(3);
  Eigen::Quaterniond board_ang_out = truth_engine_cyclic.GetBoardOrientation(3);

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(board_pos, board_pos_out, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(board_ang, board_ang_out, 1e-6));

  SimRNG rng;
  std::vector<cv::Point3d> features = truth_engine_cyclic.GetFeatures();
}

TEST(test_TruthEngineCyclic, Constructor) {
  Eigen::Vector3d pos_frequency{1, 2, 3};
  Eigen::Vector3d ang_frequency{4, 5, 6};
  Eigen::Vector3d pos_offset{1, 2, 3};
  Eigen::Vector3d ang_offset{0.1, 0.2, 0.3};
  double pos_amplitude = 1.0;
  double ang_amplitude = 0.1;
  double stationary_time{0.0};
  double max_time{1.0};
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

  TruthEngineCyclic truth_engine_cyclic(
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

  truth_engine_cyclic.SetBodyPosCycleFrequency(pos_frequency);
  truth_engine_cyclic.SetBodyAngCycleFrequency(ang_frequency);

  Eigen::Vector3d pos = truth_engine_cyclic.GetBodyPosition(1.0);
  Eigen::Vector3d vel = truth_engine_cyclic.GetBodyVelocity(1.0);
  Eigen::Vector3d acc = truth_engine_cyclic.GetBodyAcceleration(1.0);
  Eigen::Vector3d ang_rpy = truth_engine_cyclic.GetBodyRollPitchYaw(1.0);
  Eigen::Quaterniond ang_pos = truth_engine_cyclic.GetBodyAngularPosition(1.0);
  Eigen::Vector3d ang_vel = truth_engine_cyclic.GetBodyAngularRate(1.0);
  Eigen::Vector3d ang_acc = truth_engine_cyclic.GetBodyAngularAcceleration(1.0);

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(pos, pos_offset, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(vel, Eigen::Vector3d{0, 0, 0}, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(acc, Eigen::Vector3d{19.739208, 78.956835, 177.652879}, 1e-6));

  EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_rpy, ang_offset, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_vel, Eigen::Vector3d{0, 0, 0}, 1e-6));
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(ang_acc, Eigen::Vector3d{17.465062, 56.054351, 64.370144}, 1e-6));

  EXPECT_TRUE(
    EXPECT_EIGEN_NEAR(
      ang_pos,
      Eigen::Quaterniond{0.983347, 0.034270, 0.106020, 0.143572}, 1e-6));
}

TEST(test_TruthEngineCyclic, WriteTruthData) {
  Eigen::Vector3d pos_frequency{1, 2, 3};
  Eigen::Vector3d ang_frequency{4, 5, 6};
  Eigen::Vector3d pos_offset{1, 2, 3};
  Eigen::Vector3d ang_offset{0.1, 0.2, 0.3};
  double pos_amplitude = 1.0;
  double ang_amplitude = 0.1;
  double stationary_time {0.0};
  double max_time {1.0};
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

  TruthEngineCyclic truth_engine_cyclic(
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

  Eigen::Vector3d imu_pos {1, 2, 3};
  Eigen::Quaterniond imu_ang_pos{-0.5, 0.5, -0.5, 0.5};
  Eigen::Vector3d imu_acc_bias{1, 2, 3};
  Eigen::Vector3d imu_gyro_bias{1, 2, 3};
  Eigen::Vector3d cam_pos {1, 2, 3};
  Eigen::Quaterniond cam_ang_pos{-0.5, 0.5, -0.5, 0.5};
  Eigen::Vector3d board_pos {1, 2, 3};
  Eigen::Quaterniond board_ang{-0.5, 0.5, -0.5, 0.5};
  Eigen::Vector3d gps_pos {1, 2, 3};

  truth_engine_cyclic.SetImuPosition(1, imu_pos);
  truth_engine_cyclic.SetImuAngularPosition(1, imu_ang_pos);
  truth_engine_cyclic.SetImuAccelerometerBias(1, imu_acc_bias);
  truth_engine_cyclic.SetImuGyroscopeBias(1, imu_gyro_bias);
  truth_engine_cyclic.SetCameraPosition(2, cam_pos);
  truth_engine_cyclic.SetCameraAngularPosition(2, cam_ang_pos);
  truth_engine_cyclic.SetBoardPosition(3, board_pos);
  truth_engine_cyclic.SetBoardOrientation(3, board_ang);
  truth_engine_cyclic.SetGpsPosition(4, gps_pos);

  SimRNG rng;
  std::vector<cv::Point3d> features = truth_engine_cyclic.GetFeatures();

  truth_engine_cyclic.WriteTruthData(10.0, "");
}

TEST(test_TruthEngineCyclic, SetLocalPosition) {
  Eigen::Vector3d pos_frequency{1, 2, 3};
  Eigen::Vector3d ang_frequency{4, 5, 6};
  Eigen::Vector3d pos_offset{1, 2, 3};
  Eigen::Vector3d ang_offset{0.1, 0.2, 0.3};
  double pos_amplitude = 1.0;
  double ang_amplitude = 0.1;
  double stationary_time {0.0};
  double max_time {1.0};
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

  TruthEngineCyclic truth_engine_cyclic(
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

  double lla_heading{0.0};
  Eigen::Vector3d lla_reference{0, 0, 0};
  truth_engine_cyclic.SetLocalPosition(lla_reference);
  truth_engine_cyclic.SetLocalHeading(lla_heading);
  EXPECT_TRUE(EXPECT_EIGEN_NEAR(truth_engine_cyclic.GetLocalPosition(), lla_reference, 1e-6));
  EXPECT_EQ(truth_engine_cyclic.GetLocalHeading(), lla_heading);
}
