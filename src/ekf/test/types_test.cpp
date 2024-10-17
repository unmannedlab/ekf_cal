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

#include <map>
#include <memory>
#include <vector>

#include "ekf/types.hpp"

TEST(test_ekf_types, state_plus_equals_state) {
  Eigen::Quaterniond quat;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  State left_state;
  left_state.body_state.pos_b_in_l = Eigen::Vector3d::Ones() * 1.0;
  left_state.body_state.vel_b_in_l = Eigen::Vector3d::Ones() * 2.0;
  left_state.body_state.ang_b_to_l = quat;

  ImuState imu_state;
  imu_state.SetIsExtrinsic(true);
  imu_state.SetIsIntrinsic(true);
  imu_state.pos_i_in_b = Eigen::Vector3d::Ones() * 6.0;
  imu_state.ang_i_to_b = quat;
  imu_state.acc_bias = Eigen::Vector3d::Ones() * 7.0;
  imu_state.omg_bias = Eigen::Vector3d::Ones() * 8.0;

  GpsState gps_state;
  gps_state.SetIsExtrinsic(true);
  gps_state.pos_a_in_b = Eigen::Vector3d::Ones() * 9.0;

  CamState cam_state;
  cam_state.pos_c_in_b = Eigen::Vector3d::Ones() * 10.0;
  cam_state.ang_c_to_b = quat;

  FidState fid_state;
  fid_state.pos_f_in_l = Eigen::Vector3d::Ones() * 12.0;
  fid_state.ang_f_to_l = quat;

  AugState aug_state;
  aug_state.pos_b_in_l = Eigen::Vector3d::Ones() * 14.0;
  aug_state.ang_b_to_l = quat;

  left_state.imu_states[1] = imu_state;
  left_state.gps_states[2] = gps_state;
  left_state.cam_states[3] = cam_state;
  left_state.fid_states[4] = fid_state;
  left_state.aug_states[0].push_back(aug_state);

  left_state += left_state;

  EXPECT_EQ(left_state.body_state.pos_b_in_l(0), 2.0);
  EXPECT_EQ(left_state.body_state.pos_b_in_l(1), 2.0);
  EXPECT_EQ(left_state.body_state.pos_b_in_l(2), 2.0);

  EXPECT_EQ(left_state.body_state.vel_b_in_l(0), 4.0);
  EXPECT_EQ(left_state.body_state.vel_b_in_l(1), 4.0);
  EXPECT_EQ(left_state.body_state.vel_b_in_l(2), 4.0);


  EXPECT_EQ(left_state.body_state.ang_b_to_l.w(), 1.0);
  EXPECT_EQ(left_state.body_state.ang_b_to_l.x(), 0.0);
  EXPECT_EQ(left_state.body_state.ang_b_to_l.y(), 0.0);
  EXPECT_EQ(left_state.body_state.ang_b_to_l.z(), 0.0);

  EXPECT_EQ(left_state.imu_states[1].pos_i_in_b(0), 12.0);
  EXPECT_EQ(left_state.imu_states[1].pos_i_in_b(1), 12.0);
  EXPECT_EQ(left_state.imu_states[1].pos_i_in_b(2), 12.0);

  EXPECT_EQ(left_state.imu_states[1].ang_i_to_b.w(), 1.0);
  EXPECT_EQ(left_state.imu_states[1].ang_i_to_b.x(), 0.0);
  EXPECT_EQ(left_state.imu_states[1].ang_i_to_b.y(), 0.0);
  EXPECT_EQ(left_state.imu_states[1].ang_i_to_b.z(), 0.0);

  EXPECT_EQ(left_state.imu_states[1].acc_bias(0), 14.0);
  EXPECT_EQ(left_state.imu_states[1].acc_bias(1), 14.0);
  EXPECT_EQ(left_state.imu_states[1].acc_bias(2), 14.0);

  EXPECT_EQ(left_state.imu_states[1].omg_bias(0), 16.0);
  EXPECT_EQ(left_state.imu_states[1].omg_bias(1), 16.0);
  EXPECT_EQ(left_state.imu_states[1].omg_bias(2), 16.0);

  EXPECT_EQ(left_state.gps_states[2].pos_a_in_b(0), 18.0);
  EXPECT_EQ(left_state.gps_states[2].pos_a_in_b(1), 18.0);
  EXPECT_EQ(left_state.gps_states[2].pos_a_in_b(2), 18.0);

  EXPECT_EQ(left_state.cam_states[3].pos_c_in_b(0), 20.0);
  EXPECT_EQ(left_state.cam_states[3].pos_c_in_b(1), 20.0);
  EXPECT_EQ(left_state.cam_states[3].pos_c_in_b(2), 20.0);

  EXPECT_EQ(left_state.cam_states[3].ang_c_to_b.w(), 1.0);
  EXPECT_EQ(left_state.cam_states[3].ang_c_to_b.x(), 0.0);
  EXPECT_EQ(left_state.cam_states[3].ang_c_to_b.y(), 0.0);
  EXPECT_EQ(left_state.cam_states[3].ang_c_to_b.z(), 0.0);

  EXPECT_EQ(left_state.fid_states[4].pos_f_in_l(0), 24.0);
  EXPECT_EQ(left_state.fid_states[4].pos_f_in_l(1), 24.0);
  EXPECT_EQ(left_state.fid_states[4].pos_f_in_l(2), 24.0);

  EXPECT_EQ(left_state.fid_states[4].ang_f_to_l.w(), 1.0);
  EXPECT_EQ(left_state.fid_states[4].ang_f_to_l.x(), 0.0);
  EXPECT_EQ(left_state.fid_states[4].ang_f_to_l.y(), 0.0);
  EXPECT_EQ(left_state.fid_states[4].ang_f_to_l.z(), 0.0);

  EXPECT_EQ(left_state.aug_states[0][0].pos_b_in_l(0), 28.0);
  EXPECT_EQ(left_state.aug_states[0][0].pos_b_in_l(1), 28.0);
  EXPECT_EQ(left_state.aug_states[0][0].pos_b_in_l(2), 28.0);

  EXPECT_EQ(left_state.aug_states[0][0].ang_b_to_l.w(), 1.0);
  EXPECT_EQ(left_state.aug_states[0][0].ang_b_to_l.x(), 0.0);
  EXPECT_EQ(left_state.aug_states[0][0].ang_b_to_l.y(), 0.0);
  EXPECT_EQ(left_state.aug_states[0][0].ang_b_to_l.z(), 0.0);
}

TEST(test_ekf_types, state_plus_equals_vector) {
  Eigen::Quaterniond quat;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  State left_state;
  left_state.body_state.pos_b_in_l = Eigen::Vector3d::Ones() * 1.0;
  left_state.body_state.vel_b_in_l = Eigen::Vector3d::Ones() * 2.0;
  left_state.body_state.ang_b_to_l = quat;

  ImuState imu_state;
  imu_state.SetIsIntrinsic(true);
  imu_state.SetIsExtrinsic(true);
  imu_state.pos_i_in_b = Eigen::Vector3d::Ones() * 6.0;
  imu_state.ang_i_to_b = quat;
  imu_state.acc_bias = Eigen::Vector3d::Ones() * 7.0;
  imu_state.omg_bias = Eigen::Vector3d::Ones() * 8.0;

  GpsState gps_state;
  gps_state.SetIsExtrinsic(true);
  gps_state.pos_a_in_b = Eigen::Vector3d::Ones() * 9.0;

  CamState cam_state;
  cam_state.pos_c_in_b = Eigen::Vector3d::Ones() * 10.0;
  cam_state.ang_c_to_b = quat;

  FidState fid_state;
  fid_state.SetIsExtrinsic(true);
  fid_state.pos_f_in_l = Eigen::Vector3d::Ones() * 11.0;
  fid_state.ang_f_to_l = quat;

  AugState aug_state;
  aug_state.pos_b_in_l = Eigen::Vector3d::Ones() * 12.0;
  aug_state.ang_b_to_l = quat;

  left_state.imu_states[1] = imu_state;
  left_state.gps_states[2] = gps_state;
  left_state.cam_states[3] = cam_state;
  left_state.fid_states[4] = fid_state;
  left_state.aug_states[0].push_back(aug_state);

  Eigen::VectorXd state_vector = Eigen::VectorXd::Zero(51);
  // Body States
  state_vector.segment<3>(0) = Eigen::Vector3d::Ones() * 1.0;
  state_vector.segment<3>(3) = Eigen::Vector3d::Ones() * 2.0;
  state_vector.segment<3>(6) = Eigen::Vector3d::Ones() * 3.0;
  state_vector.segment<3>(12) = Eigen::Vector3d::Ones() * 4.0;
  state_vector.segment<3>(15) = Eigen::Vector3d::Ones() * 5.0;
  // IMU States
  state_vector.segment<3>(18) = Eigen::Vector3d::Ones() * 6.0;
  state_vector.segment<3>(24) = Eigen::Vector3d::Ones() * 7.0;
  state_vector.segment<3>(27) = Eigen::Vector3d::Ones() * 8.0;
  // GPS States
  state_vector.segment<3>(30) = Eigen::Vector3d::Ones() * 9.0;
  // Camera States
  state_vector.segment<3>(33) = Eigen::Vector3d::Ones() * 10.0;
  // Fiducial States
  state_vector.segment<3>(39) = Eigen::Vector3d::Ones() * 11.0;
  // Augmented States
  state_vector.segment<3>(45) = Eigen::Vector3d::Ones() * 12.0;

  left_state += state_vector;

  EXPECT_EQ(left_state.body_state.pos_b_in_l(0), 2.0);
  EXPECT_EQ(left_state.body_state.pos_b_in_l(1), 2.0);
  EXPECT_EQ(left_state.body_state.pos_b_in_l(2), 2.0);

  EXPECT_EQ(left_state.body_state.vel_b_in_l(0), 4.0);
  EXPECT_EQ(left_state.body_state.vel_b_in_l(1), 4.0);
  EXPECT_EQ(left_state.body_state.vel_b_in_l(2), 4.0);


  EXPECT_EQ(left_state.body_state.ang_b_to_l.w(), 1.0);
  EXPECT_EQ(left_state.body_state.ang_b_to_l.x(), 0.0);
  EXPECT_EQ(left_state.body_state.ang_b_to_l.y(), 0.0);
  EXPECT_EQ(left_state.body_state.ang_b_to_l.z(), 0.0);

  EXPECT_EQ(left_state.imu_states[1].pos_i_in_b(0), 12.0);
  EXPECT_EQ(left_state.imu_states[1].pos_i_in_b(1), 12.0);
  EXPECT_EQ(left_state.imu_states[1].pos_i_in_b(2), 12.0);

  EXPECT_EQ(left_state.imu_states[1].ang_i_to_b.w(), 1.0);
  EXPECT_EQ(left_state.imu_states[1].ang_i_to_b.x(), 0.0);
  EXPECT_EQ(left_state.imu_states[1].ang_i_to_b.y(), 0.0);
  EXPECT_EQ(left_state.imu_states[1].ang_i_to_b.z(), 0.0);

  EXPECT_EQ(left_state.imu_states[1].acc_bias(0), 14.0);
  EXPECT_EQ(left_state.imu_states[1].acc_bias(1), 14.0);
  EXPECT_EQ(left_state.imu_states[1].acc_bias(2), 14.0);

  EXPECT_EQ(left_state.imu_states[1].omg_bias(0), 16.0);
  EXPECT_EQ(left_state.imu_states[1].omg_bias(1), 16.0);
  EXPECT_EQ(left_state.imu_states[1].omg_bias(2), 16.0);

  EXPECT_EQ(left_state.gps_states[2].pos_a_in_b(0), 18.0);
  EXPECT_EQ(left_state.gps_states[2].pos_a_in_b(1), 18.0);
  EXPECT_EQ(left_state.gps_states[2].pos_a_in_b(2), 18.0);

  EXPECT_EQ(left_state.cam_states[3].pos_c_in_b(0), 20.0);
  EXPECT_EQ(left_state.cam_states[3].pos_c_in_b(1), 20.0);
  EXPECT_EQ(left_state.cam_states[3].pos_c_in_b(2), 20.0);

  EXPECT_EQ(left_state.cam_states[3].ang_c_to_b.w(), 1.0);
  EXPECT_EQ(left_state.cam_states[3].ang_c_to_b.x(), 0.0);
  EXPECT_EQ(left_state.cam_states[3].ang_c_to_b.y(), 0.0);
  EXPECT_EQ(left_state.cam_states[3].ang_c_to_b.z(), 0.0);

  EXPECT_EQ(left_state.fid_states[4].pos_f_in_l(0), 22.0);
  EXPECT_EQ(left_state.fid_states[4].pos_f_in_l(1), 22.0);
  EXPECT_EQ(left_state.fid_states[4].pos_f_in_l(2), 22.0);

  EXPECT_EQ(left_state.fid_states[4].ang_f_to_l.w(), 1.0);
  EXPECT_EQ(left_state.fid_states[4].ang_f_to_l.x(), 0.0);
  EXPECT_EQ(left_state.fid_states[4].ang_f_to_l.y(), 0.0);
  EXPECT_EQ(left_state.fid_states[4].ang_f_to_l.z(), 0.0);

  EXPECT_EQ(left_state.aug_states[0][0].pos_b_in_l(0), 24.0);
  EXPECT_EQ(left_state.aug_states[0][0].pos_b_in_l(1), 24.0);
  EXPECT_EQ(left_state.aug_states[0][0].pos_b_in_l(2), 24.0);

  EXPECT_EQ(left_state.aug_states[0][0].ang_b_to_l.w(), 1.0);
  EXPECT_EQ(left_state.aug_states[0][0].ang_b_to_l.x(), 0.0);
  EXPECT_EQ(left_state.aug_states[0][0].ang_b_to_l.y(), 0.0);
  EXPECT_EQ(left_state.aug_states[0][0].ang_b_to_l.z(), 0.0);
}

TEST(test_ekf_types, body_state_plus_equals_state) {
  BodyState left_state;
  left_state.pos_b_in_l = Eigen::Vector3d::Ones();
  left_state.vel_b_in_l = Eigen::Vector3d::Ones() * 2.0;
  left_state.ang_b_to_l.w() = 0.5;
  left_state.ang_b_to_l.x() = 0.5;
  left_state.ang_b_to_l.y() = 0.5;
  left_state.ang_b_to_l.z() = 0.5;

  BodyState right_state;
  right_state.pos_b_in_l = Eigen::Vector3d::Ones();
  right_state.vel_b_in_l = Eigen::Vector3d::Ones() * 2.0;
  right_state.ang_b_to_l.w() = 0.5;
  right_state.ang_b_to_l.x() = 0.5;
  right_state.ang_b_to_l.y() = 0.5;
  right_state.ang_b_to_l.z() = 0.5;

  left_state += right_state;

  EXPECT_EQ(left_state.pos_b_in_l(0), 2);
  EXPECT_EQ(left_state.pos_b_in_l(1), 2);
  EXPECT_EQ(left_state.pos_b_in_l(2), 2);

  EXPECT_EQ(left_state.vel_b_in_l(0), 4);
  EXPECT_EQ(left_state.vel_b_in_l(1), 4);
  EXPECT_EQ(left_state.vel_b_in_l(2), 4);

  EXPECT_EQ(left_state.ang_b_to_l.w(), -0.5);
  EXPECT_EQ(left_state.ang_b_to_l.x(), 0.5);
  EXPECT_EQ(left_state.ang_b_to_l.y(), 0.5);
  EXPECT_EQ(left_state.ang_b_to_l.z(), 0.5);
}

TEST(test_ekf_types, body_state_plus_equals_vector) {
  BodyState left_state;
  left_state.pos_b_in_l = Eigen::Vector3d::Ones();
  left_state.vel_b_in_l = Eigen::Vector3d::Ones() * 2.0;
  left_state.ang_b_to_l.w() = 0.5;
  left_state.ang_b_to_l.x() = 0.5;
  left_state.ang_b_to_l.y() = 0.5;
  left_state.ang_b_to_l.z() = 0.5;

  Eigen::VectorXd right_vector(18);
  right_vector(0) = 1.0;
  right_vector(1) = 1.0;
  right_vector(2) = 1.0;

  right_vector(3) = 2.0;
  right_vector(4) = 2.0;
  right_vector(5) = 2.0;

  right_vector(6) = 3.0;
  right_vector(7) = 3.0;
  right_vector(8) = 3.0;

  right_vector(9) = 0.0;
  right_vector(10) = 0.0;
  right_vector(11) = 0.0;

  right_vector(12) = 4.0;
  right_vector(13) = 4.0;
  right_vector(14) = 4.0;

  right_vector(15) = 5.0;
  right_vector(16) = 5.0;
  right_vector(17) = 5.0;

  left_state += right_vector;

  EXPECT_EQ(left_state.pos_b_in_l(0), 2);
  EXPECT_EQ(left_state.pos_b_in_l(1), 2);
  EXPECT_EQ(left_state.pos_b_in_l(2), 2);

  EXPECT_EQ(left_state.vel_b_in_l(0), 4);
  EXPECT_EQ(left_state.vel_b_in_l(1), 4);
  EXPECT_EQ(left_state.vel_b_in_l(2), 4);


  EXPECT_EQ(left_state.ang_b_to_l.w(), 0.5);
  EXPECT_EQ(left_state.ang_b_to_l.x(), 0.5);
  EXPECT_EQ(left_state.ang_b_to_l.y(), 0.5);
  EXPECT_EQ(left_state.ang_b_to_l.z(), 0.5);
}

TEST(test_ekf_types, imu_map_plus_equals) {
  ImuState imu_state;
  imu_state.SetIsIntrinsic(true);
  imu_state.SetIsExtrinsic(true);
  imu_state.pos_i_in_b = Eigen::Vector3d::Ones() * 1.0;
  imu_state.acc_bias = Eigen::Vector3d::Ones() * 2.0;
  imu_state.omg_bias = Eigen::Vector3d::Ones() * 3.0;
  imu_state.ang_i_to_b.w() = 1.0;
  imu_state.ang_i_to_b.x() = 0.0;
  imu_state.ang_i_to_b.y() = 0.0;
  imu_state.ang_i_to_b.z() = 0.0;

  std::map<unsigned int, ImuState> imu_map;
  imu_map[1] = imu_state;

  Eigen::VectorXd vec_state(12);
  vec_state.segment<3>(0) = Eigen::Vector3d::Ones() * 2.0;
  vec_state.segment<3>(3) = Eigen::Vector3d::Zero();
  vec_state.segment<3>(6) = Eigen::Vector3d::Ones() * 3.0;
  vec_state.segment<3>(9) = Eigen::Vector3d::Ones() * 4.0;

  imu_map += vec_state;

  EXPECT_EQ(imu_map[1].pos_i_in_b(0), 3.0);
  EXPECT_EQ(imu_map[1].pos_i_in_b(1), 3.0);
  EXPECT_EQ(imu_map[1].pos_i_in_b(2), 3.0);

  EXPECT_EQ(imu_map[1].ang_i_to_b.w(), 1.0);
  EXPECT_EQ(imu_map[1].ang_i_to_b.x(), 0.0);
  EXPECT_EQ(imu_map[1].ang_i_to_b.y(), 0.0);
  EXPECT_EQ(imu_map[1].ang_i_to_b.z(), 0.0);

  EXPECT_EQ(imu_map[1].acc_bias(0), 5.0);
  EXPECT_EQ(imu_map[1].acc_bias(1), 5.0);
  EXPECT_EQ(imu_map[1].acc_bias(2), 5.0);

  EXPECT_EQ(imu_map[1].omg_bias(0), 7.0);
  EXPECT_EQ(imu_map[1].omg_bias(1), 7.0);
  EXPECT_EQ(imu_map[1].omg_bias(2), 7.0);
}

TEST(test_ekf_types, cam_map_plus_equals) {
  Eigen::Quaterniond quat;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  CamState cam_state;
  cam_state.pos_c_in_b = Eigen::Vector3d::Ones() * 1.0;
  cam_state.ang_c_to_b = quat;

  std::map<unsigned int, CamState> cam_map;
  cam_map[1] = cam_state;

  Eigen::VectorXd vec_state = Eigen::VectorXd::Zero(18);
  vec_state.segment<3>(0) = Eigen::Vector3d::Ones() * 1.0;
  vec_state.segment<3>(6) = Eigen::Vector3d::Ones() * 2.0;
  vec_state.segment<3>(12) = Eigen::Vector3d::Ones() * 3.0;

  cam_map += vec_state;

  EXPECT_EQ(cam_map[1].pos_c_in_b(0), 2.0);
  EXPECT_EQ(cam_map[1].pos_c_in_b(1), 2.0);
  EXPECT_EQ(cam_map[1].pos_c_in_b(2), 2.0);

  EXPECT_EQ(cam_map[1].ang_c_to_b.w(), 1.0);
  EXPECT_EQ(cam_map[1].ang_c_to_b.x(), 0.0);
  EXPECT_EQ(cam_map[1].ang_c_to_b.y(), 0.0);
  EXPECT_EQ(cam_map[1].ang_c_to_b.z(), 0.0);
}

TEST(test_ekf_types, aug_state_plus_equals) {
  Eigen::Quaterniond quat;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  AugState aug_state_1;
  aug_state_1.pos_b_in_l = Eigen::Vector3d::Ones() * 1.0;
  aug_state_1.ang_b_to_l = quat;

  AugState aug_state_2;
  aug_state_2.pos_b_in_l = Eigen::Vector3d::Ones() * 2.0;
  aug_state_2.ang_b_to_l = quat;

  std::vector<AugState> aug_state_vec;
  aug_state_vec.push_back(aug_state_1);
  aug_state_vec.push_back(aug_state_2);

  Eigen::VectorXd vec_state = Eigen::VectorXd::Zero(24);
  vec_state.segment<3>(0) = Eigen::Vector3d::Ones() * 1.0;
  vec_state.segment<3>(6) = Eigen::Vector3d::Ones() * 2.0;

  aug_state_vec += vec_state;

  EXPECT_EQ(aug_state_vec[0].pos_b_in_l(0), 2.0);
  EXPECT_EQ(aug_state_vec[0].pos_b_in_l(1), 2.0);
  EXPECT_EQ(aug_state_vec[0].pos_b_in_l(2), 2.0);

  EXPECT_EQ(aug_state_vec[1].pos_b_in_l(0), 4.0);
  EXPECT_EQ(aug_state_vec[1].pos_b_in_l(1), 4.0);
  EXPECT_EQ(aug_state_vec[1].pos_b_in_l(2), 4.0);
}

TEST(test_ekf_types, body_state_to_vector) {
  BodyState body_state;
  body_state.pos_b_in_l = Eigen::Vector3d::Ones();
  body_state.vel_b_in_l = Eigen::Vector3d::Ones() * 2.0;
  body_state.ang_b_to_l.w() = 1.0;
  body_state.ang_b_to_l.x() = 0.0;
  body_state.ang_b_to_l.y() = 0.0;
  body_state.ang_b_to_l.z() = 0.0;

  Eigen::VectorXd state_vector = body_state.ToVector();

  EXPECT_EQ(state_vector.size(), 18);

  EXPECT_EQ(state_vector(0), 1.0);
  EXPECT_EQ(state_vector(1), 1.0);
  EXPECT_EQ(state_vector(2), 1.0);

  EXPECT_EQ(state_vector(3), 2.0);
  EXPECT_EQ(state_vector(4), 2.0);
  EXPECT_EQ(state_vector(5), 2.0);

  EXPECT_EQ(state_vector(6), 3.0);
  EXPECT_EQ(state_vector(7), 3.0);
  EXPECT_EQ(state_vector(8), 3.0);

  EXPECT_EQ(state_vector(9), 0.0);
  EXPECT_EQ(state_vector(10), 0.0);
  EXPECT_EQ(state_vector(11), 0.0);

  EXPECT_EQ(state_vector(12), 4.0);
  EXPECT_EQ(state_vector(13), 4.0);
  EXPECT_EQ(state_vector(14), 4.0);

  EXPECT_EQ(state_vector(15), 5.0);
  EXPECT_EQ(state_vector(16), 5.0);
  EXPECT_EQ(state_vector(17), 5.0);
}

TEST(test_ekf_types, cam_state_to_vector) {
  CamState cam_state;
  Eigen::Quaterniond quat;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  cam_state.pos_c_in_b = Eigen::Vector3d::Ones();
  cam_state.ang_c_to_b = quat;
  Eigen::VectorXd cam_state_vector = cam_state.ToVector();

  EXPECT_EQ(cam_state_vector.size(), 6);

  EXPECT_EQ(cam_state_vector(0), 1.0);
  EXPECT_EQ(cam_state_vector(1), 1.0);
  EXPECT_EQ(cam_state_vector(2), 1.0);

  EXPECT_EQ(cam_state_vector(3), 0.0);
  EXPECT_EQ(cam_state_vector(4), 0.0);
  EXPECT_EQ(cam_state_vector(5), 0.0);
}

TEST(test_ekf_types, fid_state_to_vector) {
  FidState fid_state;
  fid_state.SetIsExtrinsic(true);
  fid_state.pos_f_in_l = Eigen::Vector3d::Ones();
  fid_state.ang_f_to_l = Eigen::Quaterniond{1, 0, 0, 0};
  Eigen::VectorXd fid_state_vector = fid_state.ToVector();

  EXPECT_EQ(fid_state_vector.size(), 6);

  EXPECT_EQ(fid_state_vector(0), 1.0);
  EXPECT_EQ(fid_state_vector(1), 1.0);
  EXPECT_EQ(fid_state_vector(2), 1.0);

  EXPECT_EQ(fid_state_vector(3), 0.0);
  EXPECT_EQ(fid_state_vector(4), 0.0);
  EXPECT_EQ(fid_state_vector(5), 0.0);
}

TEST(test_ekf_types, imu_state_to_vector) {
  ImuState imu_state;
  imu_state.SetIsIntrinsic(true);
  imu_state.SetIsExtrinsic(true);
  imu_state.size = 12;
  imu_state.pos_i_in_b = Eigen::Vector3d::Ones() * 1.0;
  imu_state.ang_i_to_b.w() = 1.0;
  imu_state.ang_i_to_b.x() = 0.0;
  imu_state.ang_i_to_b.y() = 0.0;
  imu_state.ang_i_to_b.z() = 0.0;
  imu_state.acc_bias = Eigen::Vector3d::Ones() * 2.0;
  imu_state.omg_bias = Eigen::Vector3d::Ones() * 3.0;

  Eigen::VectorXd imu_state_vector = imu_state.ToVector();

  EXPECT_EQ(imu_state_vector.size(), 12);

  EXPECT_EQ(imu_state_vector(0), 1.0);
  EXPECT_EQ(imu_state_vector(1), 1.0);
  EXPECT_EQ(imu_state_vector(2), 1.0);

  EXPECT_EQ(imu_state_vector(3), 0.0);
  EXPECT_EQ(imu_state_vector(4), 0.0);
  EXPECT_EQ(imu_state_vector(5), 0.0);

  EXPECT_EQ(imu_state_vector(6), 2.0);
  EXPECT_EQ(imu_state_vector(7), 2.0);
  EXPECT_EQ(imu_state_vector(8), 2.0);

  EXPECT_EQ(imu_state_vector(9), 3.0);
  EXPECT_EQ(imu_state_vector(10), 3.0);
  EXPECT_EQ(imu_state_vector(11), 3.0);

  imu_state.SetIsExtrinsic(true);
  imu_state.SetIsIntrinsic(false);
  imu_state.size = 6;

  imu_state_vector = imu_state.ToVector();

  EXPECT_EQ(imu_state_vector.size(), 6);

  EXPECT_EQ(imu_state_vector(0), 1.0);
  EXPECT_EQ(imu_state_vector(1), 1.0);
  EXPECT_EQ(imu_state_vector(2), 1.0);

  EXPECT_EQ(imu_state_vector(3), 0.0);
  EXPECT_EQ(imu_state_vector(4), 0.0);
  EXPECT_EQ(imu_state_vector(5), 0.0);

  imu_state.SetIsExtrinsic(false);
  imu_state.SetIsIntrinsic(true);
  imu_state.size = 6;

  imu_state_vector = imu_state.ToVector();

  EXPECT_EQ(imu_state_vector.size(), 6);

  EXPECT_EQ(imu_state_vector(0), 2.0);
  EXPECT_EQ(imu_state_vector(1), 2.0);
  EXPECT_EQ(imu_state_vector(2), 2.0);

  EXPECT_EQ(imu_state_vector(3), 3.0);
  EXPECT_EQ(imu_state_vector(4), 3.0);
  EXPECT_EQ(imu_state_vector(5), 3.0);

  imu_state.SetIsExtrinsic(false);
  imu_state.SetIsIntrinsic(false);
  imu_state.size = 0;

  imu_state_vector = imu_state.ToVector();

  EXPECT_EQ(imu_state_vector.size(), 0);
}

TEST(test_ekf_types, state_to_vector) {
  Eigen::Quaterniond quat;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  ImuState imu_state;
  imu_state.SetIsIntrinsic(true);
  imu_state.SetIsExtrinsic(true);
  imu_state.pos_i_in_b = Eigen::Vector3d::Ones() * 6.0;
  imu_state.ang_i_to_b = quat;
  imu_state.acc_bias = Eigen::Vector3d::Ones() * 7.0;
  imu_state.omg_bias = Eigen::Vector3d::Ones() * 8.0;

  GpsState gps_state;
  gps_state.SetIsExtrinsic(true);
  gps_state.pos_a_in_b = Eigen::Vector3d::Ones() * 9.0;

  CamState cam_state;
  cam_state.pos_c_in_b = Eigen::Vector3d::Ones() * 10.0;
  cam_state.ang_c_to_b = quat;

  FidState fid_state;
  fid_state.SetIsExtrinsic(true);
  fid_state.pos_f_in_l = Eigen::Vector3d::Ones() * 11.0;

  AugState aug_state;
  aug_state.pos_b_in_l = Eigen::Vector3d::Ones() * 12.0;
  aug_state.ang_b_to_l = quat;

  State state;
  state.body_state.pos_b_in_l = Eigen::Vector3d::Ones() * 1.0;
  state.body_state.vel_b_in_l = Eigen::Vector3d::Ones() * 2.0;
  state.body_state.ang_b_to_l = quat;
  state.imu_states[1] = imu_state;
  state.gps_states[2] = gps_state;
  state.cam_states[3] = cam_state;
  state.fid_states[4] = fid_state;
  state.aug_states[0].push_back(aug_state);

  Eigen::VectorXd state_vector = state.ToVector();

  EXPECT_EQ(state_vector.size(), 51);

  EXPECT_EQ(state_vector(0), 1.0);
  EXPECT_EQ(state_vector(1), 1.0);
  EXPECT_EQ(state_vector(2), 1.0);

  EXPECT_EQ(state_vector(3), 2.0);
  EXPECT_EQ(state_vector(4), 2.0);
  EXPECT_EQ(state_vector(5), 2.0);

  EXPECT_EQ(state_vector(6), 3.0);
  EXPECT_EQ(state_vector(7), 3.0);
  EXPECT_EQ(state_vector(8), 3.0);

  EXPECT_EQ(state_vector(12), 4.0);
  EXPECT_EQ(state_vector(13), 4.0);
  EXPECT_EQ(state_vector(14), 4.0);

  EXPECT_EQ(state_vector(15), 5.0);
  EXPECT_EQ(state_vector(16), 5.0);
  EXPECT_EQ(state_vector(17), 5.0);

  EXPECT_EQ(state_vector(18), 6.0);
  EXPECT_EQ(state_vector(19), 6.0);
  EXPECT_EQ(state_vector(20), 6.0);

  EXPECT_EQ(state_vector(24), 7.0);
  EXPECT_EQ(state_vector(25), 7.0);
  EXPECT_EQ(state_vector(26), 7.0);

  EXPECT_EQ(state_vector(27), 8.0);
  EXPECT_EQ(state_vector(28), 8.0);
  EXPECT_EQ(state_vector(29), 8.0);

  EXPECT_EQ(state_vector(30), 9.0);
  EXPECT_EQ(state_vector(31), 9.0);
  EXPECT_EQ(state_vector(32), 9.0);

  EXPECT_EQ(state_vector(33), 10.0);
  EXPECT_EQ(state_vector(34), 10.0);
  EXPECT_EQ(state_vector(35), 10.0);

  EXPECT_EQ(state_vector(39), 11.0);
  EXPECT_EQ(state_vector(40), 11.0);
  EXPECT_EQ(state_vector(41), 11.0);

  EXPECT_EQ(state_vector(45), 12.0);
  EXPECT_EQ(state_vector(45), 12.0);
  EXPECT_EQ(state_vector(47), 12.0);
}

TEST(test_ekf_types, state_get_state_size) {
  State state;

  EXPECT_EQ(state.GetStateSize(), 18U);

  ImuState imu_state;
  imu_state.SetIsIntrinsic(true);
  imu_state.SetIsExtrinsic(true);
  state.imu_states[1] = imu_state;

  EXPECT_EQ(state.GetStateSize(), 30U);

  CamState cam_state;
  state.cam_states[2] = cam_state;

  EXPECT_EQ(state.GetStateSize(), 36U);

  AugState aug_state;
  state.aug_states[0].push_back(aug_state);

  EXPECT_EQ(state.GetStateSize(), 42U);
}

TEST(test_ekf_types, set_body_state) {
  BodyState body_state;
  Eigen::VectorXd body_state_vec(18);
  body_state_vec.segment<3>(0) = Eigen::Vector3d::Ones() * 1.0;
  body_state_vec.segment<3>(3) = Eigen::Vector3d::Ones() * 2.0;
  body_state_vec.segment<3>(6) = Eigen::Vector3d::Ones() * 3.0;
  body_state_vec.segment<3>(9) = Eigen::Vector3d::Ones() * 0.4;
  body_state_vec.segment<3>(12) = Eigen::Vector3d::Ones() * 5.0;
  body_state_vec.segment<3>(15) = Eigen::Vector3d::Ones() * 6.0;

  body_state.SetState(body_state_vec);

  EXPECT_EQ(body_state.ToVector(), body_state_vec);
}

TEST(test_ekf_types, gps_state_to_vector) {
  GpsState gps_state;
  gps_state.SetIsExtrinsic(true);
  gps_state.pos_a_in_b = Eigen::Vector3d{1.0, 2.0, 3.0};
  EXPECT_EQ(gps_state.ToVector(), gps_state.pos_a_in_b);
}
