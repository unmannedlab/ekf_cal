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

#include "ekf/types.hpp"

TEST(test_ekf_types, state_plus_equals_state) {
  Eigen::Quaterniond quat;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  ImuState imu_state;
  imu_state.position = Eigen::Vector3d::Ones() * 6.0;
  imu_state.orientation = quat;
  imu_state.acc_bias = Eigen::Vector3d::Ones() * 7.0;
  imu_state.omg_bias = Eigen::Vector3d::Ones() * 8.0;

  AugmentedState aug_state;
  aug_state.imu_position = Eigen::Vector3d::Ones() * 10.0;
  aug_state.imu_orientation = quat;
  aug_state.position = Eigen::Vector3d::Ones() * 11.0;
  aug_state.orientation = quat;

  CamState cam_state;
  cam_state.position = Eigen::Vector3d::Ones() * 9.0;
  cam_state.orientation = quat;
  cam_state.augmented_states.push_back(aug_state);

  State left_state;
  left_state.m_body_state.m_position = Eigen::Vector3d::Ones() * 1.0;
  left_state.m_body_state.m_velocity = Eigen::Vector3d::Ones() * 2.0;
  left_state.m_body_state.m_acceleration = Eigen::Vector3d::Ones() * 3.0;
  left_state.m_body_state.m_orientation = quat;
  left_state.m_body_state.m_angular_velocity = Eigen::Vector3d::Ones() * 4.0;
  left_state.m_body_state.m_angular_acceleration = Eigen::Vector3d::Ones() * 5.0;
  left_state.m_imu_states[1] = imu_state;
  left_state.m_cam_states[2] = cam_state;

  left_state += left_state;

  EXPECT_EQ(left_state.m_body_state.m_position(0), 2.0);
  EXPECT_EQ(left_state.m_body_state.m_position(1), 2.0);
  EXPECT_EQ(left_state.m_body_state.m_position(2), 2.0);

  EXPECT_EQ(left_state.m_body_state.m_velocity(0), 4.0);
  EXPECT_EQ(left_state.m_body_state.m_velocity(1), 4.0);
  EXPECT_EQ(left_state.m_body_state.m_velocity(2), 4.0);

  EXPECT_EQ(left_state.m_body_state.m_acceleration(0), 6.0);
  EXPECT_EQ(left_state.m_body_state.m_acceleration(1), 6.0);
  EXPECT_EQ(left_state.m_body_state.m_acceleration(2), 6.0);

  EXPECT_EQ(left_state.m_body_state.m_orientation.w(), 1.0);
  EXPECT_EQ(left_state.m_body_state.m_orientation.x(), 0.0);
  EXPECT_EQ(left_state.m_body_state.m_orientation.y(), 0.0);
  EXPECT_EQ(left_state.m_body_state.m_orientation.z(), 0.0);

  EXPECT_EQ(left_state.m_body_state.m_angular_velocity(0), 8.0);
  EXPECT_EQ(left_state.m_body_state.m_angular_velocity(1), 8.0);
  EXPECT_EQ(left_state.m_body_state.m_angular_velocity(2), 8.0);

  EXPECT_EQ(left_state.m_body_state.m_angular_acceleration(0), 10.0);
  EXPECT_EQ(left_state.m_body_state.m_angular_acceleration(1), 10.0);
  EXPECT_EQ(left_state.m_body_state.m_angular_acceleration(2), 10.0);

  EXPECT_EQ(left_state.m_imu_states[1].position(0), 12.0);
  EXPECT_EQ(left_state.m_imu_states[1].position(1), 12.0);
  EXPECT_EQ(left_state.m_imu_states[1].position(2), 12.0);

  EXPECT_EQ(left_state.m_imu_states[1].orientation.w(), 1.0);
  EXPECT_EQ(left_state.m_imu_states[1].orientation.x(), 0.0);
  EXPECT_EQ(left_state.m_imu_states[1].orientation.y(), 0.0);
  EXPECT_EQ(left_state.m_imu_states[1].orientation.z(), 0.0);

  EXPECT_EQ(left_state.m_imu_states[1].acc_bias(0), 14.0);
  EXPECT_EQ(left_state.m_imu_states[1].acc_bias(1), 14.0);
  EXPECT_EQ(left_state.m_imu_states[1].acc_bias(2), 14.0);

  EXPECT_EQ(left_state.m_imu_states[1].omg_bias(0), 16.0);
  EXPECT_EQ(left_state.m_imu_states[1].omg_bias(1), 16.0);
  EXPECT_EQ(left_state.m_imu_states[1].omg_bias(2), 16.0);

  EXPECT_EQ(left_state.m_cam_states[2].position(0), 18.0);
  EXPECT_EQ(left_state.m_cam_states[2].position(1), 18.0);
  EXPECT_EQ(left_state.m_cam_states[2].position(2), 18.0);

  EXPECT_EQ(left_state.m_cam_states[2].orientation.w(), 1.0);
  EXPECT_EQ(left_state.m_cam_states[2].orientation.x(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].orientation.y(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].orientation.z(), 0.0);

  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_position(0), 20.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_position(1), 20.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_position(2), 20.0);

  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_orientation.w(), 1.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_orientation.x(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_orientation.y(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_orientation.z(), 0.0);

  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].position(0), 22.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].position(1), 22.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].position(2), 22.0);

  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].orientation.w(), 1.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].orientation.x(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].orientation.y(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].orientation.z(), 0.0);
}

TEST(test_ekf_types, state_plus_equals_vector) {
  Eigen::Quaterniond quat;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  ImuState imu_state;
  imu_state.position = Eigen::Vector3d::Ones() * 6.0;
  imu_state.orientation = quat;
  imu_state.acc_bias = Eigen::Vector3d::Ones() * 7.0;
  imu_state.omg_bias = Eigen::Vector3d::Ones() * 8.0;

  AugmentedState aug_state;
  aug_state.imu_position = Eigen::Vector3d::Ones() * 10.0;
  aug_state.imu_orientation = quat;
  aug_state.position = Eigen::Vector3d::Ones() * 11.0;
  aug_state.orientation = quat;

  CamState cam_state;
  cam_state.position = Eigen::Vector3d::Ones() * 9.0;
  cam_state.orientation = quat;
  cam_state.augmented_states.push_back(aug_state);

  State left_state;
  left_state.m_body_state.m_position = Eigen::Vector3d::Ones() * 1.0;
  left_state.m_body_state.m_velocity = Eigen::Vector3d::Ones() * 2.0;
  left_state.m_body_state.m_acceleration = Eigen::Vector3d::Ones() * 3.0;
  left_state.m_body_state.m_orientation = quat;
  left_state.m_body_state.m_angular_velocity = Eigen::Vector3d::Ones() * 4.0;
  left_state.m_body_state.m_angular_acceleration = Eigen::Vector3d::Ones() * 5.0;
  left_state.m_imu_states[1] = imu_state;
  left_state.m_cam_states[2] = cam_state;

  Eigen::VectorXd state_vector = Eigen::VectorXd::Zero(48);
  state_vector.segment<3>(0) = Eigen::Vector3d::Ones() * 1.0;
  state_vector.segment<3>(3) = Eigen::Vector3d::Ones() * 2.0;
  state_vector.segment<3>(6) = Eigen::Vector3d::Ones() * 3.0;
  state_vector.segment<3>(12) = Eigen::Vector3d::Ones() * 4.0;
  state_vector.segment<3>(15) = Eigen::Vector3d::Ones() * 5.0;
  state_vector.segment<3>(18) = Eigen::Vector3d::Ones() * 6.0;
  state_vector.segment<3>(24) = Eigen::Vector3d::Ones() * 7.0;
  state_vector.segment<3>(27) = Eigen::Vector3d::Ones() * 8.0;
  state_vector.segment<3>(30) = Eigen::Vector3d::Ones() * 9.0;
  state_vector.segment<3>(36) = Eigen::Vector3d::Ones() * 10.0;
  state_vector.segment<3>(42) = Eigen::Vector3d::Ones() * 11.0;

  left_state += state_vector;

  EXPECT_EQ(left_state.m_body_state.m_position(0), 2.0);
  EXPECT_EQ(left_state.m_body_state.m_position(1), 2.0);
  EXPECT_EQ(left_state.m_body_state.m_position(2), 2.0);

  EXPECT_EQ(left_state.m_body_state.m_velocity(0), 4.0);
  EXPECT_EQ(left_state.m_body_state.m_velocity(1), 4.0);
  EXPECT_EQ(left_state.m_body_state.m_velocity(2), 4.0);

  EXPECT_EQ(left_state.m_body_state.m_acceleration(0), 6.0);
  EXPECT_EQ(left_state.m_body_state.m_acceleration(1), 6.0);
  EXPECT_EQ(left_state.m_body_state.m_acceleration(2), 6.0);

  EXPECT_EQ(left_state.m_body_state.m_orientation.w(), 1.0);
  EXPECT_EQ(left_state.m_body_state.m_orientation.x(), 0.0);
  EXPECT_EQ(left_state.m_body_state.m_orientation.y(), 0.0);
  EXPECT_EQ(left_state.m_body_state.m_orientation.z(), 0.0);

  EXPECT_EQ(left_state.m_body_state.m_angular_velocity(0), 8.0);
  EXPECT_EQ(left_state.m_body_state.m_angular_velocity(1), 8.0);
  EXPECT_EQ(left_state.m_body_state.m_angular_velocity(2), 8.0);

  EXPECT_EQ(left_state.m_body_state.m_angular_acceleration(0), 10.0);
  EXPECT_EQ(left_state.m_body_state.m_angular_acceleration(1), 10.0);
  EXPECT_EQ(left_state.m_body_state.m_angular_acceleration(2), 10.0);

  EXPECT_EQ(left_state.m_imu_states[1].position(0), 12.0);
  EXPECT_EQ(left_state.m_imu_states[1].position(1), 12.0);
  EXPECT_EQ(left_state.m_imu_states[1].position(2), 12.0);

  EXPECT_EQ(left_state.m_imu_states[1].orientation.w(), 1.0);
  EXPECT_EQ(left_state.m_imu_states[1].orientation.x(), 0.0);
  EXPECT_EQ(left_state.m_imu_states[1].orientation.y(), 0.0);
  EXPECT_EQ(left_state.m_imu_states[1].orientation.z(), 0.0);

  EXPECT_EQ(left_state.m_imu_states[1].acc_bias(0), 14.0);
  EXPECT_EQ(left_state.m_imu_states[1].acc_bias(1), 14.0);
  EXPECT_EQ(left_state.m_imu_states[1].acc_bias(2), 14.0);

  EXPECT_EQ(left_state.m_imu_states[1].omg_bias(0), 16.0);
  EXPECT_EQ(left_state.m_imu_states[1].omg_bias(1), 16.0);
  EXPECT_EQ(left_state.m_imu_states[1].omg_bias(2), 16.0);

  EXPECT_EQ(left_state.m_cam_states[2].position(0), 18.0);
  EXPECT_EQ(left_state.m_cam_states[2].position(1), 18.0);
  EXPECT_EQ(left_state.m_cam_states[2].position(2), 18.0);

  EXPECT_EQ(left_state.m_cam_states[2].orientation.w(), 1.0);
  EXPECT_EQ(left_state.m_cam_states[2].orientation.x(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].orientation.y(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].orientation.z(), 0.0);

  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_position(0), 20.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_position(1), 20.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_position(2), 20.0);

  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_orientation.w(), 1.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_orientation.x(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_orientation.y(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].imu_orientation.z(), 0.0);

  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].position(0), 22.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].position(1), 22.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].position(2), 22.0);

  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].orientation.w(), 1.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].orientation.x(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].orientation.y(), 0.0);
  EXPECT_EQ(left_state.m_cam_states[2].augmented_states[0].orientation.z(), 0.0);
}

TEST(test_ekf_types, body_state_plus_equals_state) {
  BodyState left_state;
  left_state.m_position = Eigen::Vector3d::Ones();
  left_state.m_velocity = Eigen::Vector3d::Ones() * 2.0;
  left_state.m_acceleration = Eigen::Vector3d::Ones() * 3.0;
  left_state.m_orientation.w() = 0.5;
  left_state.m_orientation.x() = 0.5;
  left_state.m_orientation.y() = 0.5;
  left_state.m_orientation.z() = 0.5;
  left_state.m_angular_velocity = Eigen::Vector3d::Ones() * 4.0;
  left_state.m_angular_acceleration = Eigen::Vector3d::Ones() * 5.0;

  BodyState right_state;
  right_state.m_position = Eigen::Vector3d::Ones();
  right_state.m_velocity = Eigen::Vector3d::Ones() * 2.0;
  right_state.m_acceleration = Eigen::Vector3d::Ones() * 3.0;
  right_state.m_orientation.w() = 0.5;
  right_state.m_orientation.x() = 0.5;
  right_state.m_orientation.y() = 0.5;
  right_state.m_orientation.z() = 0.5;
  right_state.m_angular_velocity = Eigen::Vector3d::Ones() * 4.0;
  right_state.m_angular_acceleration = Eigen::Vector3d::Ones() * 5.0;

  left_state += right_state;

  EXPECT_EQ(left_state.m_position(0), 2);
  EXPECT_EQ(left_state.m_position(1), 2);
  EXPECT_EQ(left_state.m_position(2), 2);

  EXPECT_EQ(left_state.m_velocity(0), 4);
  EXPECT_EQ(left_state.m_velocity(1), 4);
  EXPECT_EQ(left_state.m_velocity(2), 4);

  EXPECT_EQ(left_state.m_acceleration(0), 6);
  EXPECT_EQ(left_state.m_acceleration(1), 6);
  EXPECT_EQ(left_state.m_acceleration(2), 6);

  EXPECT_EQ(left_state.m_orientation.w(), -0.5);
  EXPECT_EQ(left_state.m_orientation.x(), 0.5);
  EXPECT_EQ(left_state.m_orientation.y(), 0.5);
  EXPECT_EQ(left_state.m_orientation.z(), 0.5);

  EXPECT_EQ(left_state.m_angular_velocity(0), 8);
  EXPECT_EQ(left_state.m_angular_velocity(1), 8);
  EXPECT_EQ(left_state.m_angular_velocity(2), 8);

  EXPECT_EQ(left_state.m_angular_acceleration(0), 10);
  EXPECT_EQ(left_state.m_angular_acceleration(1), 10);
  EXPECT_EQ(left_state.m_angular_acceleration(2), 10);
}

TEST(test_ekf_types, body_state_plus_equals_vector) {
  BodyState left_state;
  left_state.m_position = Eigen::Vector3d::Ones();
  left_state.m_velocity = Eigen::Vector3d::Ones() * 2.0;
  left_state.m_acceleration = Eigen::Vector3d::Ones() * 3.0;
  left_state.m_orientation.w() = 0.5;
  left_state.m_orientation.x() = 0.5;
  left_state.m_orientation.y() = 0.5;
  left_state.m_orientation.z() = 0.5;
  left_state.m_angular_velocity = Eigen::Vector3d::Ones() * 4.0;
  left_state.m_angular_acceleration = Eigen::Vector3d::Ones() * 5.0;

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

  EXPECT_EQ(left_state.m_position(0), 2);
  EXPECT_EQ(left_state.m_position(1), 2);
  EXPECT_EQ(left_state.m_position(2), 2);

  EXPECT_EQ(left_state.m_velocity(0), 4);
  EXPECT_EQ(left_state.m_velocity(1), 4);
  EXPECT_EQ(left_state.m_velocity(2), 4);

  EXPECT_EQ(left_state.m_acceleration(0), 6);
  EXPECT_EQ(left_state.m_acceleration(1), 6);
  EXPECT_EQ(left_state.m_acceleration(2), 6);

  EXPECT_EQ(left_state.m_orientation.w(), 0.5);
  EXPECT_EQ(left_state.m_orientation.x(), 0.5);
  EXPECT_EQ(left_state.m_orientation.y(), 0.5);
  EXPECT_EQ(left_state.m_orientation.z(), 0.5);

  EXPECT_EQ(left_state.m_angular_velocity(0), 8);
  EXPECT_EQ(left_state.m_angular_velocity(1), 8);
  EXPECT_EQ(left_state.m_angular_velocity(2), 8);

  EXPECT_EQ(left_state.m_angular_acceleration(0), 10);
  EXPECT_EQ(left_state.m_angular_acceleration(1), 10);
  EXPECT_EQ(left_state.m_angular_acceleration(2), 10);
}

TEST(test_ekf_types, imu_map_plus_equals) {
  ImuState imu_state;
  imu_state.position = Eigen::Vector3d::Ones() * 1.0;
  imu_state.acc_bias = Eigen::Vector3d::Ones() * 2.0;
  imu_state.omg_bias = Eigen::Vector3d::Ones() * 3.0;
  imu_state.orientation.w() = 1.0;
  imu_state.orientation.x() = 0.0;
  imu_state.orientation.y() = 0.0;
  imu_state.orientation.z() = 0.0;

  std::map<unsigned int, ImuState> imu_map;
  imu_map[1] = imu_state;

  Eigen::VectorXd vec_state(12);
  vec_state.segment<3>(0) = Eigen::Vector3d::Ones() * 2.0;
  vec_state.segment<3>(3) = Eigen::Vector3d::Zero();
  vec_state.segment<3>(6) = Eigen::Vector3d::Ones() * 3.0;
  vec_state.segment<3>(9) = Eigen::Vector3d::Ones() * 4.0;

  imu_map += vec_state;

  EXPECT_EQ(imu_map[1].position(0), 3.0);
  EXPECT_EQ(imu_map[1].position(1), 3.0);
  EXPECT_EQ(imu_map[1].position(2), 3.0);

  EXPECT_EQ(imu_map[1].orientation.w(), 1.0);
  EXPECT_EQ(imu_map[1].orientation.x(), 0.0);
  EXPECT_EQ(imu_map[1].orientation.y(), 0.0);
  EXPECT_EQ(imu_map[1].orientation.z(), 0.0);

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

  AugmentedState aug_state;
  aug_state.imu_position = Eigen::Vector3d::Ones() * 2.0;
  aug_state.imu_orientation = quat;
  aug_state.position = Eigen::Vector3d::Ones() * 3.0;
  aug_state.orientation = quat;

  CamState cam_state;
  cam_state.position = Eigen::Vector3d::Ones() * 1.0;
  cam_state.orientation = quat;
  cam_state.augmented_states.push_back(aug_state);

  std::map<unsigned int, CamState> cam_map;
  cam_map[1] = cam_state;

  Eigen::VectorXd vec_state = Eigen::VectorXd::Zero(18);
  vec_state.segment<3>(0) = Eigen::Vector3d::Ones() * 1.0;
  vec_state.segment<3>(6) = Eigen::Vector3d::Ones() * 2.0;
  vec_state.segment<3>(12) = Eigen::Vector3d::Ones() * 3.0;

  cam_map += vec_state;

  EXPECT_EQ(cam_map[1].position(0), 2.0);
  EXPECT_EQ(cam_map[1].position(1), 2.0);
  EXPECT_EQ(cam_map[1].position(2), 2.0);

  EXPECT_EQ(cam_map[1].orientation.w(), 1.0);
  EXPECT_EQ(cam_map[1].orientation.x(), 0.0);
  EXPECT_EQ(cam_map[1].orientation.y(), 0.0);
  EXPECT_EQ(cam_map[1].orientation.z(), 0.0);

  EXPECT_EQ(cam_map[1].augmented_states[0].imu_position(0), 4.0);
  EXPECT_EQ(cam_map[1].augmented_states[0].imu_position(1), 4.0);
  EXPECT_EQ(cam_map[1].augmented_states[0].imu_position(2), 4.0);

  EXPECT_EQ(cam_map[1].augmented_states[0].imu_orientation.w(), 1.0);
  EXPECT_EQ(cam_map[1].augmented_states[0].imu_orientation.x(), 0.0);
  EXPECT_EQ(cam_map[1].augmented_states[0].imu_orientation.y(), 0.0);
  EXPECT_EQ(cam_map[1].augmented_states[0].imu_orientation.z(), 0.0);

  EXPECT_EQ(cam_map[1].augmented_states[0].position(0), 6.0);
  EXPECT_EQ(cam_map[1].augmented_states[0].position(1), 6.0);
  EXPECT_EQ(cam_map[1].augmented_states[0].position(2), 6.0);

  EXPECT_EQ(cam_map[1].augmented_states[0].orientation.w(), 1.0);
  EXPECT_EQ(cam_map[1].augmented_states[0].orientation.x(), 0.0);
  EXPECT_EQ(cam_map[1].augmented_states[0].orientation.y(), 0.0);
  EXPECT_EQ(cam_map[1].augmented_states[0].orientation.z(), 0.0);
}

TEST(test_ekf_types, aug_state_plus_equals) {
  Eigen::Quaterniond quat;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  AugmentedState aug_state_1;
  aug_state_1.imu_position = Eigen::Vector3d::Ones() * 1.0;
  aug_state_1.imu_orientation = quat;
  aug_state_1.position = Eigen::Vector3d::Ones() * 2.0;
  aug_state_1.orientation = quat;

  AugmentedState aug_state_2;
  aug_state_2.imu_position = Eigen::Vector3d::Ones() * 3.0;
  aug_state_2.imu_orientation = quat;
  aug_state_2.position = Eigen::Vector3d::Ones() * 4.0;
  aug_state_2.orientation = quat;

  std::vector<AugmentedState> aug_state_vec;
  aug_state_vec.push_back(aug_state_1);
  aug_state_vec.push_back(aug_state_2);

  Eigen::VectorXd vec_state = Eigen::VectorXd::Zero(24);
  vec_state.segment<3>(0) = Eigen::Vector3d::Ones() * 1.0;
  vec_state.segment<3>(6) = Eigen::Vector3d::Ones() * 2.0;
  vec_state.segment<3>(12) = Eigen::Vector3d::Ones() * 3.0;
  vec_state.segment<3>(18) = Eigen::Vector3d::Ones() * 4.0;

  aug_state_vec += vec_state;

  EXPECT_EQ(aug_state_vec[0].imu_position(0), 2.0);
  EXPECT_EQ(aug_state_vec[0].imu_position(1), 2.0);
  EXPECT_EQ(aug_state_vec[0].imu_position(2), 2.0);

  EXPECT_EQ(aug_state_vec[0].position(0), 4.0);
  EXPECT_EQ(aug_state_vec[0].position(1), 4.0);
  EXPECT_EQ(aug_state_vec[0].position(2), 4.0);

  EXPECT_EQ(aug_state_vec[1].imu_position(0), 6.0);
  EXPECT_EQ(aug_state_vec[1].imu_position(1), 6.0);
  EXPECT_EQ(aug_state_vec[1].imu_position(2), 6.0);

  EXPECT_EQ(aug_state_vec[1].position(0), 8.0);
  EXPECT_EQ(aug_state_vec[1].position(1), 8.0);
  EXPECT_EQ(aug_state_vec[1].position(2), 8.0);
}

TEST(test_ekf_types, body_state_to_vector) {
  BodyState body_state;
  body_state.m_position = Eigen::Vector3d::Ones();
  body_state.m_velocity = Eigen::Vector3d::Ones() * 2.0;
  body_state.m_acceleration = Eigen::Vector3d::Ones() * 3.0;
  body_state.m_orientation.w() = 1.0;
  body_state.m_orientation.x() = 0.0;
  body_state.m_orientation.y() = 0.0;
  body_state.m_orientation.z() = 0.0;
  body_state.m_angular_velocity = Eigen::Vector3d::Ones() * 4.0;
  body_state.m_angular_acceleration = Eigen::Vector3d::Ones() * 5.0;

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

  cam_state.position = Eigen::Vector3d::Ones();
  cam_state.orientation = quat;

  AugmentedState aug_state_1;
  aug_state_1.imu_position = Eigen::Vector3d::Ones() * 2.0;
  aug_state_1.imu_orientation = quat;
  aug_state_1.position = Eigen::Vector3d::Ones() * 3.0;
  aug_state_1.orientation = quat;

  AugmentedState aug_state_2;
  aug_state_2.imu_position = Eigen::Vector3d::Ones() * 4.0;
  aug_state_2.imu_orientation = quat;
  aug_state_2.position = Eigen::Vector3d::Ones() * 5.0;
  aug_state_2.orientation = quat;

  cam_state.augmented_states.push_back(aug_state_1);
  cam_state.augmented_states.push_back(aug_state_2);

  Eigen::VectorXd cam_state_vector = cam_state.ToVector();

  EXPECT_EQ(cam_state_vector.size(), 30);

  EXPECT_EQ(cam_state_vector(0), 1.0);
  EXPECT_EQ(cam_state_vector(1), 1.0);
  EXPECT_EQ(cam_state_vector(2), 1.0);

  EXPECT_EQ(cam_state_vector(3), 0.0);
  EXPECT_EQ(cam_state_vector(4), 0.0);
  EXPECT_EQ(cam_state_vector(5), 0.0);

  EXPECT_EQ(cam_state_vector(6), 2.0);
  EXPECT_EQ(cam_state_vector(7), 2.0);
  EXPECT_EQ(cam_state_vector(8), 2.0);

  EXPECT_EQ(cam_state_vector(9), 0.0);
  EXPECT_EQ(cam_state_vector(10), 0.0);
  EXPECT_EQ(cam_state_vector(11), 0.0);

  EXPECT_EQ(cam_state_vector(12), 3.0);
  EXPECT_EQ(cam_state_vector(13), 3.0);
  EXPECT_EQ(cam_state_vector(14), 3.0);

  EXPECT_EQ(cam_state_vector(15), 0.0);
  EXPECT_EQ(cam_state_vector(16), 0.0);
  EXPECT_EQ(cam_state_vector(17), 0.0);

  EXPECT_EQ(cam_state_vector(18), 4.0);
  EXPECT_EQ(cam_state_vector(19), 4.0);
  EXPECT_EQ(cam_state_vector(20), 4.0);

  EXPECT_EQ(cam_state_vector(21), 0.0);
  EXPECT_EQ(cam_state_vector(22), 0.0);
  EXPECT_EQ(cam_state_vector(23), 0.0);

  EXPECT_EQ(cam_state_vector(24), 5.0);
  EXPECT_EQ(cam_state_vector(25), 5.0);
  EXPECT_EQ(cam_state_vector(26), 5.0);

  EXPECT_EQ(cam_state_vector(27), 0.0);
  EXPECT_EQ(cam_state_vector(28), 0.0);
  EXPECT_EQ(cam_state_vector(29), 0.0);
}

TEST(test_ekf_types, imu_state_to_vector) {
  ImuState imu_state;
  imu_state.position = Eigen::Vector3d::Ones() * 1.0;
  imu_state.orientation.w() = 1.0;
  imu_state.orientation.x() = 0.0;
  imu_state.orientation.y() = 0.0;
  imu_state.orientation.z() = 0.0;
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
}

// TEST(test_ekf_types, body_state_set_state) {
//   BodyState body_state;
// }

TEST(test_ekf_types, state_to_vector) {
  Eigen::Quaterniond quat;
  quat.w() = 1.0;
  quat.x() = 0.0;
  quat.y() = 0.0;
  quat.z() = 0.0;

  ImuState imu_state;
  imu_state.position = Eigen::Vector3d::Ones() * 6.0;
  imu_state.orientation = quat;
  imu_state.acc_bias = Eigen::Vector3d::Ones() * 7.0;
  imu_state.omg_bias = Eigen::Vector3d::Ones() * 8.0;

  AugmentedState aug_state;
  aug_state.imu_position = Eigen::Vector3d::Ones() * 10.0;
  aug_state.imu_orientation = quat;
  aug_state.position = Eigen::Vector3d::Ones() * 11.0;
  aug_state.orientation = quat;

  CamState cam_state;
  cam_state.position = Eigen::Vector3d::Ones() * 9.0;
  cam_state.orientation = quat;
  cam_state.augmented_states.push_back(aug_state);

  State state;
  state.m_body_state.m_position = Eigen::Vector3d::Ones() * 1.0;
  state.m_body_state.m_velocity = Eigen::Vector3d::Ones() * 2.0;
  state.m_body_state.m_acceleration = Eigen::Vector3d::Ones() * 3.0;
  state.m_body_state.m_orientation = quat;
  state.m_body_state.m_angular_velocity = Eigen::Vector3d::Ones() * 4.0;
  state.m_body_state.m_angular_acceleration = Eigen::Vector3d::Ones() * 5.0;
  state.m_imu_states[1] = imu_state;
  state.m_cam_states[2] = cam_state;

  Eigen::VectorXd state_vector = state.ToVector();

  EXPECT_EQ(state_vector.size(), 48);

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

  EXPECT_EQ(state_vector(36), 10.0);
  EXPECT_EQ(state_vector(37), 10.0);
  EXPECT_EQ(state_vector(38), 10.0);

  EXPECT_EQ(state_vector(42), 11.0);
  EXPECT_EQ(state_vector(43), 11.0);
  EXPECT_EQ(state_vector(44), 11.0);
}

TEST(test_ekf_types, state_get_state_size) {
  State state;

  EXPECT_EQ(state.GetStateSize(), 18);

  ImuState imu_state;
  state.m_imu_states[1] = imu_state;

  EXPECT_EQ(state.GetStateSize(), 30);

  CamState cam_state;
  state.m_cam_states[1] = cam_state;

  EXPECT_EQ(state.GetStateSize(), 36);

  AugmentedState aug_state;
  state.m_cam_states[1].augmented_states.push_back(aug_state);

  EXPECT_EQ(state.GetStateSize(), 48);
}
