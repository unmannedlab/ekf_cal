// Copyright 2023 Jacob Hartzer
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

#include "ekf/types.hpp"

#include <eigen3/Eigen/Eigen>
#include <memory>
#include <utility>
#include <vector>

#include "ekf/constants.hpp"
#include "utility/type_helper.hpp"

BodyState & operator+=(BodyState & l_m_body_state, BodyState & r_m_body_state)
{
  l_m_body_state.m_position += r_m_body_state.m_position;
  l_m_body_state.m_velocity += r_m_body_state.m_velocity;
  l_m_body_state.m_acceleration += r_m_body_state.m_acceleration;
  l_m_body_state.m_ang_b_to_g = r_m_body_state.m_ang_b_to_g * l_m_body_state.m_ang_b_to_g;
  l_m_body_state.m_angular_velocity += r_m_body_state.m_angular_velocity;
  l_m_body_state.m_angular_acceleration += r_m_body_state.m_angular_acceleration;

  return l_m_body_state;
}

BodyState & operator+=(BodyState & l_m_body_state, Eigen::VectorXd & r_vector)
{
  l_m_body_state.m_position += r_vector.segment<3>(0);
  l_m_body_state.m_velocity += r_vector.segment<3>(3);
  l_m_body_state.m_acceleration += r_vector.segment<3>(6);
  l_m_body_state.m_ang_b_to_g =
    l_m_body_state.m_ang_b_to_g * RotVecToQuat(r_vector.segment<3>(9));
  l_m_body_state.m_angular_velocity += r_vector.segment<3>(12);
  l_m_body_state.m_angular_acceleration += r_vector.segment<3>(15);

  return l_m_body_state;
}

State & operator+=(State & l_state, State & r_state)
{
  l_state.m_body_state += r_state.m_body_state;

  for (auto & imu_iter : l_state.m_imu_states) {
    unsigned int imu_id = imu_iter.first;
    l_state.m_imu_states[imu_id].pos_i_in_b += r_state.m_imu_states[imu_id].pos_i_in_b;
    l_state.m_imu_states[imu_id].ang_i_to_b =
      r_state.m_imu_states[imu_id].ang_i_to_b * l_state.m_imu_states[imu_id].ang_i_to_b;
    l_state.m_imu_states[imu_id].acc_bias += r_state.m_imu_states[imu_id].acc_bias;
    l_state.m_imu_states[imu_id].omg_bias += r_state.m_imu_states[imu_id].omg_bias;
  }

  for (auto & cam_iter : l_state.m_cam_states) {
    unsigned int cam_id = cam_iter.first;
    l_state.m_cam_states[cam_id].pos_c_in_b += r_state.m_cam_states[cam_id].pos_c_in_b;
    l_state.m_cam_states[cam_id].ang_c_to_b =
      r_state.m_cam_states[cam_id].ang_c_to_b * l_state.m_cam_states[cam_id].ang_c_to_b;
    for (unsigned int i = 0; i < l_state.m_cam_states[cam_id].augmented_states.size(); ++i) {
      AugmentedState & l_aug_state = l_state.m_cam_states[cam_id].augmented_states[i];
      AugmentedState & r_Aug_state = r_state.m_cam_states[cam_id].augmented_states[i];
      l_aug_state.pos_b_in_g += r_Aug_state.pos_b_in_g;
      l_aug_state.ang_b_to_g = r_Aug_state.ang_b_to_g * l_aug_state.ang_b_to_g;
      l_aug_state.pos_c_in_b += r_Aug_state.pos_c_in_b;
      l_aug_state.ang_c_to_b = r_Aug_state.ang_c_to_b * l_aug_state.ang_c_to_b;
    }
  }

  return l_state;
}

State & operator+=(State & l_state, Eigen::VectorXd & r_vector)
{
  Eigen::VectorXd r_m_body_state = r_vector.segment<18>(0);
  l_state.m_body_state += r_m_body_state;

  unsigned int n = 18;
  for (auto & imu_iter : l_state.m_imu_states) {
    unsigned int imu_id = imu_iter.first;
    if (imu_iter.second.is_extrinsic) {
      l_state.m_imu_states[imu_id].pos_i_in_b += r_vector.segment<3>(n + 0);
      l_state.m_imu_states[imu_id].ang_i_to_b =
        l_state.m_imu_states[imu_id].ang_i_to_b * RotVecToQuat(r_vector.segment<3>(n + 3));
      n += 6;
    }
    if (imu_iter.second.is_intrinsic) {
      l_state.m_imu_states[imu_id].acc_bias += r_vector.segment<3>(n + 0);
      l_state.m_imu_states[imu_id].omg_bias += r_vector.segment<3>(n + 3);
      n += 6;
    }
  }

  for (auto & cam_iter : l_state.m_cam_states) {
    cam_iter.second.pos_c_in_b += r_vector.segment<3>(n + 0);
    cam_iter.second.ang_c_to_b =
      cam_iter.second.ang_c_to_b * RotVecToQuat(r_vector.segment<3>(n + 3));
    n += 6;
    for (unsigned int i = 0; i < cam_iter.second.augmented_states.size(); ++i) {
      cam_iter.second.augmented_states[i].pos_b_in_g += r_vector.segment<3>(n + 0);
      cam_iter.second.augmented_states[i].ang_b_to_g =
        cam_iter.second.augmented_states[i].ang_b_to_g *
        RotVecToQuat(r_vector.segment<3>(n + 3));
      cam_iter.second.augmented_states[i].pos_c_in_b += r_vector.segment<3>(n + 6);
      cam_iter.second.augmented_states[i].ang_c_to_b =
        cam_iter.second.augmented_states[i].ang_c_to_b *
        RotVecToQuat(r_vector.segment<3>(n + 9));
      n += 12;
    }
  }

  return l_state;
}

std::map<unsigned int, ImuState> & operator+=(
  std::map<unsigned int, ImuState> & l_imu_state, Eigen::VectorXd & r_vector)
{
  unsigned int n {0};
  for (auto & imu_iter : l_imu_state) {
    unsigned int imu_id = imu_iter.first;
    if (l_imu_state[imu_id].is_extrinsic) {
      l_imu_state[imu_id].pos_i_in_b += r_vector.segment<3>(n + 0);
      l_imu_state[imu_id].ang_i_to_b = l_imu_state[imu_id].ang_i_to_b * RotVecToQuat(
        r_vector.segment<3>(n + 3));
      n += 6;
    }
    if (l_imu_state[imu_id].is_intrinsic) {
      l_imu_state[imu_id].acc_bias += r_vector.segment<3>(n + 0);
      l_imu_state[imu_id].omg_bias += r_vector.segment<3>(n + 3);
      n += 6;
    }
  }

  return l_imu_state;
}

std::map<unsigned int, CamState> & operator+=(
  std::map<unsigned int, CamState> & lCamState, Eigen::VectorXd & r_vector)
{
  unsigned int n {0};
  for (auto & cam_iter : lCamState) {
    unsigned int cam_id = cam_iter.first;
    lCamState[cam_id].pos_c_in_b += r_vector.segment<3>(n + 0);
    lCamState[cam_id].ang_c_to_b =
      lCamState[cam_id].ang_c_to_b * RotVecToQuat(r_vector.segment<3>(n + 3));
    unsigned int augSize = lCamState[cam_id].augmented_states.size() * 12U;
    Eigen::VectorXd augUpdate = r_vector.segment(n + 6, augSize);
    lCamState[cam_id].augmented_states += augUpdate;
    n += 6 + augSize;
  }

  return lCamState;
}


std::vector<AugmentedState> & operator+=(
  std::vector<AugmentedState> & l_aug_state, Eigen::VectorXd & r_vector)
{
  unsigned int n {0};
  for (auto & aug_iter : l_aug_state) {
    aug_iter.pos_b_in_g += r_vector.segment<3>(n + 0);
    aug_iter.ang_b_to_g = aug_iter.ang_b_to_g * RotVecToQuat(r_vector.segment<3>(n + 3));
    aug_iter.pos_c_in_b += r_vector.segment<3>(n + 6);
    aug_iter.ang_c_to_b = aug_iter.ang_c_to_b * RotVecToQuat(r_vector.segment<3>(n + 9));
    n += 12;
  }

  return l_aug_state;
}


Eigen::VectorXd BodyState::ToVector()
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(18);

  out_vec.segment<3>(0) = m_position;
  out_vec.segment<3>(3) = m_velocity;
  out_vec.segment<3>(6) = m_acceleration;
  out_vec.segment<3>(9) = QuatToRotVec(m_ang_b_to_g);
  out_vec.segment<3>(12) = m_angular_velocity;
  out_vec.segment<3>(15) = m_angular_acceleration;

  return out_vec;
}

Eigen::VectorXd CamState::ToVector()
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(6 + 12 * augmented_states.size());

  out_vec.segment<3>(0) = pos_c_in_b;
  out_vec.segment<3>(3) = QuatToRotVec(ang_c_to_b);

  unsigned int n = 6;
  for (auto const & aug_state : augmented_states) {
    out_vec.segment<3>(n + 0) = aug_state.pos_b_in_g;
    out_vec.segment<3>(n + 3) = QuatToRotVec(aug_state.ang_b_to_g);
    out_vec.segment<3>(n + 6) = aug_state.pos_c_in_b;
    out_vec.segment<3>(n + 9) = QuatToRotVec(aug_state.ang_c_to_b);
    n += 12;
  }

  return out_vec;
}

Eigen::VectorXd ImuState::ToVector()
{
  Eigen::VectorXd out_vec;

  if (is_extrinsic && is_intrinsic) {
    out_vec = Eigen::VectorXd::Zero(12);
    out_vec.segment<3>(0) = pos_i_in_b;
    out_vec.segment<3>(3) = QuatToRotVec(ang_i_to_b);
    out_vec.segment<3>(6) = acc_bias;
    out_vec.segment<3>(9) = omg_bias;
  } else if (is_extrinsic) {
    out_vec = Eigen::VectorXd::Zero(6);
    out_vec.segment<3>(0) = pos_i_in_b;
    out_vec.segment<3>(3) = QuatToRotVec(ang_i_to_b);
  } else if (is_intrinsic) {
    out_vec = Eigen::VectorXd::Zero(6);
    out_vec.segment<3>(6) = acc_bias;
    out_vec.segment<3>(9) = omg_bias;
  } else if (is_extrinsic) {
    out_vec = Eigen::VectorXd::Zero(0);
  }

  return out_vec;
}

void BodyState::SetState(Eigen::VectorXd state)
{
  m_position = state.segment<3>(0);
  m_velocity = state.segment<3>(3);
  m_acceleration = state.segment<3>(6);
  m_ang_b_to_g = RotVecToQuat(state.segment<3>(9));
  m_angular_velocity = state.segment<3>(12);
  m_angular_acceleration = state.segment<3>(15);
}


Eigen::VectorXd State::ToVector()
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(GetStateSize());

  out_vec.segment<g_body_state_size>(0) = m_body_state.ToVector();
  unsigned int n = g_body_state_size;

  for (auto const & imu_iter : m_imu_states) {
    if (imu_iter.second.is_extrinsic) {
      out_vec.segment<3>(n + 0) = imu_iter.second.pos_i_in_b;
      out_vec.segment<3>(n + 3) = QuatToRotVec(imu_iter.second.ang_i_to_b);
      n += g_imu_extrinsic_state_size;
    }
    if (imu_iter.second.is_intrinsic) {
      out_vec.segment<3>(n + 0) = imu_iter.second.acc_bias;
      out_vec.segment<3>(n + 3) = imu_iter.second.omg_bias;
      n += g_imu_intrinsic_state_size;
    }
  }

  for (auto const & cam_iter : m_cam_states) {
    out_vec.segment<3>(n + 0) = cam_iter.second.pos_c_in_b;
    out_vec.segment<3>(n + 3) = QuatToRotVec(cam_iter.second.ang_c_to_b);
    n += g_cam_state_size;
    for (auto const & aug_state : cam_iter.second.augmented_states) {
      out_vec.segment<3>(n + 0) = aug_state.pos_b_in_g;
      out_vec.segment<3>(n + 3) = QuatToRotVec(aug_state.ang_b_to_g);
      out_vec.segment<3>(n + 6) = aug_state.pos_c_in_b;
      out_vec.segment<3>(n + 9) = QuatToRotVec(aug_state.ang_c_to_b);
      n += g_aug_state_size;
    }
  }

  return out_vec;
}


unsigned int State::GetStateSize()
{
  unsigned int state_size = g_body_state_size;

  for (auto const & imu_iter : m_imu_states) {
    if (imu_iter.second.is_extrinsic) {
      state_size += g_imu_extrinsic_state_size;
    }
    if (imu_iter.second.is_intrinsic) {
      state_size += g_imu_intrinsic_state_size;
    }
  }

  for (auto const & cam_iter : m_cam_states) {
    state_size += 6 + 12 * cam_iter.second.augmented_states.size();
  }
  return state_size;
}
