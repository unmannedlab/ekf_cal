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

BodyState & operator+=(BodyState & l_body_state, BodyState & r_body_state)
{
  l_body_state.pos_b_in_l += r_body_state.pos_b_in_l;
  l_body_state.vel_b_in_l += r_body_state.vel_b_in_l;
  l_body_state.acc_b_in_l += r_body_state.acc_b_in_l;
  l_body_state.ang_b_to_l = r_body_state.ang_b_to_l * l_body_state.ang_b_to_l;
  l_body_state.ang_vel_b_in_l += r_body_state.ang_vel_b_in_l;
  l_body_state.ang_acc_b_in_l += r_body_state.ang_acc_b_in_l;

  return l_body_state;
}

BodyState & operator+=(BodyState & l_body_state, Eigen::VectorXd & r_vector)
{
  l_body_state.pos_b_in_l += r_vector.segment<3>(0);
  l_body_state.vel_b_in_l += r_vector.segment<3>(3);
  l_body_state.acc_b_in_l += r_vector.segment<3>(6);
  l_body_state.ang_b_to_l =
    l_body_state.ang_b_to_l * RotVecToQuat(r_vector.segment<3>(9));
  l_body_state.ang_vel_b_in_l += r_vector.segment<3>(12);
  l_body_state.ang_acc_b_in_l += r_vector.segment<3>(15);

  return l_body_state;
}

State & operator+=(State & l_state, State & r_state)
{
  l_state.body_state += r_state.body_state;

  for (auto & imu_iter : l_state.imu_states) {
    unsigned int imu_id = imu_iter.first;
    l_state.imu_states[imu_id].pos_i_in_b += r_state.imu_states[imu_id].pos_i_in_b;
    l_state.imu_states[imu_id].ang_i_to_b =
      r_state.imu_states[imu_id].ang_i_to_b * l_state.imu_states[imu_id].ang_i_to_b;
    l_state.imu_states[imu_id].acc_bias += r_state.imu_states[imu_id].acc_bias;
    l_state.imu_states[imu_id].omg_bias += r_state.imu_states[imu_id].omg_bias;
  }

  for (auto & gps_iter : l_state.gps_states) {
    unsigned int gps_id = gps_iter.first;
    l_state.gps_states[gps_id].pos_a_in_b += r_state.gps_states[gps_id].pos_a_in_b;
  }

  for (auto & cam_iter : l_state.cam_states) {
    unsigned int cam_id = cam_iter.first;
    l_state.cam_states[cam_id].pos_c_in_b += r_state.cam_states[cam_id].pos_c_in_b;
    l_state.cam_states[cam_id].ang_c_to_b =
      r_state.cam_states[cam_id].ang_c_to_b * l_state.cam_states[cam_id].ang_c_to_b;
    for (unsigned int i = 0; i < l_state.cam_states[cam_id].augmented_states.size(); ++i) {
      AugmentedState & l_aug_state = l_state.cam_states[cam_id].augmented_states[i];
      AugmentedState & r_aug_state = r_state.cam_states[cam_id].augmented_states[i];
      l_aug_state.pos_b_in_l += r_aug_state.pos_b_in_l;
      l_aug_state.ang_b_to_l = r_aug_state.ang_b_to_l * l_aug_state.ang_b_to_l;
      l_aug_state.pos_c_in_b += r_aug_state.pos_c_in_b;
      l_aug_state.ang_c_to_b = r_aug_state.ang_c_to_b * l_aug_state.ang_c_to_b;
    }
  }

  for (auto & fid_iter : l_state.fid_states) {
    unsigned int fid_id = fid_iter.first;
    l_state.fid_states[fid_id].pos_f_in_l += r_state.fid_states[fid_id].pos_f_in_l;
    l_state.fid_states[fid_id].ang_f_to_l =
      r_state.fid_states[fid_id].ang_f_to_l * l_state.fid_states[fid_id].ang_f_to_l;
  }

  return l_state;
}

State & operator+=(State & l_state, Eigen::VectorXd & r_vector)
{
  Eigen::VectorXd r_body_state = r_vector.segment<g_body_state_size>(0);
  l_state.body_state += r_body_state;

  unsigned int n = g_body_state_size;
  for (auto & imu_iter : l_state.imu_states) {
    if (imu_iter.second.is_extrinsic) {
      imu_iter.second.pos_i_in_b += r_vector.segment<3>(n + 0);
      imu_iter.second.ang_i_to_b =
        imu_iter.second.ang_i_to_b * RotVecToQuat(r_vector.segment<3>(n + 3));
      n += g_imu_extrinsic_state_size;
    }
    if (imu_iter.second.is_intrinsic) {
      imu_iter.second.acc_bias += r_vector.segment<3>(n + 0);
      imu_iter.second.omg_bias += r_vector.segment<3>(n + 3);
      n += g_imu_intrinsic_state_size;
    }
  }

  for (auto & gps_iter : l_state.gps_states) {
    if (gps_iter.second.is_extrinsic) {
      gps_iter.second.pos_a_in_b += r_vector.segment<3>(n);
      n += g_gps_extrinsic_state_size;
    }
  }

  for (auto & cam_iter : l_state.cam_states) {
    cam_iter.second.pos_c_in_b += r_vector.segment<3>(n + 0);
    cam_iter.second.ang_c_to_b =
      cam_iter.second.ang_c_to_b * RotVecToQuat(r_vector.segment<3>(n + 3));
    n += g_cam_state_size;
    for (unsigned int i = 0; i < cam_iter.second.augmented_states.size(); ++i) {
      cam_iter.second.augmented_states[i].pos_b_in_l += r_vector.segment<3>(n + 0);
      cam_iter.second.augmented_states[i].ang_b_to_l =
        cam_iter.second.augmented_states[i].ang_b_to_l *
        RotVecToQuat(r_vector.segment<3>(n + 3));
      cam_iter.second.augmented_states[i].pos_c_in_b += r_vector.segment<3>(n + 6);
      cam_iter.second.augmented_states[i].ang_c_to_b =
        cam_iter.second.augmented_states[i].ang_c_to_b *
        RotVecToQuat(r_vector.segment<3>(n + 9));
      n += g_aug_state_size;
    }
  }

  for (auto & fid_iter : l_state.fid_states) {
    if (fid_iter.second.is_extrinsic) {
      fid_iter.second.pos_f_in_l += r_vector.segment<3>(n);
      fid_iter.second.ang_f_to_l =
        fid_iter.second.ang_f_to_l * RotVecToQuat(r_vector.segment<3>(n + 3));
      n += g_fid_extrinsic_state_size;
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
      n += g_imu_extrinsic_state_size;
    }
    if (l_imu_state[imu_id].is_intrinsic) {
      l_imu_state[imu_id].acc_bias += r_vector.segment<3>(n + 0);
      l_imu_state[imu_id].omg_bias += r_vector.segment<3>(n + 3);
      n += g_imu_intrinsic_state_size;
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
    unsigned int aug_size = lCamState[cam_id].augmented_states.size() * g_aug_state_size;
    Eigen::VectorXd augUpdate = r_vector.segment(n + g_cam_state_size, aug_size);
    lCamState[cam_id].augmented_states += augUpdate;
    n += g_cam_state_size + aug_size;
  }

  return lCamState;
}

std::map<unsigned int, FidState> & operator+=(
  std::map<unsigned int, FidState> & l_fid_state, Eigen::VectorXd & r_vector)
{
  unsigned int n {0};
  for (auto & fid_iter : l_fid_state) {
    unsigned int fid_id = fid_iter.first;
    if (l_fid_state[fid_id].is_extrinsic) {
      l_fid_state[fid_id].pos_f_in_l += r_vector.segment<3>(n + 0);
      l_fid_state[fid_id].ang_f_to_l =
        l_fid_state[fid_id].ang_f_to_l * RotVecToQuat(r_vector.segment<3>(n + 3));
      n += g_fid_extrinsic_state_size;
    }
  }

  return l_fid_state;
}

std::vector<AugmentedState> & operator+=(
  std::vector<AugmentedState> & l_aug_state, Eigen::VectorXd & r_vector)
{
  unsigned int n {0};
  for (auto & aug_iter : l_aug_state) {
    aug_iter.pos_b_in_l += r_vector.segment<3>(n + 0);
    aug_iter.ang_b_to_l = aug_iter.ang_b_to_l * RotVecToQuat(r_vector.segment<3>(n + 3));
    aug_iter.pos_c_in_b += r_vector.segment<3>(n + 6);
    aug_iter.ang_c_to_b = aug_iter.ang_c_to_b * RotVecToQuat(r_vector.segment<3>(n + 9));
    n += 12;
  }

  return l_aug_state;
}

Eigen::VectorXd BodyState::ToVector()
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(g_body_state_size);

  out_vec.segment<3>(0) = pos_b_in_l;
  out_vec.segment<3>(3) = vel_b_in_l;
  out_vec.segment<3>(6) = acc_b_in_l;
  out_vec.segment<3>(9) = QuatToRotVec(ang_b_to_l);
  out_vec.segment<3>(12) = ang_vel_b_in_l;
  out_vec.segment<3>(15) = ang_acc_b_in_l;

  return out_vec;
}

Eigen::VectorXd CamState::ToVector()
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(
    g_cam_state_size + g_aug_state_size * augmented_states.size());

  out_vec.segment<3>(0) = pos_c_in_b;
  out_vec.segment<3>(3) = QuatToRotVec(ang_c_to_b);

  unsigned int n = g_cam_state_size;
  for (auto const & aug_state : augmented_states) {
    out_vec.segment<3>(n + 0) = aug_state.pos_b_in_l;
    out_vec.segment<3>(n + 3) = QuatToRotVec(aug_state.ang_b_to_l);
    out_vec.segment<3>(n + 6) = aug_state.pos_c_in_b;
    out_vec.segment<3>(n + 9) = QuatToRotVec(aug_state.ang_c_to_b);
    n += g_aug_state_size;
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
    out_vec.segment<3>(0) = acc_bias;
    out_vec.segment<3>(3) = omg_bias;
  } else {
    out_vec = Eigen::VectorXd::Zero(0);
  }

  return out_vec;
}

Eigen::VectorXd GpsState::ToVector()
{
  return pos_a_in_b;
}

void BodyState::SetState(Eigen::VectorXd state)
{
  pos_b_in_l = state.segment<3>(0);
  vel_b_in_l = state.segment<3>(3);
  acc_b_in_l = state.segment<3>(6);
  ang_b_to_l = RotVecToQuat(state.segment<3>(9));
  ang_vel_b_in_l = state.segment<3>(12);
  ang_acc_b_in_l = state.segment<3>(15);
}


Eigen::VectorXd State::ToVector()
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(GetStateSize());

  out_vec.segment<g_body_state_size>(0) = body_state.ToVector();
  unsigned int n = g_body_state_size;

  for (auto const & imu_iter : imu_states) {
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

  for (auto const & gps_iter : gps_states) {
    if (gps_iter.second.is_extrinsic) {
      out_vec.segment<3>(n) = gps_iter.second.pos_a_in_b;
      n += g_gps_extrinsic_state_size;
    }
  }

  for (auto const & cam_iter : cam_states) {
    out_vec.segment<3>(n + 0) = cam_iter.second.pos_c_in_b;
    out_vec.segment<3>(n + 3) = QuatToRotVec(cam_iter.second.ang_c_to_b);
    n += g_cam_state_size;
    for (auto const & aug_state : cam_iter.second.augmented_states) {
      out_vec.segment<3>(n + 0) = aug_state.pos_b_in_l;
      out_vec.segment<3>(n + 3) = QuatToRotVec(aug_state.ang_b_to_l);
      out_vec.segment<3>(n + 6) = aug_state.pos_c_in_b;
      out_vec.segment<3>(n + 9) = QuatToRotVec(aug_state.ang_c_to_b);
      n += g_aug_state_size;
    }
  }

  for (auto const & fid_iter : fid_states) {
    if (fid_iter.second.is_extrinsic) {
      out_vec.segment<3>(n + 0) = fid_iter.second.pos_f_in_l;
      out_vec.segment<3>(n + 3) = QuatToRotVec(fid_iter.second.ang_f_to_l);
      n += g_fid_extrinsic_state_size;
    }
  }

  return out_vec;
}

unsigned int State::GetStateSize()
{
  unsigned int state_size = g_body_state_size;

  for (auto const & imu_iter : imu_states) {
    if (imu_iter.second.is_extrinsic) {
      state_size += g_imu_extrinsic_state_size;
    }
    if (imu_iter.second.is_intrinsic) {
      state_size += g_imu_intrinsic_state_size;
    }
  }

  for (auto const & gps_iter : gps_states) {
    if (gps_iter.second.is_extrinsic) {
      state_size += g_gps_extrinsic_state_size;
    }
  }

  for (auto const & cam_iter : cam_states) {
    state_size += g_cam_state_size + g_aug_state_size * cam_iter.second.augmented_states.size();
  }

  for (auto const & fid_iter : fid_states) {
    if (fid_iter.second.is_extrinsic) {
      state_size += g_fid_extrinsic_state_size;
    }
  }

  return state_size;
}
