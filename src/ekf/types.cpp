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
  }

  for (auto & fid_iter : l_state.fid_states) {
    unsigned int fid_id = fid_iter.first;
    l_state.fid_states[fid_id].pos_f_in_l += r_state.fid_states[fid_id].pos_f_in_l;
    l_state.fid_states[fid_id].ang_f_to_l =
      r_state.fid_states[fid_id].ang_f_to_l * l_state.fid_states[fid_id].ang_f_to_l;
  }

  for (auto & aug_iter : l_state.aug_states) {
    unsigned int aug_id = aug_iter.first;
    for (unsigned int i = 0; i < l_state.aug_states[aug_id].size(); ++i) {
      l_state.aug_states[aug_id][i].pos_b_in_l += r_state.aug_states[aug_id][i].pos_b_in_l;
      l_state.aug_states[aug_id][i].ang_b_to_l = r_state.aug_states[aug_id][i].ang_b_to_l *
        l_state.aug_states[aug_id][i].ang_b_to_l;
    }
  }

  return l_state;
}

State & operator+=(State & l_state, Eigen::VectorXd & r_vector)
{
  Eigen::VectorXd r_body_state = r_vector.segment<g_body_state_size>(0);
  l_state.body_state += r_body_state;

  unsigned int n = g_body_state_size;
  for (auto & imu_iter : l_state.imu_states) {
    if (imu_iter.second.get_is_extrinsic()) {
      imu_iter.second.pos_i_in_b += r_vector.segment<3>(n + 0);
      imu_iter.second.ang_i_to_b =
        imu_iter.second.ang_i_to_b * RotVecToQuat(r_vector.segment<3>(n + 3));
      n += g_imu_extrinsic_state_size;
    }
    if (imu_iter.second.get_is_intrinsic()) {
      imu_iter.second.acc_bias += r_vector.segment<3>(n + 0);
      imu_iter.second.omg_bias += r_vector.segment<3>(n + 3);
      n += g_imu_intrinsic_state_size;
    }
  }

  for (auto & gps_iter : l_state.gps_states) {
    if (gps_iter.second.get_is_extrinsic()) {
      gps_iter.second.pos_a_in_b += r_vector.segment<3>(n);
      n += g_gps_extrinsic_state_size;
    }
  }

  for (auto & cam_iter : l_state.cam_states) {
    cam_iter.second.pos_c_in_b += r_vector.segment<3>(n + 0);
    cam_iter.second.ang_c_to_b =
      cam_iter.second.ang_c_to_b * RotVecToQuat(r_vector.segment<3>(n + 3));
    n += g_cam_state_size;
  }

  for (auto & fid_iter : l_state.fid_states) {
    if (fid_iter.second.get_is_extrinsic()) {
      fid_iter.second.pos_f_in_l += r_vector.segment<3>(n);
      fid_iter.second.ang_f_to_l =
        fid_iter.second.ang_f_to_l * RotVecToQuat(r_vector.segment<3>(n + 3));
      n += g_fid_extrinsic_state_size;
    }
  }

  for (auto & aug_iter : l_state.aug_states) {
    unsigned int aug_id = aug_iter.first;
    for (unsigned int i = 0; i < l_state.aug_states[aug_id].size(); ++i) {
      l_state.aug_states[aug_id][i].pos_b_in_l += r_vector.segment<3>(n + 0);
      l_state.aug_states[aug_id][i].ang_b_to_l = l_state.aug_states[aug_id][i].ang_b_to_l *
        RotVecToQuat(r_vector.segment<3>(n + 3));
      n += g_aug_state_size;
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
    if (l_imu_state[imu_id].get_is_extrinsic()) {
      l_imu_state[imu_id].pos_i_in_b += r_vector.segment<3>(n + 0);
      l_imu_state[imu_id].ang_i_to_b = l_imu_state[imu_id].ang_i_to_b * RotVecToQuat(
        r_vector.segment<3>(n + 3));
      n += g_imu_extrinsic_state_size;
    }
    if (l_imu_state[imu_id].get_is_intrinsic()) {
      l_imu_state[imu_id].acc_bias += r_vector.segment<3>(n + 0);
      l_imu_state[imu_id].omg_bias += r_vector.segment<3>(n + 3);
      n += g_imu_intrinsic_state_size;
    }
  }

  return l_imu_state;
}

std::map<unsigned int, CamState> & operator+=(
  std::map<unsigned int, CamState> & l_cam_state, Eigen::VectorXd & r_vector)
{
  unsigned int n {0};
  for (auto & cam_iter : l_cam_state) {
    unsigned int cam_id = cam_iter.first;
    l_cam_state[cam_id].pos_c_in_b += r_vector.segment<3>(n + 0);
    l_cam_state[cam_id].ang_c_to_b =
      l_cam_state[cam_id].ang_c_to_b * RotVecToQuat(r_vector.segment<3>(n + 3));
  }

  return l_cam_state;
}

std::map<unsigned int, FidState> & operator+=(
  std::map<unsigned int, FidState> & l_fid_state, Eigen::VectorXd & r_vector)
{
  unsigned int n {0};
  for (auto & fid_iter : l_fid_state) {
    unsigned int fid_id = fid_iter.first;
    if (l_fid_state[fid_id].get_is_extrinsic()) {
      l_fid_state[fid_id].pos_f_in_l += r_vector.segment<3>(n + 0);
      l_fid_state[fid_id].ang_f_to_l =
        l_fid_state[fid_id].ang_f_to_l * RotVecToQuat(r_vector.segment<3>(n + 3));
      n += g_fid_extrinsic_state_size;
    }
  }

  return l_fid_state;
}

std::vector<AugState> & operator+=(
  std::vector<AugState> & l_aug_state, Eigen::VectorXd & r_vector)
{
  unsigned int n {0};
  for (auto & aug_iter : l_aug_state) {
    aug_iter.pos_b_in_l += r_vector.segment<3>(n + 0);
    aug_iter.ang_b_to_l = aug_iter.ang_b_to_l * RotVecToQuat(r_vector.segment<3>(n + 3));
    n += g_aug_state_size;
  }

  return l_aug_state;
}

Eigen::VectorXd BodyState::ToVector() const
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

Eigen::VectorXd CamState::ToVector() const
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(g_cam_state_size);

  out_vec.segment<3>(0) = pos_c_in_b;
  out_vec.segment<3>(3) = QuatToRotVec(ang_c_to_b);

  return out_vec;
}

Eigen::VectorXd AugState::ToVector() const
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(g_aug_state_size);

  out_vec.segment<3>(0) = pos_b_in_l;
  out_vec.segment<3>(3) = QuatToRotVec(ang_b_to_l);

  return out_vec;
}

Eigen::VectorXd ImuState::ToVector() const
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(size);
  unsigned int n {0};

  if (is_extrinsic) {
    out_vec.segment<3>(n + 0) = pos_i_in_b;
    out_vec.segment<3>(n + 3) = QuatToRotVec(ang_i_to_b);
    n += g_imu_extrinsic_state_size;
  }
  if (is_intrinsic) {
    out_vec.segment<3>(n + 0) = acc_bias;
    out_vec.segment<3>(n + 3) = omg_bias;
    n += g_imu_intrinsic_state_size;
  }

  return out_vec;
}

Eigen::VectorXd GpsState::ToVector() const
{
  return pos_a_in_b;
}

Eigen::VectorXd FidState::ToVector() const
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(size);

  if (is_extrinsic) {
    out_vec.segment<3>(0) = pos_f_in_l;
    out_vec.segment<3>(3) = QuatToRotVec(ang_f_to_l);
  }

  return out_vec;
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


Eigen::VectorXd State::ToVector() const
{
  Eigen::VectorXd out_vec = Eigen::VectorXd::Zero(GetStateSize());

  out_vec.segment<g_body_state_size>(0) = body_state.ToVector();
  unsigned int n = g_body_state_size;

  for (auto const & imu_iter : imu_states) {
    if (imu_iter.second.get_is_extrinsic() || imu_iter.second.get_is_extrinsic()) {
      Eigen::VectorXd temp_vec = imu_iter.second.ToVector();
      out_vec.segment(n, temp_vec.size()) = temp_vec;
      n += temp_vec.size();
    }
  }

  for (auto const & gps_iter : gps_states) {
    if (gps_iter.second.get_is_extrinsic()) {
      out_vec.segment<3>(n) = gps_iter.second.pos_a_in_b;
      n += g_gps_extrinsic_state_size;
    }
  }

  for (auto const & cam_iter : cam_states) {
    Eigen::VectorXd temp_vec = cam_iter.second.ToVector();
    out_vec.segment(n, temp_vec.size()) = temp_vec;
    n += temp_vec.size();
  }

  for (auto const & fid_iter : fid_states) {
    if (fid_iter.second.get_is_extrinsic()) {
      out_vec.segment<3>(n + 0) = fid_iter.second.pos_f_in_l;
      out_vec.segment<3>(n + 3) = QuatToRotVec(fid_iter.second.ang_f_to_l);
      n += g_fid_extrinsic_state_size;
    }
  }

  for (auto const & aug_iter : aug_states) {
    for (unsigned int i = 0; i < aug_iter.second.size(); ++i) {
      out_vec.segment<3>(n + 0) = aug_iter.second[i].pos_b_in_l;
      out_vec.segment<3>(n + 3) = QuatToRotVec(aug_iter.second[i].ang_b_to_l);
      n += g_aug_state_size;
    }
  }

  return out_vec;
}

unsigned int State::GetStateSize() const
{
  unsigned int state_size = g_body_state_size;

  for (auto const & imu_iter : imu_states) {
    if (imu_iter.second.get_is_extrinsic()) {
      state_size += g_imu_extrinsic_state_size;
    }
    if (imu_iter.second.get_is_intrinsic()) {
      state_size += g_imu_intrinsic_state_size;
    }
  }

  for (auto const & gps_iter : gps_states) {
    if (gps_iter.second.get_is_extrinsic()) {
      state_size += g_gps_extrinsic_state_size;
    }
  }

  state_size += g_cam_state_size * cam_states.size();

  for (auto const & fid_iter : fid_states) {
    if (fid_iter.second.get_is_extrinsic()) {
      state_size += g_fid_extrinsic_state_size;
    }
  }

  state_size += g_aug_state_size * aug_states.size();

  return state_size;
}

bool ImuState::get_is_extrinsic() const
{
  return is_extrinsic;
}

bool ImuState::get_is_intrinsic() const
{
  return is_intrinsic;
}

bool GpsState::get_is_extrinsic() const
{
  return is_extrinsic;
}

bool FidState::get_is_extrinsic() const
{
  return is_extrinsic;
}

void ImuState::set_is_extrinsic(bool extrinsic)
{
  is_extrinsic = extrinsic;
  refresh_size();
}

void ImuState::set_is_intrinsic(bool intrinsic)
{
  is_intrinsic = intrinsic;
  refresh_size();
}

void ImuState::refresh_size()
{
  size = 0;
  if (is_extrinsic) {size += g_imu_extrinsic_state_size;}
  if (is_intrinsic) {size += g_imu_intrinsic_state_size;}
}

void GpsState::set_is_extrinsic(bool extrinsic)
{
  is_extrinsic = extrinsic;
  refresh_size();
}

void GpsState::refresh_size()
{
  size = 0;
  if (is_extrinsic) {size += g_gps_extrinsic_state_size;}
}

void FidState::set_is_extrinsic(bool extrinsic)
{
  is_extrinsic = extrinsic;
  refresh_size();
}

void FidState::refresh_size()
{
  size = 0;
  if (is_extrinsic) {size += g_imu_extrinsic_state_size;}
}

cv::Mat Intrinsics::ToCameraMatrix()
{
  cv::Mat camera_matrix = cv::Mat(3, 3, CV_64F, 0.0);

  camera_matrix.at<double>(0, 0) = f_x;
  camera_matrix.at<double>(1, 1) = f_y;
  camera_matrix.at<double>(0, 2) = width / 2;
  camera_matrix.at<double>(1, 2) = height / 2;
  camera_matrix.at<double>(2, 2) = 1.0;

  return camera_matrix;
}

cv::Mat Intrinsics::ToDistortionVector()
{
  cv::Mat distortion_vector = cv::Mat(4, 1, CV_64F, 0.0);

  distortion_vector.at<double>(0) = k_1;
  distortion_vector.at<double>(1) = k_2;
  distortion_vector.at<double>(2) = p_1;
  distortion_vector.at<double>(3) = p_2;

  return distortion_vector;
}
