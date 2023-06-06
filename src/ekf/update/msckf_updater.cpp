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

#include "ekf/update/msckf_updater.hpp"

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <chrono>
#include <string>

#include "utility/math_helper.hpp"
#include "utility/string_helper.hpp"
#include "ekf/constants.hpp"

MsckfUpdater::MsckfUpdater(
  unsigned int cam_id, std::string log_file_directory,
  bool data_logging_on)
: Updater(cam_id), m_data_logger(log_file_directory, "msckf_" + std::to_string(cam_id) + ".csv")
{
  std::stringstream msg;
  msg << "time";
  msg << EnumerateHeader("body_state", g_body_state_size);
  msg << EnumerateHeader("cam_state", g_cam_state_size);
  msg << EnumerateHeader("body_update", g_body_state_size);
  msg << EnumerateHeader("cam_update", g_cam_state_size);
  msg << EnumerateHeader("time", 1);
  msg << "\n";

  m_data_logger.DefineHeader(msg.str());
  m_data_logger.SetLogging(data_logging_on);
}

AugmentedState MsckfUpdater::MatchState(
  unsigned int frame_id)
{
  AugmentedState aug_state_match;

  for (auto & aug_state : m_aug_states) {
    if (aug_state.frame_id == frame_id) {
      aug_state_match = aug_state;
      break;
    }
  }

  return aug_state_match;
}

/// @todo possible move into separate source for re-compilation speed
void MsckfUpdater::ApplyLeftNullspace(
  Eigen::MatrixXd & H_f,
  Eigen::MatrixXd & H_x,
  Eigen::VectorXd & res)
{
  // Apply the left nullspace of H_f to the jacobians and the residual
  Eigen::JacobiRotation<double> givens;
  for (int n = 0; n < H_f.cols(); ++n) {
    for (int m = static_cast<int>(H_f.rows()) - 2; m >= n; --m) {
      // Givens matrix G
      givens.makeGivens(H_f(m, n), H_f(m, n));

      // Apply nullspace
      (H_f.block(m, n, 2, H_f.cols() - n)).applyOnTheLeft(0, 1, givens.adjoint());
      (H_x.block(m, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, givens.adjoint());
      (res.block(m, 0, 2, 1)).applyOnTheLeft(0, 1, givens.adjoint());
    }
  }

  H_x = H_x.block(H_f.cols(), 0, H_x.rows() - H_f.cols(), H_x.cols()).eval();
  res = res.block(H_f.cols(), 0, res.rows() - H_f.cols(), res.cols()).eval();
}

/// @todo possible move into separate source for re-compilation speed
void MsckfUpdater::CompressMeasurements(Eigen::MatrixXd & jacobian, Eigen::VectorXd & residual)
{
  // Cannot compress fat matrices
  if (jacobian.rows() > jacobian.cols()) {
    Eigen::JacobiRotation<double> givens;
    for (int n = 0; n < jacobian.cols(); n++) {
      for (int m = static_cast<int>(jacobian.rows()) - 2; m >= n; --m) {
        // Givens matrix
        givens.makeGivens(jacobian(m, n), jacobian(m, n));

        // Compress measurements
        (jacobian.block(m, n, 2, jacobian.cols() - n)).applyOnTheLeft(0, 1, givens.adjoint());
        (residual.block(m, 0, 2, 1)).applyOnTheLeft(0, 1, givens.adjoint());
      }
    }
  }

  // Jacobian is ill-formed if either rows or columns are size 1
  int r = std::min(jacobian.rows(), jacobian.cols());
  if (r == 1) {
    return;
  }

  // Construct the smaller jacobian and residual after measurement compression
  jacobian.conservativeResize(r, jacobian.cols());
  residual.conservativeResize(r, residual.cols());
}

/// @todo possible move into separate source for re-compilation speed
Eigen::Vector3d MsckfUpdater::TriangulateFeature(std::vector<FeatureTrack> & feature_track)
{
  AugmentedState aug_state_0 = MatchState(feature_track[0].frame_id);

  // 3D Cartesian Triangulation
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  Eigen::Vector3d b = Eigen::Vector3d::Zero();

  const Eigen::Matrix<double, 3, 3> rotation_c0_to_g = aug_state_0.orientation.toRotationMatrix();
  const Eigen::Matrix<double, 3, 3> rotation_g_to_c0 = rotation_c0_to_g.transpose();
  const Eigen::Matrix<double, 3, 1> position_c0_in_g = aug_state_0.position;

  for (unsigned int i = 0; i < feature_track.size(); ++i) {
    AugmentedState aug_state_i = MatchState(feature_track[i].frame_id);

    const Eigen::Matrix<double, 3, 3> rotation_ci_to_g = aug_state_i.orientation.toRotationMatrix();
    const Eigen::Vector3d position_ci_in_g = aug_state_i.position;

    // Convert current position relative to anchor
    Eigen::Matrix3d rotation_ci_to_c0 = rotation_ci_to_g * rotation_g_to_c0;
    Eigen::Vector3d position_ci_in_c0 = rotation_g_to_c0 * (position_ci_in_g - position_c0_in_g);

    // Get the UV coordinate normal
    Eigen::Vector3d b_i;
    b_i(0) = (feature_track[i].key_point.pt.x - (static_cast<double>(m_image_width) / 2)) / 200.0;
    b_i(1) = (feature_track[i].key_point.pt.y - (static_cast<double>(m_image_height) / 2)) / 200.0;
    b_i(2) = 1;

    // Rotate and normalize
    b_i = rotation_ci_to_c0 * b_i;
    b_i = b_i / b_i.norm();

    Eigen::Matrix3d b_i_skew = SkewSymmetric(b_i);
    Eigen::Matrix3d A_i = b_i_skew.transpose() * b_i_skew;
    A += A_i;
    b += A_i * position_ci_in_c0;
  }

  // Solve linear triangulation for 3D cartesian estimate of feature position
  Eigen::Vector3d position_f_in_c0 = A.colPivHouseholderQr().solve(b);
  Eigen::Vector3d position_f_in_g = rotation_c0_to_g * position_f_in_c0 + position_c0_in_g;

  /// @todo condition check
  /// @todo max and min distance check

  return position_f_in_g;
}


void MsckfUpdater::UpdateEKF(double time, unsigned int camera_id, FeatureTracks feature_tracks)
{
  m_ekf->ProcessModel(time);
  RefreshStates();
  auto t_start = std::chrono::high_resolution_clock::now();

  m_logger->Log(LogLevel::DEBUG, "Called update_msckf for camera ID: " + std::to_string(camera_id));

  if (feature_tracks.size() == 0) {
    return;
  }

  // Calculate the max possible measurement size
  unsigned int max_meas_size = 0;
  for (unsigned int i = 0; i < feature_tracks.size(); ++i) {
    max_meas_size += 2 * feature_tracks[i].size();
  }

  unsigned int ct_meas = 0;
  unsigned int state_size = m_ekf->GetState().GetStateSize();
  unsigned int cam_state_start = m_ekf->GetCamStateStartIndex(camera_id);

  Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, state_size);

  m_logger->Log(LogLevel::DEBUG, "Update track count: " + std::to_string(feature_tracks.size()));

  // MSCKF Update
  for (auto & feature_track : feature_tracks) {
    m_logger->Log(LogLevel::DEBUG, "Feature Track size: " + std::to_string(feature_track.size()));

    // Get triangulated estimate of feature position
    Eigen::Vector3d pos_f_in_g = TriangulateFeature(feature_track);
    /// @todo Additional non-linear optimization

    if (pos_f_in_g.norm() < 1e-3) {
      m_logger->Log(LogLevel::DEBUG, "MSCKF Triangulation is Zero");
      continue;
    }

    AugmentedState aug_state_0 = MatchState(feature_track[0].frame_id);

    Eigen::Matrix3d rot_c0_to_g = aug_state_0.orientation.toRotationMatrix();
    // Eigen::Matrix3d rot_i0_to_g = aug_state_0.imu_orientation.toRotationMatrix();

    // Anchor pose orientation and position
    Eigen::Vector3d pos_i_in_g = aug_state_0.imu_position;

    // Allocate our residual and Jacobians
    int jacob_size = 3;

    unsigned int aug_state_size = g_aug_state_size *
      m_ekf->GetCamState(camera_id).augmented_states.size();
    Eigen::VectorXd res = Eigen::VectorXd::Zero(2 * feature_track.size());
    Eigen::MatrixXd H_f = Eigen::MatrixXd::Zero(2 * feature_track.size(), jacob_size);
    Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(2 * feature_track.size(), aug_state_size);

    // Jacobian for our anchor pose
    Eigen::Matrix<double, 3, 6> H_anc;
    H_anc.setZero();

    /// @todo CHECK
    // H_anc.block(0, 0, 3, 3).noalias() =
    //   -rot_i0_to_g * SkewSymmetric(rot_i0_to_g.transpose() * (pos_f_in_g - pos_i_in_g));
    H_anc.block(0, 3, 3, 3).setIdentity();

    // Get calibration Jacobians (for anchor clone)
    Eigen::Matrix<double, 3, 6> H_calib;
    H_calib.setZero();

    /// @todo CHECK
    // H_calib.block(0, 0, 3, 3).noalias() =
    //   -rot_c0_to_g * SkewSymmetric(rot_c0_to_g.transpose() * (pos_f_in_g - pos_i_in_g));
    H_calib.block(0, 3, 3, 3) = -rot_c0_to_g;

    // Get the Jacobian for this feature
    // Loop through each camera for this feature
    for (unsigned int i = 1; i < feature_track.size(); ++i) {
      AugmentedState aug_state_i = MatchState(feature_track[i].frame_id);

      // Our calibration between the IMU and CAMi frames
      Eigen::Matrix3d rot_ci_to_g = aug_state_i.orientation.toRotationMatrix();
      Eigen::Vector3d pos_ci_in_g = aug_state_i.position;

      // Project the current feature into the current frame of reference
      Eigen::Vector3d pos_f_in_ci = rot_ci_to_g.transpose() * (pos_f_in_g - pos_ci_in_g);
      Eigen::Vector2d uv_predicted;
      uv_predicted(0) = pos_f_in_ci(0) / pos_f_in_ci(2);
      uv_predicted(1) = pos_f_in_ci(1) / pos_f_in_ci(2);

      // Our residual
      Eigen::Vector2d uv_measured;
      uv_measured(0) = feature_track[i].key_point.pt.x;
      uv_measured(1) = feature_track[i].key_point.pt.y;
      res.block(2 * (i - 1), 0, 2, 1) = uv_measured - uv_predicted;

      // Normalized coordinates in respect to projection function
      Eigen::MatrixXd dzn_dPFC = Eigen::MatrixXd::Zero(2, 3);
      dzn_dPFC(0, 0) = 1 / pos_f_in_ci(2);
      dzn_dPFC(1, 1) = 1 / pos_f_in_ci(2);
      dzn_dPFC(0, 2) = -pos_f_in_ci(0) / (pos_f_in_ci(2) * pos_f_in_ci(2));
      dzn_dPFC(1, 1) = -pos_f_in_ci(1) / (pos_f_in_ci(2) * pos_f_in_ci(2));

      // Derivative of p_FinCi in respect to p_FinIi
      Eigen::MatrixXd dPFC_dPFG = rot_ci_to_g.transpose();

      // Derivative of p_FinCi in respect to camera clone state
      Eigen::MatrixXd dPFC_dClone = Eigen::MatrixXd::Zero(3, 6);
      dPFC_dClone.block(0, 0, 3, 3) = SkewSymmetric(pos_f_in_ci);
      dPFC_dClone.block(0, 3, 3, 3) = -dPFC_dPFG;

      unsigned int augStateStart =
        m_ekf->GetAugStateStartIndex(camera_id, feature_track[i].frame_id);

      // Precompute some matrices
      Eigen::MatrixXd dz_dPFC = dzn_dPFC;
      Eigen::MatrixXd dz_dPFG = dz_dPFC * dPFC_dPFG;

      // Get the total feature Jacobian
      H_f.block(2 * (i - 1), 0, 2, H_f.cols()) = dz_dPFG * rot_c0_to_g;

      // Get state clone Jacobian
      H_x.block(2 * (i - 1), 0, 2, 6) = dz_dPFC * dPFC_dClone;

      // loop through all extra states and add their
      // NOTE: we add the Jacobian here as we might be in the anchoring pose for this measurement
      H_x.block(2 * (i - 1), augStateStart - cam_state_start - 6, 2, 6) += dz_dPFG * H_anc;
      // H_x.block(2 * (i - 1), augStateStart - cam_state_start, 2, 6) += dz_dPFG * H_calib;

      // Calculate the Jacobian
      Eigen::MatrixXd dPFC_dCalib = Eigen::MatrixXd::Zero(3, 6);
      dPFC_dCalib.block(0, 0, 3, 3) =
        rot_ci_to_g.transpose() * SkewSymmetric(pos_f_in_g - pos_i_in_g);

      dPFC_dCalib.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

      // Add result to the state jacobian
      H_x.block(2 * (i - 1), cam_state_start - cam_state_start, 2, 6) += dz_dPFC * dPFC_dCalib;
    }

    ApplyLeftNullspace(H_f, H_x, res);

    /// @todo Chi2 distance check

    // Append our jacobian and residual
    Hx_big.block(ct_meas, cam_state_start, H_x.rows(), H_x.cols()) = H_x;
    res_big.block(ct_meas, 0, res.rows(), 1) = res;

    ct_meas += H_x.rows();
  }

  if (ct_meas == 0) {
    return;
  }

  CompressMeasurements(Hx_big, res_big);

  /// @todo get this value from config file
  unsigned int SIGMA_PIX {1U};
  // Our noise is isotropic, so make it here after our compression
  Eigen::MatrixXd R_big = SIGMA_PIX * SIGMA_PIX * Eigen::MatrixXd::Identity(
    res_big.rows(), res_big.rows());

  // 6. With all good features update the state
  Eigen::MatrixXd S = Hx_big * m_ekf->GetCov() * Hx_big.transpose() + R_big;
  Eigen::MatrixXd K = m_ekf->GetCov() * Hx_big.transpose() * S.inverse();

  unsigned int imu_states_size = m_ekf->GetImuCount() * g_imu_state_size;
  unsigned int cam_states_size = state_size - g_body_state_size - imu_states_size;

  Eigen::VectorXd update = K * res_big;
  Eigen::VectorXd body_update = update.segment<g_body_state_size>(0);
  Eigen::VectorXd imu_update = update.segment(g_body_state_size, imu_states_size);
  Eigen::VectorXd cam_update = update.segment(g_body_state_size + imu_states_size, cam_states_size);

  m_ekf->GetState().m_body_state += body_update;
  m_ekf->GetState().m_imu_states += imu_update;
  m_ekf->GetState().m_cam_states += cam_update;

  m_ekf->GetCov() =
    (Eigen::MatrixXd::Identity(state_size, state_size) - K * Hx_big) * m_ekf->GetCov();

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;
  Eigen::VectorXd body_state = m_ekf->GetState().m_body_state.ToVector();
  Eigen::VectorXd cam_state = m_ekf->GetState().m_cam_states[camera_id].ToVector();
  Eigen::VectorXd cam_sub_update = update.segment(cam_state_start, g_cam_state_size);

  msg << time;
  msg << VectorToCommaString(body_state);
  msg << VectorToCommaString(cam_state.segment(0, g_cam_state_size));
  msg << VectorToCommaString(body_update);
  msg << VectorToCommaString(cam_update.segment(0, g_cam_state_size));
  msg << "," << t_execution.count();
  msg << "\n";
  m_data_logger.Log(msg.str());
}

void MsckfUpdater::RefreshStates()
{
  BodyState body_state = m_ekf->GetBodyState();
  m_body_pos = body_state.m_position;
  m_body_vel = body_state.m_velocity;
  m_body_acc = body_state.m_acceleration;
  m_body_ang_pos = body_state.m_orientation;
  m_body_ang_vel = body_state.m_angular_velocity;
  m_body_ang_acc = body_state.m_angular_acceleration;

  CamState cam_state = m_ekf->GetCamState(m_id);
  m_pos_offset = cam_state.position;
  m_ang_offset = cam_state.orientation;
  m_aug_states = cam_state.augmented_states;
}
