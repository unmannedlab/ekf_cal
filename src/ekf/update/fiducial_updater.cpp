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

#include "ekf/update/fiducial_updater.hpp"

#include <eigen3/Eigen/Eigen>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <ostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "ekf/constants.hpp"
#include "ekf/ekf.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/types.hpp"
#include "utility/math_helper.hpp"
#include "utility/string_helper.hpp"

FiducialUpdater::FiducialUpdater(
  int cam_id, std::string log_file_directory, bool data_logging_on)
: Updater(cam_id), m_data_logger(log_file_directory, "camera_" + std::to_string(cam_id) + ".csv")
{
  std::stringstream msg;
  msg << "time";
  msg << EnumerateHeader("cam_state", g_cam_state_size);
  msg << EnumerateHeader("body_update", g_body_state_size);
  msg << EnumerateHeader("cam_update", g_cam_state_size);
  msg << ",FeatureTracks";
  msg << EnumerateHeader("time", 1);
  msg << std::endl;

  m_data_logger.DefineHeader(msg.str());
  m_data_logger.SetLogging(data_logging_on);
}

void FiducialUpdater::UpdateEKF(
  double time, int camera_id, BoardTrack board_track,
  Intrinsics intrinsics)
{
  m_ekf->ProcessModel(time);
  RefreshStates();
  auto t_start = std::chrono::high_resolution_clock::now();

  // m_logger->Log(LogLevel::DEBUG, "Called update_msckf for camera ID: "
  // + std::to_string(camera_id));

  // if (feature_tracks.size() == 0) {
  //   return;
  // }

  // Eigen::VectorXd res_x = Eigen::VectorXd::Zero(max_meas_size);
  // Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(max_meas_size, state_size);

  // m_logger->Log(LogLevel::DEBUG, "Update track count: " + std::to_string(feature_tracks.size()));

  // ApplyLeftNullspace(H_f, H_c, res_f);

  // /// @todo Chi^2 distance check

  // // Append our Jacobian and residual
  // H_x.block(ct_meas, cam_state_start, H_c.rows(), H_c.cols()) = H_c;
  // res_x.block(ct_meas, 0, res_f.rows(), 1) = res_f;

  // ct_meas += H_c.rows();


  // if (ct_meas == 0) {
  //   return;
  // }

  // CompressMeasurements(H_x, res_x);

  // // Jacobian is ill-formed if either rows or columns post-compression are size 1
  // if (res_x.size() == 1) {
  //   m_logger->Log(LogLevel::INFO, "Compressed MSCKF Jacobian is ill-formed");
  //   return;
  // }

  // Eigen::MatrixXd R = px_error * px_error *
  // Eigen::MatrixXd::Identity(res_x.rows(), res_x.rows());

  // // Apply Kalman update
  // Eigen::MatrixXd S = H_x * m_ekf->GetCov() * H_x.transpose() + R;
  // Eigen::MatrixXd K = m_ekf->GetCov() * H_x.transpose() * S.inverse();

  // unsigned int imu_states_size = m_ekf->GetImuStateSize();
  // unsigned int cam_states_size = state_size - g_body_state_size - imu_states_size;

  // Eigen::VectorXd update = K * res_x;
  // Eigen::VectorXd body_update = update.segment<g_body_state_size>(0);
  // Eigen::VectorXd imu_update = update.segment(g_body_state_size, imu_states_size);
  // Eigen::VectorXd cam_update = update.segment(g_body_state_size +
  // imu_states_size, cam_states_size);

  // m_ekf->GetState().m_body_state += body_update;
  // m_ekf->GetState().m_imu_states += imu_update;
  // m_ekf->GetState().m_cam_states += cam_update;

  // m_ekf->GetCov() =
  //   (Eigen::MatrixXd::Identity(state_size, state_size) - K * H_x) * m_ekf->GetCov();

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // // Write outputs
  // std::stringstream msg;
  // Eigen::VectorXd cam_state = m_ekf->GetState().m_cam_states[camera_id].ToVector();
  // Eigen::VectorXd cam_sub_update = update.segment(cam_state_start, g_cam_state_size);

  // msg << time;
  // msg << VectorToCommaString(cam_state.segment(0, g_cam_state_size));
  // msg << VectorToCommaString(body_update);
  // msg << VectorToCommaString(cam_update.segment(0, g_cam_state_size));
  // msg << "," << std::to_string(feature_tracks.size());
  // msg << "," << t_execution.count();
  // msg << std::endl;
  // m_data_logger.Log(msg.str());
}

void FiducialUpdater::RefreshStates()
{
  BodyState body_state = m_ekf->GetBodyState();
  m_body_pos = body_state.m_position;
  m_body_vel = body_state.m_velocity;
  m_body_acc = body_state.m_acceleration;
  m_ang_b_to_g = body_state.m_ang_b_to_g;
  m_body_ang_vel = body_state.m_angular_velocity;
  m_body_ang_acc = body_state.m_angular_acceleration;

  CamState cam_state = m_ekf->GetCamState(m_id);
  m_pos_c_in_b = cam_state.pos_c_in_b;
  m_ang_c_to_b = cam_state.ang_c_to_b;
}
