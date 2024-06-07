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

#include "trackers/sim/sim_fiducial_tracker.hpp"

#include <opencv2/opencv.hpp>

#include "utility/string_helper.hpp"
#include "utility/type_helper.hpp"

SimFiducialTracker::SimFiducialTracker(
  SimFiducialTracker::Parameters params,
  std::shared_ptr<TruthEngine> truth_engine)
: FiducialTracker(params.fiducial_params),
  m_rng(params.rng)
{
  m_no_errors = params.no_errors;
  m_truth = truth_engine;

  m_pos_error = params.pos_error;
  m_ang_error = params.ang_error;
  m_t_vec_error = params.t_vec_error;
  m_r_vec_error = params.r_vec_error;
  m_min_track_length = params.fiducial_params.min_track_length;
  m_max_track_length = params.fiducial_params.max_track_length;

  m_intrinsics = params.fiducial_params.intrinsics;
  m_proj_matrix = cv::Mat(3, 3, cv::DataType<double>::type, 0.0);
  m_proj_matrix.at<double>(0, 0) = m_intrinsics.f_x / m_intrinsics.pixel_size;
  m_proj_matrix.at<double>(1, 1) = m_intrinsics.f_y / m_intrinsics.pixel_size;
  m_proj_matrix.at<double>(0, 2) = static_cast<double>(m_intrinsics.width) / 2.0;
  m_proj_matrix.at<double>(1, 2) = static_cast<double>(m_intrinsics.height) / 2.0;
  m_proj_matrix.at<double>(2, 2) = 1;

  Eigen::Vector3d pos_f_in_g_true;
  Eigen::Quaterniond ang_f_to_g_true;
  if (m_no_errors) {
    pos_f_in_g_true = params.fiducial_params.pos_f_in_g;
    ang_f_to_g_true = params.fiducial_params.ang_f_to_g;
  } else {
    pos_f_in_g_true = m_rng.VecNormRand(params.fiducial_params.pos_f_in_g, params.pos_error);
    ang_f_to_g_true = m_rng.QuatNormRand(params.fiducial_params.ang_f_to_g, params.ang_error);
  }
  truth_engine->SetBoardPosition(m_id, pos_f_in_g_true);
  truth_engine->SetBoardOrientation(m_id, ang_f_to_g_true);
}

bool SimFiducialTracker::IsBoardVisible(double time, int sensor_id)
{
  Eigen::Vector3d pos_b_in_g = m_truth->GetBodyPosition(time);
  Eigen::Quaterniond ang_b_to_g = m_truth->GetBodyAngularPosition(time);
  Eigen::Vector3d pos_c_in_b_true = m_truth->GetCameraPosition(sensor_id);
  Eigen::Quaterniond ang_c_to_b_true = m_truth->GetCameraAngularPosition(sensor_id);
  Eigen::Quaterniond ang_c_to_b = ang_c_to_b_true;
  Eigen::Matrix3d ang_g_to_c = (ang_b_to_g * ang_c_to_b).toRotationMatrix().transpose();
  cv::Mat ang_g_to_c_cv(3, 3, cv::DataType<double>::type);
  EigenMatrixToCv(ang_g_to_c, ang_g_to_c_cv);

  // Creating Rodrigues rotation vector
  cv::Mat r_vec(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(ang_g_to_c_cv, r_vec);

  Eigen::Vector3d pos_g_in_c = ang_g_to_c * (-(pos_b_in_g + ang_b_to_g * pos_c_in_b_true));

  cv::Mat t_vec(3, 1, cv::DataType<double>::type);
  t_vec.at<double>(0) = pos_g_in_c[0];
  t_vec.at<double>(1) = pos_g_in_c[1];
  t_vec.at<double>(2) = pos_g_in_c[2];

  // Create distortion matrix. Defined in CV2 as (k1,k2,p1,p2)
  cv::Mat distortion(4, 1, cv::DataType<double>::type);
  distortion.at<double>(0) = m_intrinsics.k_1;
  distortion.at<double>(1) = m_intrinsics.k_2;
  distortion.at<double>(2) = m_intrinsics.p_1;
  distortion.at<double>(3) = m_intrinsics.p_2;

  // Project points
  std::vector<cv::Point2d> projected_points;
  std::vector<cv::Point3d> board_position_vector;
  cv::Point3d board_position;
  Eigen::Vector3d pos_f_in_g_true = m_truth->GetBoardPosition(0);
  board_position.x = pos_f_in_g_true[0];
  board_position.y = pos_f_in_g_true[1];
  board_position.z = pos_f_in_g_true[2];
  board_position_vector.push_back(board_position);

  cv::projectPoints(
    board_position_vector, r_vec, t_vec, m_proj_matrix, distortion, projected_points);

  Eigen::Vector3d cam_plane_vec = ang_g_to_c.transpose() * Eigen::Vector3d(0, 0, 1);
  // Check that board is in front of camera plane
  if (cam_plane_vec.dot(pos_f_in_g_true) > 0 &&
    projected_points[0].x >= 0 &&
    projected_points[0].y >= 0 &&
    projected_points[0].x <= m_intrinsics.width &&
    projected_points[0].y <= m_intrinsics.height)
  {
    return true;
  } else {
    return false;
  }
}

std::vector<std::shared_ptr<SimFiducialTrackerMessage>> SimFiducialTracker::GenerateMessages(
  std::vector<double> message_times, int sensor_id)
{
  m_logger->Log(
    LogLevel::INFO,
    "Generating " + std::to_string(message_times.size()) + " Fiducial measurements");

  std::vector<std::shared_ptr<SimFiducialTrackerMessage>> fiducial_tracker_messages;
  Eigen::Vector3d pos_f_in_g_true = m_truth->GetBoardPosition(0);
  Eigen::Quaterniond ang_f_to_g_true = m_truth->GetBoardOrientation(0);

  BoardTrack board_track;
  for (int frame_id = 0; static_cast<unsigned int>(frame_id) < message_times.size(); ++frame_id) {
    std::vector<std::vector<FeaturePoint>> feature_tracks;
    auto tracker_message = std::make_shared<SimFiducialTrackerMessage>();
    tracker_message->m_time = message_times[frame_id];
    tracker_message->m_tracker_id = m_id;
    tracker_message->m_sensor_id = sensor_id;
    tracker_message->m_sensor_type = SensorType::Tracker;
    tracker_message->m_pos_error = m_t_vec_error;
    tracker_message->m_ang_error = m_r_vec_error;

    bool is_board_visible = IsBoardVisible(message_times[frame_id], sensor_id);
    if (is_board_visible) {
      Eigen::Vector3d pos_b_in_g = m_truth->GetBodyPosition(message_times[frame_id]);
      Eigen::Quaterniond ang_b_to_g = m_truth->GetBodyAngularPosition(message_times[frame_id]);
      Eigen::Vector3d pos_c_in_b_true = m_truth->GetCameraPosition(sensor_id);
      Eigen::Quaterniond ang_c_to_b_true = m_truth->GetCameraAngularPosition(sensor_id);

      Eigen::Matrix3d rot_g_to_b = ang_b_to_g.toRotationMatrix().transpose();
      Eigen::Matrix3d rot_b_to_c = ang_c_to_b_true.toRotationMatrix().transpose();

      Eigen::Vector3d pos_f_in_c_true =
        rot_b_to_c * (rot_g_to_b * (pos_f_in_g_true - pos_b_in_g) - pos_c_in_b_true);

      BoardDetection board_detection;
      board_detection.frame_id = frame_id;
      board_detection.t_vec_f_in_c[0] = m_rng.NormRand(pos_f_in_c_true[0], m_t_vec_error[0]);
      board_detection.t_vec_f_in_c[1] = m_rng.NormRand(pos_f_in_c_true[1], m_t_vec_error[1]);
      board_detection.t_vec_f_in_c[2] = m_rng.NormRand(pos_f_in_c_true[2], m_t_vec_error[2]);

      Eigen::Vector3d ang_f_to_c_error_rpy;
      ang_f_to_c_error_rpy(0) = m_rng.NormRand(0.0, m_r_vec_error[0]);
      ang_f_to_c_error_rpy(1) = m_rng.NormRand(0.0, m_r_vec_error[1]);
      ang_f_to_c_error_rpy(2) = m_rng.NormRand(0.0, m_r_vec_error[2]);
      Eigen::Quaterniond ang_f_to_c = EigVecToQuat(ang_f_to_c_error_rpy) *
        ang_c_to_b_true.inverse() * ang_b_to_g.inverse() * ang_f_to_g_true;
      board_detection.r_vec_f_to_c = QuatToRodrigues(ang_f_to_c);

      board_track.push_back(board_detection);
    } else if (board_track.size() < m_min_track_length) {
      board_track.clear();
    }

    if (!is_board_visible || board_track.size() >= m_max_track_length) {
      tracker_message->m_board_track = board_track;
      board_track.clear();
    }

    fiducial_tracker_messages.push_back(tracker_message);
  }
  return fiducial_tracker_messages;
}

void SimFiducialTracker::Callback(double time, std::shared_ptr<SimFiducialTrackerMessage> msg)
{
  m_fiducial_updater.UpdateEKF(
    m_ekf,
    time,
    msg->m_board_track,
    msg->m_pos_error.norm(),
    msg->m_ang_error.norm());
}
