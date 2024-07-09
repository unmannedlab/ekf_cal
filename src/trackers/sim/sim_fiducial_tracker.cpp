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

  Eigen::Vector3d pos_f_in_g_true;
  Eigen::Quaterniond ang_f_to_g_true;
  if (m_no_errors) {
    pos_f_in_g_true = params.fiducial_params.pos_f_in_l;
    ang_f_to_g_true = params.fiducial_params.ang_f_to_l;
  } else {
    pos_f_in_g_true = m_rng.VecNormRand(params.fiducial_params.pos_f_in_l, params.pos_error);
    ang_f_to_g_true = m_rng.QuatNormRand(params.fiducial_params.ang_f_to_l, params.ang_error);
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
  Intrinsics intrinsics = m_truth->GetCameraIntrinsics(sensor_id);
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

  // Create intrinsic matrices
  cv::Mat camera_matrix = intrinsics.ToCameraMatrix();
  cv::Mat distortion = intrinsics.ToDistortionVector();

  // Project points
  std::vector<cv::Point2d> projected_points;
  std::vector<cv::Point3d> board_position_vector;
  cv::Point3d board_position;
  Eigen::Vector3d pos_f_in_g_true = m_truth->GetBoardPosition(m_id);
  board_position.x = pos_f_in_g_true.x();
  board_position.y = pos_f_in_g_true.y();
  board_position.z = pos_f_in_g_true.z();
  board_position_vector.push_back(board_position);

  cv::projectPoints(
    board_position_vector, r_vec, t_vec, camera_matrix, distortion, projected_points);

  Eigen::Vector3d cam_plane_vec = ang_g_to_c.transpose() * Eigen::Vector3d(0, 0, 1);
  // Check that board is in front of camera plane

  if (cam_plane_vec.dot(pos_f_in_g_true) > 0 &&
    projected_points[0].x >= 0 &&
    projected_points[0].y >= 0 &&
    projected_points[0].x <= intrinsics.width &&
    projected_points[0].y <= intrinsics.height)
  {
    return true;
  } else {
    return false;
  }
}

std::shared_ptr<SimFiducialTrackerMessage> SimFiducialTracker::GenerateMessage(
  double message_time, int frame_id, int sensor_id)
{
  std::vector<std::shared_ptr<SimFiducialTrackerMessage>> fiducial_tracker_messages;
  Eigen::Vector3d pos_f_in_g_true = m_truth->GetBoardPosition(m_id);
  Eigen::Quaterniond ang_f_to_g_true = m_truth->GetBoardOrientation(m_id);

  std::vector<std::vector<FeaturePoint>> feature_tracks;

  bool is_board_visible = IsBoardVisible(message_time, sensor_id);
  if (is_board_visible) {
    Eigen::Vector3d pos_b_in_g = m_truth->GetBodyPosition(message_time);
    Eigen::Quaterniond ang_b_to_g = m_truth->GetBodyAngularPosition(message_time);
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

    m_board_track.push_back(board_detection);
  } else if (m_board_track.size() < m_min_track_length) {
    m_board_track.clear();
  }

  auto tracker_message = std::make_shared<SimFiducialTrackerMessage>();
  tracker_message->time = message_time;
  tracker_message->tracker_id = m_id;
  tracker_message->sensor_id = sensor_id;
  tracker_message->sensor_type = SensorType::Tracker;
  tracker_message->pos_error = m_t_vec_error;
  tracker_message->ang_error = m_r_vec_error;

  if (!is_board_visible || m_board_track.size() >= m_max_track_length) {
    tracker_message->board_track = m_board_track;
    m_board_track.clear();
  }

  return tracker_message;
}

void SimFiducialTracker::Callback(double time, std::shared_ptr<SimFiducialTrackerMessage> msg)
{
  m_fiducial_updater.UpdateEKF(
    m_ekf,
    time,
    msg->board_track,
    msg->pos_error.norm(),
    msg->ang_error.norm());
}
