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

  Eigen::Vector3d pos_f_in_l_true;
  Eigen::Quaterniond ang_f_to_l_true;
  if (m_no_errors) {
    pos_f_in_l_true = params.fiducial_params.pos_f_in_l;
    ang_f_to_l_true = params.fiducial_params.ang_f_to_l;
  } else {
    pos_f_in_l_true = m_rng.VecNormRand(params.fiducial_params.pos_f_in_l, params.pos_error);
    ang_f_to_l_true = m_rng.QuatNormRand(params.fiducial_params.ang_f_to_l, params.ang_error);
  }
  truth_engine->SetBoardPosition(m_id, pos_f_in_l_true);
  truth_engine->SetBoardOrientation(m_id, ang_f_to_l_true);
}

bool SimFiducialTracker::IsBoardVisible(double time)
{
  return true;
  Eigen::Vector3d pos_b_in_l = m_truth->GetBodyPosition(time);
  Eigen::Quaterniond ang_b_to_l = m_truth->GetBodyAngularPosition(time);
  Eigen::Vector3d pos_c_in_b = m_truth->GetCameraPosition(m_camera_id);
  Eigen::Quaterniond ang_c_to_b = m_truth->GetCameraAngularPosition(m_camera_id);
  Eigen::Matrix3d rot_l_to_c = (ang_b_to_l * ang_c_to_b).toRotationMatrix().transpose();
  cv::Mat ang_l_to_c_cv(3, 3, cv::DataType<double>::type);
  EigenMatrixToCv(rot_l_to_c, ang_l_to_c_cv);

  // Creating Rodrigues rotation vector
  cv::Mat r_vec(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(ang_l_to_c_cv, r_vec);

  Eigen::Vector3d pos_l_in_c = rot_l_to_c * (-(pos_b_in_l + ang_b_to_l * pos_c_in_b));

  cv::Mat t_vec(3, 1, cv::DataType<double>::type);
  t_vec.at<double>(0) = pos_l_in_c[0];
  t_vec.at<double>(1) = pos_l_in_c[1];
  t_vec.at<double>(2) = pos_l_in_c[2];

  // Create intrinsic matrices
  Intrinsics intrinsics = m_truth->GetCameraIntrinsics(m_camera_id);
  cv::Mat camera_matrix = intrinsics.ToCameraMatrix();
  cv::Mat distortion = intrinsics.ToDistortionVector();

  // Project points
  std::vector<cv::Point2d> projected_points;
  std::vector<cv::Point3d> board_position_vector;
  cv::Point3d board_position;
  Eigen::Vector3d pos_f_in_l_true = m_truth->GetBoardPosition(m_id);
  board_position.x = pos_f_in_l_true.x();
  board_position.y = pos_f_in_l_true.y();
  board_position.z = pos_f_in_l_true.z();
  board_position_vector.push_back(board_position);

  cv::projectPoints(
    board_position_vector, r_vec, t_vec, camera_matrix, distortion, projected_points);

  Eigen::Vector3d cam_plane_vec = rot_l_to_c.transpose() * Eigen::Vector3d(0, 0, 1);
  // Check that board is in front of camera plane

  if (cam_plane_vec.dot(pos_f_in_l_true) > 0 &&
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
  double message_time, unsigned int frame_id)
{
  std::vector<std::shared_ptr<SimFiducialTrackerMessage>> fiducial_tracker_messages;
  Eigen::Vector3d pos_f_in_l_true = m_truth->GetBoardPosition(m_id);
  Eigen::Quaterniond ang_f_to_l_true = m_truth->GetBoardOrientation(m_id);

  FeatureTracks feature_tracks;

  auto tracker_message = std::make_shared<SimFiducialTrackerMessage>();
  tracker_message->time = message_time;
  tracker_message->tracker_id = m_id;
  tracker_message->sensor_id = m_camera_id;
  tracker_message->sensor_type = SensorType::Tracker;
  tracker_message->is_board_visible = IsBoardVisible(message_time);

  if (tracker_message->is_board_visible) {
    Eigen::Vector3d pos_b_in_l = m_truth->GetBodyPosition(message_time);
    Eigen::Quaterniond ang_b_to_l = m_truth->GetBodyAngularPosition(message_time);
    Eigen::Vector3d pos_c_in_b_true = m_truth->GetCameraPosition(m_camera_id);
    Eigen::Quaterniond ang_c_to_b_true = m_truth->GetCameraAngularPosition(m_camera_id);

    Eigen::Matrix3d rot_l_to_b = ang_b_to_l.toRotationMatrix().transpose();
    Eigen::Matrix3d rot_b_to_c = ang_c_to_b_true.toRotationMatrix().transpose();

    Eigen::Vector3d pos_f_in_c_true =
      rot_b_to_c * (rot_l_to_b * (pos_f_in_l_true - pos_b_in_l) - pos_c_in_b_true);

    Eigen::Quaterniond ang_f_to_c_true =
      ang_c_to_b_true.inverse() * ang_b_to_l.inverse() * ang_f_to_l_true;

    BoardDetection board_detection;
    board_detection.frame_id = frame_id;
    board_detection.frame_time = message_time;
    board_detection.pos_error = m_t_vec_error;
    board_detection.ang_error = m_r_vec_error;
    if (!m_no_errors) {
      board_detection.pos_f_in_c = m_rng.VecNormRand(pos_f_in_c_true, m_t_vec_error);
      board_detection.ang_f_to_c = m_rng.QuatNormRand(ang_f_to_c_true, m_r_vec_error);
    } else {
      board_detection.pos_f_in_c = pos_f_in_c_true;
      board_detection.ang_f_to_c = ang_f_to_c_true;
    }
    tracker_message->board_detection = board_detection;
  }


  return tracker_message;
}

void SimFiducialTracker::Callback(double time, SimFiducialTrackerMessage & msg)
{
  if (msg.is_board_visible) {
    m_fiducial_updater.UpdateEKF(*m_ekf, time, msg.board_detection);
  }
}
