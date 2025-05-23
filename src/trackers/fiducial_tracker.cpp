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

#include "trackers/fiducial_tracker.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/opencv.hpp>

#include "trackers/tracker.hpp"
#include "utility/type_helper.hpp"

FiducialTracker::FiducialTracker(FiducialTracker::Parameters params)
: Tracker(params),
  m_fiducial_updater(
    params.id,
    params.camera_id,
    params.is_extrinsic,
    params.is_cam_extrinsic,
    params.log_directory,
    params.data_log_rate,
    params.logger
  ),
  m_detector_type(params.detector_type)
{
  m_pos_error = params.variance.segment<3>(0);
  m_ang_error = params.variance.segment<3>(3);

  auto dict_name = static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(params.predefined_dict);
  m_dict = cv::aruco::getPredefinedDictionary(dict_name);

  if (params.detector_type == FiducialType::ARUCO_BOARD) {
    m_board = cv::aruco::GridBoard::create(
      static_cast<int>(params.squares_x),
      static_cast<int>(params.squares_y),
      static_cast<float>(params.square_length),
      static_cast<float>(params.marker_length),
      m_dict);
  } else if (params.detector_type == FiducialType::CHARUCO_BOARD) {
    m_board = cv::aruco::CharucoBoard::create(
      static_cast<int>(params.squares_x),
      static_cast<int>(params.squares_y),
      static_cast<float>(params.square_length),
      static_cast<float>(params.marker_length),
      m_dict);
  }

  FidState fid_state;
  fid_state.SetIsExtrinsic(params.is_extrinsic);
  fid_state.pos_f_in_l = params.pos_f_in_l;
  fid_state.ang_f_to_l = params.ang_f_to_l;
  fid_state.id = params.id;
  Eigen::MatrixXd covariance(g_fid_extrinsic_state_size, g_fid_extrinsic_state_size);
  covariance = params.variance.asDiagonal();

  m_ekf->RegisterFiducial(fid_state, covariance);
}

int FiducialTracker::InterpolateCorners(
  std::vector<std::vector<cv::Point2f>> & marker_corners,
  std::vector<int> & marker_ids,
  cv::Mat image,
  cv::Ptr<cv::aruco::Board> & board,
  std::vector<cv::Point2f> & corners,
  std::vector<int> & ids,
  cv::Mat camera_matrix,
  cv::Mat dist_coefficients
) const
{
  int corners_found{0};
  if (m_detector_type == FiducialType::CHARUCO_BOARD) {
    auto temp_board = board.staticCast<cv::aruco::CharucoBoard>();
    corners_found = cv::aruco::interpolateCornersCharuco(
      marker_corners, marker_ids, image, temp_board,
      corners, ids, camera_matrix, dist_coefficients);
  } else if (m_detector_type == FiducialType::ARUCO_BOARD) {
    corners_found = static_cast<int>(marker_ids.size());
  }
  return corners_found;
}

void FiducialTracker::DrawDetectedCorners(
  cv::Mat image,
  std::vector<std::vector<cv::Point2f>> & marker_corners,
  std::vector<cv::Point2f> & corners,
  std::vector<int> & ids,
  cv::Scalar corner_color
) const
{
  if (m_detector_type == FiducialType::CHARUCO_BOARD) {
    cv::aruco::drawDetectedCornersCharuco(image, corners, ids, corner_color);
  } else if (m_detector_type == FiducialType::ARUCO_BOARD) {
    cv::aruco::drawDetectedMarkers(image, marker_corners, ids, corner_color);
  }
}

bool FiducialTracker::EstimatePoseBoard(
  std::vector<std::vector<cv::Point2f>> & marker_corners,
  cv::InputArray & corners,
  cv::InputArray & ids,
  cv::Ptr<cv::aruco::Board> board,
  cv::InputArray & camera_matrix,
  cv::InputArray & dist_coefficients,
  cv::Vec3d & r_vec,
  cv::Vec3d & t_vec
) const
{
  bool is_valid{false};
  if (m_detector_type == FiducialType::CHARUCO_BOARD) {
    auto temp_board = board.staticCast<cv::aruco::CharucoBoard>();
    is_valid = cv::aruco::estimatePoseCharucoBoard(
      corners, ids, temp_board, camera_matrix, dist_coefficients, r_vec, t_vec);
  } else if (m_detector_type == FiducialType::ARUCO_BOARD) {
    is_valid = cv::aruco::estimatePoseBoard(
      marker_corners, ids, board, camera_matrix, dist_coefficients, r_vec, t_vec) != 0;
  }
  return is_valid;
}

void FiducialTracker::Track(
  double time,
  unsigned int frame_id,
  const cv::Mat & img_in,
  cv::Mat & img_out)
{
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::makePtr<cv::aruco::DetectorParameters>();
  std::vector<int> marker_ids;
  cv::Mat camera_matrix = m_ekf->m_state.cam_states[m_camera_id].intrinsics.ToCameraMatrix();
  cv::Mat distortion = m_ekf->m_state.cam_states[m_camera_id].intrinsics.ToDistortionVector();

  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(img_in, m_dict, marker_corners, marker_ids, params);

  // if at least one marker detected
  if (!marker_ids.empty()) {
    cv::aruco::drawDetectedMarkers(img_out, marker_corners, marker_ids);
    std::vector<cv::Point2f> corners;
    std::vector<int> ids;

    FiducialTracker::InterpolateCorners(
      marker_corners, marker_ids, img_in, m_board, corners, ids, camera_matrix, distortion);

    // If at least one corner detected
    if (!ids.empty()) {
      cv::Scalar color = cv::Scalar(255, 0, 0);
      FiducialTracker::DrawDetectedCorners(img_out, marker_corners, corners, ids, color);
      cv::Vec3d r_vec;
      cv::Vec3d t_vec;
      bool valid = FiducialTracker::EstimatePoseBoard(
        marker_corners, corners, ids, m_board, camera_matrix, distortion, r_vec, t_vec);

      // if marker pose is valid
      if (valid) {
        cv::drawFrameAxes(img_out, camera_matrix, distortion, r_vec, t_vec, 0.5);
      }

      Eigen::Vector3d pos_f_in_c = CvVectorToEigen(t_vec);
      Eigen::Quaterniond ang_f_to_c = RodriguesToQuat(r_vec);

      BoardDetection board_detection;
      board_detection.frame_id = frame_id;
      board_detection.pos_f_in_c = pos_f_in_c;
      board_detection.ang_f_to_c = ang_f_to_c;
      board_detection.pos_error = m_pos_error;
      board_detection.ang_error = m_ang_error;

      m_fiducial_updater.UpdateEKF(*m_ekf, time, board_detection);
    }
  }
}
