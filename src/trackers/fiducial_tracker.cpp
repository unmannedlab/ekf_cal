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

#include "trackers/tracker.hpp"
#include "utility/type_helper.hpp"

FiducialTracker::FiducialTracker(FiducialTracker::Parameters params)
: Tracker(params),
  m_fiducial_updater(
    params.camera_id,
    params.pos_f_in_g,
    params.ang_f_to_g,
    params.output_directory,
    params.data_logging_on,
    params.data_log_rate,
    params.logger
  ),
  m_detector_type(params.detector_type)
{
  m_pos_error = params.variance.segment<3>(0);
  m_ang_error = params.variance.segment<3>(3);
}

void FiducialTracker::Track(
  double time,
  int frame_id,
  cv::Mat & img_in,
  cv::Mat & img_out)
{
  /// @todo(jhartzer): Make this part of constructor / input params
  const cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::Ptr<cv::aruco::CharucoBoard> board =
    cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
  cv::Ptr<cv::aruco::DetectorParameters> params = cv::makePtr<cv::aruco::DetectorParameters>();
  std::vector<int> marker_ids;
  cv::Mat camera_matrix = GenerateCameraMatrix(m_intrinsics);
  cv::Mat dist_coeff = GenerateDistortionVector(m_intrinsics);

  std::vector<std::vector<cv::Point2f>> marker_corners;
  cv::aruco::detectMarkers(img_in, dictionary, marker_corners, marker_ids, params);
  bool detection_made {false};

  // if at least one marker detected
  if (marker_ids.size() > 0) {
    img_out = img_in.clone();
    cv::aruco::drawDetectedMarkers(img_out, marker_corners, marker_ids);
    std::vector<cv::Point2f> charuco_corners;
    std::vector<int> charuco_ids;
    cv::aruco::interpolateCornersCharuco(
      marker_corners, marker_ids, img_in, board, charuco_corners,
      charuco_ids, camera_matrix, dist_coeff);

    // if at least one charuco corner detected
    if (charuco_ids.size() > 0) {
      cv::Scalar color = cv::Scalar(255, 0, 0);
      cv::aruco::drawDetectedCornersCharuco(img_out, charuco_corners, charuco_ids, color);
      cv::Vec3d r_vec, t_vec;
      bool valid = cv::aruco::estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, board, camera_matrix, dist_coeff, r_vec, t_vec);

      // if charuco pose is valid
      if (valid) {
        cv::drawFrameAxes(img_out, camera_matrix, dist_coeff, r_vec, t_vec, 0.1f);
      }
      BoardDetection board_detection;
      board_detection.frame_id = frame_id;
      board_detection.t_vec_f_in_c = t_vec;
      board_detection.r_vec_f_to_c = r_vec;
      m_board_track.push_back(board_detection);
      detection_made = true;
    }
  }

  bool update_ekf {false};
  if (detection_made) {
    if (m_board_track.size() >= m_max_track_length) {
      update_ekf = true;
    }
  } else {
    if (m_board_track.size() < m_min_track_length) {
      m_board_track.clear();
    } else {
      update_ekf = true;
    }
  }

  if (update_ekf) {
    m_fiducial_updater.UpdateEKF(
      m_ekf,
      time,
      m_board_track,
      m_pos_error.norm(),
      m_ang_error.norm());
    m_board_track.clear();
  }
}
