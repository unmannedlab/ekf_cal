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

#ifndef TRACKERS__FIDUCIAL_TRACKER_HPP_
#define TRACKERS__FIDUCIAL_TRACKER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "ekf/update/fiducial_updater.hpp"
#include "infrastructure/debug_logger.hpp"
#include "trackers/tracker.hpp"

///
/// @brief Detector Enumerations
///
enum class FiducialType
{
  ARUCO_BOARD,
  CHARUCO_BOARD
};

///
/// @class FiducialTracker
/// @brief FiducialTracker Class
///
class FiducialTracker : public Tracker
{
public:
  ///
  /// @brief Feature Tracker Initialization parameters structure
  ///
  typedef struct Parameters : public Tracker::Parameters
  {
    FiducialType detector_type;                     ///< @brief Detector type
    unsigned int predefined_dict{0};                ///< @brief Predefined dictionary
    unsigned int squares_x {1};                     ///< @brief Number of squares in the x direction
    unsigned int squares_y {1};                     ///< @brief Number of squares in the y direction
    double square_length {1.0};                     ///< @brief Checkerboard square length
    double marker_length {1.0};                     ///< @brief Marker length
    unsigned int id{0};                             ///< @brief Initial ID
    Eigen::Vector3d pos_f_in_l;                     ///< @brief Fiducial position
    Eigen::Quaterniond ang_f_to_l;                  ///< @brief Fiducial orientation
    bool is_extrinsic{false};                       ///< @brief Perform extrinsic calibration
    Eigen::VectorXd variance {{1, 1, 1, 1, 1, 1}};  ///< @brief Fiducial marker variance
    bool is_cam_extrinsic{false};  ///< @brief Flag for extrinsic camera calibration
  } Parameters;

  ///
  /// @brief FeatureTracker sensor constructor
  /// @param params Parameter struct for feature tracker
  ///
  explicit FiducialTracker(FiducialTracker::Parameters params);

  ///
  /// @brief Perform track on new image frame
  /// @param time Frame time
  /// @param frame_id Frame ID
  /// @param img_in Input frame
  /// @param img_out Output frame with drawn track lines
  ///
  void Track(
    double time,
    unsigned int frame_id,
    const cv::Mat & img_in,
    cv::Mat & img_out
  );

  ///
  /// @brief Function to interpolate corners of charuco boards
  /// @param marker_corners Input marker corners
  /// @param marker_ids Input marker IDs
  /// @param image Image on which to perform interpolation
  /// @param board Fiducial board
  /// @param corners Output corners
  /// @param ids Output ids
  /// @param camera_matrix Input camera matrix
  /// @param dist_coefficients Input camera distortion coefficients
  ///
  int InterpolateCorners(
    std::vector<std::vector<cv::Point2f>> & marker_corners,
    std::vector<int> & marker_ids,
    cv::Mat image,
    cv::Ptr<cv::aruco::Board> & board,
    std::vector<cv::Point2f> & corners,
    std::vector<int> & ids,
    cv::Mat camera_matrix,
    cv::Mat dist_coefficients
  );

  ///
  /// @brief DrawDetectedCorners
  /// @param image Image on which to draw corners
  /// @param marker_corners Input aruco corners
  /// @param corners Input charuco corners
  /// @param ids Detected IDs
  /// @param corner_color Colors for drawing
  ///
  void DrawDetectedCorners(
    cv::Mat image,
    std::vector<std::vector<cv::Point2f>> & marker_corners,
    std::vector<cv::Point2f> & corners,
    std::vector<int> & ids,
    cv::Scalar corner_color
  );

  ///
  /// @brief Estimate pose of board
  /// @param marker_corners Input aruco corners
  /// @param corners Input charuco corners
  /// @param ids Fiducial IDs
  /// @param board Fiducial board
  /// @param camera_matrix Input camera matrix
  /// @param dist_coefficients Input camera distortion coefficients
  /// @param r_vec Output rotation vector of board
  /// @param t_vec Output translation vector of board
  ///
  bool EstimatePoseBoard(
    std::vector<std::vector<cv::Point2f>> & marker_corners,
    cv::InputArray & corners,
    cv::InputArray & ids,
    cv::Ptr<cv::aruco::Board> board,
    cv::InputArray & camera_matrix,
    cv::InputArray & dist_coefficients,
    cv::Vec3d & r_vec,
    cv::Vec3d & t_vec
  );

  cv::Ptr<cv::aruco::Dictionary> m_dict;  ///< @brief Fiducial board dictionary
  cv::Ptr<cv::aruco::Board> m_board;      ///< @brief Fiducial board

protected:
  FiducialUpdater m_fiducial_updater;  ///< @brief MSCKF updater object
  FiducialType m_detector_type;        ///< @brief Detector type

private:
  Eigen::Vector3d m_pos_error;
  Eigen::Vector3d m_ang_error;
};

#endif  // TRACKERS__FIDUCIAL_TRACKER_HPP_
