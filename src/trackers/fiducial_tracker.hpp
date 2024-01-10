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
#include <string>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "ekf/update/fiducial_updater.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/types.hpp"

///
/// @class FiducialTracker
/// @brief FiducialTracker Class
///
class FiducialTracker
{
public:
  ///
  /// @brief Detector Enumerations
  ///
  enum class FiducialTypeEnum
  {
    ARUCO_BOARD,
    CHARUCO_BOARD,
    APRIL_GRID,
  };

  ///
  /// @brief Feature Tracker Initialization parameters structure
  ///
  typedef struct Parameters
  {
    std::string name {""};                          ///< @brief Feature Tracker name
    int sensor_id{-1};                              ///< @brief Associated sensor ID
    std::string output_directory {""};              ///< @brief Feature Tracker data logging directory
    bool data_logging_on {false};                   ///< @brief Feature Tracker data logging flag
    FiducialTypeEnum detector_type;                 ///< @brief Detector type
    unsigned int squares_x {1U};                    ///< @brief Number of squares in the x direction
    unsigned int squares_y {1U};                    ///< @brief Number of squares in the y direction
    double square_length {1.0};                     ///< @brief Checkerboard square length
    double marker_length {1.0};                     ///< @brief Marker length
    unsigned int initial_id{0};                     ///< @brief Initial ID
    Intrinsics intrinsics;                          ///< @brief Camera intrinsic parameters
    Eigen::Vector3d pos_b_in_g;                     ///< @brief Board position
    Eigen::Quaterniond ang_b_to_g;                  ///< @brief Board orientation
    Eigen::VectorXd variance {{1, 1, 1, 1, 1, 1}};  ///< @brief Fiducial marker variance
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
  /// @param board_track Output complete board track
  ///
  void Track(
    double time,
    int frame_id,
    cv::Mat & img_in,
    cv::Mat & img_out,
    BoardTrack board_track);

  ///
  /// @brief Tracker ID getter method
  /// @return Tracker ID
  ///
  unsigned int GetID();

protected:
  DebugLogger * m_logger = DebugLogger::GetInstance();  ///< @brief Logger singleton
  unsigned int max_track_length{20};     ///< @brief Maximum track length before forced output
  unsigned int min_track_length{2};      ///< @brief Minimum track length to consider
  FiducialUpdater m_fiducial_updater;    ///< @brief MSCKF updater object
  int m_camera_id{-1};                   ///< @brief Associated camera ID of tracker
  unsigned int m_id;                     ///< @brief Tracker ID
  Intrinsics m_intrinsics;               ///< @brief Camera intrinsics
  FiducialTypeEnum m_detector_type;  ///< @brief Detector type

private:
  void InitFiducialDetector(FiducialTypeEnum detector);

  EKF * m_ekf = EKF::GetInstance();           ///< @brief EKF singleton

  static unsigned int m_tracker_count;
};

#endif  // TRACKERS__FIDUCIAL_TRACKER_HPP_
