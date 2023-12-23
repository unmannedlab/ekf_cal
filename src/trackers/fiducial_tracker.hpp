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
  enum class FiducialDetectorEnum
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
    std::string name {""};               ///< @brief Feature Tracker name
    int sensor_id{-1};                   ///< @brief Associated sensor ID
    std::string output_directory {""};   ///< @brief Feature Tracker data logging directory
    bool data_logging_on {false};        ///< @brief Feature Tracker data logging flag
    double pos_error{1e-9};              ///< @brief Position error standard deviation
    double ang_error{1e-9};              ///< @brief Angular error standard deviation
    FiducialDetectorEnum detector_type;  ///< @brief Detector type
    unsigned int initial_id{0};          ///< @brief Initial ID
    Intrinsics intrinsics;
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
  /// @param feature_tracks Output complete feature tracks
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
  unsigned int max_track_length{20};  ///< @brief Maximum track length before forced output
  unsigned int min_track_length{2};   ///< @brief Minimum track length to consider
  FiducialUpdater m_fiducial_updater;    ///< @brief MSCKF updater object
  int m_camera_id{-1};                ///< @brief Associated camera ID of tracker
  unsigned int m_id;                  ///< @brief Tracker ID
  double m_px_error;

private:
  void InitFiducialDetector(FiducialDetectorEnum detector);

  EKF * m_ekf = EKF::GetInstance();           ///< @brief EKF singleton

  static unsigned int m_tracker_count;
};

#endif  // TRACKERS__FIDUCIAL_TRACKER_HPP_
