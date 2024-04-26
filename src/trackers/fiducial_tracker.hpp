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

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "ekf/update/fiducial_updater.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/types.hpp"
#include "trackers/tracker.hpp"

///
/// @class FiducialTracker
/// @brief FiducialTracker Class
///
class FiducialTracker : public Tracker
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
  typedef struct Parameters : public Tracker::Parameters
  {
    FiducialTypeEnum detector_type;                 ///< @brief Detector type
    unsigned int squares_x {1U};                    ///< @brief Number of squares in the x direction
    unsigned int squares_y {1U};                    ///< @brief Number of squares in the y direction
    double square_length {1.0};                     ///< @brief Checkerboard square length
    double marker_length {1.0};                     ///< @brief Marker length
    unsigned int initial_id{0};                     ///< @brief Initial ID
    Eigen::Vector3d pos_f_in_g;                     ///< @brief Fiducial position
    Eigen::Quaterniond ang_f_to_g;                  ///< @brief Fiducial orientation
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
  ///
  void Track(double time, int frame_id, cv::Mat & img_in, cv::Mat & img_out);

protected:
  FiducialUpdater m_fiducial_updater;  ///< @brief MSCKF updater object
  FiducialTypeEnum m_detector_type;    ///< @brief Detector type

private:
  BoardTrack m_board_track;
  Eigen::Vector3d m_pos_error;
  Eigen::Vector3d m_ang_error;
};

#endif  // TRACKERS__FIDUCIAL_TRACKER_HPP_
