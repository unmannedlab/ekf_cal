// Copyright 2022 Jacob Hartzer
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

#ifndef TRACKERS__SIM__SIM_FEATURE_TRACKER_HPP_
#define TRACKERS__SIM__SIM_FEATURE_TRACKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ekf/types.hpp"
#include "infrastructure/data_logger.hpp"
#include "infrastructure/debug_logger.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/camera.hpp"
#include "sensors/sensor.hpp"
#include "trackers/feature_tracker.hpp"
#include "trackers/sim/sim_feature_tracker_message.hpp"
#include "utility/sim/sim_rng.hpp"


///
/// @class SimFeatureTracker
/// @brief Simulated Tracker class
/// @todo Update ang_offset to be more descriptive
///
class SimFeatureTracker : public FeatureTracker
{
public:
  ///
  /// @brief Sim IMU initialization parameters structure
  ///
  typedef struct Parameters
  {
    unsigned int feature_count{0};                      ///< @brief Number of features to generate
    double room_size{10.0};                             ///< @brief Size of "Room" for features
    Eigen::Vector3d pos_offset{0.0, 0.0, 0.0};          ///< @brief Camera position offset
    Eigen::Quaterniond ang_offset{1.0, 0.0, 0.0, 0.0};  ///< @brief Camera angular offset
    bool no_errors {false};                             ///< @brief Perfect measurements flag
    FeatureTracker::Parameters tracker_params;          ///< @brief Tracker parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truth_engine Truth engine
  /// @param log_file_directory Directory to save data log file
  /// @param data_logging_on Flag to enable data logging
  ///
  SimFeatureTracker(
    Parameters params,
    std::shared_ptr<TruthEngine> truth_engine,
    std::string log_file_directory,
    bool data_logging_on);

  ///
  /// @brief Generate simulated tracker messages
  ///
  std::vector<std::shared_ptr<SimFeatureTrackerMessage>> GenerateMessages(
    std::vector<double> message_times,
    unsigned int sensor_id);

  ///
  /// @brief Return currently visible keypoints
  /// @param time Current time
  ///
  std::vector<cv::KeyPoint> VisibleKeypoints(double time);

  ///
  /// @brief
  /// @param time Message time
  /// @param camera_id Camera ID
  /// @param msg Feature track message
  ///
  void Callback(double time, unsigned int camera_id, std::shared_ptr<SimFeatureTrackerMessage> msg);

private:
  double m_px_error;
  Eigen::Vector3d m_pos_offset;
  Eigen::Quaterniond m_ang_offset;
  std::shared_ptr<TruthEngine> m_truth;
  bool m_no_errors {false};
  SimRNG m_rng;
  unsigned int m_feature_count {0};
  std::vector<cv::Point3d> m_feature_points;

  double m_focal_length {1};
  double m_pixel_size {0.010};
  unsigned int m_image_width {640};
  unsigned int m_image_height {480};
  cv::Mat m_proj_matrix;
  DataLogger m_data_logger;
};


#endif  // TRACKERS__SIM__SIM_FEATURE_TRACKER_HPP_
