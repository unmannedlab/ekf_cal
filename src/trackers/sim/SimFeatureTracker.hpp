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

#ifndef TRACKERS__SIM__SIMFEATURETRACKER_HPP_
#define TRACKERS__SIM__SIMFEATURETRACKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ekf/Types.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "infrastructure/sim/TruthEngine.hpp"
#include "sensors/Camera.hpp"
#include "sensors/Sensor.hpp"
#include "trackers/FeatureTracker.hpp"
#include "utility/sim/SimRNG.hpp"

///
/// @class SimFeatureTrackerMessage
/// @brief Simulated Tracker Message class
///
class SimFeatureTrackerMessage : public SensorMessage
{
public:
  SimFeatureTrackerMessage() {}
  unsigned int m_tracker_id {0};
  std::vector<std::vector<FeatureTrack>> m_feature_tracks;
};

///
/// @class SimFeatureTracker
/// @brief Simulated Tracker class
///
class SimFeatureTracker : public FeatureTracker
{
public:
  ///
  /// @brief Sim IMU initialization parameters structure
  ///
  typedef struct Parameters
  {
    unsigned int feature_count{0};                       ///< @brief Number of features to generate
    double room_size{10.0};                              ///< @brief Size of "Room" for features
    double px_error{1.0};                                ///< @brief Pixel Error
    FeatureTracker::Parameters tracker_params;           ///< @brief Tracker parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truth_engine Truth engine
  ///
  SimFeatureTracker(Parameters params, std::shared_ptr<TruthEngine> truth_engine);

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

  void Callback(double time, unsigned int camera_id, std::shared_ptr<SimFeatureTrackerMessage> msg);

private:
  double m_px_error{1e-9};
  Eigen::Vector3d m_pos_offset{0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_offset{1.0, 0.0, 0.0, 0.0};
  std::shared_ptr<TruthEngine> m_truth;
  double m_rate{1.0};
  SimRNG m_rng;
  unsigned int m_feature_count {0};
  std::vector<cv::Point3d> m_feature_points;

  double m_focal_length {100};
  unsigned int m_image_width {640};
  unsigned int m_image_height {480};
  cv::Mat m_proj_matrix;
};


#endif  // TRACKERS__SIM__SIMFEATURETRACKER_HPP_
