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

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "infrastructure/data_logger.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/types.hpp"
#include "trackers/feature_tracker.hpp"
#include "trackers/sim/sim_feature_tracker_message.hpp"
#include "utility/sim/sim_rng.hpp"

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
    unsigned int feature_count{0};              ///< @brief Total feature count
    double room_size{10.0};                     ///< @brief Size of "Room" for features
    bool no_errors {false};                     ///< @brief Perfect measurements flag
    FeatureTracker::Parameters tracker_params;  ///< @brief Tracker parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truth_engine Truth engine
  ///
  SimFeatureTracker(
    Parameters params,
    std::shared_ptr<TruthEngine> truth_engine);

  ///
  /// @brief Generate simulated tracker messages
  ///
  std::vector<std::shared_ptr<SimFeatureTrackerMessage>> GenerateMessages(
    std::vector<double> message_times, int sensor_id);

  ///
  /// @brief Return currently visible keypoints
  /// @param time Current time
  /// @param sensor_id Camera sensor ID
  ///
  std::vector<cv::KeyPoint> VisibleKeypoints(double time, int sensor_id);

  ///
  /// @brief Callback for feature tracker
  /// @param time Message time
  /// @param msg Feature track message
  ///
  void Callback(double time, std::shared_ptr<SimFeatureTrackerMessage> msg);

private:
  double m_px_error;
  std::shared_ptr<TruthEngine> m_truth;
  bool m_no_errors {false};
  SimRNG m_rng;
  unsigned int m_feature_count {0};

  Intrinsics m_intrinsics;
  cv::Mat m_proj_matrix;
};


#endif  // TRACKERS__SIM__SIM_FEATURE_TRACKER_HPP_
