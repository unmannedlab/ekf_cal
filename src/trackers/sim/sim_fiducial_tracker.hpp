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

#ifndef TRACKERS__SIM__SIM_FIDUCIAL_TRACKER_HPP_
#define TRACKERS__SIM__SIM_FIDUCIAL_TRACKER_HPP_

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "ekf/types.hpp"
#include "infrastructure/data_logger.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "trackers/fiducial_tracker.hpp"
#include "trackers/sim/sim_fiducial_tracker_message.hpp"
#include "utility/sim/sim_rng.hpp"

///
/// @class SimFiducialTracker
/// @brief Simulated Tracker class
///
class SimFiducialTracker : public FiducialTracker
{
public:
  ///
  /// @brief Sim IMU initialization parameters structure
  ///
  typedef struct Parameters
  {
    bool no_errors {false};                         ///< @brief Perfect measurements flag
    Eigen::Vector3d pos_error{1e-9, 1e-9, 1e-9};    ///< @brief Position error standard deviation
    Eigen::Vector3d ang_error{1e-9, 1e-9, 1e-9};    ///< @brief Angular error standard deviation
    Eigen::Vector3d t_vec_error{1e-9, 1e-9, 1e-9};  ///< @brief t_vec error standard deviation
    Eigen::Vector3d r_vec_error{1e-9, 1e-9, 1e-9};  ///< @brief r_vec error standard deviation
    SimRNG rng;                                     ///< @brief Random number generator
    FiducialTracker::Parameters fiducial_params;    ///< @brief Tracker parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truth_engine Truth engine
  ///
  SimFiducialTracker(
    Parameters params,
    std::shared_ptr<TruthEngine> truth_engine);

  ///
  /// @brief Generate simulated tracker messages
  /// @param message_time Message time
  /// @param frame_id Camera frame ID
  /// @param sensor_id Camera sensor ID
  /// @return Generated fiducial tracker message
  ///
  std::shared_ptr<SimFiducialTrackerMessage> GenerateMessage(
    double message_time, int frame_id, int sensor_id);

  ///
  /// @brief Return currently visible keypoints
  /// @param time Current time
  /// @param sensor_id Camera sensor ID
  ///
  bool IsBoardVisible(double time, int sensor_id);

  ///
  /// @brief Callback for feature tracker
  /// @param time Message time
  /// @param msg Feature track message
  ///
  void Callback(double time, std::shared_ptr<SimFiducialTrackerMessage> msg);

private:
  SimRNG m_rng;
  Eigen::Vector3d m_pos_error;
  Eigen::Vector3d m_ang_error;
  Eigen::Vector3d m_t_vec_error;
  Eigen::Vector3d m_r_vec_error;
  std::shared_ptr<TruthEngine> m_truth;
  bool m_no_errors {false};
  BoardTrack m_board_track;

  unsigned int m_min_track_length {0U};
  unsigned int m_max_track_length {1U};
};


#endif  // TRACKERS__SIM__SIM_FIDUCIAL_TRACKER_HPP_
