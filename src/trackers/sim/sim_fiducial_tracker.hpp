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

#include "infrastructure/data_logger.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/types.hpp"
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
    bool no_errors {false};                       ///< @brief Perfect measurements flag
    Eigen::Vector3d pos_error{1e-9, 1e-9, 1e-9};  ///< @brief Position error standard deviation
    Eigen::Vector3d ang_error{1e-9, 1e-9, 1e-9};  ///< @brief Angular error standard deviation
    FiducialTracker::Parameters fiducial_params;  ///< @brief Tracker parameters
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
  ///
  std::vector<std::shared_ptr<SimFiducialTrackerMessage>> GenerateMessages(
    std::vector<double> message_times, int sensor_id);

  ///
  /// @brief Return currently visible keypoints
  /// @param time Current time
  ///
  bool IsBoardVisible(double time);

  ///
  /// @brief Callback for feature tracker
  /// @param time Message time
  /// @param msg Feature track message
  ///
  void Callback(double time, std::shared_ptr<SimFiducialTrackerMessage> msg);

  ///
  /// @brief True camera offset setter
  /// @param pos_c_in_b_true True position offset of camera in body frame
  /// @param ang_c_to_b_true True angular offset of camera in body frame
  ///
  void SetTrueCameraOffsets(
    Eigen::Vector3d pos_c_in_b_true,
    Eigen::Quaterniond ang_c_to_b_true);

private:
  Eigen::Vector3d m_pos_error;
  Eigen::Vector3d m_ang_error;
  Eigen::Vector3d m_pos_c_in_b_true {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_c_to_b_true {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_pos_f_in_g_true {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_f_to_g_true {1.0, 0.0, 0.0, 0.0};
  std::shared_ptr<TruthEngine> m_truth;
  bool m_no_errors {false};
  SimRNG m_rng;

  Intrinsics m_intrinsics;
  cv::Mat m_proj_matrix;
  DataLogger m_data_logger;
};


#endif  // TRACKERS__SIM__SIM_FIDUCIAL_TRACKER_HPP_
