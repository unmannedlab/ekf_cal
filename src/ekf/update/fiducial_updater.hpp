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

#ifndef EKF__UPDATE__FIDUCIAL_UPDATER_HPP_
#define EKF__UPDATE__FIDUCIAL_UPDATER_HPP_

#include <eigen3/Eigen/Eigen>

#include <string>
#include <vector>

#include "ekf/types.hpp"
#include "ekf/update/updater.hpp"
#include "infrastructure/data_logger.hpp"
#include "sensors/types.hpp"

///
/// @class FiducialUpdater
/// @brief EKF Updater Class for Fiducial Camera Measurements
///
class FiducialUpdater : public Updater
{
public:
  ///
  /// @brief MSCKF EKF Updater constructor
  /// @param cam_id Camera sensor ID
  /// @param log_file_directory Directory to save log files
  /// @param data_logging_on Flag to enable data logging
  ///
  explicit FiducialUpdater(
    int cam_id,
    std::string log_file_directory,
    bool data_logging_on);

  ///
  /// @brief EKF updater function
  /// @param time Time of update
  /// @param board_track Board track to be used for state update
  /// @param pos_error Standard deviation of the position error
  /// @param ang_error Standard deviation of the angle error
  ///
  void UpdateEKF(
    double time, BoardTrack board_track, Eigen::Vector3d pos_error, Eigen::Vector3d ang_error);

  ///
  /// @brief Refresh internal states with EKF values
  ///
  void RefreshStates();

private:
  Eigen::Vector3d m_body_pos {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_vel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_acc {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_b_to_g {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_ang_vel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_ang_acc {0.0, 0.0, 0.0};

  Eigen::Vector3d m_pos_c_in_b {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_c_to_b {1.0, 0.0, 0.0, 0.0};

  DataLogger m_data_logger;
  DataLogger m_fiducial_logger;
  Intrinsics m_intrinsics;
};

#endif  // EKF__UPDATE__FIDUCIAL_UPDATER_HPP_
