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

#include <memory>
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
  /// @param data_log_rate Maximum average rate to log data
  /// @param logger Debug logger pointer
  ///
  explicit FiducialUpdater(
    int cam_id,
    std::string log_file_directory,
    bool data_logging_on,
    double data_log_rate,
    std::shared_ptr<DebugLogger> logger
  );

  ///
  /// @brief EKF updater function
  /// @param ekf EKF pointer
  /// @param time Time of update
  /// @param board_track Board track to be used for state update
  /// @param pos_error Standard deviation of the position error
  /// @param ang_error Standard deviation of the angle error
  ///
  void UpdateEKF(
    std::shared_ptr<EKF> ekf,
    double time, BoardTrack board_track, double pos_error, double ang_error);

private:
  DataLogger m_fiducial_logger;
  DataLogger m_triangulation_logger;
  Intrinsics m_intrinsics;
};

#endif  // EKF__UPDATE__FIDUCIAL_UPDATER_HPP_
