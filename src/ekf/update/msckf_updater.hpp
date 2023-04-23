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

#ifndef EKF__UPDATE__MSCKF_UPDATER_HPP_
#define EKF__UPDATE__MSCKF_UPDATER_HPP_

#include <string>
#include <vector>

#include "ekf/update/updater.hpp"
#include "ekf/ekf.hpp"
#include "infrastructure/data_logger.hpp"

///
/// @class MsckfUpdater
/// @brief EKF Updater Class for MSCKF Camera Measurements
///
class MsckfUpdater : public Updater
{
public:
  ///
  /// @brief MSCKF EKF Updater constructor
  /// @param cam_id Camera sensor ID
  /// @param log_file_directory Directory to save log files
  /// @param data_logging_on Flag to enable data logging
  ///
  explicit MsckfUpdater(
    unsigned int cam_id,
    std::string log_file_directory,
    bool data_logging_on);

  ///
  /// @brief
  /// @param frame_id
  ///
  AugmentedState MatchState(unsigned int frame_id);

  ///
  /// @brief
  /// @param time Time of update
  /// @param camera_id ID of camera associated with update
  /// @param feature_tracks Feature tracks to be used for state update
  ///
  void UpdateEKF(double time, unsigned int camera_id, FeatureTracks feature_tracks);

  ///
  /// @brief Refresh internal states with EKF values
  ///
  void RefreshStates();

private:
  Eigen::Vector3d m_body_pos {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_vel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_acc {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_body_ang_pos {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_ang_vel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_ang_acc {0.0, 0.0, 0.0};
  Eigen::Vector3d m_pos_offset {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_offset {1.0, 0.0, 0.0, 0.0};
  std::vector<AugmentedState> m_aug_states {};
  DataLogger m_data_logger;
};

#endif  // EKF__UPDATE__MSCKF_UPDATER_HPP_
