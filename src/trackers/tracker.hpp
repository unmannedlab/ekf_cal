// Copyright 2024 Jacob Hartzer
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

#ifndef TRACKERS__TRACKER_HPP_
#define TRACKERS__TRACKER_HPP_

#include <memory>
#include <string>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"

///
/// @class Tracker
/// @brief Tracker Class
///
class Tracker
{
public:
  ///
  /// @brief Tracker Initialization parameters structure
  ///
  typedef struct Parameters
  {
    std::string name {""};                ///< @brief Feature Tracker name
    unsigned int camera_id{0};            ///< @brief Associated sensor ID
    std::string log_directory {""};       ///< @brief Feature Tracker data logging directory
    Intrinsics intrinsics;                ///< @brief Camera intrinsic parameters
    unsigned int min_track_length{2};     ///< @brief Minimum track length to consider
    unsigned int max_track_length{20};    ///< @brief Maximum track length before forced output
    double data_log_rate {0.0};           ///< @brief Data logging rate
    std::shared_ptr<DebugLogger> logger;  ///< @brief Debug logger
    std::shared_ptr<EKF> ekf;             ///< @brief EKF to update
  } Parameters;

  ///
  /// @brief Tracker Initialization parameters structure
  /// @param params
  ///
  explicit Tracker(Parameters params);

  ///
  /// @brief Tracker ID getter method
  /// @return Tracker ID
  ///
  unsigned int GetID();

protected:
  unsigned int m_id;                      ///< @brief Tracker ID
  unsigned int m_camera_id;               ///< @brief Associated camera ID of tracker
  unsigned int m_min_track_length{2};     ///< @brief Minimum track length to consider
  unsigned int m_max_track_length{20};    ///< @brief Maximum track length before forced output
  std::shared_ptr<EKF> m_ekf;             ///< @brief EKF
  std::shared_ptr<DebugLogger> m_logger;  ///< @brief Debug logger

private:
  static unsigned int m_tracker_count;    ///< @brief Static tracker count
};

#endif  // TRACKERS__TRACKER_HPP_
