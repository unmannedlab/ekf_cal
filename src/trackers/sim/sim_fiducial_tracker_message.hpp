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

#ifndef TRACKERS__SIM__SIM_FIDUCIAL_TRACKER_MESSAGE_HPP_
#define TRACKERS__SIM__SIM_FIDUCIAL_TRACKER_MESSAGE_HPP_

#include <eigen3/Eigen/Eigen>

#include "ekf/types.hpp"
#include "sensors/sensor_message.hpp"

///
/// @class SimFiducialTrackerMessage
/// @brief Simulated Fiducial Tracker Message class
///
class SimFiducialTrackerMessage : public SensorMessage
{
public:
  SimFiducialTrackerMessage() {}
  unsigned int tracker_id {0};     ///< @brief Associated Tracker ID
  BoardDetection board_detection;  ///< @brief Board detection
  bool is_board_visible {false};   ///< @brief Is board visible
};

#endif  // TRACKERS__SIM__SIM_FIDUCIAL_TRACKER_MESSAGE_HPP_
