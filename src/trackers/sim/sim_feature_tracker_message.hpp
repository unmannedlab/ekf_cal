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

#ifndef TRACKERS__SIM__SIM_FEATURE_TRACKER_MESSAGE_HPP_
#define TRACKERS__SIM__SIM_FEATURE_TRACKER_MESSAGE_HPP_

#include "sensors/sensor_message.hpp"
#include "ekf/types.hpp"

///
/// @class SimFeatureTrackerMessage
/// @brief Simulated Tracker Message class
///
class SimFeatureTrackerMessage : public SensorMessage
{
public:
  SimFeatureTrackerMessage() {}
  unsigned int m_tracker_id {0};                            ///< @brief Associated Tracker ID
  std::vector<std::vector<FeatureTrack>> m_feature_tracks;  ///< @brief Feature Tracks
};

#endif  // TRACKERS__SIM__SIM_FEATURE_TRACKER_MESSAGE_HPP_
