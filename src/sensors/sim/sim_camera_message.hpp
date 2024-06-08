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


#ifndef SENSORS__SIM__SIM_CAMERA_MESSAGE_HPP_
#define SENSORS__SIM__SIM_CAMERA_MESSAGE_HPP_

#include <memory>

#include "sensors/camera_message.hpp"
#include "trackers/sim/sim_feature_tracker_message.hpp"
#include "trackers/sim/sim_fiducial_tracker_message.hpp"

///
/// @class SimCameraMessage
/// @brief Camera message subclass for simulated camera
///
class SimCameraMessage : public CameraMessage
{
public:
  ///
  /// @brief Define SimCameraMessage constructor with CameraMessage's
  ///
  using CameraMessage::CameraMessage;

  /// @brief Message output from feature tracker
  std::shared_ptr<SimFeatureTrackerMessage> feature_track_message;

  /// @brief Message output from fiducial tracker
  std::shared_ptr<SimFiducialTrackerMessage> fiducial_track_message;
};

#endif  // SENSORS__SIM__SIM_CAMERA_MESSAGE_HPP_
