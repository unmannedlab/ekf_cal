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


#ifndef SENSORS__SIM__SIM_CAMERA_HPP_
#define SENSORS__SIM__SIM_CAMERA_HPP_

#include <eigen3/Eigen/Eigen>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/camera.hpp"
#include "sensors/sensor.hpp"
#include "utility/sim/sim_rng.hpp"
#include "trackers/sim/sim_feature_tracker.hpp"

class SimCameraMessage : public CameraMessage
{
public:
  ///
  /// @brief Define SimCameraMessage constructor with CameraMessage's
  ///
  using CameraMessage::CameraMessage;
  std::shared_ptr<SimFeatureTrackerMessage> m_feature_track_message;
};

///
/// @class SimCamera
/// @brief Simulated Camera Sensor Class
///
class SimCamera : public Camera
{
public:
  ///
  /// @brief Sim IMU initialization parameters structure
  ///
  typedef struct Parameters
  {
    double time_bias {0.0};                                 ///< @brief Time offset bias
    double time_skew {1.0};                               ///< @brief Time offset error
    double time_error {1e-9};                               ///< @brief Time offset error
    Eigen::Vector3d pos_offset {0.0, 0.0, 0.0};          ///< @brief Sensor position offset
    Eigen::Quaterniond ang_offset {1.0, 0.0, 0.0, 0.0};  ///< @brief Sensor angular offset
    Camera::Parameters cam_params;                              ///< @brief IMU sensor parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truth_engine Truth engine
  ///
  SimCamera(SimCamera::Parameters params, std::shared_ptr<TruthEngine> truth_engine);

  void AddTracker(std::shared_ptr<SimFeatureTracker> tracker);

  std::vector<double> GenerateMessageTimes(double max_time);

  void Callback(std::shared_ptr<SimCameraMessage> sim_camera_message);

  ///
  /// @brief Generate simulated IMU messages
  /// @param max_time Maximum time of generated messages
  ///
  std::vector<std::shared_ptr<SimCameraMessage>> GenerateMessages(double max_time);

private:
  double m_time_bias{0.0};
  double m_time_skew{0.0};
  double m_time_error{1e-9};
  Eigen::Vector3d m_pos_offset{0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_offset{1.0, 0.0, 0.0, 0.0};
  SimRNG m_rng;
  std::shared_ptr<TruthEngine> m_truth;

  /// @todo create vector of trackers
  std::map<unsigned int, std::shared_ptr<SimFeatureTracker>> m_trackers;
};


#endif  // SENSORS__SIM__SIM_CAMERA_HPP_
