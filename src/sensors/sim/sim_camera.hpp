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
#include <vector>

#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/camera.hpp"
#include "sensors/sim/sim_camera_message.hpp"
#include "trackers/sim/sim_feature_tracker.hpp"
#include "trackers/sim/sim_fiducial_tracker.hpp"
#include "utility/sim/sim_rng.hpp"


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
    bool no_errors {false};                     ///< @brief Perfect measurements flag
    double time_error {0.0};                    ///< @brief Time offset error
    double time_bias_error {0.0};               ///< @brief Time offset bias
    double time_skew_error {0.0};               ///< @brief Time offset error
    Eigen::Vector3d pos_error {0.0, 0.0, 0.0};  ///< @brief Position offset error
    Eigen::Vector3d ang_error {0.0, 0.0, 0.0};  ///< @brief Angular offset error
    Camera::Parameters cam_params;              ///< @brief Camera sensor parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truth_engine Truth engine
  ///
  SimCamera(SimCamera::Parameters params, std::shared_ptr<TruthEngine> truth_engine);

  ///
  /// @brief Method to add tracker object to camera sensor
  /// @param tracker Tracker pointer for camera to use during callbacks
  ///
  void AddTracker(std::shared_ptr<SimFeatureTracker> tracker);

  ///
  /// @brief Method to add fiducial object to camera sensor
  /// @param fiducial Fiducial pointer for camera to use during callbacks
  ///
  void AddFiducial(std::shared_ptr<SimFiducialTracker> fiducial);

  ///
  /// @brief Generate simulated camera message times
  /// @param rng Random number generator
  /// @param max_time Maximum time to generate frame times
  ///
  std::vector<double> GenerateMessageTimes(SimRNG rng, double max_time);

  ///
  /// @brief Callback method for simulated camera
  /// @param sim_camera_message Simulated camera data message
  ///
  void Callback(std::shared_ptr<SimCameraMessage> sim_camera_message);

  ///
  /// @brief Generate simulated IMU messages
  /// @param rng Random number generator
  /// @param max_time Maximum time of generated messages
  /// @return Generated camera messages
  ///
  std::vector<std::shared_ptr<SimCameraMessage>> GenerateMessages(SimRNG rng, double max_time);

private:
  bool m_no_errors {false};
  double m_time_error{0.0};
  double m_time_bias_error{0.0};
  double m_time_skew_error{0.0};
  Eigen::Vector3d m_pos_error;
  Eigen::Vector3d m_ang_error;
  std::shared_ptr<TruthEngine> m_truth;

  std::map<unsigned int, std::shared_ptr<SimFeatureTracker>> m_trackers;
  std::map<unsigned int, std::shared_ptr<SimFiducialTracker>> m_fiducials;
};


#endif  // SENSORS__SIM__SIM_CAMERA_HPP_
