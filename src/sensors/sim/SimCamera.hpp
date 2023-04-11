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


#ifndef SENSORS__SIM__SIMCAMERA_HPP_
#define SENSORS__SIM__SIMCAMERA_HPP_

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>
#include <vector>

#include "ekf/Types.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "infrastructure/sim/TruthEngine.hpp"
#include "sensors/Camera.hpp"
#include "sensors/Sensor.hpp"
#include "utility/sim/SimRNG.hpp"
#include "trackers/sim/SimFeatureTracker.hpp"

class SimCameraMessage : public CameraMessage
{
public:
  ///
  /// @brief Define SimCameraMessage constructor with CameraMessage's
  ///
  using CameraMessage::CameraMessage;
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
    double tBias {0.0};                                 ///< @brief Time offset bias
    double tSkew {1.0};                               ///< @brief Time offset error
    double tError {1e-9};                               ///< @brief Time offset error
    Eigen::Vector3d posOffset {0.0, 0.0, 0.0};          ///< @brief Sensor position offset
    Eigen::Quaterniond angOffset {1.0, 0.0, 0.0, 0.0};  ///< @brief Sensor angular offset
    Camera::Parameters camParams;                              ///< @brief IMU sensor parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truthEngine Truth engine
  ///
  SimCamera(SimCamera::Parameters params, std::shared_ptr<TruthEngine> truthEngine);

  void addTracker(std::shared_ptr<SimFeatureTracker> tracker);

  ///
  /// @brief Generate simulated IMU messages
  /// @param maxTime Maximum time of generated messages
  ///
  std::vector<std::shared_ptr<SimCameraMessage>> generateMessages(double maxTime);

private:
  double m_tBias{0.0};
  double m_tSkew{0.0};
  double m_tError{1e-9};
  Eigen::Vector3d m_posOffset{0.0, 0.0, 0.0};
  Eigen::Quaterniond m_angOffset{1.0, 0.0, 0.0, 0.0};
  SimRNG m_rng;
  std::shared_ptr<TruthEngine> m_truth;

  /// @todo create vector of trackers
  std::vector<std::shared_ptr<SimFeatureTracker>> m_trackers;
};


#endif  // SENSORS__SIM__SIMCAMERA_HPP_
