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

#ifndef SENSORS__GPS_HPP_
#define SENSORS__GPS_HPP_

#include <eigen3/Eigen/Eigen>

#include <string>
#include <memory>

#include "ekf/types.hpp"
#include "ekf/update/gps_updater.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/gps_message.hpp"
#include "sensors/sensor.hpp"

///
/// @class GPS
/// @brief GPS Sensor Class
/// @todo Bias Stability and Noise process inputs for GPSs
///
class GPS : public Sensor
{
public:
  ///
  /// @brief GPS initialization parameters structure
  ///
  typedef struct Parameters : public Sensor::Parameters
  {
    Eigen::Vector3d pos_a_in_b {0, 0, 0};           ///< @brief GPS antenna position offset vector
    Eigen::Vector3d pos_l_in_g {0, 0, 0};           ///< @brief Local LLA position in global
    double ang_l_to_g {0.0};                        ///< @brief Local angle to global
    Eigen::Vector3d variance {{1, 1, 1}};           ///< @brief Initial state variance
    bool is_extrinsic {false};                      ///< @brief Online extrinsic calibration flag
    double pos_stability {1e-9};                    ///< @brief Position stability
  } Parameters;

  ///
  /// @brief GPS class constructor
  /// @param params GPS sensor parameters
  ///
  explicit GPS(GPS::Parameters params);

  ///
  /// @brief Callback method for GPS measurements
  /// @param gps_message GPS measurement message
  ///
  void Callback(std::shared_ptr<GpsMessage> gps_message);

private:
  std::shared_ptr<EKF> m_ekf;
  GpsUpdater m_gps_updater;
};

#endif  // SENSORS__GPS_HPP_
