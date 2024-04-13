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

#include "ekf/update/gps_updater.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/gps_message.hpp"
#include "sensors/sensor.hpp"

///
/// @class GPS
/// @brief GPS Sensor Class
/// @todo Add parameter input/defaults for covariance
/// @todo Bias Stability and Noise process inputs for GPSs
///
class GPS : public Sensor
{
public:
  ///
  /// @brief GPS initialization parameters structure
  /// @todo remove topic from parameters
  ///
  typedef struct Parameters
  {
    std::string name {"Name"};             ///< @brief GPS name
    std::string topic {"Topic"};           ///< @brief GPS topic
    double rate{1.0};                      ///< @brief GPS update rate
    Eigen::Vector3d pos_a_in_b {0, 0, 0};  ///< @brief GPS antenna position offset vector
    std::string output_directory {""};     ///< @brief GPS data logging directory
    bool data_logging_on {false};          ///< @brief GPS data logging flag
    Eigen::Vector3d variance {{1, 1, 1}};  ///< @brief Initial state variance
    double data_log_rate {0.0};            ///< @brief Data logging rate
    std::shared_ptr<DebugLogger> logger;   ///< @brief Debug logger
    std::shared_ptr<EKF> ekf;              ///< @brief EKF to update
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
