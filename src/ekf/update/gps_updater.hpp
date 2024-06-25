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

#ifndef EKF__UPDATE__GPS_UPDATER_HPP_
#define EKF__UPDATE__GPS_UPDATER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "ekf/update/updater.hpp"
#include "infrastructure/data_logger.hpp"

///
/// @class GpsUpdater
/// @brief EKF Updater Class for GPS Sensors
///
class GpsUpdater : public Updater
{
public:
  ///
  /// @brief GPS EKF Updater Constructor
  /// @param gps_id GPS Sensor ID
  /// @param log_file_directory Logging file directory
  /// @param is_extrinsic Is GPS extrinsic calibrating
  /// @param data_logging_on Logging flag
  /// @param data_log_rate Maximum data logging rate
  /// @param logger Debug logger pointer
  ///
  GpsUpdater(
    unsigned int gps_id,
    bool is_extrinsic,
    std::string log_file_directory,
    bool data_logging_on,
    double data_log_rate,
    std::shared_ptr<DebugLogger> logger
  );

  ///
  /// @brief Measurement Jacobian method
  /// @param ekf EKF pointer
  /// @return Measurement Jacobian matrix
  ///
  Eigen::MatrixXd GetMeasurementJacobian(std::shared_ptr<EKF> ekf);

  ///
  /// @brief EKF update method for GPS measurements
  /// @param ekf EKF pointer
  /// @param time Measurement time
  /// @param gps_lla GPS measured lat-lon-alt
  ///
  void UpdateEKF(
    std::shared_ptr<EKF> ekf,
    double time,
    Eigen::Vector3d gps_lla);

  ///
  /// @brief Update/marginalize EKF using GPS measurements used to initialize local frame
  /// @param ekf EKF pointer
  ///
  void MultiUpdateEKF(std::shared_ptr<EKF> ekf);

private:
  bool m_is_extrinsic {false};
  DataLogger m_data_logger;
};

#endif  // EKF__UPDATE__GPS_UPDATER_HPP_
