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
  /// @param init_err_thresh Error limit to initialize local reference frame
  /// @param baseline_initialization Flag to use baseline distance initialization
  /// @param baseline_distance Distance threshold to initialize
  /// @param log_file_directory Logging file directory
  /// @param data_logging_on Logging flag
  /// @param data_log_rate Maximum data logging rate
  /// @param logger Debug logger pointer
  ///
  GpsUpdater(
    unsigned int gps_id,
    unsigned int initialization_type,
    double init_err_thresh,
    double init_baseline_dist,
    std::string log_file_directory,
    bool data_logging_on,
    double data_log_rate,
    std::shared_ptr<DebugLogger> logger
  );

  ///
  /// @brief GPS LLA to ENU Initialization Routine
  /// @param time GPS measured time
  /// @param ekf EKF pointer
  /// @param gps_lla GPS measured lat-lon-alt
  ///
  void AttemptInitialization(
    double time,
    std::shared_ptr<EKF> ekf,
    Eigen::Vector3d gps_lla);

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
  /// @brief
  /// @param ekf
  /// @param gps_time_vec
  /// @param gps_ecef_vec
  /// @param local_xyz_vec
  ///
  void MultiUpdateEKF(
    std::shared_ptr<EKF> ekf,
    std::vector<double> gps_time_vec,
    std::vector<Eigen::Vector3d> gps_ecef_vec,
    std::vector<Eigen::Vector3d> local_xyz_vec);

private:
  double m_projection_stddev {0.0};
  unsigned int m_initialization_type;
  double m_init_err_thresh;
  double m_init_baseline_dist;
  DataLogger m_data_logger;
  std::vector<double> m_gps_time_vec;
  std::vector<Eigen::Vector3d> m_gps_ecef_vec;
  std::vector<Eigen::Vector3d> m_local_xyz_vec;
};

#endif  // EKF__UPDATE__GPS_UPDATER_HPP_
