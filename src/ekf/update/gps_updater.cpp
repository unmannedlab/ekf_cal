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

#include "ekf/update/gps_updater.hpp"

#include <eigen3/Eigen/Eigen>
#include <unistd.h>

#include <memory>
#include <string>
#include <sstream>

#include "ekf/constants.hpp"
#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "utility/gps_helper.hpp"
#include "utility/math_helper.hpp"
#include "utility/string_helper.hpp"
#include "utility/type_helper.hpp"


GpsUpdater::GpsUpdater(
  unsigned int gps_id,
  std::string log_file_directory,
  bool data_logging_on,
  std::shared_ptr<DebugLogger> logger
)
: Updater(gps_id, logger),
  m_data_logger(
    log_file_directory,
    "gps_" + std::to_string(gps_id) + ".csv")
{
  std::stringstream header;
  header << "time,lat,lon,alt";
  header << EnumerateHeader("time", 1);
  header << std::endl;

  m_data_logger.DefineHeader(header.str());
  m_data_logger.SetLogging(data_logging_on);
}

Eigen::VectorXd GpsUpdater::PredictMeasurement()
{
  /// @todo(jhartzer): This
  Eigen::VectorXd predicted_measurement(3);
  return predicted_measurement;
}

Eigen::MatrixXd GpsUpdater::GetMeasurementJacobian()
{
  /// @todo(jhartzer): This
  Eigen::MatrixXd measurement_jacobian;
  return measurement_jacobian;
}

void GpsUpdater::RefreshStates()
{
  /// @todo(jhartzer): This
}


void GpsUpdater::UpdateEKF(
  std::shared_ptr<EKF> ekf,
  double time,
  double latitude,
  double longitude,
  double altitude)
{
  ekf->ProcessModel(time);
  RefreshStates();

  Eigen::VectorXd predicted_measurement = PredictMeasurement();
  Eigen::MatrixXd measurement_jacobian = GetMeasurementJacobian();

  auto t_start = std::chrono::high_resolution_clock::now();

  /// @todo(jhartzer): Implement this updater
  m_logger->Log(LogLevel::WARN, "GPS Updater not yet implemented");

  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;
  msg << time;
  msg << "," << latitude;
  msg << "," << longitude;
  msg << "," << altitude;
  msg << "," << t_execution.count();
  msg << std::endl;
  m_data_logger.Log(msg.str());
}
