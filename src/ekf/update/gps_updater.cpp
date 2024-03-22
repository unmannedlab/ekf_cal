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

void GpsUpdater::Initialize(
  double latitude,
  double longitude,
  double altitude)
{
  m_reference_lla[0] = latitude;
  m_reference_lla[1] = longitude;
  m_reference_lla[2] = altitude;
}

Eigen::VectorXd GpsUpdater::PredictMeasurement(
  std::shared_ptr<EKF> ekf)
{
  Eigen::VectorXd predicted_measurement =
    enu_to_lla(ekf->GetBodyState().m_position, m_reference_lla);

  return predicted_measurement;
}

Eigen::MatrixXd GpsUpdater::GetMeasurementJacobian()
{
  /// @todo(jhartzer): This
  Eigen::MatrixXd measurement_jacobian;
  return measurement_jacobian;
}

void GpsUpdater::UpdateEKF(
  std::shared_ptr<EKF> ekf,
  double time,
  double latitude,
  double longitude,
  double altitude)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  if (!m_is_lla_initialized) {
    Initialize(latitude, longitude, altitude);
    m_logger->Log(LogLevel::WARN, "GPS Updater Initialization");
  } else {
    ekf->ProcessModel(time);

    Eigen::VectorXd measurement{latitude, longitude, altitude};
    Eigen::VectorXd predicted_measurement = PredictMeasurement(ekf);
    Eigen::VectorXd resid = measurement - predicted_measurement;
    Eigen::MatrixXd H = GetMeasurementJacobian();

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
    R.block<3, 3>(0, 0) = MinBoundDiagonal(acceleration_covariance * 3, 1e-3);
    R.block<3, 3>(3, 3) = MinBoundDiagonal(angular_rate_covariance * 3, 1e-2);

    Eigen::MatrixXd S = H * ekf->GetCov().block(0, 0, update_size, update_size) * H.transpose() + R;
    Eigen::MatrixXd K =
      ekf->GetCov().block(0, 0, update_size, update_size) * H.transpose() * S.inverse();

    Eigen::VectorXd update = K * resid;
    Eigen::VectorXd body_update = update.segment<g_body_state_size>(0);
    Eigen::VectorXd imu_update = update.segment(g_body_state_size, update_size - g_body_state_size);

    ekf->GetState().m_body_state += body_update;
    ekf->GetState().m_imu_states += imu_update;

    // ekf->GetCov().block(0, 0, update_size, update_size) =
    //   (Eigen::MatrixXd::Identity(update_size, update_size) - K * H) *
    //   ekf->GetCov().block(0, 0, update_size, update_size) *
    //   (Eigen::MatrixXd::Identity(update_size, update_size) - K * H).transpose() +
    //   K * R * K.transpose();

    /// @todo(jhartzer): Implement this updater
    m_logger->Log(LogLevel::WARN, "GPS Updater Update");
  }
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
