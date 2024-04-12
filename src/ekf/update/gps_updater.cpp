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
#include <sstream>
#include <string>
#include <vector>

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
  m_data_logger(log_file_directory, "gps_" + std::to_string(gps_id) + ".csv")
{
  std::stringstream header;
  header << "time,lat,lon,alt";
  header << EnumerateHeader("time", 1);
  header << std::endl;

  m_data_logger.DefineHeader(header.str());
  m_data_logger.SetLogging(data_logging_on);
}

/// @todo(jhartzer): Add option to check max distance of baseline to initialize
void GpsUpdater::AttemptInitialization(
  double time,
  Eigen::Vector3d gps_lla,
  Eigen::Vector3d pos_xyz)
{
  Eigen::Vector3d gps_ecef = lla_to_ecef(gps_lla);
  m_augmented_gps_states.time.push_back(time);
  m_augmented_gps_states.gps_ecef.push_back(gps_ecef);
  m_augmented_gps_states.local_xyz.push_back(pos_xyz);

  if (m_augmented_gps_states.time.size() >= 4) {
    // Check eigenvalue of SVD from Kabsch

    Eigen::Vector3d init_ref_ecef = average_vectors(m_augmented_gps_states.gps_ecef);
    Eigen::Vector3d init_ref_lla = ecef_to_lla(init_ref_ecef);

    std::vector<Eigen::Vector3d> gps_states_enu;
    for (auto gps_ecef : m_augmented_gps_states.gps_ecef) {
      gps_states_enu.push_back(ecef_to_enu(gps_ecef, init_ref_lla));
    }

    Eigen::Affine3d transformation;
    Eigen::Vector2d singular_values;
    double residual_rms;
    bool is_successful = kabsch_2d(
      m_augmented_gps_states.local_xyz,
      gps_states_enu,
      transformation,
      singular_values,
      residual_rms);

    /// @todo(jhartzer): Get quality limit from input
    if (is_successful && (singular_values.maxCoeff() > 2.0)) {
      Eigen::Vector3d delta_ref_enu = transformation.translation();
      m_reference_lla = enu_to_lla(delta_ref_enu, init_ref_lla);

      /// @todo(jhartzer): Get heading from alignment output
      // m_reference_heading = transformation.linear()
      m_is_lla_initialized = true;
      m_logger->Log(LogLevel::INFO, "GPS Updater Initialized");
      // Perform single update with compressed measurements
    }
  }
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
  /// @todo(jhartzer): Implement this Jacobian
  Eigen::MatrixXd measurement_jacobian;
  return measurement_jacobian;
}

void GpsUpdater::UpdateEKF(
  std::shared_ptr<EKF> ekf,
  double time,
  Eigen::Vector3d gps_lla)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  if (!m_is_lla_initialized) {
    m_logger->Log(LogLevel::INFO, "GPS Updater Attempt Initialization");
    AttemptInitialization(time, gps_lla, ekf->GetState().m_body_state.m_position);
  } else {
    ekf->ProcessModel(time);


    Eigen::Vector3d predicted_measurement = PredictMeasurement(ekf);
    Eigen::Vector3d resid = gps_lla - predicted_measurement;
    Eigen::MatrixXd H = GetMeasurementJacobian();
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd S = H * ekf->GetCov() * H.transpose() + R;
    Eigen::MatrixXd K = ekf->GetCov() * H.transpose() * S.inverse();
    Eigen::VectorXd update = K * resid;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(update.size(), update.size());

    ekf->GetCov() = (I - K * H) * ekf->GetCov() * (I - K * H).transpose() + K * R * K.transpose();

    m_logger->Log(LogLevel::INFO, "GPS Updater Update");
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;
  msg << time;
  msg << "," << VectorToCommaString(gps_lla);
  msg << "," << t_execution.count();
  msg << std::endl;
  m_data_logger.Log(msg.str());
}
