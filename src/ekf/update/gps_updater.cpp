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
bool GpsUpdater::AttemptInitialization(
  double time,
  Eigen::Vector3d gps_lla,
  Eigen::Vector3d pos_xyz)
{
  Eigen::Vector3d gps_ecef = lla_to_ecef(gps_lla);

  m_gps_time_vec.push_back(time);
  m_gps_ecef_vec.push_back(gps_ecef);
  m_local_xyz_vec.push_back(pos_xyz);

  if (m_gps_time_vec.size() >= 4) {
    // Check eigenvalue of SVD from Kabsch

    Eigen::Vector3d init_ref_ecef = average_vectors(m_gps_ecef_vec);
    Eigen::Vector3d init_ref_lla = ecef_to_lla(init_ref_ecef);

    std::vector<Eigen::Vector3d> gps_states_enu;
    for (auto gps_ecef : m_gps_ecef_vec) {
      gps_states_enu.push_back(ecef_to_enu(gps_ecef, init_ref_lla));
    }

    Eigen::Affine3d transformation;
    Eigen::Vector2d singular_values;
    double residual_rms;
    bool is_successful = kabsch_2d(
      m_local_xyz_vec,
      gps_states_enu,
      transformation,
      singular_values,
      residual_rms);

    /// @todo(jhartzer): Get quality limit from input
    if (is_successful && (singular_values.maxCoeff() > 2.0)) {
      Eigen::Vector3d delta_ref_enu = transformation.translation();
      m_reference_lla = enu_to_lla(delta_ref_enu, init_ref_lla);
      m_ang_l_to_g = affine_angle(transformation);

      m_logger->Log(LogLevel::INFO, "GPS Updater Initialized");
      return true;
    }
  }
  return false;
}

Eigen::MatrixXd GpsUpdater::GetMeasurementJacobian(unsigned int state_size)
{
  Eigen::MatrixXd measurement_jacobian = Eigen::MatrixXd::Zero(3, state_size);
  return measurement_jacobian;
}

void GpsUpdater::UpdateEKF(std::shared_ptr<EKF> ekf, double time, Eigen::Vector3d gps_lla)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  if (!m_is_lla_initialized) {
    m_logger->Log(LogLevel::INFO, "GPS Updater Attempt Initialization");
    m_is_lla_initialized =
      AttemptInitialization(time, gps_lla, ekf->GetState().m_body_state.m_position);
    if (m_is_lla_initialized) {
      // Perform single update with compressed measurements
      MultiUpdateEKF(ekf, m_gps_time_vec, m_gps_ecef_vec, m_local_xyz_vec);
    }
  } else {
    ekf->ProcessModel(time);

    Eigen::Vector3d gps_enu = lla_to_enu(gps_lla, m_reference_lla);
    Eigen::Vector3d gps_local = enu_to_local(gps_enu, m_ang_l_to_g);

    Eigen::Vector3d y = gps_local - ekf->GetBodyState().m_position;
    Eigen::MatrixXd H = GetMeasurementJacobian(ekf->GetStateSize());
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
    KalmanUpdate(ekf, H, y, R);

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

void GpsUpdater::MultiUpdateEKF(
  std::shared_ptr<EKF> ekf,
  std::vector<double> gps_time_vec,
  std::vector<Eigen::Vector3d> gps_ecef_vec,
  std::vector<Eigen::Vector3d> local_xyz_vec)
{
  unsigned int measurement_size = 3 * gps_time_vec.size();
  unsigned int state_size = ekf->GetStateSize();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(measurement_size, state_size);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(measurement_size);

  for (unsigned int i = 0; i < gps_time_vec.size(); ++i) {
    Eigen::Vector3d gps_enu = ecef_to_enu(gps_ecef_vec[i], m_reference_lla);
    Eigen::Vector3d gps_local = enu_to_local(gps_enu, m_ang_l_to_g);

    H.block(3 * i, 0, 3, state_size) = GetMeasurementJacobian(state_size);
    y.segment(3 * i, 3) = gps_local - local_xyz_vec[i];
  }

  CompressMeasurements(H, y);

  // Jacobian is ill-formed if either rows or columns post-compression are size 1
  if (y.size() == 1) {
    m_logger->Log(LogLevel::INFO, "Compressed MSCKF Jacobian is ill-formed");
    return;
  }

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(y.rows(), y.rows());

  KalmanUpdate(ekf, H, y, R);
  m_logger->Log(LogLevel::INFO, "GPS Updater Update");
}
