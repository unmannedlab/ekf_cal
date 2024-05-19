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
  GpsInitializationType initialization_type,
  double init_pos_thresh,
  double init_ang_thresh,
  double init_baseline_dist,
  std::string log_file_directory,
  bool data_logging_on,
  double data_log_rate,
  std::shared_ptr<DebugLogger> logger
)
: Updater(gps_id, logger),
  m_initialization_type(initialization_type),
  m_init_pos_thresh(init_pos_thresh),
  m_init_ang_thresh(init_ang_thresh),
  m_init_baseline_dist(init_baseline_dist),
  m_data_logger(log_file_directory, "gps_" + std::to_string(gps_id) + ".csv")
{
  std::stringstream header;
  header <<
    "time,lat,lon,alt,x,y,z,ref_lat,ref_lon,ref_alt,ref_heading,pos_stddev,ang_stddev,is_initialized";
  header << EnumerateHeader("antenna", g_gps_state_size);
  header << EnumerateHeader("gps_cov", g_gps_state_size);
  header << EnumerateHeader("residual", g_gps_state_size);
  header << EnumerateHeader("duration", 1);

  m_data_logger.DefineHeader(header.str());
  m_data_logger.SetLogging(data_logging_on);
  m_data_logger.SetLogRate(data_log_rate);
}

/// @todo(jhartzer): Add option to check max distance of baseline to initialize
void GpsUpdater::AttemptInitialization(
  double time,
  std::shared_ptr<EKF> ekf,
  Eigen::Vector3d gps_lla)
{
  Eigen::Vector3d gps_ecef = lla_to_ecef(gps_lla);

  m_gps_time_vec.push_back(time);
  m_gps_ecef_vec.push_back(gps_ecef);
  m_local_xyz_vec.push_back(ekf->m_state.m_body_state.m_position);

  if (m_gps_time_vec.size() >= 4) {
    Eigen::Vector3d init_ref_ecef = average_vectors(m_gps_ecef_vec);
    Eigen::Vector3d init_ref_lla = ecef_to_lla(init_ref_ecef);

    std::vector<Eigen::Vector3d> gps_states_enu;
    for (auto gps_ecef : m_gps_ecef_vec) {
      gps_states_enu.push_back(ecef_to_enu(gps_ecef, init_ref_lla));
    }

    Eigen::Affine3d transformation;
    bool is_successful = kabsch_2d(
      m_local_xyz_vec,
      gps_states_enu,
      transformation,
      m_pos_stddev,
      m_ang_stddev);

    double max_distance = maximum_distance(gps_states_enu);

    if (((m_initialization_type == GpsInitializationType::BASELINE_DIST) &&
      (max_distance > m_init_baseline_dist)) ||
      ((m_initialization_type == GpsInitializationType::ERROR_THRESHOLD) && is_successful &&
      (m_pos_stddev < m_init_pos_thresh) && m_ang_stddev && (m_ang_stddev < m_init_ang_thresh)))
    {
      Eigen::Vector3d delta_ref_enu = transformation.translation();
      Eigen::Vector3d reference_lla = enu_to_lla(-delta_ref_enu, init_ref_lla);
      double ang_l_to_g = affine_angle(transformation);

      m_logger->Log(LogLevel::INFO, "GPS Updater Initialized");
      ekf->SetGpsReference(reference_lla, ang_l_to_g);
    }
  }
}

Eigen::MatrixXd GpsUpdater::GetMeasurementJacobian(std::shared_ptr<EKF> ekf)
{
  unsigned int state_size = ekf->GetStateSize();
  Eigen::Vector3d pos_a_in_b = ekf->m_state.m_body_state.m_position;
  Eigen::Quaterniond ang_b_to_g = ekf->m_state.m_body_state.m_ang_b_to_g;
  unsigned int gps_state_start = ekf->GetGpsStateStartIndex(m_id);

  Eigen::MatrixXd measurement_jacobian = Eigen::MatrixXd::Zero(3, state_size);
  measurement_jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
  /// @todo(jhartzer): Need to debug this Jacobian
  measurement_jacobian.block<3, 3>(0, 9) =
    -ang_b_to_g.toRotationMatrix() * SkewSymmetric(pos_a_in_b) * quaternion_jacobian(ang_b_to_g);
  measurement_jacobian.block<3, 3>(0, gps_state_start) = ang_b_to_g.toRotationMatrix();
  return measurement_jacobian;
}

void GpsUpdater::UpdateEKF(std::shared_ptr<EKF> ekf, double time, Eigen::Vector3d gps_lla)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  ekf->ProcessModel(time);

  Eigen::Vector3d pos_a_in_g = Eigen::Vector3d::Zero();
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference_lla = Eigen::Vector3d::Zero();
  double ang_l_to_g = ekf->GetReferenceAngle();
  if (!ekf->IsLlaInitialized()) {
    m_logger->Log(LogLevel::INFO, "GPS Updater Attempt Initialization");
    AttemptInitialization(time, ekf, gps_lla);
    if (ekf->IsLlaInitialized()) {
      // Perform single update with compressed measurements
      MultiUpdateEKF(ekf, m_gps_time_vec, m_gps_ecef_vec, m_local_xyz_vec);
    }
  } else {
    reference_lla = ekf->GetReferenceLLA();
    Eigen::Vector3d gps_enu = lla_to_enu(gps_lla, reference_lla);
    pos_a_in_g = enu_to_local(gps_enu, ang_l_to_g);

    Eigen::Vector3d pos_b_in_g = ekf->m_state.m_body_state.m_position;
    Eigen::Quaterniond ang_b_to_g = ekf->m_state.m_body_state.m_ang_b_to_g;
    Eigen::Vector3d pos_a_in_b = ekf->GetGpsState(m_id).pos_a_in_b;
    Eigen::Vector3d pos_a_in_g_hat = pos_b_in_g + ang_b_to_g * pos_a_in_b;
    residual = pos_a_in_g - pos_a_in_g_hat;
    Eigen::MatrixXd jacobian = GetMeasurementJacobian(ekf);
    Eigen::MatrixXd measurement_noise = Eigen::MatrixXd::Identity(3, 3);
    KalmanUpdate(ekf, jacobian, residual, measurement_noise);

    m_logger->Log(LogLevel::INFO, "GPS Updater Update");
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  unsigned int gps_state_start = ekf->GetGpsStateStartIndex(m_id);
  Eigen::VectorXd cov_diag = ekf->m_cov.block(
    gps_state_start, gps_state_start, g_gps_state_size, g_gps_state_size).diagonal();

  // Write outputs
  std::stringstream msg;
  msg << time;
  msg << VectorToCommaString(gps_lla, 12);
  msg << VectorToCommaString(pos_a_in_g);
  msg << VectorToCommaString(reference_lla, 12);
  msg << "," << ang_l_to_g;
  msg << "," << m_pos_stddev;
  msg << "," << m_ang_stddev;
  msg << "," << ekf->IsLlaInitialized();
  msg << VectorToCommaString(ekf->GetGpsState(m_id).pos_a_in_b);
  msg << VectorToCommaString(cov_diag);
  msg << VectorToCommaString(residual);
  msg << "," << t_execution.count();
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
  Eigen::Vector3d reference_lla = ekf->GetReferenceLLA();
  double ang_l_to_g = ekf->GetReferenceAngle();

  for (unsigned int i = 0; i < gps_time_vec.size(); ++i) {
    Eigen::Vector3d gps_enu = ecef_to_enu(gps_ecef_vec[i], reference_lla);
    Eigen::Vector3d gps_local = enu_to_local(gps_enu, ang_l_to_g);

    H.block(3 * i, 0, 3, state_size) = GetMeasurementJacobian(ekf);
    y.segment(3 * i, 3) = gps_local - local_xyz_vec[i];
  }

  CompressMeasurements(H, y);

  // Jacobian is ill-formed if either rows or columns post-compression are size 1
  if (y.size() <= 1) {
    m_logger->Log(LogLevel::INFO, "Compressed MSCKF Jacobian is ill-formed");
    return;
  }

  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(y.rows(), y.rows());

  KalmanUpdate(ekf, H, y, R);
  m_logger->Log(LogLevel::INFO, "GPS Updater Update");
}
