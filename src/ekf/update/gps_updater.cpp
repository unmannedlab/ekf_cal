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
  bool is_extrinsic,
  std::string log_file_directory,
  double data_log_rate,
  std::shared_ptr<DebugLogger> logger
)
: Updater(gps_id, logger),
  m_is_extrinsic(is_extrinsic),
  m_data_logger(log_file_directory, "gps_" + std::to_string(gps_id) + ".csv")
{
  std::stringstream header;
  header << "time,lat,lon,alt,x,y,z,ref_lat,ref_lon,ref_alt,ref_heading,is_initialized";
  if (m_is_extrinsic) {
    header << EnumerateHeader("ant_pos", g_gps_extrinsic_state_size);
    header << EnumerateHeader("gps_cov", g_gps_extrinsic_state_size);
  }
  header << EnumerateHeader("residual", g_gps_extrinsic_state_size);
  header << EnumerateHeader("duration", 1);

  m_data_logger.DefineHeader(header.str());
  if (data_log_rate) {m_data_logger.EnableLogging();}
  m_data_logger.SetLogRate(data_log_rate);
}

Eigen::MatrixXd GpsUpdater::GetMeasurementJacobian(std::shared_ptr<EKF> ekf)
{
  Eigen::Quaterniond ang_b_to_l = ekf->m_state.body_state.ang_b_to_l;

  Eigen::MatrixXd measurement_jacobian = Eigen::MatrixXd::Zero(3, ekf->GetStateSize());
  measurement_jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
  measurement_jacobian.block<3, 3>(0, 6) = -ang_b_to_l.toRotationMatrix() *
    SkewSymmetric(m_pos_a_in_b) * quaternion_jacobian(ang_b_to_l);

  if (m_is_extrinsic) {
    unsigned int gps_index = ekf->m_state.gps_states[m_id].index;
    measurement_jacobian.block<3, 3>(0, gps_index) = ang_b_to_l.toRotationMatrix();
  }
  return measurement_jacobian;
}

void GpsUpdater::UpdateEKF(
  std::shared_ptr<EKF> ekf, double time, Eigen::Vector3d gps_lla, Eigen::MatrixXd pos_covariance)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  ekf->PredictModel(time);

  if (!ekf->GetUseFirstEstimateJacobian() || m_is_first_estimate) {
    m_pos_a_in_b = ekf->m_state.gps_states[m_id].pos_a_in_b;
    m_is_first_estimate = false;
  }

  Eigen::Vector3d pos_a_in_l = Eigen::Vector3d::Zero();
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  Eigen::Vector3d pos_e_in_g = Eigen::Vector3d::Zero();
  double ang_l_to_e = ekf->GetReferenceAngle();
  if (!ekf->IsLlaInitialized()) {
    m_logger->Log(LogLevel::INFO, "GPS Updater Attempt Initialization");
    ekf->AttemptGpsInitialization(time, gps_lla);
    if (ekf->IsLlaInitialized()) {
      // Perform single update with compressed measurements
      MultiUpdateEKF(ekf);
    }
  } else {
    pos_e_in_g = ekf->GetReferenceLLA();
    Eigen::Vector3d gps_enu = lla_to_enu(gps_lla, pos_e_in_g);
    pos_a_in_l = enu_to_local(gps_enu, ang_l_to_e);

    Eigen::Vector3d pos_b_in_l = ekf->m_state.body_state.pos_b_in_l;
    Eigen::Quaterniond ang_b_to_l = ekf->m_state.body_state.ang_b_to_l;

    Eigen::Vector3d pos_a_in_l_hat = pos_b_in_l + ang_b_to_l * m_pos_a_in_b;
    residual = pos_a_in_l - pos_a_in_l_hat;
    Eigen::MatrixXd jacobian = GetMeasurementJacobian(ekf);
    KalmanUpdate(ekf, jacobian, residual, pos_covariance);

    m_logger->Log(LogLevel::INFO, "GPS Updater Update");
  }
  auto t_end = std::chrono::high_resolution_clock::now();
  auto t_execution = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

  // Write outputs
  std::stringstream msg;
  msg << time;
  msg << VectorToCommaString(gps_lla, 12);
  msg << VectorToCommaString(pos_a_in_l);
  msg << VectorToCommaString(pos_e_in_g, 12);
  msg << "," << ang_l_to_e;
  msg << "," << ekf->IsLlaInitialized();
  if (m_is_extrinsic) {
    msg << VectorToCommaString(ekf->m_state.gps_states[m_id].pos_a_in_b);
    unsigned int gps_index = ekf->m_state.gps_states[m_id].index;
    Eigen::VectorXd cov_diag = ekf->m_cov.block(
      gps_index, gps_index, g_gps_extrinsic_state_size,
      g_gps_extrinsic_state_size).diagonal();
    if (ekf->GetUseRootCovariance()) {
      cov_diag = cov_diag.cwiseProduct(cov_diag);
    }
    msg << VectorToCommaString(cov_diag);
  }
  msg << VectorToCommaString(residual);
  msg << "," << t_execution.count();
  m_data_logger.Log(msg.str());
}

void GpsUpdater::MultiUpdateEKF(std::shared_ptr<EKF> ekf)
{
  std::vector<double> gps_time_vec = ekf->GetGpsTimeVector();
  std::vector<Eigen::Vector3d> gps_ecef_vec = ekf->GetGpsEcefVector();
  std::vector<Eigen::Vector3d> local_xyz_vec = ekf->GetGpsXyzVector();

  unsigned int measurement_size = 3 * gps_time_vec.size();
  unsigned int state_size = ekf->GetStateSize();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(measurement_size, state_size);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(measurement_size);
  Eigen::Vector3d pos_e_in_g = ekf->GetReferenceLLA();
  double ang_l_to_e = ekf->GetReferenceAngle();

  for (unsigned int i = 0; i < gps_time_vec.size(); ++i) {
    Eigen::Vector3d gps_enu = ecef_to_enu(gps_ecef_vec[i], pos_e_in_g);
    Eigen::Vector3d gps_local = enu_to_local(gps_enu, ang_l_to_e);

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
