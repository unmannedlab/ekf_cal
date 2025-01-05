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

#include "sensors/sim/sim_gps.hpp"

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <memory>
#include <cmath>

#include "ekf/constants.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/sim/sim_gps_message.hpp"
#include "utility/gps_helper.hpp"
#include "utility/sim/sim_rng.hpp"

SimGPS::SimGPS(SimGPS::Parameters params, std::shared_ptr<TruthEngine> truth_engine)
: GPS(params.gps_params), SimSensor(params)
{
  m_lla_error = params.lla_error;
  m_truth = truth_engine;

  // Set true camera values
  Eigen::Vector3d pos_a_in_b;
  if (m_no_errors) {
    pos_a_in_b = params.gps_params.pos_a_in_b;
  } else {
    pos_a_in_b = m_rng.VecNormRand(params.gps_params.pos_a_in_b, params.pos_a_in_b_err);
  }
  m_truth->SetGpsPosition(m_id, pos_a_in_b);
}

std::vector<std::shared_ptr<SimGpsMessage>> SimGPS::GenerateMessages()
{
  std::vector<double> measurement_times = GenerateMeasurementTimes(m_rate);

  m_logger->Log(
    LogLevel::INFO, "Generating " + std::to_string(measurement_times.size()) + " GPS measurements");

  std::vector<std::shared_ptr<SimGpsMessage>> messages;
  for (auto measurement_time : measurement_times) {
    auto sim_gps_msg = std::make_shared<SimGpsMessage>();
    sim_gps_msg->time = measurement_time;
    sim_gps_msg->sensor_id = m_id;
    sim_gps_msg->sensor_type = SensorType::GPS;

    Eigen::Vector3d pos_b_in_l = m_truth->GetBodyPosition(measurement_time);
    Eigen::Quaterniond ang_b_to_l = m_truth->GetBodyAngularPosition(measurement_time);
    Eigen::Vector3d pos_a_in_b = m_truth->GetGpsPosition(m_id);
    Eigen::Vector3d pos_e_in_g = m_truth->GetLocalPosition();
    double ang_l_to_e = m_truth->GetLocalHeading();

    Eigen::Vector3d pos_a_in_l = pos_b_in_l + ang_b_to_l * pos_a_in_b;
    if (!m_no_errors) {
      pos_a_in_l(0) += m_rng.NormRand(0, m_lla_error(0));
      pos_a_in_l(1) += m_rng.NormRand(0, m_lla_error(1));
      pos_a_in_l(2) += m_rng.NormRand(0, m_lla_error(2));
    }

    Eigen::Vector3d antenna_enu = local_to_enu(pos_a_in_l, ang_l_to_e);

    sim_gps_msg->gps_lla = enu_to_lla(antenna_enu, pos_e_in_g);
    sim_gps_msg->pos_covariance = m_lla_error.cwiseProduct(m_lla_error).asDiagonal();

    messages.push_back(sim_gps_msg);
  }
  return messages;
}
