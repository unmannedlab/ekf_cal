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

SimGPS::SimGPS(SimGPS::Parameters params, std::shared_ptr<TruthEngine> truthEngine)
: GPS(params.gps_params)
{
  m_time_bias = params.time_bias;
  m_time_skew = params.time_skew;
  m_time_error = std::max(params.time_error, 1e-9);
  m_pos_a_in_b = params.pos_a_in_b;
  m_gps_error = params.gps_error;
  m_pos_l_in_g = params.pos_l_in_g;
  m_ang_l_to_g = params.ang_l_to_g;
  m_truth = truthEngine;
}

std::vector<std::shared_ptr<SimGpsMessage>> SimGPS::GenerateMessages(double max_time)
{
  unsigned int num_measurements =
    static_cast<int>(std::floor(max_time * m_rate / (1 + m_time_skew)));

  m_logger->Log(
    LogLevel::INFO, "Generating " + std::to_string(num_measurements) + " GPS measurements");

  double time_init = m_no_errors ? 0 : m_rng.UniRand(0.0, 1.0 / m_rate);

  std::vector<std::shared_ptr<SimGpsMessage>> messages;
  for (unsigned int i = 0; i < num_measurements; ++i) {
    auto sim_gps_msg = std::make_shared<SimGpsMessage>();
    double measurement_time = (1.0 + m_time_skew) / m_rate * static_cast<double>(i) + time_init;
    sim_gps_msg->m_time = measurement_time;
    sim_gps_msg->m_sensor_id = m_id;
    sim_gps_msg->m_sensor_type = SensorType::GPS;

    /// @todo(jhartzer): Implement GPS error model
    Eigen::Vector3d pos_b_in_l = m_truth->GetBodyPosition(measurement_time);
    Eigen::Quaterniond ang_b_to_l = m_truth->GetBodyAngularPosition(measurement_time);

    Eigen::Vector3d pos_a_in_l = pos_b_in_l + ang_b_to_l * m_pos_a_in_b;
    if (!m_no_errors) {
      pos_a_in_l(0) += m_rng.NormRand(0, m_gps_error(0));
      pos_a_in_l(1) += m_rng.NormRand(0, m_gps_error(1));
      pos_a_in_l(2) += m_rng.NormRand(0, m_gps_error(2));
    }
    Eigen::Vector3d pos_a_lla = enu_to_lla(pos_a_in_l, m_pos_l_in_g);

    sim_gps_msg->m_latitude = pos_a_lla(0);
    sim_gps_msg->m_longitude = pos_a_lla(1);
    sim_gps_msg->m_altitude = pos_a_lla(2);

    messages.push_back(sim_gps_msg);
  }
  return messages;
}
