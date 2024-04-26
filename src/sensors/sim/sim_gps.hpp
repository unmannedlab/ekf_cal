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


#ifndef SENSORS__SIM__SIM_GPS_HPP_
#define SENSORS__SIM__SIM_GPS_HPP_

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>
#include <vector>

#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/gps.hpp"
#include "sensors/sensor.hpp"
#include "sensors/sim/sim_gps_message.hpp"
#include "sensors/sim/sim_sensor.hpp"
#include "utility/sim/sim_rng.hpp"

///
/// @class SimGPS
/// @brief Simulated GPS Sensor Class
///
class SimGPS : public GPS, public SimSensor
{
public:
  ///
  /// @brief Sim GPS initialization parameters structure
  ///
  typedef struct Parameters : public SimSensor::Parameters
  {
    Eigen::Vector3d lla_error {1e-9, 1e-9, 1e-9};       ///< @brief LLA errors
    Eigen::Vector3d pos_a_in_b_err {1e-9, 1e-9, 1e-9};  ///< @brief Position error of GPS antenna
    Eigen::Vector3d pos_l_in_g_err {1e-9, 1e-9, 1e-9};  ///< @brief Error in local frame position
    double ang_l_to_g_err {1e-9};                       ///< @brief Error in local to global angle
    double ang_l_to_g {0.0};                            ///< @brief Local orientation in global
    GPS::Parameters gps_params;                         ///< @brief GPS sensor parameters
  } Parameters;

  ///
  /// @brief Simulation GPS constructor
  /// @param params Simulation GPS parameters
  /// @param truth_engine Truth engine
  ///
  SimGPS(SimGPS::Parameters params, std::shared_ptr<TruthEngine> truth_engine);

  ///
  /// @brief Generate simulated GPS messages
  /// @param rng Random number generator
  /// @param max_time Maximum time of generated messages
  /// @return Generated GPS messages
  ///
  std::vector<std::shared_ptr<SimGpsMessage>> GenerateMessages(SimRNG rng, double max_time);

private:
  Eigen::Vector3d m_lla_error{1e-9, 1e-9, 1e-9};
};


#endif  // SENSORS__SIM__SIM_GPS_HPP_
