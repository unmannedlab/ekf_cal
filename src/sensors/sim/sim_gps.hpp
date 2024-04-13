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
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/gps.hpp"
#include "sensors/sensor.hpp"
#include "sensors/sim/sim_gps_message.hpp"
#include "utility/sim/sim_rng.hpp"

///
/// @class SimGPS
/// @brief Simulated GPS Sensor Class
///
class SimGPS : public GPS
{
public:
  ///
  /// @brief Sim GPS initialization parameters structure
  ///
  typedef struct Parameters
  {
    double time_bias {0.0};                        ///< @brief Time offset bias
    double time_skew {0.0};                        ///< @brief Time offset error
    double time_error {1e-9};                      ///< @brief Time offset error
    Eigen::Vector3d pos_a_in_b {0.0, 0.0, 0.0};    ///< @brief Antenna position in body frame
    Eigen::Vector3d gps_error {1e-9, 1e-9, 1e-9};  ///< @brief LLA errors
    Eigen::Vector3d pos_l_in_g {0.0, 0.0, 0.0};    ///< @brief Local in global LLA
    double ang_l_to_g {0.0};                       ///< @brief Local orientation in global
    GPS::Parameters gps_params;                    ///< @brief GPS sensor parameters
    bool no_errors {false};                        ///< @brief Perfect measurements flag
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
  double m_time_bias{0.0};
  double m_time_skew{0.0};
  double m_time_error{1e-9};
  Eigen::Vector3d m_pos_a_in_b {0.0, 0.0, 0.0};
  Eigen::Vector3d m_gps_error{1e-9, 1e-9, 1e-9};
  Eigen::Vector3d m_pos_l_in_g {0.0, 0.0, 0.0};
  double m_ang_l_to_g {0.0};
  std::shared_ptr<TruthEngine> m_truth;
  bool m_no_errors {false};
};


#endif  // SENSORS__SIM__SIM_GPS_HPP_
