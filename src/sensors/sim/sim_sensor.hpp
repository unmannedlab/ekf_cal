// Copyright 2022 Jacob Hartzer
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


#ifndef SENSORS__SIM__SIM_SENSOR_HPP_
#define SENSORS__SIM__SIM_SENSOR_HPP_

#include <memory>
#include <vector>

#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/sensor.hpp"
#include "utility/sim/sim_rng.hpp"

///
/// @brief SimSensor namespace
///
class SimSensor
{
public:
  ///
  /// @brief Sim IMU initialization parameters structure
  ///
  typedef struct Parameters
  {
    bool no_errors {false};        ///< @brief Perfect measurements flag
    double time_error {0.0};       ///< @brief Time offset error
    double time_bias_error {0.0};  ///< @brief Time offset bias
    double time_skew_error {0.0};  ///< @brief Time offset error
  } Parameters;

  ///
  /// @brief SimSensor constructor
  ///
  SimSensor() {}

  ///
  /// @brief Generate list of true measurement times
  /// @param rng Random number generator
  /// @param m_rate Sensor rate
  /// @return List of sensor measurement times
  ///
  std::vector<double> GenerateMeasurementTimes(SimRNG rng, double m_rate);

  ///
  /// @brief Apply errors, if necessary, to sensor measurement time
  /// @param rng Random number generator
  /// @param true_time True measurement time
  /// @return Time with error
  ///
  double ApplyTimeError(SimRNG rng, double true_time);

protected:
  double m_time_error {1e-9};            ///< @brief Time offset error
  double m_time_bias_error{0.0};         ///< @brief Time offset bias
  double m_time_skew_error{0.0};         ///< @brief Time offset error
  bool m_no_errors {false};              ///< @brief Flag to remove measurement errors
  std::shared_ptr<TruthEngine> m_truth;  ///< @brief Truth engine pointer
};

#endif  // SENSORS__SIM__SIM_SENSOR_HPP_
