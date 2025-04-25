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
    SimRNG rng;                    ///< @brief Random number generator
  } Parameters;

  ///
  /// @brief SimSensor constructor
  ///
  explicit SimSensor(Parameters params);

  ///
  /// @brief Generate list of true measurement times
  /// @param m_rate Sensor rate
  /// @return List of sensor measurement times
  ///
  std::vector<double> GenerateMeasurementTimes(double m_rate) const;

  ///
  /// @brief Apply errors, if necessary, to sensor measurement time
  /// @param true_time True measurement time
  /// @return Time with error
  ///
  double ApplyTimeError(double true_time) const;

protected:
  bool m_no_errors {false};              ///< @brief Flag to remove measurement errors
  double m_time_error {0.0};            ///< @brief Time offset error
  SimRNG m_rng;                          ///< @brief Random number generator
  std::shared_ptr<TruthEngine> m_truth;  ///< @brief Truth engine pointer
};

#endif  // SENSORS__SIM__SIM_SENSOR_HPP_
