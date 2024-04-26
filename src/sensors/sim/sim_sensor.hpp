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

#include "infrastructure/sim/truth_engine.hpp"


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

  SimSensor() {}

protected:
  double m_time_error{1e-9};             ///< @brief Time offset error
  double m_time_bias_error{0.0};         ///< @brief Time offset bias
  double m_time_skew_error{0.0};         ///< @brief Time offset error
  bool m_no_errors {false};              ///< @brief Flag to remove measurement errors
  std::shared_ptr<TruthEngine> m_truth;  ///< @brief Truth engine pointer
};

#endif  // SENSORS__SIM__SIM_SENSOR_HPP_
