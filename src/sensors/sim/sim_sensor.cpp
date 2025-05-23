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

#include <vector>

#include "sensors/sim/sim_sensor.hpp"
#include "utility/sim/sim_rng.hpp"


SimSensor::SimSensor(Parameters params)
: m_no_errors(params.no_errors)
{
  if (m_no_errors) {
    m_time_error = 0.0;
  } else {
    m_time_error = params.time_error;
  }
}

std::vector<double> SimSensor::GenerateMeasurementTimes(double m_rate) const
{
  auto num_measurements = static_cast<unsigned int>(std::floor(m_truth->m_max_time * m_rate));
  double time_init = m_no_errors ? 0 : SimRNG::UniRand(0.0, 1.0 / m_rate);

  std::vector<double> message_times;
  for (unsigned int i = 0; i < num_measurements; ++i) {
    message_times.push_back(ApplyTimeError(static_cast<double>(i) / m_rate + time_init));
  }
  return message_times;
}

double SimSensor::ApplyTimeError(double true_time) const
{
  double time_err;
  if (m_no_errors) {
    time_err = true_time;
  } else {
    time_err = SimRNG::NormRand(true_time, m_time_error);
  }
  return time_err;
}
