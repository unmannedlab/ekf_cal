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

#include "ekf/sensors/Imu.hpp"

#include <rclcpp/rclcpp.hpp>

#include "ekf/sensors/Sensor.hpp"

Imu::Imu(Imu::Params params)
: Sensor(params.name)
{
  if (params.intrinsic) {
    if (params.baseSensor) {
      RCLCPP_WARN(rclcpp::get_logger("IMU"), "Base IMU should be extrinsic for filter stability");
    }
    m_stateSize = 12U;
  } else {
    m_stateSize = 6U;
  }

  m_baseSensor = params.baseSensor;
  m_intrinsic = params.intrinsic;
  m_rate = params.rate;
  m_posOffset = params.posOffset;
  m_quatOffset = params.quatOffset;
  m_accBias = params.accBias;
  m_omgBias = params.omgBias;
  m_accBiasStability = params.accBiasStability;
  m_omgBiasStability = params.omgBiasStability;
}


double Imu::GetAccBiasStability()
{
  return m_accBiasStability;
}

double Imu::GetOmgBiasStability()
{
  return m_omgBiasStability;
}

bool Imu::IsBaseSensor()
{
  return m_baseSensor;
}
bool Imu::IsIntrinsic()
{
  return m_intrinsic;
}

/// @todo Update size and function for different types of IMUs
Eigen::VectorXd Imu::PredictMeasurement()
{
  Eigen::VectorXd predictedMeasurement(m_stateSize);
  return predictedMeasurement;
}

Eigen::MatrixXd Imu::GetMeasurementJacobian()
{
  Eigen::MatrixXd measurementJacobian(m_stateSize, m_stateSize);
  return measurementJacobian;
}

Eigen::MatrixXd Imu::GetMeasurementCovariance()
{
  Eigen::MatrixXd measurementCovariance(m_stateSize, m_stateSize);
  return measurementCovariance;
}
