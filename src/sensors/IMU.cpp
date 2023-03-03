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

#include "sensors/IMU.hpp"

#include <eigen3/Eigen/Eigen>

#include "ekf/Types.hpp"
#include "infrastructure/Logger.hpp"
#include "sensors/Sensor.hpp"
#include "utility/MathHelper.hpp"
#include "utility/TypeHelper.hpp"


IMU::IMU(IMU::Params params)
: Sensor(params.name), m_imuUpdater(m_id, params.accBiasStability, params.omgBiasStability)
{
  m_isBaseSensor = params.baseSensor;
  m_isIntrinsic = params.intrinsic;
  m_rate = params.rate;

  ImuState imuState;
  imuState.position = params.posOffset;
  imuState.orientation = params.angOffset;
  imuState.accBias = params.accBias;
  imuState.omgBias = params.omgBias;

  Eigen::MatrixXd cov = minBoundVector(params.variance, 1e-6).asDiagonal();

  m_ekf->registerIMU(m_id, imuState, cov);
}

void IMU::callback(ImuMessage imuMessage)
{
  m_logger->log(
    LogLevel::DEBUG,
    "IMU \"" + m_name + "\" callback at time " + std::to_string(imuMessage.time));
  m_imuUpdater.updateEKF(
    imuMessage.time,
    imuMessage.acceleration,
    imuMessage.accelerationCovariance,
    imuMessage.angularRate,
    imuMessage.angularRateCovariance);
  m_logger->log(LogLevel::DEBUG, "IMU \"" + m_name + "\" callback complete");
}
