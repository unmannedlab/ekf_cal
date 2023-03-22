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

#include "sensors/sim/SimIMU.hpp"

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <memory>

#include "infrastructure/sim/TruthEngine.hpp"
#include "utility/sim/SimRNG.hpp"
#include "utility/MathHelper.hpp"

SimIMU::SimIMU(SimImuParams params, std::shared_ptr<TruthEngine> truthEngine)
: IMU(params.imuParams)
{
  m_tBias = params.tBias;
  m_tSkew = params.tSkew;
  m_tError = std::max(params.tError, 1e-9);
  m_accBias = params.accBias;
  m_accError = minBoundVector(params.accError, 1e-9);
  m_omgBias = params.omgBias;
  m_omgError = minBoundVector(params.omgError, 1e-9);
  m_posOffset = params.posOffset;
  m_angOffset = params.angOffset;
  m_rate = params.imuParams.rate;
  m_truth = truthEngine;
}

std::vector<std::shared_ptr<SimImuMessage>> SimIMU::generateMessages(double maxTime)
{
  double nMeasurements = maxTime * m_rate / (1 + m_tSkew);
  std::vector<std::shared_ptr<SimImuMessage>> messages;
  m_logger->log(LogLevel::INFO, "Generating " + std::to_string(nMeasurements) + " measurements");

  for (unsigned int i = 0; i < nMeasurements; ++i) {
    auto simImuMsg = std::make_shared<SimImuMessage>();
    double measurementTime = (1.0 + m_tSkew) / m_rate * static_cast<double>(i);
    simImuMsg->time = measurementTime + m_rng.NormRand(m_tBias, m_tError);
    simImuMsg->sensorID = m_id;
    simImuMsg->sensorType = SensorType::IMU;

    Eigen::Vector3d bodyAcc = m_truth->GetBodyAcceleration(measurementTime);
    Eigen::Quaterniond bodyAngPos = m_truth->GetBodyAngularPosition(measurementTime);
    Eigen::Vector3d bodyAngVel = m_truth->GetBodyAngularRate(measurementTime);
    Eigen::Vector3d bodyAngAcc = m_truth->GetBodyAngularAcceleration(measurementTime);

    // Transform acceleration to IMU location
    Eigen::Vector3d imuAcc = bodyAcc +
      bodyAngAcc.cross(m_posOffset) +
      bodyAngVel.cross((bodyAngVel.cross(m_posOffset)));

    // Rotate measurements in place
    Eigen::Vector3d imuAccRot = m_angOffset * imuAcc;
    Eigen::Vector3d imuOmgRot = m_angOffset * bodyAngVel;

    /// @todo move gravity to constants file
    imuAccRot += bodyAngPos * m_angOffset * Eigen::Vector3d(0, 0, 9.80665);

    simImuMsg->acceleration = imuAccRot;
    simImuMsg->acceleration[0] += m_rng.NormRand(m_accBias[0], m_accError[0]);
    simImuMsg->acceleration[1] += m_rng.NormRand(m_accBias[1], m_accError[1]);
    simImuMsg->acceleration[2] += m_rng.NormRand(m_accBias[2], m_accError[2]);

    simImuMsg->angularRate = imuOmgRot;
    simImuMsg->angularRate[0] += m_rng.NormRand(m_omgBias[0], m_omgError[0]);
    simImuMsg->angularRate[1] += m_rng.NormRand(m_omgBias[1], m_omgError[1]);
    simImuMsg->angularRate[2] += m_rng.NormRand(m_omgBias[2], m_omgError[2]);

    Eigen::Vector3d accSigmas(m_accError[0], m_accError[1], m_accError[2]);
    Eigen::Vector3d omgSigmas(m_omgError[0], m_omgError[1], m_omgError[2]);

    simImuMsg->accelerationCovariance = accSigmas.asDiagonal();
    simImuMsg->angularRateCovariance = omgSigmas.asDiagonal();
    messages.push_back(simImuMsg);
  }
  return messages;
}
