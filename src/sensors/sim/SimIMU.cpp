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
#include <random>

#include "infrastructure/sim/TruthEngine.hpp"
#include "utility/sim/SimRNG.hpp"

SimIMU::SimIMU(SimImuParams params, std::shared_ptr<TruthEngine> truthEngine)
: IMU(params.imuParams)
{
  m_tBias = params.tBias;
  m_tError = params.tError;
  m_accBias = params.accBias;
  m_accError = params.accError;
  m_omgBias = params.omgBias;
  m_omgError = params.omgError;
  m_posOffset = params.posOffset;
  m_angOffset = params.angOffset;
  m_truth = truthEngine;
}

std::vector<SimImuMessage> SimIMU::generateMeasurements(double maxTime)
{
  double nMeasurements = maxTime / m_tSkew / m_rate;
  std::vector<SimImuMessage> messages;

  for (unsigned int i = 0; i < nMeasurements; ++i) {
    SimImuMessage simImuMsg;
    double measurementTime = m_tSkew * static_cast<double>(i);
    simImuMsg.time = measurementTime += m_rng.NormRand(m_tBias, m_tError);
    simImuMsg.sensorID = m_id;

    Eigen::Vector3d bodyAcc = m_truth->GetBodyAcceleration();
    Eigen::Vector3d bodyOmg = m_truth->GetBodyAngularRate();

    /// @todo Insert acceleration model here
    /// @todo Add gravity

    simImuMsg.acceleration = Eigen::Vector3d::Zero(3);
    simImuMsg.acceleration[0] += m_rng.NormRand(m_accBias, m_accError);
    simImuMsg.acceleration[1] += m_rng.NormRand(m_accBias, m_accError);
    simImuMsg.acceleration[2] += m_rng.NormRand(m_accBias, m_accError);

    simImuMsg.angularRate = Eigen::Vector3d::Zero(3);
    simImuMsg.angularRate[0] += m_rng.NormRand(m_omgBias, m_omgError);
    simImuMsg.angularRate[1] += m_rng.NormRand(m_omgBias, m_omgError);
    simImuMsg.angularRate[2] += m_rng.NormRand(m_omgBias, m_omgError);

    Eigen::Vector3d accSigmas(m_accError, m_accError, m_accError);
    Eigen::Vector3d omgSigmas(m_omgError, m_omgError, m_omgError);

    simImuMsg.accelerationCovariance = accSigmas.asDiagonal();
    simImuMsg.angularRateCovariance = omgSigmas.asDiagonal();
    messages.push_back(simImuMsg);
  }
  return messages;
}
