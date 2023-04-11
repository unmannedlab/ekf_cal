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

#include "sensors/sim/SimCamera.hpp"

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <memory>

#include "infrastructure/sim/TruthEngine.hpp"
#include "utility/sim/SimRNG.hpp"
#include "utility/MathHelper.hpp"

SimCamera::SimCamera(
  Parameters params,
  std::shared_ptr<TruthEngine> truthEngine)
: Camera(params.camParams)
{
  m_tBias = params.tBias;
  m_tSkew = params.tSkew;
  m_tError = std::max(params.tError, 1e-9);
  m_posOffset = params.posOffset;
  m_angOffset = params.angOffset;
  m_truth = truthEngine;
}

std::vector<std::shared_ptr<SimCameraMessage>> SimCamera::generateMessages(double maxTime)
{
  double nMeasurements = maxTime * m_rate / (1 + m_tSkew);
  std::vector<std::shared_ptr<SimCameraMessage>> messages;
  m_logger->log(LogLevel::INFO, "Generating " + std::to_string(nMeasurements) + " measurements");

  /// @todo Next: Iterate over trackers to return measurements
  return messages;
}

void SimCamera::addTracker(std::shared_ptr<SimFeatureTracker> tracker)
{
  m_trackers.push_back(tracker);
}
