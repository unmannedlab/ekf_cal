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

#include "sensors/sim/SimTracker.hpp"


SimTracker::SimTracker(SimTrackerParams params, std::shared_ptr<TruthEngine> truthEngine)
: Tracker(params.trackerParams, params.cameraID)
{
  m_rate = params.rate;
  m_tBias = params.tBias;
  m_tSkew = params.tSkew;
  m_tError = params.tError;
  m_uvError = params.uvError;
  m_posOffset = params.posOffset;
  m_angOffset = params.angOffset;
  m_featureCount = params.featureCount;
  m_truth = truthEngine;

  for (unsigned int i = 0; i < m_featureCount; ++i) {
    Eigen::Vector3d vec;
    vec[0] = m_rng.UniRand(-params.roomSize, params.roomSize);
    vec[1] = m_rng.UniRand(-params.roomSize, params.roomSize);
    vec[2] = m_rng.UniRand(-params.roomSize, params.roomSize);
    m_featurePoints.push_back(vec);
  }
}

/// @todo Write visibleKeypoints function
std::vector<Eigen::Vector3d> SimTracker::visibleKeypoints(double maxTime)
{
  std::vector<Eigen::Vector3d> keypoints;
  return keypoints;
}

/// @todo Write generateMessages function
std::vector<FeatureTracks> SimTracker::generateMessages(double maxTime)
{
  double nMeasurements = maxTime * m_rate / (1 + m_tSkew);
  std::vector<FeatureTracks> messages;
  m_logger->log(LogLevel::INFO, "Generating " + std::to_string(nMeasurements) + " measurements");
  unsigned int frameID{0};
  for (unsigned int i = 0; i < nMeasurements; ++i) {
    double measurementTime = (1.0 + m_tSkew) / m_rate * static_cast<double>(i);
    ++frameID;
    // keyPoint.class_id = generateFeatureID()
  }
  return messages;
}
