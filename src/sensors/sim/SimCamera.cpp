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

std::vector<double> SimCamera::generateMessageTimes(double maxTime)
{
  std::vector<double> messageTimes;
  double nMeasurements = maxTime * m_rate / (1 + m_tSkew);
  for (unsigned int i = 0; i < nMeasurements; ++i) {
    double measurementTime = (1.0 + m_tSkew) / m_rate * static_cast<double>(i);
    messageTimes.push_back(measurementTime + m_rng.NormRand(m_tBias, m_tError));
  }
  return messageTimes;
}

std::vector<std::shared_ptr<SimCameraMessage>> SimCamera::generateMessages(double maxTime)
{
  std::vector<std::shared_ptr<SimCameraMessage>> messages;
  std::vector<double> messageTimes = generateMessageTimes(maxTime);

  for (auto const & trkIter : m_trackers) {
    auto trkMsgs = m_trackers[trkIter.first]->generateMessages(messageTimes, m_id);
    cv::Mat blankImg;
    for (auto trkMsg : trkMsgs) {
      auto camMsg = std::make_shared<SimCameraMessage>(blankImg);

      camMsg->featureTrackMessage = trkMsg;
      camMsg->sensorID = m_id;
      camMsg->time = trkMsg->time;
      camMsg->sensorType = SensorType::Camera;

      messages.push_back(camMsg);
    }
  }
  return messages;
}

void SimCamera::addTracker(std::shared_ptr<SimFeatureTracker> tracker)
{
  m_trackers[tracker->getID()] = tracker;
}

void SimCamera::callback(std::shared_ptr<SimCameraMessage> simCameraMessage)
{
  m_trackers[simCameraMessage->featureTrackMessage->trackerID]->callback(
    simCameraMessage->time, simCameraMessage->sensorID, simCameraMessage->featureTrackMessage);
}
