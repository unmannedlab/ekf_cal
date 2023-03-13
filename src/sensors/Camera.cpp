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

#include "sensors/Camera.hpp"

#include <string>
#include <vector>
#include <unordered_map>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "ekf/Types.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "sensors/Sensor.hpp"
#include "sensors/Tracker.hpp"
#include "utility/MathHelper.hpp"
#include "utility/TypeHelper.hpp"


CameraMessage::CameraMessage(cv::Mat & imgIn)
: SensorMessage(), image(imgIn) {}

/// @todo add detector/extractor parameters to input
Camera::Camera(Camera::Params cParams, Tracker::Params tParams)
: Sensor(cParams.name), m_tracker(tParams, m_id)
{
  m_rate = cParams.rate;
  m_posOffset = cParams.posOffset;
  m_angOffset = cParams.angOffset;

  CamState camState;
  camState.position = m_posOffset;
  camState.orientation = m_angOffset;

  Eigen::MatrixXd cov = minBoundVector(cParams.variance, 1e-6).asDiagonal();

  m_ekf->registerCamera(m_id, camState, cov);
}

void Camera::callback(std::shared_ptr<CameraMessage> cameraMessage)
{
  m_logger->log(
    LogLevel::DEBUG, "Camera " + std::to_string(
      cameraMessage->sensorID) + " callback called at time = " +
    std::to_string(cameraMessage->time));

  unsigned int frameID = generateFrameID();

  m_ekf->augmentState(m_id, frameID);

  FeatureTracks featureTracks;
  // m_tracker.track(frameID, cameraMessage->image, m_outImg, featureTracks);
  /// @todo Undistort points post track?
  // cv::undistortPoints();
  /// @todo Call a EKF updater method
}

unsigned int Camera::generateFrameID()
{
  static unsigned int FrameID = 0;
  return FrameID++;
}
