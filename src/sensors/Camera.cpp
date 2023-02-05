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

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "ekf/Types.hpp"
#include "infrastructure/Logger.hpp"
#include "sensors/Sensor.hpp"
#include "sensors/Tracker.hpp"
#include "utility/TypeHelper.hpp"

/// @todo add detector/extractor parameters to input
Camera::Camera(Camera::Params cParams, Tracker::Params tParams)
: Sensor(cParams.name), m_tracker(tParams)
{}

// Eigen::VectorXd Camera::predictMeasurement()
// {
//   Eigen::VectorXd predictedMeasurement(m_stateSize);
//   return predictedMeasurement;
// }

// Eigen::MatrixXd Camera::getMeasurementJacobian()
// {
//   Eigen::MatrixXd measurementJacobian(m_stateSize, m_stateSize);
//   return measurementJacobian;
// }

void Camera::callback(double time, cv::Mat & imgIn)
{
  m_logger->log(LogLevel::DEBUG, "Camera callback called at time = " + std::to_string(time));

  unsigned int frameID = generateFrameID();

  FeatureTracks featureTracks;
  m_tracker.track(frameID, imgIn, m_outImg, featureTracks);
  /// @todo Undistort points post track?
  // cv::undistortPoints();
  /// @todo Call a EKF updater method
}

unsigned int Camera::generateFrameID()
{
  static unsigned int FrameID = 0;
  return FrameID++;
}
