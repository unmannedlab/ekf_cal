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

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "sensors/Sensor.hpp"
#include "utility/TypeHelper.hpp"

/// @todo add detector/extractor parameters to input
Camera::Camera(Camera::Params params)
: Sensor(params.name)
{
  m_featureDetector = cv::ORB::create(
    500, 1.2f, 8, 31,
    0, 2, cv::ORB::HARRIS_SCORE, 31, 15);
  m_descriptorExtractor = cv::ORB::create();
  m_descriptorMatcher = cv::FlannBasedMatcher::create();
}

Eigen::VectorXd Camera::PredictMeasurement()
{
  Eigen::VectorXd predictedMeasurement(m_stateSize);
  return predictedMeasurement;
}

Eigen::MatrixXd Camera::GetMeasurementJacobian()
{
  Eigen::MatrixXd measurementJacobian(m_stateSize, m_stateSize);
  return measurementJacobian;
}

Eigen::VectorXd Camera::GetState()
{
  Eigen::AngleAxisd angAxis{m_angOffset};
  Eigen::Vector3d rotVec = angAxis.axis() * angAxis.angle();
  Eigen::VectorXd stateVec(m_posOffset.size() + rotVec.size());
  stateVec.segment<3>(0) = m_posOffset;
  stateVec.segment<3>(3) = rotVec;

  return stateVec;
}

void Camera::Callback(double time, cv::Mat & imgIn)
{
  m_logger->log(LogLevel::INFO, "Camera callback called at time = " + std::to_string(time));

  m_featureDetector->detect(imgIn, m_currKeyPoints);
  m_descriptorExtractor->compute(imgIn, m_currKeyPoints, m_currDescriptors);
  m_currDescriptors.convertTo(m_currDescriptors, CV_32F);
  cv::drawKeypoints(imgIn, m_currKeyPoints, m_outImg);

  if (m_prevDescriptors.rows > 0 && m_currDescriptors.rows > 0) {
    std::vector<cv::DMatch> matches;
    m_descriptorMatcher->match(m_prevDescriptors, m_currDescriptors, matches);

    // Use only "good" matches (i.e. whose distance is less than 3*min_dist )
    double max_dist = 0;
    double min_dist = 100;
    for (int i = 0; i < matches.size(); ++i) {
      double dist = matches[i].distance;
      if (dist < min_dist) {min_dist = dist;}
      if (dist > max_dist) {max_dist = dist;}
    }

    for (unsigned int i = 0; i < matches.size(); ++i) {
      if (matches[i].distance < 3 * min_dist) {
        cv::Point2d point_old = m_prevKeyPoints[matches[i].queryIdx].pt;
        cv::Point2d point_new = m_currKeyPoints[matches[i].trainIdx].pt;
        cv::line(m_outImg, point_old, point_new, cv::Scalar(0, 255, 0), 2, 8, 0);
      }
    }
  }

  m_prevKeyPoints = m_currKeyPoints;
  m_prevDescriptors = m_currDescriptors;
}

void Camera::SetState()
{
  Eigen::VectorXd state = m_ekf->GetState();

  m_posOffset = state.segment<3>(m_stateStartIndex);
  Eigen::Vector3d rotVec = state.segment<3>(m_stateStartIndex + 3);

  double angle = rotVec.norm();
  Eigen::Vector3d axis = rotVec / angle;

  m_angOffset = Eigen::AngleAxisd(angle, axis);
}
