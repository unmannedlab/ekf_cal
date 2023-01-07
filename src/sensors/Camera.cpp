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

#include "sensors/Sensor.hpp"
#include "utility/TypeHelper.hpp"

Camera::Camera(Camera::Params params)
: Sensor(params.name) {}

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
  std::vector<cv::KeyPoint> keyPoints;
  m_fastDetector->detect(imgIn, keyPoints);
  cv::drawKeypoints(imgIn, keyPoints, m_outImg);
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
