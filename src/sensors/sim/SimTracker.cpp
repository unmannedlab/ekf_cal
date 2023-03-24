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

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

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
    cv::Point3d vec;
    vec.x = m_rng.UniRand(-params.roomSize, params.roomSize);
    vec.y = m_rng.UniRand(-params.roomSize, params.roomSize);
    vec.z = m_rng.UniRand(-params.roomSize / 10, params.roomSize / 10);
    m_featurePoints.push_back(vec);
  }

  double camera_mat[3][3] = {
    {m_focalLength, 0, static_cast<double>(m_imageWidth) / 2.0},
    {0, m_focalLength, static_cast<double>(m_imageHeight) / 2.0},
    {0, 0, 1}
  };

  m_projMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_mat);
}

/// @todo Write visibleKeypoints function
std::vector<cv::KeyPoint> SimTracker::visibleKeypoints(double time)
{
  std::vector<Eigen::Vector3d> keypoints;
  Eigen::Vector3d bodyPos = m_truth->GetBodyPosition(time);
  Eigen::Quaterniond bodyAng = m_truth->GetBodyAngularPosition(time);
  Eigen::Quaterniond camAng = bodyAng * m_angOffset;
  Eigen::Matrix3d camAngEigMat = camAng.toRotationMatrix();

  cv::Mat camAngCvMat(3, 3, cv::DataType<double>::type);
  camAngCvMat.at<double>(0, 0) = camAngEigMat(0, 0);
  camAngCvMat.at<double>(1, 0) = camAngEigMat(1, 0);
  camAngCvMat.at<double>(2, 0) = camAngEigMat(2, 0);

  camAngCvMat.at<double>(0, 1) = camAngEigMat(0, 1);
  camAngCvMat.at<double>(1, 1) = camAngEigMat(1, 1);
  camAngCvMat.at<double>(2, 1) = camAngEigMat(2, 1);

  camAngCvMat.at<double>(0, 2) = camAngEigMat(0, 2);
  camAngCvMat.at<double>(1, 2) = camAngEigMat(1, 2);
  camAngCvMat.at<double>(2, 2) = camAngEigMat(2, 2);

  // Creating Rodrigues rotation matrix
  cv::Mat rVec(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(camAngCvMat, rVec);

  Eigen::Vector3d camPos = bodyPos + bodyAng * m_posOffset;
  cv::Mat T(3, 1, cv::DataType<double>::type);
  T.at<double>(0) = camPos[0];
  T.at<double>(1) = camPos[1];
  T.at<double>(2) = camPos[2];

  // Create zero distortion
  /// @todo grab this from input
  cv::Mat distortion(4, 1, cv::DataType<double>::type);
  distortion.at<double>(0) = 0;
  distortion.at<double>(1) = 0;
  distortion.at<double>(2) = 0;
  distortion.at<double>(3) = 0;

  // Project points
  std::vector<cv::Point2d> projectedPoints;

  /// @todo 2D projection is not correct
  cv::projectPoints(m_featurePoints, rVec, T, m_projMatrix, distortion, projectedPoints);

  // Convert to feature points
  std::vector<cv::KeyPoint> projectedFeatures;
  for (auto & point : projectedPoints) {
    cv::KeyPoint feat;
    feat.pt.x = point.x;
    feat.pt.y = point.y;
    projectedFeatures.push_back(feat);
  }

  return projectedFeatures;
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
    std::vector<cv::KeyPoint> keypoints = visibleKeypoints(measurementTime);
    // keyPoint.class_id = generateFeatureID()
  }
  return messages;
}
