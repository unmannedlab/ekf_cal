// Copyright 2023 Jacob Hartzer
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

#include "sensors/Tracker.hpp"

#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "ekf/Types.hpp"
#include "ekf/EKF.hpp"

/// @todo add detector/extractor parameters to input
Tracker::Tracker(Tracker::Params params, unsigned int cameraID)
: Sensor(params.name), m_msckfUpdater(cameraID)
{
  m_featureDetector = initFeatureDetector(params.detector, params.threshold);
  m_descriptorExtractor = initDescriptorExtractor(params.descriptor, params.threshold);
  m_descriptorMatcher = initDescriptorMatcher(params.matcher);
  m_cameraID = cameraID;
}

/// @todo Check what parameters are used by open_vins
cv::Ptr<cv::FeatureDetector> Tracker::initFeatureDetector(
  FeatureDetectorEnum detector,
  double threshold)
{
  cv::Ptr<cv::FeatureDetector> featureDetector;
  switch (detector) {
    case FeatureDetectorEnum::BRISK:
      featureDetector = cv::BRISK::create(threshold, 3, 1.0);
      break;
    case FeatureDetectorEnum::FAST:
      featureDetector = cv::FastFeatureDetector::create(
        threshold, true,
        cv::FastFeatureDetector::TYPE_9_16);
      break;
    case FeatureDetectorEnum::GFTT:
      featureDetector = cv::GFTTDetector::create();
      break;
    case FeatureDetectorEnum::MSER:
      featureDetector = cv::MSER::create();
      break;
    case FeatureDetectorEnum::ORB:
      featureDetector = cv::ORB::create(
        500, 1.2f, 8, 31,
        0, 2, cv::ORB::HARRIS_SCORE, 31, threshold);
      break;
    case FeatureDetectorEnum::SIFT:
      featureDetector = cv::SIFT::create();
      break;
  }

  return featureDetector;
}


cv::Ptr<cv::DescriptorExtractor> Tracker::initDescriptorExtractor(
  DescriptorExtractorEnum extractor,
  double threshold)
{
  cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
  switch (extractor) {
    case DescriptorExtractorEnum::ORB:
      descriptorExtractor = cv::ORB::create(
        500, 1.2f, 8, 31,
        0, 2, cv::ORB::HARRIS_SCORE, 31, threshold);
      break;
    case DescriptorExtractorEnum::SIFT:
      descriptorExtractor = cv::SIFT::create();
      break;
  }

  return descriptorExtractor;
}


cv::Ptr<cv::DescriptorMatcher> Tracker::initDescriptorMatcher(DescriptorMatcherEnum matcher)
{
  cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
  switch (matcher) {
    case DescriptorMatcherEnum::BRUTE_FORCE:
      descriptorMatcher = cv::BFMatcher::create();
      break;
    case DescriptorMatcherEnum::FLANN:
      descriptorMatcher = cv::FlannBasedMatcher::create();
      break;
  }

  return descriptorMatcher;
}

/// @todo Do keypoint vector editing in place
std::vector<cv::KeyPoint> Tracker::gridFeatures(
  std::vector<cv::KeyPoint> keyPoints,
  unsigned int rows,
  unsigned int cols)
{
  unsigned int minPixelDistance = 20;
  unsigned int gridCols = (int)((float)cols / (float)minPixelDistance);
  unsigned int gridRows = (int)((float)rows / (float)minPixelDistance);
  cv::Size size(gridCols, gridRows);

  /// @todo replace this with some kind of boolean array. Eigen?
  // Eigen::  Array<bool, 1, 5> false_array(5);
  // false_array = Array<bool, 1, 5>::Zero(5);
  /// @todo Implement non-maximal suppression instead

  cv::Mat grid_2d = cv::Mat::zeros(size, CV_8UC1);

  std::vector<cv::KeyPoint> gridKeyPoints;
  for (size_t i = 0; i < keyPoints.size(); i++) {
    // Get current left keypoint, check that it is in bounds
    cv::KeyPoint kpt = keyPoints.at(i);
    int x = (int)kpt.pt.x;
    int y = (int)kpt.pt.y;
    int x_grid = (int)(kpt.pt.x / (float)minPixelDistance);
    int y_grid = (int)(kpt.pt.y / (float)minPixelDistance);
    if (x_grid < 0 || x_grid >= size.width || y_grid < 0 || y_grid >= size.height || x < 0 ||
      x >= cols || y < 0 || y >= rows)
    {
      continue;
    }
    // Check if this keypoint is near another point
    if (grid_2d.at<uint8_t>(y_grid, x_grid) > 127) {
      continue;
    }
    // Else we are good, append our keypoints and descriptors
    gridKeyPoints.push_back(keyPoints.at(i));

    grid_2d.at<uint8_t>(y_grid, x_grid) = 255;
  }
  return gridKeyPoints;
}

void Tracker::track(
  unsigned int frameID, cv::Mat & imgIn, cv::Mat & imgOut,
  FeatureTracks featureTracks)
{
  m_featureDetector->detect(imgIn, m_currKeyPoints);
  /// @todo create occupancy grid of keypoints using minimal pixel distance
  m_currKeyPoints = gridFeatures(m_currKeyPoints, imgIn.rows, imgIn.cols);

  m_descriptorExtractor->compute(imgIn, m_currKeyPoints, m_currDescriptors);
  m_currDescriptors.convertTo(m_currDescriptors, CV_32F);
  cv::drawKeypoints(imgIn, m_currKeyPoints, imgOut);

  m_logger->log(LogLevel::DEBUG, "Called Tracker for frame ID: " + std::to_string(frameID));

  if (m_prevDescriptors.rows > 0 && m_currDescriptors.rows > 0) {
    std::vector<std::vector<cv::DMatch>> matches;

    /// @todo Mask using maximum distance from predicted IMU rotations
    m_descriptorMatcher->knnMatch(m_prevDescriptors, m_currDescriptors, matches, 5);

    // Use only "good" matches (i.e. whose distance is less than 3*min_dist )
    double max_dist = 0;
    double min_dist = 100;

    for (unsigned int i = 0; i < matches.size(); ++i) {
      // for (unsigned int j = 0; j < matches[i].size(); ++j) {
      double dist = matches[i][0].distance;

      if (dist < min_dist && dist >= 5.0) {min_dist = dist;}
      if (dist > max_dist) {max_dist = dist;}
      // }
    }

    auto goodMatches = std::vector<cv::DMatch>{};
    for (unsigned int i = 0; i < matches.size(); ++i) {
      for (unsigned int j = 0; j < matches[i].size(); ++j) {
        if (matches[i][j].distance < 3 * min_dist) {
          cv::Point2d point_old = m_prevKeyPoints[matches[i][j].queryIdx].pt;
          cv::Point2d point_new = m_currKeyPoints[matches[i][j].trainIdx].pt;
          cv::line(imgOut, point_old, point_new, cv::Scalar(0, 255, 0), 2, 8, 0);
          goodMatches.push_back(matches[i][j]);
        }
      }
    }

    // Assign previous Key Point ID for each match
    for (const auto & m : goodMatches) {
      m_currKeyPoints[m.trainIdx].class_id = m_prevKeyPoints[m.queryIdx].class_id;
    }

    // Only generate feature IDs for unmatched features
    for (auto & keyPoint : m_currKeyPoints) {
      if (keyPoint.class_id == -1) {
        keyPoint.class_id = generateFeatureID();
      }
    }

    // Store feature tracks
    for (const auto & keyPoint : m_currKeyPoints) {
      auto featureTrack = FeatureTrack{frameID, keyPoint};
      m_featureTrackMap[keyPoint.class_id].push_back(featureTrack);
    }

    // Update MSCKF on features no longer detected
    for (auto it = m_featureTrackMap.cbegin(); it != m_featureTrackMap.cend(); ) {
      const auto & featureTrack = it->second;
      if ((featureTrack.back().frameID < frameID) ||
        (featureTrack.size() >= max_track_length))
      {
        // This feature does not exist in the latest frame
        if (featureTrack.size() >= min_track_length) {
          featureTracks.push_back(featureTrack);
        }
        it = m_featureTrackMap.erase(it);
      } else {
        ++it;
      }
    }
  }

  m_msckfUpdater.updateEKF(m_cameraID, featureTracks);

  m_prevKeyPoints = m_currKeyPoints;
  m_prevDescriptors = m_currDescriptors;
}


unsigned int Tracker::generateFeatureID()
{
  static unsigned int featureID = 0;
  return featureID++;
}
