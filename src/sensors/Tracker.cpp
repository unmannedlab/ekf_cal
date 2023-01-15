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


/// @todo add detector/extractor parameters to input
Tracker::Tracker(Tracker::Params params)
{
  m_featureDetector = InitFeatureDetector(FeatureDetectorEnum::ORB, 4);
  m_descriptorExtractor = InitDescriptorExtractor(DescriptorExtractorEnum::ORB, 4);
  m_descriptorMatcher = InitDescriptorMatcher(DescriptorMatcherEnum::FLANN);
}


cv::Ptr<cv::FeatureDetector> Tracker::InitFeatureDetector(
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


cv::Ptr<cv::DescriptorExtractor> Tracker::InitDescriptorExtractor(
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


cv::Ptr<cv::DescriptorMatcher> Tracker::InitDescriptorMatcher(DescriptorMatcherEnum matcher)
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

/// @todo create maximum distance using predicted IMU rotations
void Tracker::Track(double time, cv::Mat & imgIn, cv::Mat & imgOut)
{
  m_featureDetector->detect(imgIn, m_currKeyPoints);
  m_descriptorExtractor->compute(imgIn, m_currKeyPoints, m_currDescriptors);
  m_currDescriptors.convertTo(m_currDescriptors, CV_32F);
  cv::drawKeypoints(imgIn, m_currKeyPoints, imgOut);

  if (m_prevDescriptors.rows > 0 && m_currDescriptors.rows > 0) {
    std::vector<std::vector<cv::DMatch>> matches;

    m_descriptorMatcher->knnMatch(m_prevDescriptors, m_currDescriptors, matches, 2);

    // Use only "good" matches (i.e. whose distance is less than 3*min_dist )
    double max_dist = 0;
    double min_dist = 100;

    for (unsigned int i = 0; i < matches.size(); ++i) {
      for (unsigned int j = 0; j < matches[i].size(); ++j) {
        double dist = matches[i][j].distance;
        if (dist < min_dist) {min_dist = dist;}
        if (dist > max_dist) {max_dist = dist;}
      }
    }

    for (unsigned int i = 0; i < matches.size(); ++i) {
      for (unsigned int j = 0; j < matches[i].size(); ++j) {
        if (matches[i][j].distance < 3 * min_dist) {
          cv::Point2d point_old = m_prevKeyPoints[matches[i][j].queryIdx].pt;
          cv::Point2d point_new = m_currKeyPoints[matches[i][j].trainIdx].pt;
          cv::line(imgOut, point_old, point_new, cv::Scalar(0, 255, 0), 2, 8, 0);
        }
      }
    }
  }

  m_prevKeyPoints = m_currKeyPoints;
  m_prevDescriptors = m_currDescriptors;
}


unsigned int Tracker::generateFeatureID()
{
  static unsigned int featureID = 0;
  return featureID++;
}

unsigned int Tracker::generateSequenceID()
{
  static unsigned int SequenceID = 0;
  return SequenceID++;
}
