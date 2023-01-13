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

cv::Ptr<cv::FeatureDetector> Tracker::GetFeatureDetector()
{
  return m_featureDetector;
}


cv::Ptr<cv::DescriptorExtractor> Tracker::GetDescriptorExtractor()
{
  return m_descriptorExtractor;
}


cv::Ptr<cv::DescriptorMatcher> Tracker::GetDescriptorMatcher()
{
  return m_descriptorMatcher;
}
