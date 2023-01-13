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

#ifndef SENSORS__TRACKER_HPP_
#define SENSORS__TRACKER_HPP_

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "sensors/Sensor.hpp"

///
/// @class Tracker
/// @brief Tracker Class
///
class Tracker
{
public:
  ///
  /// @brief Detector Enumerations
  ///
  enum class FeatureDetectorEnum
  {
    BRISK,
    FAST,
    GFTT,
    MSER,
    ORB,
    SIFT,
  };

  enum class DescriptorExtractorEnum
  {
    ORB,
    SIFT
  };

  enum class DescriptorMatcherEnum
  {
    BRUTE_FORCE,
    FLANN
  };

  ///
  /// @brief Tracker initialization parameters structure
  ///
  typedef struct Params
  {
    FeatureDetectorEnum featureDetector {FeatureDetectorEnum::ORB};
    DescriptorExtractorEnum descriptorExtractor {DescriptorExtractorEnum::ORB};
    DescriptorMatcherEnum descriptorMatcher {DescriptorMatcherEnum::FLANN};
    double detectorThreshold {20.0};
  } Params;

  ///
  /// @brief Tracker sensor constructor
  /// @param params Parameter struct for Tracker sensor
  ///
  explicit Tracker(Tracker::Params params);

  cv::Ptr<cv::FeatureDetector> GetFeatureDetector();
  cv::Ptr<cv::DescriptorExtractor> GetDescriptorExtractor();
  cv::Ptr<cv::DescriptorMatcher> GetDescriptorMatcher();

private:
  cv::Ptr<cv::FeatureDetector> InitFeatureDetector(
    FeatureDetectorEnum detector,
    double threshold);
  cv::Ptr<cv::DescriptorExtractor> InitDescriptorExtractor(
    DescriptorExtractorEnum extractor,
    double threshold);
  cv::Ptr<cv::DescriptorMatcher> InitDescriptorMatcher(
    DescriptorMatcherEnum matcher);

  cv::Ptr<cv::FeatureDetector> m_featureDetector;
  cv::Ptr<cv::DescriptorExtractor> m_descriptorExtractor;
  cv::Ptr<cv::DescriptorMatcher> m_descriptorMatcher;

  std::vector<cv::KeyPoint> m_prevKeyPoints;
  std::vector<cv::KeyPoint> m_currKeyPoints;
  cv::Mat m_prevDescriptors;
  cv::Mat m_currDescriptors;
};

#endif  // SENSORS__TRACKER_HPP_
