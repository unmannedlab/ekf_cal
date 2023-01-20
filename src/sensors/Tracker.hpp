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
#include <map>

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

  ///
  /// @brief Descriptor Enumerations
  ///
  enum class DescriptorExtractorEnum
  {
    ORB,
    SIFT
  };

  ///
  /// @brief Matcher Enumerations
  ///
  enum class DescriptorMatcherEnum
  {
    BRUTE_FORCE,
    FLANN
  };


  ///
  /// @brief Feature Track structure
  ///
  typedef struct FeatureTrack
  {
    unsigned int frameID;   /// <@brief Sequence ID of feature detection
    cv::KeyPoint keyPoint;  /// <@brief Detected key point
  } FeatureTrack;

  typedef std::vector<std::vector<Tracker::FeatureTrack>> FeatureTracks;

  ///
  /// @brief Tracker initialization parameters structure
  ///
  typedef struct Params
  {
    FeatureDetectorEnum detector {FeatureDetectorEnum::ORB};
    DescriptorExtractorEnum descriptor {DescriptorExtractorEnum::ORB};
    DescriptorMatcherEnum matcher {DescriptorMatcherEnum::FLANN};
    double threshold {20.0};
  } Params;

  ///
  /// @brief Tracker sensor constructor
  /// @param params Parameter struct for Tracker sensor
  ///
  explicit Tracker(Tracker::Params params);

  void Track(
    unsigned int frameID, cv::Mat & imgIn, cv::Mat & imgOut,
    FeatureTracks featureTracks);

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

  std::map<unsigned int, std::vector<FeatureTrack>> m_featureTrackMap;

  unsigned int generateFeatureID();

  Logger * m_logger = Logger::getInstance();  ///< @brief Logger singleton

  /// @todo Use parameter inputs for these values
  bool use_qr_decomposition{false};
  unsigned int max_track_length{30};
  unsigned int min_track_length{2};
};

#endif  // SENSORS__TRACKER_HPP_
