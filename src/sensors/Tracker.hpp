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

#include "infrastructure/Logger.hpp"
#include "sensors/Sensor.hpp"
#include "ekf/Types.hpp"
#include "ekf/EKF.hpp"
#include "ekf/update/MsckfUpdater.hpp"

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
  /// @brief Tracker initialization parameters structure
  ///
  typedef struct Params
  {
    FeatureDetectorEnum detector {FeatureDetectorEnum::ORB};            ///< @brief Detector
    DescriptorExtractorEnum descriptor {DescriptorExtractorEnum::ORB};  ///< @brief Descriptor
    DescriptorMatcherEnum matcher {DescriptorMatcherEnum::FLANN};       ///< @brief Matcher
    double threshold {20.0};                                            ///< @brief Threshold
  } Params;

  ///
  /// @brief Tracker sensor constructor
  /// @param params Parameter struct for Tracker sensor
  /// @param cameraID Associated camera ID
  ///
  explicit Tracker(Tracker::Params params, unsigned int cameraID);

  void track(
    unsigned int frameID, cv::Mat & imgIn, cv::Mat & imgOut,
    FeatureTracks featureTracks);

private:
  cv::Ptr<cv::FeatureDetector> initFeatureDetector(
    FeatureDetectorEnum detector,
    double threshold);
  cv::Ptr<cv::DescriptorExtractor> initDescriptorExtractor(
    DescriptorExtractorEnum extractor,
    double threshold);
  cv::Ptr<cv::DescriptorMatcher> initDescriptorMatcher(
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
  EKF * m_ekf = EKF::getInstance();           ///< @brief EKF singleton

  /// @todo Use parameter inputs for these values
  bool use_qr_decomposition{false};
  unsigned int max_track_length{30};
  unsigned int min_track_length{2};
  unsigned int m_cameraID;
  MsckfUpdater m_msckfUpdater;
};

#endif  // SENSORS__TRACKER_HPP_
