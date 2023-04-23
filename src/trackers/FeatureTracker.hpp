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

#ifndef TRACKERS__FEATURETRACKER_HPP_
#define TRACKERS__FEATURETRACKER_HPP_

#include <string>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "infrastructure/DebugLogger.hpp"
#include "sensors/Sensor.hpp"
#include "ekf/Types.hpp"
#include "ekf/EKF.hpp"
#include "ekf/update/MsckfUpdater.hpp"

///
/// @class FeatureTracker
/// @brief FeatureTracker Class
///
class FeatureTracker
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
  /// @brief Feature Tracker Initialization parameters structure
  ///
  typedef struct Parameters
  {
    std::string name {""};                                          ///< @brief Feature Tracker name
    FeatureDetectorEnum detector {FeatureDetectorEnum::ORB};            ///< @brief Detector
    DescriptorExtractorEnum descriptor {DescriptorExtractorEnum::ORB};  ///< @brief Descriptor
    DescriptorMatcherEnum matcher {DescriptorMatcherEnum::FLANN};       ///< @brief Matcher
    double threshold {20.0};                                            ///< @brief Threshold
    unsigned int sensor_id;              ///< @brief Associated sensor ID
    std::string output_directory {""};   ///< @brief Feature Tracker data logging directory
    bool data_logging_on {false};        ///< @brief Feature Tracker data logging flag
  } Parameters;

  ///
  /// @brief FeatureTracker sensor constructor
  /// @param params Parameter struct for feature tracker
  /// @param cameraID Associated camera ID
  ///
  explicit FeatureTracker(FeatureTracker::Parameters params);

  std::vector<cv::KeyPoint> GridFeatures(
    std::vector<cv::KeyPoint> points,
    unsigned int rows,
    unsigned int cols);

  void Track(
    double time,
    unsigned int frame_id, cv::Mat & img_in, cv::Mat & img_out,
    FeatureTracks feature_tracks);

  unsigned int GetID();

protected:
  DebugLogger * m_logger = DebugLogger::GetInstance();  ///< @brief Logger singleton
  unsigned int max_track_length{30};
  unsigned int min_track_length{2};
  unsigned int m_camera_id;
  MsckfUpdater m_msckf_updater;
  unsigned int m_id;

private:
  cv::Ptr<cv::FeatureDetector> InitFeatureDetector(
    FeatureDetectorEnum detector,
    double threshold);
  cv::Ptr<cv::DescriptorExtractor> InitDescriptorExtractor(
    DescriptorExtractorEnum extractor,
    double threshold);
  cv::Ptr<cv::DescriptorMatcher> InitDescriptorMatcher(
    DescriptorMatcherEnum matcher);

  cv::Ptr<cv::FeatureDetector> m_feature_detector;
  cv::Ptr<cv::DescriptorExtractor> m_descriptor_extractor;
  cv::Ptr<cv::DescriptorMatcher> m_descriptor_matcher;

  std::vector<cv::KeyPoint> m_prev_key_points;
  std::vector<cv::KeyPoint> m_curr_key_points;
  cv::Mat m_prev_descriptors;
  cv::Mat m_curr_descriptors;

  std::map<unsigned int, std::vector<FeatureTrack>> m_feature_track_map;

  unsigned int GenerateFeatureID();

  EKF * m_ekf = EKF::GetInstance();           ///< @brief EKF singleton

  static unsigned int m_tracker_count;
};

#endif  // TRACKERS__FEATURETRACKER_HPP_
