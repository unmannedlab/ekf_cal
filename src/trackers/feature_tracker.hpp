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

#ifndef TRACKERS__FEATURE_TRACKER_HPP_
#define TRACKERS__FEATURE_TRACKER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "ekf/update/msckf_updater.hpp"
#include "infrastructure/debug_logger.hpp"
#include "trackers/tracker.hpp"

///
/// @brief Detector Enumerations
///
enum class Detector
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
enum class Descriptor
{
  ORB,
  SIFT
};

///
/// @brief Matcher Enumerations
///
enum class Matcher
{
  BRUTE_FORCE,
  FLANN
};

///
/// @class FeatureTracker
/// @brief FeatureTracker Class
///
class FeatureTracker : public Tracker
{
public:
  ///
  /// @brief Feature Tracker Initialization parameters structure
  ///
  typedef struct Parameters : public Tracker::Parameters
  {
    Detector detector {Detector::FAST};       ///< @brief Detector
    Descriptor descriptor {Descriptor::ORB};  ///< @brief Descriptor
    Matcher matcher {Matcher::FLANN};         ///< @brief Matcher
    int threshold {20};                       ///< @brief Threshold
    double px_error{1e-9};                    ///< @brief Pixel error standard deviation
    double min_feat_dist {1.0};               ///< @brief Minimum feature distance to consider
    bool down_sample {false};                 ///< @brief Flag to perform down-sampling
    int down_sample_height {480};             ///< @brief Down-sampled height to use for tracking
    int down_sample_width {640};              ///< @brief Down-sampled width to use for tracking
    bool is_cam_extrinsic{false};             ///< @brief Flag for extrinsic camera calibration
  } Parameters;

  ///
  /// @brief FeatureTracker sensor constructor
  /// @param params Parameter struct for feature tracker
  ///
  explicit FeatureTracker(FeatureTracker::Parameters params);

  ///
  /// @brief Down sample features to grid
  /// @param key_points Key points to down sample
  /// @param rows Number of final rows to consider
  /// @param cols Number of final columns to consider
  /// @return Down sampled key points
  ///
  std::vector<cv::KeyPoint> GridFeatures(
    std::vector<cv::KeyPoint> & key_points,
    int rows,
    int cols
  ) const;

  ///
  /// @brief Perform track on new image frame
  /// @param time Frame time
  /// @param frame_id Frame ID
  /// @param img_in Input frame
  /// @param img_out Output frame with drawn track lines
  ///
  void Track(
    double time,
    unsigned int frame_id,
    const cv::Mat & img_in,
    cv::Mat & img_out
  );

  ///
  /// @brief Perform ratio test on a set of matches
  /// @param matches List of matches to perform test over
  ///
  void RatioTest(std::vector<std::vector<cv::DMatch>> & matches) const;

  ///
  /// @brief Perform symmetry test given forward and backward matches
  /// @param matches_forward Forward matches
  /// @param matches_backward Backward matches
  /// @param matches_out Passing matches
  ///
  void SymmetryTest(
    std::vector<std::vector<cv::DMatch>> & matches_forward,
    std::vector<std::vector<cv::DMatch>> & matches_backward,
    std::vector<cv::DMatch> & matches_out
  ) const;

  ///
  /// @brief Perform RANSAC filtering test given matches and key points
  /// @param matches_in Current matches
  /// @param curr_key_points Current detected key points
  /// @param matches_out Passing matches
  ///
  void RANSAC(
    std::vector<cv::DMatch> & matches_in,
    std::vector<cv::KeyPoint> & curr_key_points,
    std::vector<cv::DMatch> & matches_out
  ) const;

  ///
  /// @brief Perform distance test given matches and key points
  /// @param matches_in Current matches
  /// @param curr_key_points Current detected key points
  /// @param matches_out Passing matches
  ///
  void DistanceTest(
    std::vector<cv::DMatch> & matches_in,
    std::vector<cv::KeyPoint> & curr_key_points,
    std::vector<cv::DMatch> & matches_out
  ) const;

protected:
  MsckfUpdater m_msckf_updater;  ///< @brief MSCKF updater object

private:
  cv::Ptr<cv::FeatureDetector> InitFeatureDetector(Detector detector, int threshold);
  cv::Ptr<cv::DescriptorExtractor> InitDescriptorExtractor(Descriptor extractor, int threshold);
  cv::Ptr<cv::DescriptorMatcher> InitDescriptorMatcher(Matcher matcher);

  cv::Ptr<cv::FeatureDetector> m_feature_detector;
  cv::Ptr<cv::DescriptorExtractor> m_descriptor_extractor;
  cv::Ptr<cv::DescriptorMatcher> m_descriptor_matcher;

  unsigned int m_prev_frame_id;
  double m_prev_frame_time;
  cv::Mat m_prev_descriptors;
  std::vector<cv::KeyPoint> m_prev_key_points;

  std::map<unsigned int, std::vector<FeaturePoint>> m_feature_points_map;

  int GenerateFeatureID();

  double m_px_error;
  bool m_down_sample;
  int m_down_sample_height;
  int m_down_sample_width;
  double m_knn_ratio{0.7};
};

#endif  // TRACKERS__FEATURE_TRACKER_HPP_
