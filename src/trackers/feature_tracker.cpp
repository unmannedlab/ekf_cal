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

#include "trackers/feature_tracker.hpp"

#include <stddef.h>
#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "ekf/types.hpp"
#include "trackers/tracker.hpp"


/// @todo add detector/extractor parameters to input
FeatureTracker::FeatureTracker(FeatureTracker::Parameters params)
: Tracker(params),
  m_msckf_updater(
    params.camera_id,
    params.is_cam_extrinsic,
    params.log_directory,
    params.data_log_rate,
    params.min_feat_dist,
    params.logger
  ),
  m_px_error(params.px_error),
  m_down_sample(params.down_sample),
  m_down_sample_height(params.down_sample_height),
  m_down_sample_width(params.down_sample_width)
{
  m_feature_detector = InitFeatureDetector(params.detector, params.threshold);
  m_descriptor_extractor = InitDescriptorExtractor(params.descriptor, params.threshold);
  m_descriptor_matcher = InitDescriptorMatcher(params.matcher);
  m_prev_frame_time = m_ekf->GetCurrentTime();
}

/// @todo Check what parameters are used by open_vins
cv::Ptr<cv::FeatureDetector> FeatureTracker::InitFeatureDetector(
  Detector detector, int threshold)
{
  cv::Ptr<cv::FeatureDetector> feature_detector;
  switch (detector) {
    case Detector::BRISK:
      feature_detector = cv::BRISK::create(threshold, 3, 1);
      break;
    case Detector::FAST:
      feature_detector =
        cv::FastFeatureDetector::create(threshold, true, cv::FastFeatureDetector::TYPE_9_16);
      break;
    case Detector::GFTT:
      feature_detector = cv::GFTTDetector::create();
      break;
    case Detector::MSER:
      feature_detector = cv::MSER::create();
      break;
    case Detector::ORB:
      feature_detector =
        cv::ORB::create(1000, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, threshold);
      break;
    case Detector::SIFT:
      feature_detector = cv::SIFT::create();
      break;
  }

  return feature_detector;
}

cv::Ptr<cv::DescriptorExtractor> FeatureTracker::InitDescriptorExtractor(
  Descriptor extractor, int threshold)
{
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
  switch (extractor) {
    case Descriptor::ORB:
      descriptor_extractor =
        cv::ORB::create(500, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, threshold);
      break;
    case Descriptor::SIFT:
      descriptor_extractor = cv::SIFT::create();
      break;
  }

  return descriptor_extractor;
}

cv::Ptr<cv::DescriptorMatcher> FeatureTracker::InitDescriptorMatcher(Matcher matcher)
{
  cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
  switch (matcher) {
    case Matcher::BRUTE_FORCE:
      descriptor_matcher = cv::BFMatcher::create();
      break;
    case Matcher::FLANN:
      descriptor_matcher = cv::FlannBasedMatcher::create();
      break;
  }

  return descriptor_matcher;
}

/// @todo Do keypoint vector editing in place
std::vector<cv::KeyPoint> FeatureTracker::GridFeatures(
  std::vector<cv::KeyPoint> & key_points,
  int rows,
  int cols
)
{
  double min_pixel_distance = 10.0;
  auto double_rows = static_cast<double>(rows);
  auto double_cols = static_cast<double>(cols);
  auto grid_rows = static_cast<int>(double_rows / min_pixel_distance);
  auto grid_cols = static_cast<int>(double_cols / min_pixel_distance);
  cv::Size size(grid_cols, grid_rows);

  /// @todo replace this with some kind of boolean array. Eigen?
  // Eigen::Array<bool, 1, 5> false_array(5);
  // false_array = Array<bool, 1, 5>::Zero(5);

  /// @todo Implement non-maximal suppression instead

  cv::Mat grid_2d = cv::Mat::zeros(size, CV_8UC1);

  std::vector<cv::KeyPoint> grid_key_points;
  for (unsigned int i = 0; i < key_points.size(); i++) {
    // Get current left keypoint, check that it is in bounds
    cv::KeyPoint kpt = key_points.at(i);
    int pt_x = static_cast<int>(kpt.pt.x);
    int pt_y = static_cast<int>(kpt.pt.y);
    int x_grid = static_cast<int>(static_cast<double>(kpt.pt.x) / min_pixel_distance);
    int y_grid = static_cast<int>(static_cast<double>(kpt.pt.y) / min_pixel_distance);
    if (x_grid < 0 || x_grid >= size.width || y_grid < 0 || y_grid >= size.height || pt_x < 0 ||
      pt_x >= cols || pt_y < 0 || pt_y >= rows)
    {
      continue;
    }
    // Check if this keypoint is near another point
    if (grid_2d.at<uint8_t>(y_grid, x_grid) > 127) {
      continue;
    }
    // Else we are good, append our key_points and descriptors
    grid_key_points.push_back(key_points.at(i));

    grid_2d.at<uint8_t>(y_grid, x_grid) = 255;
  }
  return grid_key_points;
}

void FeatureTracker::Track(
  double time, unsigned int frame_id, const cv::Mat & img_in, cv::Mat & img_out)
{
  // Down sample image
  /// @todo: Get down-sample parameters from input
  cv::Mat img_down;
  cv::Size down_sample_size;
  if (m_down_sample) {
    down_sample_size.height = m_down_sample_height;
    down_sample_size.width = m_down_sample_width;
    cv::resize(img_in, img_down, down_sample_size);
  } else {
    down_sample_size.height = img_in.rows;
    down_sample_size.width = img_in.cols;
    img_down = img_in;
  }

  std::vector<cv::KeyPoint> curr_key_points;
  m_feature_detector->detect(img_down, curr_key_points);
  /// @todo create occupancy grid of key_points using minimal pixel distance
  curr_key_points = GridFeatures(curr_key_points, img_down.rows, img_down.cols);

  // double threshold_dist =
  //   0.1 * sqrt(
  //   static_cast<double>(
  //     down_sample_size.height * down_sample_size.height +
  //     down_sample_size.width * down_sample_size.width
  //   ));

  cv::Mat curr_descriptors;
  m_descriptor_extractor->compute(img_down, curr_key_points, curr_descriptors);
  curr_descriptors.convertTo(curr_descriptors, CV_32F);

  m_logger->Log(LogLevel::DEBUG, "Called Tracker for frame ID: " + std::to_string(frame_id));

  if (m_prev_descriptors.rows > 0 && curr_descriptors.rows > 0) {
    std::vector<std::vector<cv::DMatch>> matches_forward;
    std::vector<std::vector<cv::DMatch>> matches_backward;

    /// @todo Mask using maximum distance from predicted IMU rotations
    m_descriptor_matcher->knnMatch(m_prev_descriptors, curr_descriptors, matches_forward, 2);
    m_descriptor_matcher->knnMatch(curr_descriptors, m_prev_descriptors, matches_backward, 2);

    // Perform test on matches
    RatioTest(matches_forward);
    RatioTest(matches_backward);
    std::vector<cv::DMatch> matches_symmetry;
    std::vector<cv::DMatch> matches_ransac;
    std::vector<cv::DMatch> matches_final;
    SymmetryTest(matches_forward, matches_backward, matches_symmetry);
    RANSAC(matches_symmetry, curr_key_points, matches_ransac);
    DistanceTest(matches_ransac, curr_key_points, matches_final);

    // Draw tracks
    for (const auto & match : matches_final) {
      cv::Point2f point_old = m_prev_key_points[static_cast<unsigned int>(match.queryIdx)].pt;
      cv::Point2f point_new = curr_key_points[static_cast<unsigned int>(match.trainIdx)].pt;
      cv::line(img_out, point_old, point_new, cv::Scalar(0, 255, 0), 2, 8, 0);
    }

    // Generate IDs and add to track map features that persist along at least two frames
    for (const auto & match : matches_final) {
      if (m_prev_key_points[static_cast<unsigned int>(match.queryIdx)].class_id == -1) {
        m_prev_key_points[static_cast<unsigned int>(match.queryIdx)].class_id = GenerateFeatureID();
        FeaturePoint feat_point {m_prev_frame_id, m_prev_frame_time,
          m_prev_key_points[static_cast<unsigned int>(match.queryIdx)]};
        m_feature_points_map[static_cast<unsigned int>(m_prev_key_points[
            static_cast<unsigned int>(match.queryIdx)].class_id)].push_back(feat_point);
      }
      curr_key_points[static_cast<unsigned int>(match.trainIdx)].class_id =
        m_prev_key_points[static_cast<unsigned int>(match.queryIdx)].class_id;
      FeaturePoint feat_point {frame_id, m_ekf->GetCurrentTime(),
        curr_key_points[static_cast<unsigned int>(match.trainIdx)]};
      m_feature_points_map[static_cast<unsigned int>(curr_key_points[
          static_cast<unsigned int>(match.trainIdx)].class_id)].
      push_back(feat_point);
    }

    // Update MSCKF on features no longer detected
    FeatureTracks feature_tracks;
    for (auto it = m_feature_points_map.cbegin(); it != m_feature_points_map.cend(); ) {
      const auto & feature_points = it->second;
      if ((feature_points.back().frame_id < frame_id) ||
        (feature_points.size() >= m_max_track_length))
      {
        // This feature does not exist in the latest frame
        if (feature_points.size() >= m_min_track_length) {
          FeatureTrack feature_track;
          feature_track.track = feature_points;
          feature_tracks.push_back(feature_track);
        }
        it = m_feature_points_map.erase(it);
      } else {
        ++it;
      }
    }
    if (!feature_tracks.empty()) {
      m_msckf_updater.UpdateEKF(*m_ekf, time, feature_tracks, m_px_error);
    }
  }

  m_prev_frame_id = frame_id;
  m_prev_frame_time = m_ekf->GetCurrentTime();
  m_prev_key_points = curr_key_points;
  m_prev_descriptors = curr_descriptors;
}

int FeatureTracker::GenerateFeatureID()
{
  static int featureID = 0;
  return featureID++;
}

void FeatureTracker::RatioTest(std::vector<std::vector<cv::DMatch>> & matches) const
{
  for (auto & match : matches) {
    // Remove matches without two nearest neighbors or that fail the ratio test
    if ((match.size() == 1) ||
      ((match.size() == 2) &&
      (static_cast<double>(match[0].distance / match[1].distance) > m_knn_ratio)))
    {
      match.clear();
    }
  }
}

void FeatureTracker::SymmetryTest(
  std::vector<std::vector<cv::DMatch>> & matches_forward,
  std::vector<std::vector<cv::DMatch>> & matches_backward,
  std::vector<cv::DMatch> & matches_out
)
{
  for (auto & match_f : matches_forward) {
    if (match_f.size() != 2) {
      continue;
    }
    for (auto & match_b : matches_backward) {
      if (match_b.size() != 2) {
        continue;
      }
      // Test if matches are found symmetrically forward and backward
      if (match_f[0].queryIdx == match_b[0].trainIdx &&
        match_b[0].queryIdx == match_f[0].trainIdx)
      {
        cv::DMatch match {match_f[0].queryIdx, match_f[0].trainIdx, match_f[0].distance};
        matches_out.emplace_back(match);
        break;
      }
    }
  }
}

void FeatureTracker::RANSAC(
  std::vector<cv::DMatch> & matches_in,
  std::vector<cv::KeyPoint> & curr_key_points,
  std::vector<cv::DMatch> & matches_out
) const
{
  // Minimum threshold for RANSAC
  if (matches_in.size() < 10) {
    return;
  }

  // Convert points into points for RANSAC
  std::vector<cv::Point2f> points_good_prev;
  std::vector<cv::Point2f> points_good_curr;
  for (unsigned int i = 0; i < matches_in.size(); i++) {
    points_good_prev.push_back(
      m_prev_key_points[static_cast<unsigned int>(matches_in.at(i).queryIdx)].pt);
    points_good_curr.push_back(
      curr_key_points[static_cast<unsigned int>(matches_in.at(i).trainIdx)].pt);
  }

  /// @todo: Undistort?

  std::vector<uint8_t> mask;
  cv::findFundamentalMat(points_good_prev, points_good_curr, cv::FM_RANSAC, 1.0, 0.99, mask);

  // Apply mask
  for (unsigned int i = 0; i < matches_in.size(); i++) {
    if (mask[i] == 1) {
      matches_out.push_back(matches_in.at(i));
    }
  }
}

void FeatureTracker::DistanceTest(
  std::vector<cv::DMatch> & matches_in,
  std::vector<cv::KeyPoint> & curr_key_points,
  std::vector<cv::DMatch> & matches_out
) const
{
  double dist_sum{0.0};
  std::vector<cv::Point2f> points_good_prev;
  std::vector<cv::Point2f> points_good_curr;
  for (unsigned int i = 0; i < matches_in.size(); i++) {
    cv::Point2f point_prev =
      m_prev_key_points[static_cast<unsigned int>(matches_in.at(i).queryIdx)].pt;
    cv::Point2f point_curr =
      curr_key_points[static_cast<unsigned int>(matches_in.at(i).trainIdx)].pt;
    points_good_prev.push_back(point_prev);
    points_good_curr.push_back(point_curr);
    dist_sum += std::sqrt(
      std::pow(point_curr.x - point_prev.x, 2) +
      std::pow(point_curr.y - point_prev.y, 2));
  }

  double dist_mean = dist_sum / static_cast<double>(matches_in.size());

  for (unsigned int i = 0; i < matches_in.size(); i++) {
    cv::Point2f point_prev =
      m_prev_key_points[static_cast<unsigned int>(matches_in.at(i).queryIdx)].pt;
    cv::Point2f point_curr =
      curr_key_points[static_cast<unsigned int>(matches_in.at(i).trainIdx)].pt;
    double dist = std::sqrt(
      std::pow(point_curr.x - point_prev.x, 2) +
      std::pow(point_curr.y - point_prev.y, 2));
    if (dist < dist_mean * 3.0) {
      matches_out.push_back(matches_in[i]);
    }
  }
}
