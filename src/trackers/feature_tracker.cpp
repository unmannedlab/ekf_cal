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
    params.output_directory,
    params.data_logging_on,
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
}

/// @todo Check what parameters are used by open_vins
cv::Ptr<cv::FeatureDetector> FeatureTracker::InitFeatureDetector(
  Detector detector,
  double threshold)
{
  cv::Ptr<cv::FeatureDetector> feature_detector;
  switch (detector) {
    case Detector::BRISK:
      feature_detector = cv::BRISK::create(threshold, 3, 1.0);
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
        cv::ORB::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, threshold);
      break;
    case Detector::SIFT:
      feature_detector = cv::SIFT::create();
      break;
  }

  return feature_detector;
}


cv::Ptr<cv::DescriptorExtractor> FeatureTracker::InitDescriptorExtractor(
  Descriptor extractor,
  double threshold)
{
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor;
  switch (extractor) {
    case Descriptor::ORB:
      descriptor_extractor =
        cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, threshold);
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
  std::vector<cv::KeyPoint> key_points,
  unsigned int rows,
  unsigned int cols)
{
  unsigned int min_pixel_distance = 10;
  double double_rows = static_cast<double>(rows);
  double double_cols = static_cast<double>(cols);
  double double_min_pixel_distance = static_cast<double>(min_pixel_distance);
  unsigned int grid_rows = static_cast<unsigned int>(double_rows / double_min_pixel_distance);
  unsigned int grid_cols = static_cast<unsigned int>(double_cols / double_min_pixel_distance);
  cv::Size size(grid_cols, grid_rows);

  /// @todo replace this with some kind of boolean array. Eigen?
  // Eigen::Array<bool, 1, 5> false_array(5);
  // false_array = Array<bool, 1, 5>::Zero(5);

  /// @todo Implement non-maximal suppression instead

  cv::Mat grid_2d = cv::Mat::zeros(size, CV_8UC1);

  std::vector<cv::KeyPoint> grid_key_points;
  for (size_t i = 0; i < key_points.size(); i++) {
    // Get current left keypoint, check that it is in bounds
    cv::KeyPoint kpt = key_points.at(i);
    int x = static_cast<int>(kpt.pt.x);
    int y = static_cast<int>(kpt.pt.y);
    int x_grid = static_cast<int>(kpt.pt.x / double_min_pixel_distance);
    int y_grid = static_cast<int>(kpt.pt.y / double_min_pixel_distance);
    if (x_grid < 0 || x_grid >= size.width || y_grid < 0 || y_grid >= size.height || x < 0 ||
      x >= static_cast<int>(cols) || y < 0 || y >= static_cast<int>(rows))
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

void FeatureTracker::Track(double time, int frame_id, cv::Mat & img_in, cv::Mat & img_out)
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

  double threshold_dist =
    0.1 * sqrt(
    static_cast<double>(
      down_sample_size.height * down_sample_size.height +
      down_sample_size.width * down_sample_size.width
    ));

  cv::Mat curr_descriptors;
  m_descriptor_extractor->compute(img_down, curr_key_points, curr_descriptors);
  curr_descriptors.convertTo(curr_descriptors, CV_32F);
  cv::drawKeypoints(img_down, curr_key_points, img_out);

  m_logger->Log(LogLevel::DEBUG, "Called Tracker for frame ID: " + std::to_string(frame_id));

  if (m_prev_descriptors.rows > 0 && curr_descriptors.rows > 0) {
    std::vector<std::vector<cv::DMatch>> matches_forward, matches_backward;
    std::vector<cv::DMatch> matches_good;

    /// @todo Mask using maximum distance from predicted IMU rotations
    m_descriptor_matcher->knnMatch(m_prev_descriptors, curr_descriptors, matches_forward, 2);
    m_descriptor_matcher->knnMatch(curr_descriptors, m_prev_descriptors, matches_backward, 2);

    ratio_test(matches_forward);
    ratio_test(matches_backward);

    /// @todo: Symmetry testing?

    matches_good.reserve(matches_forward.size());
    for (unsigned int i = 0; i < matches_forward.size(); ++i) {
      for (unsigned int j = 0; j < matches_forward[i].size(); j++) {
        cv::Point2f point_old = m_prev_key_points[matches_forward[i][j].queryIdx].pt;
        cv::Point2f point_new = curr_key_points[matches_forward[i][j].trainIdx].pt;

        // Calculate local distance for each possible match
        double dist = sqrt(
          (point_old.x - point_new.x) * (point_old.x - point_new.x) +
          (point_old.y - point_new.y) * (point_old.y - point_new.y));

        // Save as best match if local distance is in specified area and on same height
        if (dist < threshold_dist) {
          cv::line(img_out, point_old, point_new, cv::Scalar(0, 255, 0), 2, 8, 0);
          matches_good.push_back(matches_forward[i][j]);
          j = matches_forward[i].size();
        }
      }
    }

    // Generate IDs and add to track map features that persist along at least two frames
    for (const auto & m : matches_good) {
      if (m_prev_key_points[m.queryIdx].class_id == -1) {
        m_prev_key_points[m.queryIdx].class_id = GenerateFeatureID();
        auto feature_point = FeaturePoint{
          m_prev_frame_id, m_prev_frame_time, m_prev_key_points[m.queryIdx]};
        m_feature_points_map[m_prev_key_points[m.queryIdx].class_id].push_back(feature_point);
      }
      curr_key_points[m.trainIdx].class_id = m_prev_key_points[m.queryIdx].class_id;
      auto feature_point = FeaturePoint{
        frame_id, m_ekf->GetCurrentTime(), curr_key_points[m.trainIdx]};
      m_feature_points_map[curr_key_points[m.trainIdx].class_id].push_back(feature_point);
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
    if (feature_tracks.size() > 0) {
      m_msckf_updater.UpdateEKF(m_ekf, time, feature_tracks, m_px_error);
    }
  }

  m_prev_frame_id = frame_id;
  m_prev_frame_time = m_ekf->GetCurrentTime();
  m_prev_key_points = curr_key_points;
  m_prev_descriptors = curr_descriptors;
}


unsigned int FeatureTracker::GenerateFeatureID()
{
  static unsigned int featureID = 0;
  return featureID++;
}

void FeatureTracker::ratio_test(std::vector<std::vector<cv::DMatch>> & matches)
{
  for (auto & match : matches) {
    if (match.size() > 1) {
      // Remove matches that exceed KNN ratio
      if (match[0].distance / match[1].distance > m_knn_ratio) {
        match.clear();
      }
    } else {
      // Remove matches without 2 neighbors
      match.clear();
    }
  }
}
