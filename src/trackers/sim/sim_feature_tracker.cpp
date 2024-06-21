// Copyright 2022 Jacob Hartzer
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

#include "trackers/sim/sim_feature_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <ostream>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "ekf/types.hpp"
#include "ekf/update/msckf_updater.hpp"
#include "infrastructure/debug_logger.hpp"
#include "trackers/sim/sim_feature_tracker_message.hpp"
#include "utility/type_helper.hpp"

SimFeatureTracker::SimFeatureTracker(
  SimFeatureTracker::Parameters params,
  std::shared_ptr<TruthEngine> truthEngine)
: FeatureTracker(params.tracker_params),
  m_rng(params.rng)
{
  m_px_error = params.tracker_params.px_error;
  m_no_errors = params.no_errors;
  m_feature_count = params.feature_count;
  m_max_track_length = params.tracker_params.max_track_length;
  m_truth = truthEngine;
}

std::vector<cv::KeyPoint> SimFeatureTracker::VisibleKeypoints(double time, int sensor_id)
{
  Eigen::Vector3d pos_b_in_g = m_truth->GetBodyPosition(time);
  Eigen::Quaterniond ang_b_to_g = m_truth->GetBodyAngularPosition(time);
  Eigen::Vector3d pos_c_in_b = m_truth->GetCameraPosition(sensor_id);
  Eigen::Quaterniond ang_c_to_b = m_truth->GetCameraAngularPosition(sensor_id);
  Intrinsics intrinsics = m_truth->GetCameraIntrinsics(sensor_id);
  Eigen::Matrix3d ang_g_to_c = (ang_b_to_g * ang_c_to_b).toRotationMatrix().transpose();
  cv::Mat ang_g_to_c_cv(3, 3, cv::DataType<double>::type);
  EigenMatrixToCv(ang_g_to_c, ang_g_to_c_cv);

  // Creating Rodrigues rotation matrix
  cv::Mat r_vec(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(ang_g_to_c_cv, r_vec);

  Eigen::Vector3d pos_g_in_c = ang_g_to_c * (-(pos_b_in_g + ang_b_to_g * pos_c_in_b));

  cv::Mat t_vec(3, 1, cv::DataType<double>::type);
  t_vec.at<double>(0) = pos_g_in_c[0];
  t_vec.at<double>(1) = pos_g_in_c[1];
  t_vec.at<double>(2) = pos_g_in_c[2];

  // Create intrinsic matrices
  cv::Mat camera_matrix = intrinsics.ToCameraMatrix();
  cv::Mat distortion = intrinsics.ToDistortionVector();

  // Project points
  std::vector<cv::Point2d> projected_points;

  std::vector<cv::Point3d> feature_points = m_truth->GetFeatures();
  cv::projectPoints(feature_points, r_vec, t_vec, camera_matrix, distortion, projected_points);

  // Convert to feature points
  std::vector<cv::KeyPoint> projected_features;
  Eigen::Vector3d cam_plane_vec = ang_g_to_c.transpose() * Eigen::Vector3d(0, 0, 1);
  for (unsigned int i = 0; i < projected_points.size(); ++i) {
    cv::Point3d pointCV = feature_points[i];
    Eigen::Vector3d pointEig(pointCV.x, pointCV.y, pointCV.z);

    // Check that point is in front of camera plane and within sensor limits
    if (
      cam_plane_vec.dot(pointEig) > 0 &&
      projected_points[i].x >= 0 &&
      projected_points[i].y >= 0 &&
      projected_points[i].x <= intrinsics.width &&
      projected_points[i].y <= intrinsics.height)
    {
      cv::KeyPoint feat;
      feat.class_id = i;
      if (m_no_errors) {
        feat.pt.x = projected_points[i].x;
        feat.pt.y = projected_points[i].y;
      } else {
        feat.pt.x = round(m_rng.NormRand(projected_points[i].x, m_px_error));
        feat.pt.y = round(m_rng.NormRand(projected_points[i].y, m_px_error));
      }
      projected_features.push_back(feat);
    }
  }

  return projected_features;
}

std::vector<std::shared_ptr<SimFeatureTrackerMessage>> SimFeatureTracker::GenerateMessages(
  std::vector<double> message_times, int sensor_id)
{
  m_logger->Log(
    LogLevel::INFO, "Generating " + std::to_string(message_times.size()) + " Feature measurements");

  std::map<unsigned int, std::vector<FeaturePoint>> feature_track_map;
  std::vector<std::shared_ptr<SimFeatureTrackerMessage>> tracker_messages;

  for (int frame_id = 0; static_cast<unsigned int>(frame_id) < message_times.size(); ++frame_id) {
    std::vector<std::vector<FeaturePoint>> feature_tracks;

    std::vector<cv::KeyPoint> key_points =
      VisibleKeypoints(message_times[frame_id], sensor_id);

    for (auto & key_point : key_points) {
      auto feature_track = FeaturePoint{frame_id, key_point};
      feature_track_map[key_point.class_id].push_back(feature_track);
    }

    // Update MSCKF on features no longer detected
    for (auto it = feature_track_map.cbegin(); it != feature_track_map.cend(); ) {
      const auto & feature_track = it->second;
      if ((feature_track.back().frame_id < frame_id) ||
        (feature_track.size() >= m_max_track_length))
      {
        // This feature does not exist in the latest frame
        if (feature_track.size() > 1) {
          feature_tracks.push_back(feature_track);
        }
        it = feature_track_map.erase(it);
      } else {
        ++it;
      }
    }
    auto tracker_message = std::make_shared<SimFeatureTrackerMessage>();
    tracker_message->feature_tracks = feature_tracks;
    tracker_message->time = message_times[frame_id];
    tracker_message->tracker_id = m_id;
    tracker_message->sensor_id = sensor_id;
    tracker_message->sensor_type = SensorType::Tracker;
    tracker_messages.push_back(tracker_message);
  }
  return tracker_messages;
}

void SimFeatureTracker::Callback(double time, std::shared_ptr<SimFeatureTrackerMessage> msg)
{
  m_msckf_updater.UpdateEKF(
    m_ekf,
    time,
    msg->feature_tracks,
    m_px_error);
}
