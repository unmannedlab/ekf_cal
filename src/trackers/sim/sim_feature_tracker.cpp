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

#include <map>
#include <memory>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

SimFeatureTracker::SimFeatureTracker(
  SimFeatureTracker::Parameters params,
  std::shared_ptr<TruthEngine> truthEngine)
: FeatureTracker(params.tracker_params)
{
  m_px_error = params.px_error;
  m_feature_count = params.feature_count;
  m_truth = truthEngine;

  m_feature_points.push_back(cv::Point3d(1, 0, 0));
  m_feature_points.push_back(cv::Point3d(-1, 0, 0));
  m_feature_points.push_back(cv::Point3d(0, 1, 0));
  m_feature_points.push_back(cv::Point3d(0, -1, 0));
  m_feature_points.push_back(cv::Point3d(0, 0, 1));
  m_feature_points.push_back(cv::Point3d(0, 0, -1));
  for (unsigned int i = 0; i < m_feature_count; ++i) {
    cv::Point3d vec;
    vec.x = m_rng.UniRand(-params.room_size, params.room_size);
    vec.y = m_rng.UniRand(-params.room_size, params.room_size);
    vec.z = m_rng.UniRand(-params.room_size / 10, params.room_size / 10);
    m_feature_points.push_back(vec);
  }

  m_proj_matrix = cv::Mat(3, 3, cv::DataType<double>::type, 0.0);
  m_proj_matrix.at<double>(0, 0) = m_focal_length;
  m_proj_matrix.at<double>(1, 1) = m_focal_length;
  m_proj_matrix.at<double>(0, 2) = static_cast<double>(m_image_width) / 2.0;
  m_proj_matrix.at<double>(1, 2) = static_cast<double>(m_image_height) / 2.0;
  m_proj_matrix.at<double>(2, 2) = 1;
}

/// @todo Write visibleKeypoints function
std::vector<cv::KeyPoint> SimFeatureTracker::VisibleKeypoints(double time)
{
  Eigen::Vector3d body_pos = m_truth->GetBodyPosition(time);
  Eigen::Quaterniond body_ang = m_truth->GetBodyAngularPosition(time);
  Eigen::Quaterniond cam_ang = body_ang * m_ang_offset;
  Eigen::Matrix3d cam_ang_eig_mat = cam_ang.toRotationMatrix();
  Eigen::Vector3d cam_plane_vec = cam_ang * Eigen::Vector3d(0, 0, 1);

  cv::Mat cam_ang_cv_mat(3, 3, cv::DataType<double>::type);
  cam_ang_cv_mat.at<double>(0, 0) = cam_ang_eig_mat(0, 0);
  cam_ang_cv_mat.at<double>(1, 0) = cam_ang_eig_mat(1, 0);
  cam_ang_cv_mat.at<double>(2, 0) = cam_ang_eig_mat(2, 0);

  cam_ang_cv_mat.at<double>(0, 1) = cam_ang_eig_mat(0, 1);
  cam_ang_cv_mat.at<double>(1, 1) = cam_ang_eig_mat(1, 1);
  cam_ang_cv_mat.at<double>(2, 1) = cam_ang_eig_mat(2, 1);

  cam_ang_cv_mat.at<double>(0, 2) = cam_ang_eig_mat(0, 2);
  cam_ang_cv_mat.at<double>(1, 2) = cam_ang_eig_mat(1, 2);
  cam_ang_cv_mat.at<double>(2, 2) = cam_ang_eig_mat(2, 2);

  // Creating Rodrigues rotation matrix
  cv::Mat rot_vec(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(cam_ang_cv_mat, rot_vec);

  Eigen::Vector3d camPos = body_pos + body_ang * m_pos_offset;
  cv::Mat T(3, 1, cv::DataType<double>::type);
  T.at<double>(0) = camPos[0];
  T.at<double>(1) = camPos[1];
  T.at<double>(2) = camPos[2];

  // Create zero distortion
  /// @todo grab this from input
  cv::Mat distortion(4, 1, cv::DataType<double>::type);
  distortion.at<double>(0) = 0;
  distortion.at<double>(1) = 0;
  distortion.at<double>(2) = 0;
  distortion.at<double>(3) = 0;

  // Project points
  std::vector<cv::Point2d> projected_points;

  /// @todo 2D projection is not correct
  cv::projectPoints(m_feature_points, rot_vec, T, m_proj_matrix, distortion, projected_points);

  // Convert to feature points
  std::vector<cv::KeyPoint> projected_features;
  for (unsigned int i = 0; i < projected_points.size(); ++i) {
    cv::Point3d pointCV = m_feature_points[i];
    Eigen::Vector3d pointEig(pointCV.x, pointCV.y, pointCV.z);

    // Check that point is in front of camera plane
    if (cam_plane_vec.dot(pointEig) > 0) {
      cv::KeyPoint feat;
      feat.pt.x = projected_points[i].x;
      feat.pt.y = projected_points[i].y;
      feat.class_id = i;
      if (
        feat.pt.x > 0 &&
        feat.pt.y > 0 &&
        feat.pt.x < m_image_width &&
        feat.pt.y < m_image_height)
      {
        projected_features.push_back(feat);
      }
    }
  }

  return projected_features;
}

/// @todo Write generateMessages function
std::vector<std::shared_ptr<SimFeatureTrackerMessage>> SimFeatureTracker::GenerateMessages(
  std::vector<double> message_times, unsigned int sensor_id)
{
  m_logger->Log(
    LogLevel::INFO, "Generating " + std::to_string(message_times.size()) + " measurements");

  std::map<unsigned int, std::vector<FeatureTrack>> feature_track_map;
  std::vector<std::shared_ptr<SimFeatureTrackerMessage>> tracker_messages;

  for (unsigned int frame_id = 0; frame_id < message_times.size(); ++frame_id) {
    std::vector<std::vector<FeatureTrack>> feature_tracks;

    std::vector<cv::KeyPoint> key_points = VisibleKeypoints(message_times[frame_id]);

    for (auto & key_point : key_points) {
      auto feature_track = FeatureTrack{frame_id, key_point};
      feature_track_map[key_point.class_id].push_back(feature_track);
    }

    // Update MSCKF on features no longer detected
    for (auto it = feature_track_map.cbegin(); it != feature_track_map.cend(); ) {
      const auto & feature_track = it->second;
      /// @todo get constant from tracker
      if ((feature_track.size() > 1) &&
        ((feature_track.back().frame_id < frame_id) || (feature_track.size() >= 20)))
      {
        // This feature does not exist in the latest frame
        feature_tracks.push_back(feature_track);
        it = feature_track_map.erase(it);
      } else {
        ++it;
      }
    }
    auto tracker_message = std::make_shared<SimFeatureTrackerMessage>();
    tracker_message->m_feature_tracks = feature_tracks;
    tracker_message->m_time = message_times[frame_id];
    tracker_message->m_tracker_id = m_id;
    tracker_message->m_sensor_id = sensor_id;
    tracker_message->m_sensor_type = SensorType::Tracker;
    tracker_messages.push_back(tracker_message);
  }
  return tracker_messages;
}

void SimFeatureTracker::Callback(
  double time, unsigned int camera_id,
  std::shared_ptr<SimFeatureTrackerMessage> msg)
{
  m_msckf_updater.UpdateEKF(time, camera_id, msg->m_feature_tracks);
}
