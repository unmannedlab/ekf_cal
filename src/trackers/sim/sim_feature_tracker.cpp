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
  std::shared_ptr<TruthEngine> truth_engine)
: FeatureTracker(params.tracker_params)
{
  m_px_error = params.tracker_params.px_error;
  m_no_errors = params.no_errors;
  m_feature_count = params.feature_count;
  m_max_track_length = params.tracker_params.max_track_length;
  m_truth = truth_engine;
}

std::vector<cv::KeyPoint> SimFeatureTracker::GetVisibleKeypoints(double time) const
{
  Eigen::Vector3d pos_b_in_l = m_truth->GetBodyPosition(time);
  Eigen::Quaterniond ang_b_to_l = m_truth->GetBodyAngularPosition(time);
  Eigen::Vector3d pos_c_in_b = m_truth->GetCameraPosition(m_camera_id);
  Eigen::Quaterniond ang_c_to_b = m_truth->GetCameraAngularPosition(m_camera_id);
  Eigen::Matrix3d rot_c_to_l = (ang_b_to_l * ang_c_to_b).toRotationMatrix();
  Eigen::Matrix3d rot_l_to_c = rot_c_to_l.transpose();

  // Create OpenCV rotation matrix
  cv::Mat ang_l_to_c_cv(3, 3, cv::DataType<double>::type);
  EigenMatrixToCv(rot_l_to_c, ang_l_to_c_cv);

  // Creating Rodrigues rotation matrix
  cv::Mat r_vec(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(ang_l_to_c_cv, r_vec);

  Eigen::Vector3d pos_l_in_c = rot_l_to_c * (-(pos_b_in_l + ang_b_to_l * pos_c_in_b));
  Eigen::Vector3d pos_c_in_l = pos_b_in_l + ang_b_to_l * pos_c_in_b;

  cv::Mat t_vec(3, 1, cv::DataType<double>::type);
  t_vec.at<double>(0) = pos_l_in_c[0];
  t_vec.at<double>(1) = pos_l_in_c[1];
  t_vec.at<double>(2) = pos_l_in_c[2];

  // Create intrinsic matrices
  Intrinsics intrinsics = m_truth->GetCameraIntrinsics(m_camera_id);
  cv::Mat camera_matrix = intrinsics.ToCameraMatrix();
  cv::Mat distortion = intrinsics.ToDistortionVector();

  // Project points
  std::vector<cv::Point2d> projected_points;

  std::vector<cv::Point3d> feature_points = m_truth->GetFeatures();
  cv::projectPoints(feature_points, r_vec, t_vec, camera_matrix, distortion, projected_points);
  std::vector<cv::KeyPoint> projected_features =
    FilterInvisiblePoints(feature_points, projected_points, rot_c_to_l, pos_c_in_l, intrinsics);

  if (m_feature_count > projected_features.size()) {
    auto new_feature_count = m_feature_count - static_cast<unsigned int>(projected_features.size());
    feature_points = m_truth->GenerateVisibleFeatures(time, m_camera_id, new_feature_count);
    cv::projectPoints(feature_points, r_vec, t_vec, camera_matrix, distortion, projected_points);
    projected_features =
      FilterInvisiblePoints(feature_points, projected_points, rot_c_to_l, pos_c_in_l, intrinsics);
  }

  return projected_features;
}

std::vector<cv::KeyPoint> SimFeatureTracker::FilterInvisiblePoints(
  const std::vector<cv::Point3d> & feature_points,
  const std::vector<cv::Point2d> & projected_points,
  const Eigen::Matrix3d & rot_c_to_l,
  const Eigen::Vector3d & pos_c_in_l,
  const Intrinsics & intrinsics
) const
{
  // Convert to feature points
  Eigen::Vector3d cam_plane_vec = rot_c_to_l * Eigen::Vector3d(0, 0, 1);
  std::vector<cv::KeyPoint> projected_features;
  for (unsigned int i = 0; i < projected_points.size(); ++i) {
    cv::Point3d point_cv = feature_points[i];
    Eigen::Vector3d point_eig(
      point_cv.x - pos_c_in_l[0],
      point_cv.y - pos_c_in_l[1],
      point_cv.z - pos_c_in_l[2]);

    // Check that point is in front of camera plane and within sensor limits
    if (
      cam_plane_vec.dot(point_eig) > 0 &&
      projected_points[i].x >= 0 &&
      projected_points[i].y >= 0 &&
      projected_points[i].x <= intrinsics.width &&
      projected_points[i].y <= intrinsics.height)
    {
      cv::KeyPoint feat;
      feat.class_id = static_cast<int>(i);
      if (m_no_errors) {
        feat.pt.x = static_cast<float>(projected_points[i].x);
        feat.pt.y = static_cast<float>(projected_points[i].y);
      } else {
        feat.pt.x = static_cast<float>(round(projected_points[i].x));
        feat.pt.y = static_cast<float>(round(projected_points[i].y));
      }
      projected_features.push_back(feat);
    }
  }
  return projected_features;
}

std::shared_ptr<SimFeatureTrackerMessage> SimFeatureTracker::GenerateMessage(
  double message_time,
  unsigned int frame_id
)
{
  FeatureTracks feature_tracks;

  std::vector<cv::KeyPoint> key_points = GetVisibleKeypoints(message_time);

  for (auto & key_point : key_points) {
    auto feature_point = FeaturePoint{frame_id, message_time, key_point};
    m_feature_points_map[static_cast<unsigned int>(key_point.class_id)].push_back(feature_point);
  }

  // Update MSCKF on features no longer detected
  for (auto feat_it = m_feature_points_map.cbegin(); feat_it != m_feature_points_map.cend(); ) {
    const auto & feature_points = feat_it->second;
    if ((feature_points.back().frame_id < frame_id) ||
      (feature_points.size() >= m_max_track_length))
    {
      // This feature does not exist in the latest frame
      if (feature_points.size() >= m_min_track_length) {
        FeatureTrack feature_track;
        feature_track.track = feature_points;
        auto feature_point_cv = m_truth->GetFeature(feature_points[0].key_point.class_id);
        feature_track.true_feature_position.x() = feature_point_cv.x;
        feature_track.true_feature_position.y() = feature_point_cv.y;
        feature_track.true_feature_position.z() = feature_point_cv.z;

        feature_tracks.push_back(feature_track);
      }
      feat_it = m_feature_points_map.erase(feat_it);
    } else {
      ++feat_it;
    }
  }

  auto tracker_message = std::make_shared<SimFeatureTrackerMessage>();
  tracker_message->feature_tracks = feature_tracks;
  tracker_message->time = message_time;
  tracker_message->tracker_id = m_id;
  tracker_message->sensor_id = m_camera_id;
  tracker_message->sensor_type = SensorType::Tracker;
  return tracker_message;
}

void SimFeatureTracker::Callback(const double time, const SimFeatureTrackerMessage & msg)
{
  m_msckf_updater.UpdateEKF(
    *m_ekf,
    time,
    msg.feature_tracks,
    m_px_error);
}
