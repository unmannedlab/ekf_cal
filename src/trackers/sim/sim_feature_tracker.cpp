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
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "trackers/sim/sim_feature_tracker_message.hpp"
#include "utility/constants.hpp"

SimFeatureTracker::SimFeatureTracker(
  SimFeatureTracker::Parameters params,
  std::shared_ptr<TruthEngine> truthEngine,
  std::string log_file_directory,
  bool data_logging_on)
: FeatureTracker(params.tracker_params),
  m_data_logger(log_file_directory, "feature_points.csv")
{
  m_px_error = params.tracker_params.px_error;
  m_pos_offset = params.pos_offset;
  m_ang_offset = params.ang_offset;
  m_no_errors = params.no_errors;
  m_feature_count = params.feature_count;
  m_truth = truthEngine;

  m_data_logger.DefineHeader("Feature,x,y,z\n");
  m_data_logger.SetLogging(data_logging_on);

  m_feature_points.push_back(cv::Point3d(params.room_size, 0, 0));
  m_feature_points.push_back(cv::Point3d(params.room_size, params.room_size / 10, 0));
  m_feature_points.push_back(cv::Point3d(params.room_size, 0, params.room_size / 10));
  m_feature_points.push_back(cv::Point3d(-params.room_size, 0, 0));
  m_feature_points.push_back(cv::Point3d(0, params.room_size, 0));
  m_feature_points.push_back(cv::Point3d(0, -params.room_size, 0));
  m_feature_points.push_back(cv::Point3d(0, 0, params.room_size));
  m_feature_points.push_back(cv::Point3d(params.room_size / 10, 0, params.room_size));
  m_feature_points.push_back(cv::Point3d(0, params.room_size / 10, params.room_size));
  m_feature_points.push_back(cv::Point3d(0, 0, -params.room_size));
  for (unsigned int i = 0; i < m_feature_count; ++i) {
    cv::Point3d vec;
    vec.x = params.room_size;
    /// @todo(jhartzer): Re-enable x-axis randomness
    // vec.x = m_rng.UniRand(-params.room_size, params.room_size);
    vec.y = m_rng.UniRand(-params.room_size, params.room_size);
    vec.z = m_rng.UniRand(-params.room_size / 10, params.room_size / 10);
    m_feature_points.push_back(vec);
  }

  for (unsigned int i = 0; i < m_feature_points.size(); ++i) {
    std::stringstream msg;
    msg << std::to_string(i);
    msg << "," << m_feature_points[i].x;
    msg << "," << m_feature_points[i].y;
    msg << "," << m_feature_points[i].z;
    msg << std::endl;
    m_data_logger.Log(msg.str());
  }

  m_proj_matrix = cv::Mat(3, 3, cv::DataType<double>::type, 0.0);
  m_proj_matrix.at<double>(0, 0) = m_focal_length / m_pixel_size;
  m_proj_matrix.at<double>(1, 1) = m_focal_length / m_pixel_size;
  m_proj_matrix.at<double>(0, 2) = static_cast<double>(m_image_width) / 2.0;
  m_proj_matrix.at<double>(1, 2) = static_cast<double>(m_image_height) / 2.0;
  m_proj_matrix.at<double>(2, 2) = 1;
}

std::vector<cv::KeyPoint> SimFeatureTracker::VisibleKeypoints(double time)
{
  Eigen::Vector3d pos_i_in_g = m_truth->GetBodyPosition(time);
  Eigen::Quaterniond ang_i_to_g = m_truth->GetBodyAngularPosition(time);
  Eigen::Quaterniond ang_c_to_i = m_ang_offset;
  Eigen::Matrix3d ang_g_to_c = (ang_c_to_i * ang_i_to_g).toRotationMatrix().transpose();

  /// @todo put into type helper
  cv::Mat ang_g_to_c_cv(3, 3, cv::DataType<double>::type);
  ang_g_to_c_cv.at<double>(0, 0) = ang_g_to_c(0, 0);
  ang_g_to_c_cv.at<double>(1, 0) = ang_g_to_c(1, 0);
  ang_g_to_c_cv.at<double>(2, 0) = ang_g_to_c(2, 0);

  ang_g_to_c_cv.at<double>(0, 1) = ang_g_to_c(0, 1);
  ang_g_to_c_cv.at<double>(1, 1) = ang_g_to_c(1, 1);
  ang_g_to_c_cv.at<double>(2, 1) = ang_g_to_c(2, 1);

  ang_g_to_c_cv.at<double>(0, 2) = ang_g_to_c(0, 2);
  ang_g_to_c_cv.at<double>(1, 2) = ang_g_to_c(1, 2);
  ang_g_to_c_cv.at<double>(2, 2) = ang_g_to_c(2, 2);

  // Creating Rodrigues rotation matrix
  /// @todo put into type helper
  /// @todo(jhartzer): fix names
  cv::Mat r_vec(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(ang_g_to_c_cv, r_vec);

  Eigen::Vector3d pos_g_in_c = ang_g_to_c * (-(pos_i_in_g + ang_i_to_g * m_pos_offset));

  cv::Mat t_vec(3, 1, cv::DataType<double>::type);
  /// @todo convert to be in C
  t_vec.at<double>(0) = pos_g_in_c[0];
  t_vec.at<double>(1) = pos_g_in_c[1];
  t_vec.at<double>(2) = pos_g_in_c[2];

  // Create zero distortion
  /// @todo grab this from input
  cv::Mat distortion(4, 1, cv::DataType<double>::type, 0.0);

  // Project points
  std::vector<cv::Point2d> projected_points;

  /// @todo 2D projection is not correct
  cv::projectPoints(
    m_feature_points, r_vec, t_vec, m_proj_matrix, distortion, projected_points);

  // Convert to feature points
  std::vector<cv::KeyPoint> projected_features;
  Eigen::Vector3d cam_plane_vec = ang_g_to_c.transpose() * Eigen::Vector3d(0, 0, 1);
  for (unsigned int i = 0; i < projected_points.size(); ++i) {
    cv::Point3d pointCV = m_feature_points[i];
    Eigen::Vector3d pointEig(pointCV.x, pointCV.y, pointCV.z);

    // Check that point is in front of camera plane
    if (cam_plane_vec.dot(pointEig) > 0) {
      if (
        projected_points[i].x >= 0 &&
        projected_points[i].y >= 0 &&
        projected_points[i].x <= m_image_width &&
        projected_points[i].y <= m_image_height)
      {
        cv::KeyPoint feat;
        feat.class_id = i;
        if (m_no_errors) {
          feat.pt.x = projected_points[i].x;
          feat.pt.y = projected_points[i].y;
        } else {
          feat.pt.x = round(projected_points[i].x + m_rng.NormRand(0.0, m_px_error));
          feat.pt.y = round(projected_points[i].y + m_rng.NormRand(0.0, m_px_error));
        }
        projected_features.push_back(feat);
      }
    }
  }

  return projected_features;
}

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
      if ((feature_track.back().frame_id < frame_id) || (feature_track.size() >= 20)) {
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
  m_msckf_updater.UpdateEKF(time, camera_id, msg->m_feature_tracks, m_px_error);
}
