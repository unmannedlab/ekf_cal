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

#include "trackers/sim/sim_fiducial_tracker.hpp"

// m_intrinsics = params.tracker_params.intrinsics;


SimFiducialTracker::SimFiducialTracker(
  SimFiducialTracker::Parameters params,
  std::shared_ptr<TruthEngine> truthEngine,
  std::string log_file_directory,
  bool data_logging_on)
: FiducialTracker(params.fiducial_params),
  m_data_logger(log_file_directory, "feature_points.csv")
{
  m_no_errors = params.no_errors;
  m_truth = truthEngine;

  m_data_logger.DefineHeader("Feature,x,y,z\n");
  m_data_logger.SetLogging(data_logging_on);
  m_pos_error = params.fiducial_params.pos_error;
  m_ang_error = params.fiducial_params.ang_error;

  m_intrinsics = params.fiducial_params.intrinsics;
}

bool SimFiducialTracker::IsBoardVisible(double time)
{
  /// @todo(jhartzer): calculate visibility
  return true;
}


std::vector<std::shared_ptr<SimFiducialTrackerMessage>> SimFiducialTracker::GenerateMessages(
  std::vector<double> message_times, int sensor_id)
{
  m_logger->Log(
    LogLevel::INFO, "Generating " + std::to_string(message_times.size()) + " measurements");

  std::vector<std::shared_ptr<SimFiducialTrackerMessage>> fiducial_tracker_messages;

  BoardTrack board_track;
  for (int frame_id = 0; static_cast<unsigned int>(frame_id) < message_times.size(); ++frame_id) {
    std::vector<std::vector<FeatureTrack>> feature_tracks;

    if (IsBoardVisible(message_times[frame_id])) {
      BoardDetection board_detection;
      /// @todo(jhartzer): calculate detection
      board_track.push_back(board_detection);
    } else {
      if (board_track.size() == 1) {
        board_track.clear();
      } else {
        auto tracker_message = std::make_shared<SimFiducialTrackerMessage>();
        tracker_message->m_board_track = board_track;
        tracker_message->m_time = message_times[frame_id];
        tracker_message->m_tracker_id = m_id;
        tracker_message->m_sensor_id = sensor_id;
        tracker_message->m_sensor_type = SensorType::Tracker;
        fiducial_tracker_messages.push_back(tracker_message);
      }
    }
  }
  return fiducial_tracker_messages;
}

void SimFiducialTracker::Callback(
  double time, unsigned int camera_id,
  std::shared_ptr<SimFiducialTrackerMessage> msg)
{
  m_fiducial_updater.UpdateEKF(time, camera_id, msg->m_board_track, m_intrinsics);
}

void SimFiducialTracker::SetTrueOffsets(
  Eigen::Vector3d pos_c_in_b_true,
  Eigen::Quaterniond ang_c_to_b_true)
{
  m_pos_c_in_b_true = pos_c_in_b_true;
  m_ang_c_to_b_true = ang_c_to_b_true;
}
