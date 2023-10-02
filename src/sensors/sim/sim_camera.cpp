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

#include "sensors/sim/sim_camera.hpp"

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <memory>

#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/sim/sim_camera_message.hpp"
#include "utility/sim/sim_rng.hpp"
#include "utility/math_helper.hpp"

SimCamera::SimCamera(
  Parameters params,
  std::shared_ptr<TruthEngine> truth_engine)
: Camera(params.cam_params)
{
  m_time_bias_error = params.time_bias_error;
  m_time_skew_error = params.time_skew_error;
  m_time_error = params.time_error;
  m_pos_error = params.pos_error;
  m_ang_error = params.ang_error;
  m_no_errors = params.no_errors;
  m_truth = truth_engine;

  m_pos_c_in_b_true[0] = m_rng.NormRand(0.0, m_pos_error[0]);
  m_pos_c_in_b_true[1] = m_rng.NormRand(0.0, m_pos_error[1]);
  m_pos_c_in_b_true[2] = m_rng.NormRand(0.0, m_pos_error[2]);
  double ang_c_to_b_true_r = m_rng.NormRand(0.0, m_ang_error[0]);
  double ang_c_to_b_true_p = m_rng.NormRand(0.0, m_ang_error[1]);
  double ang_c_to_b_true_y = m_rng.NormRand(0.0, m_ang_error[2]);
  m_ang_c_to_b_true =
    Eigen::AngleAxisd(ang_c_to_b_true_y, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(ang_c_to_b_true_p, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(ang_c_to_b_true_r, Eigen::Vector3d::UnitX());
}

std::vector<double> SimCamera::GenerateMessageTimes(double max_time)
{
  unsigned int num_measurements =
    static_cast<int>(std::floor(max_time * m_rate / (1 + m_time_skew_error)));

  m_logger->Log(
    LogLevel::INFO, "Generating " + std::to_string(num_measurements) + " Camera measurements");

  double time_init = m_no_errors ? 0 : m_rng.UniRand(0.0, 1.0 / m_rate);

  std::vector<double> message_times;
  for (unsigned int i = 0; i < num_measurements; ++i) {
    double measurement_time =
      (1.0 + m_time_skew_error) / m_rate * static_cast<double>(i) + time_init;
    if (!m_no_errors) {
      measurement_time += m_rng.NormRand(m_time_bias_error, m_time_error);
    }
    message_times.push_back(measurement_time);
  }
  return message_times;
}

std::vector<std::shared_ptr<SimCameraMessage>> SimCamera::GenerateMessages(double max_time)
{
  std::vector<std::shared_ptr<SimCameraMessage>> messages;
  std::vector<double> message_times = GenerateMessageTimes(max_time);

  for (auto const & trk_iter : m_trackers) {
    auto trk_msgs = m_trackers[trk_iter.first]->GenerateMessages(message_times, m_id);
    cv::Mat blank_img;
    for (auto trk_msg : trk_msgs) {
      auto cam_msg = std::make_shared<SimCameraMessage>(blank_img);

      cam_msg->m_feature_track_message = trk_msg;
      cam_msg->m_sensor_id = m_id;
      cam_msg->m_time = trk_msg->m_time;
      cam_msg->m_sensor_type = SensorType::Camera;

      messages.push_back(cam_msg);
    }
  }
  return messages;
}

void SimCamera::AddTracker(std::shared_ptr<SimFeatureTracker> tracker)
{
  tracker->SetTrueOffsets(m_pos_c_in_b_true, m_ang_c_to_b_true);
  m_trackers[tracker->GetID()] = tracker;
}

void SimCamera::Callback(std::shared_ptr<SimCameraMessage> sim_camera_message)
{
  int frame_id = GenerateFrameID();

  m_ekf->AugmentState(m_id, frame_id);

  if (sim_camera_message->m_feature_track_message->m_feature_tracks.size() > 0) {
    m_trackers[sim_camera_message->m_feature_track_message->m_tracker_id]->Callback(
      sim_camera_message->m_time, sim_camera_message->m_sensor_id,
      sim_camera_message->m_feature_track_message);
  }
}
