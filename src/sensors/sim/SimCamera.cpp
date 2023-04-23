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

#include "sensors/sim/SimCamera.hpp"

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <memory>

#include "infrastructure/sim/TruthEngine.hpp"
#include "utility/sim/SimRNG.hpp"
#include "utility/MathHelper.hpp"

SimCamera::SimCamera(
  Parameters params,
  std::shared_ptr<TruthEngine> truth_engine)
: Camera(params.cam_params)
{
  m_time_bias = params.time_bias;
  m_time_skew = params.time_skew;
  m_time_error = std::max(params.time_error, 1e-9);
  m_pos_offset = params.pos_offset;
  m_ang_offset = params.ang_offset;
  m_truth = truth_engine;
}

std::vector<double> SimCamera::GenerateMessageTimes(double max_time)
{
  std::vector<double> message_times;
  double num_measurements = max_time * m_rate / (1 + m_time_skew);
  for (unsigned int i = 0; i < num_measurements; ++i) {
    double measurement_time = (1.0 + m_time_skew) / m_rate * static_cast<double>(i);
    message_times.push_back(measurement_time + m_rng.NormRand(m_time_bias, m_time_error));
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
  m_trackers[tracker->GetID()] = tracker;
}

void SimCamera::Callback(std::shared_ptr<SimCameraMessage> sim_camera_message)
{
  unsigned int frame_id = GenerateFrameID();

  m_ekf->AugmentState(m_id, frame_id);

  if (sim_camera_message->m_feature_track_message->m_feature_tracks.size() > 0) {
    m_trackers[sim_camera_message->m_feature_track_message->m_tracker_id]->Callback(
      sim_camera_message->m_time, sim_camera_message->m_sensor_id,
      sim_camera_message->m_feature_track_message);
  }
}
