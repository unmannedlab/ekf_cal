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
#include <string>
#include <cmath>
#include <memory>
#include <utility>

#include <opencv2/opencv.hpp>

#include "ekf/ekf.hpp"
#include "infrastructure/debug_logger.hpp"
#include "infrastructure/sim/truth_engine.hpp"
#include "sensors/sim/sim_camera_message.hpp"
#include "ekf/types.hpp"
#include "trackers/sim/sim_feature_tracker_message.hpp"
#include "trackers/sim/sim_feature_tracker.hpp"
#include "trackers/sim/sim_fiducial_tracker.hpp"
#include "trackers/sim/sim_fiducial_tracker_message.hpp"
#include "utility/sim/sim_rng.hpp"
#include "utility/type_helper.hpp"


SimCamera::SimCamera(
  Parameters params,
  std::shared_ptr<TruthEngine> truth_engine)
: Camera(params.cam_params), SimSensor(params)
{
  m_pos_error = params.pos_error;
  m_ang_error = params.ang_error;
  m_truth = truth_engine;

  // Set true camera values
  Eigen::Vector3d pos_c_in_b_true;
  Eigen::Quaterniond ang_c_to_b_true;
  if (m_no_errors) {
    pos_c_in_b_true = params.cam_params.pos_c_in_b;
    ang_c_to_b_true = params.cam_params.ang_c_to_b;
  } else {
    pos_c_in_b_true = m_rng.VecNormRand(params.cam_params.pos_c_in_b, params.pos_error);
    ang_c_to_b_true = m_rng.QuatNormRand(params.cam_params.ang_c_to_b, params.ang_error);
  }

  truth_engine->SetCameraPosition(m_id, pos_c_in_b_true);
  truth_engine->SetCameraAngularPosition(m_id, ang_c_to_b_true);
  truth_engine->SetCameraIntrinsics(m_id, params.cam_params.intrinsics);
}

std::vector<std::shared_ptr<SimCameraMessage>> SimCamera::GenerateMessages()
{
  std::vector<std::shared_ptr<SimCameraMessage>> messages;
  std::vector<double> measurement_times = GenerateMeasurementTimes(m_rate);

  // Tracker Messages
  for (auto const & trk_iter : m_trackers) {
    auto trk_msgs = m_trackers[trk_iter.first]->GenerateMessages(measurement_times, m_id);
    cv::Mat blank_img;
    for (auto trk_msg : trk_msgs) {
      auto cam_msg = std::make_shared<SimCameraMessage>(blank_img);

      cam_msg->feature_track_message = trk_msg;
      cam_msg->sensor_id = m_id;
      cam_msg->time = trk_msg->time;
      cam_msg->sensor_type = SensorType::Camera;

      messages.push_back(cam_msg);
    }
  }

  // Fiducial Messages
  for (auto const & fid_iter : m_fiducials) {
    auto fid_msgs = m_fiducials[fid_iter.first]->GenerateMessages(measurement_times, m_id);
    cv::Mat blank_img;
    for (auto fid_msg : fid_msgs) {
      auto cam_msg = std::make_shared<SimCameraMessage>(blank_img);

      cam_msg->fiducial_track_message = fid_msg;
      cam_msg->sensor_id = m_id;
      cam_msg->time = fid_msg->time;
      cam_msg->sensor_type = SensorType::Camera;

      messages.push_back(cam_msg);
    }
  }
  return messages;
}

void SimCamera::AddTracker(std::shared_ptr<SimFeatureTracker> tracker)
{
  m_trackers[tracker->GetID()] = tracker;
}

void SimCamera::AddFiducial(std::shared_ptr<SimFiducialTracker> fiducial)
{
  m_fiducials[fiducial->GetID()] = fiducial;
}

void SimCamera::Callback(std::shared_ptr<SimCameraMessage> sim_camera_message)
{
  m_ekf->ProcessModel(sim_camera_message->time);

  int frame_id = GenerateFrameID();

  m_ekf->AugmentStateIfNeeded(m_id, frame_id);

  if (sim_camera_message->feature_track_message != nullptr) {
    if (sim_camera_message->feature_track_message->feature_tracks.size() > 0) {
      m_trackers[sim_camera_message->feature_track_message->tracker_id]->Callback(
        sim_camera_message->time, sim_camera_message->feature_track_message);
    }
  } else if (sim_camera_message->fiducial_track_message != nullptr) {
    m_fiducials[sim_camera_message->fiducial_track_message->tracker_id]->Callback(
      sim_camera_message->time, sim_camera_message->fiducial_track_message);
  }
}
