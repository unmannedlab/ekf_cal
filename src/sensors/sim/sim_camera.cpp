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

  m_logger->Log(
    LogLevel::INFO, "Generating " + std::to_string(measurement_times.size()) + " Camera frames");

  for (double measurement_time : measurement_times) {
    int frame_id = GenerateFrameID();
    cv::Mat blank_img;
    auto cam_msg = std::make_shared<SimCameraMessage>(blank_img);
    cam_msg->sensor_id = m_id;
    cam_msg->sensor_type = SensorType::Camera;
    cam_msg->time = measurement_time;
    cam_msg->frame_id = frame_id;

    // Tracker Messages
    for (auto const & trk_iter : m_trackers) {
      auto trk_msg = m_trackers[trk_iter.first]->GenerateMessage(measurement_time, frame_id);
      cam_msg->feature_track_messages.push_back(trk_msg);
    }

    // Fiducial Messages
    for (auto const & fid_iter : m_fiducials) {
      auto fid_msg = m_fiducials[fid_iter.first]->GenerateMessage(measurement_time, frame_id);
      cam_msg->fiducial_track_messages.push_back(fid_msg);
    }

    messages.push_back(cam_msg);
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
  double local_time = m_ekf->CalculateLocalTime(sim_camera_message->time);
  m_ekf->PredictModel(local_time);
  m_ekf->AugmentStateIfNeeded(m_id, sim_camera_message->frame_id);

  for (auto feature_track_message : sim_camera_message->feature_track_messages) {
    if (feature_track_message->feature_tracks.size() > 0) {
      m_trackers[feature_track_message->tracker_id]->Callback(
        sim_camera_message->time, feature_track_message);
    }
  }
  for (auto fiducial_track_message : sim_camera_message->fiducial_track_messages) {
    m_fiducials[fiducial_track_message->tracker_id]->Callback(
      sim_camera_message->time, fiducial_track_message);
  }
}
