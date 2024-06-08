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

#include "sensors/camera.hpp"

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "ekf/ekf.hpp"
#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/camera_message.hpp"
#include "sensors/sensor.hpp"
#include "trackers/feature_tracker.hpp"
#include "trackers/fiducial_tracker.hpp"
#include "utility/math_helper.hpp"


/// @todo add detector/extractor parameters to input
Camera::Camera(Camera::Parameters cam_params)
: Sensor(cam_params), m_ekf(cam_params.ekf)
{
  m_rate = cam_params.rate;
  m_intrinsics = cam_params.intrinsics;

  CamState cam_state;
  cam_state.pos_c_in_b = cam_params.pos_c_in_b;
  cam_state.ang_c_to_b = cam_params.ang_c_to_b;
  cam_state.pos_stability = cam_params.pos_stability;
  cam_state.ang_stability = cam_params.ang_stability;
  MinBoundVector(cam_params.variance, 1e-6);

  Eigen::MatrixXd cov = cam_params.variance.asDiagonal();

  m_ekf->RegisterCamera(m_id, cam_state, cov);
}

void Camera::Callback(std::shared_ptr<CameraMessage> camera_message)
{
  m_logger->Log(
    LogLevel::DEBUG, "Camera " + std::to_string(
      camera_message->sensor_id) + " callback called at time = " +
    std::to_string(camera_message->time));

  if (!camera_message->image.empty()) {
    if (!m_trackers.empty() || !m_fiducials.empty()) {
      m_ekf->ProcessModel(camera_message->time);

      unsigned int frameID = GenerateFrameID();

      m_ekf->AugmentState(m_id, frameID);

      for (auto const & track_iter : m_trackers) {
        m_trackers[track_iter.first]->Track(
          camera_message->time, frameID, camera_message->image, m_out_img);
        /// @todo Undistort points post track?
        // cv::undistortPoints();
      }

      for (auto const & fiducial_iter : m_fiducials) {
        m_fiducials[fiducial_iter.first]->Track(
          camera_message->time, frameID, camera_message->image, m_out_img);
      }
    } else {
      m_logger->Log(LogLevel::WARN, "Camera has no trackers ");
    }
  } else {
    m_logger->Log(LogLevel::INFO, "Camera received empty image");
  }
  m_logger->Log(
    LogLevel::DEBUG, "Camera " + std::to_string(m_id) + " callback complete");
}

/// @todo apply similar function to sensor/tracker IDs
unsigned int Camera::GenerateFrameID()
{
  static int frame_id = 0;
  return frame_id++;
}

void Camera::AddTracker(std::shared_ptr<FeatureTracker> tracker)
{
  m_trackers[tracker->GetID()] = tracker;
}

void Camera::AddFiducial(std::shared_ptr<FiducialTracker> fiducial)
{
  m_fiducials[fiducial->GetID()] = fiducial;
}
