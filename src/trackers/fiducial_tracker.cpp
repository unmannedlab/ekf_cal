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

#include "trackers/fiducial_tracker.hpp"

FiducialTracker::FiducialTracker(FiducialTracker::Parameters params)
: m_fiducial_updater(
    params.sensor_id,
    params.output_directory,
    params.data_logging_on
)
{
  m_camera_id = params.sensor_id;
  m_intrinsics = params.intrinsics;
  m_detector_type = params.detector_type;
}

void FiducialTracker::Track(
  double time,
  int frame_id,
  cv::Mat & img_in,
  cv::Mat & img_out,
  BoardTrack board_track)
{}

unsigned int FiducialTracker::GetID()
{
  return m_id;
}
