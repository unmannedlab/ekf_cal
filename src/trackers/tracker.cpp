// Copyright 2024 Jacob Hartzer
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

#include "trackers/tracker.hpp"

// Initialize static variable
unsigned int Tracker::m_tracker_count = 0;

Tracker::Tracker(Tracker::Parameters params)
: m_id(++m_tracker_count),
  m_camera_id(params.camera_id),
  m_intrinsics(params.intrinsics),
  m_min_track_length(params.min_track_length),
  m_max_track_length(params.max_track_length),
  m_ekf(params.ekf),
  m_logger(params.logger)
{}

unsigned int Tracker::GetID()
{
  return m_id;
}
