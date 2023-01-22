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

#include "ekf/MSCKF.hpp"


MSCKF::MSCKF()
{}


void MSCKF::processFrame(double time, unsigned int frameID)
{
  // Propagate to current time

  // Insert state into history
  // std::rotate(m_stateHistory.rbegin(), m_stateHistory.rbegin() + 1, m_stateHistory.rend());
  // m_stateHistory[0] = m_ekf->GetBodyState();

  // Map frameID to copy of current state
}

// void MSCKF::processTracks(Tracker::FeatureTracks featureTracks)
// {
//   // Retrieve states
// }
