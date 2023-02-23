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

#ifndef EKF__UPDATE__MSCKFUPDATER_HPP_
#define EKF__UPDATE__MSCKFUPDATER_HPP_

#include "ekf/update/Updater.hpp"
#include "ekf/EKF.hpp"

///
/// @class MsckfUpdater
/// @brief EKF Updater Class for MSCKF Camera Measurements
///
class MsckfUpdater : public Updater
{
public:
  ///
  /// @brief MSCKF EKF Updater constructor
  /// @param camID Camera sensor ID
  ///
  explicit MsckfUpdater(unsigned int camID);

  ///
  /// @brief
  /// @param augmentedStates
  /// @param frameID
  ///
  AugmentedState matchState(unsigned int frameID);

  ///
  /// @brief
  /// @param cameraID
  /// @param featureTracks
  ///
  void updateEKF(unsigned int cameraID, FeatureTracks featureTracks);

  ///
  /// @brief Refresh internal states with EKF values
  ///
  void RefreshStates();

private:
  static const Eigen::Vector3d GRAVITY;
  Eigen::Vector3d m_bodyPos {0.0, 0.0, 0.0};
  Eigen::Vector3d m_bodyVel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_bodyAcc {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_bodyAngPos {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_bodyAngVel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_bodyAngAcc {0.0, 0.0, 0.0};
  Eigen::Vector3d m_posOffset {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_angOffset {1.0, 0.0, 0.0, 0.0};
  std::vector<AugmentedState> m_augStates {};
};

#endif  // EKF__UPDATE__MSCKFUPDATER_HPP_
