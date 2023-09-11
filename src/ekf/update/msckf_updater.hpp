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

#ifndef EKF__UPDATE__MSCKF_UPDATER_HPP_
#define EKF__UPDATE__MSCKF_UPDATER_HPP_

#include <string>
#include <vector>

#include "ekf/ekf.hpp"
#include "ekf/update/updater.hpp"
#include "infrastructure/data_logger.hpp"
#include "sensors/types.hpp"

///
/// @class MsckfUpdater
/// @brief EKF Updater Class for MSCKF Camera Measurements
///
class MsckfUpdater : public Updater
{
public:
  ///
  /// @brief MSCKF EKF Updater constructor
  /// @param cam_id Camera sensor ID
  /// @param log_file_directory Directory to save log files
  /// @param data_logging_on Flag to enable data logging
  ///
  explicit MsckfUpdater(
    int cam_id,
    std::string log_file_directory,
    bool data_logging_on);

  ///
  /// @brief Find augmented state matching a frame ID
  /// @param frame_id Desired frame ID
  /// @return Matching augmented state
  ///
  AugmentedState MatchState(int frame_id);

  ///
  /// @brief Triangulate feature seen from multiple camera frames
  /// @param feature_track Single feature track
  /// @return Estimate of feature position in camera frame given observations
  ///
  Eigen::Vector3d TriangulateFeature(std::vector<FeatureTrack> & feature_track);

  ///
  /// @brief
  /// @param time Time of update
  /// @param camera_id ID of camera associated with update
  /// @param feature_tracks Feature tracks to be used for state update
  /// @param px_error Standard deviation of pixel error
  ///
  void UpdateEKF(
    double time,
    int camera_id,
    FeatureTracks feature_tracks,
    double px_error);

  ///
  /// @brief Refresh internal states with EKF values
  ///
  void RefreshStates();


  ///
  /// @brief Computes the derivative of raw distorted to normalized coordinate.
  /// @param uv_norm Normalized coordinates we wish to distort
  /// @param intrinsics Camera intrinsics
  /// @param H_d Derivative of measurement z in respect to normalized
  ///
  void distortion_jacobian(
    const Eigen::Vector2d & uv_norm,
    Intrinsics intrinsics,
    Eigen::MatrixXd & H_d);

  ///
  /// @brief Function to calculate jacobian for camera projection function
  /// @param position Position in camera coordinates
  /// @param jacobian Resulting camera projection jacobian
  ///
  void projection_jacobian(const Eigen::Vector3d & position, Eigen::MatrixXd & jacobian);

private:
  Eigen::Vector3d m_body_pos {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_vel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_acc {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_b_to_g {1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_ang_vel {0.0, 0.0, 0.0};
  Eigen::Vector3d m_body_ang_acc {0.0, 0.0, 0.0};

  Eigen::Vector3d m_pos_c_in_b {0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_c_to_b {1.0, 0.0, 0.0, 0.0};
  std::vector<AugmentedState> m_aug_states {};

  DataLogger m_data_logger;
  DataLogger m_triangulation_logger;
  unsigned int m_image_width {640};
  unsigned int m_image_height {480};
  double m_focal_length {1};
  double m_pixel_size {0.010};
};

#endif  // EKF__UPDATE__MSCKF_UPDATER_HPP_
