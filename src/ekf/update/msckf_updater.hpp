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

#ifndef EKF__UPDATE__MSCKF_UPDATER_HPP_
#define EKF__UPDATE__MSCKF_UPDATER_HPP_

#include <eigen3/Eigen/Eigen>

#include <memory>
#include <string>
#include <vector>

#include "ekf/types.hpp"
#include "ekf/update/updater.hpp"
#include "infrastructure/data_logger.hpp"

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
  /// @param data_log_rate Maximum average rate to log data
  /// @param min_feat_dist Closest feature distance to consider
  /// @param logger Debug logger pointer
  ///
  explicit MsckfUpdater(
    int cam_id,
    std::string log_file_directory,
    bool data_logging_on,
    double data_log_rate,
    double min_feat_dist,
    std::shared_ptr<DebugLogger> logger
  );

  ///
  /// @brief Triangulate feature seen from multiple camera frames
  /// @param ekf EKF pointer
  /// @param feature_track Single feature track
  /// @param pos_f_in_l Output estimate of feature position in camera frame given observations
  /// @return If triangulation was successful
  ///
  bool TriangulateFeature(
    std::shared_ptr<EKF> ekf,
    std::vector<FeaturePoint> & feature_track,
    Eigen::Vector3d & pos_f_in_l);

  ///
  /// @brief EKF updater function
  /// @param ekf EKF pointer
  /// @param time Time of update
  /// @param feature_tracks Feature tracks to be used for state update
  /// @param px_error Standard deviation of pixel error
  ///
  void UpdateEKF(
    std::shared_ptr<EKF> ekf,
    double time,
    FeatureTracks feature_tracks,
    double px_error);

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
  DataLogger m_msckf_logger;
  DataLogger m_triangulation_logger;
  double m_min_feat_dist{1.0};
  double m_max_feat_dist{100.0};  /// @todo: Get from input
};

#endif  // EKF__UPDATE__MSCKF_UPDATER_HPP_
