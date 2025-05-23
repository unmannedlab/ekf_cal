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
  /// @param is_extrinsic Camera extrinsic calibration flag
  /// @param log_file_directory Directory to save log files
  /// @param data_log_rate Maximum average rate to log data
  /// @param min_feat_dist Closest feature distance to consider
  /// @param logger Debug logger pointer
  ///
  explicit MsckfUpdater(
    unsigned int cam_id,
    bool is_extrinsic,
    const std::string & log_file_directory,
    double data_log_rate,
    double min_feat_dist,
    std::shared_ptr<DebugLogger> logger
  );

  ///
  /// @brief Triangulate feature seen from multiple camera frames
  /// @param ekf EKF address
  /// @param local_time Measurement in EKF time
  /// @param feature_track Single feature track
  /// @param pos_f_in_l Output estimate of feature position in camera frame given observations
  /// @return If triangulation was successful
  ///
  bool TriangulateFeature(
    const double local_time,
    EKF & ekf,
    const FeatureTrack & feature_track,
    Eigen::Vector3d & pos_f_in_l);

  ///
  /// @brief EKF updater function
  /// @param ekf EKF address
  /// @param time Time of update
  /// @param feature_tracks Feature tracks to be used for state update
  /// @param px_error Standard deviation of pixel error
  ///
  void UpdateEKF(
    EKF & ekf,
    const double time,
    const FeatureTracks & feature_tracks,
    double px_error);

  ///
  /// @brief Computes the derivative of raw distorted to normalized coordinate.
  /// @param xy_norm Normalized coordinates we wish to distort
  /// @param intrinsics Camera intrinsics
  /// @param H_d Derivative of measurement z in respect to normalized
  ///
  static void DistortionJacobian(
    const Eigen::Vector2d & xy_norm,
    const Intrinsics & intrinsics,
    Eigen::MatrixXd & H_d);

  ///
  /// @brief Function to calculate jacobian for camera projection function
  /// @param position Position in camera coordinates
  /// @param jacobian Resulting camera projection jacobian
  ///
  static void ProjectionJacobian(const Eigen::Vector3d & position, Eigen::MatrixXd & jacobian);

  ///
  /// @brief Project a 3D position in the camera frame to a 2D bearing
  /// @param pos_f_in_c Feature position in the camera frame
  /// @return Projected 2D position
  ///
  static Eigen::Vector2d Project(const Eigen::Vector3d pos_f_in_c);

  ///
  /// @brief Distort a normalized camera coordinate using intrinsics
  /// @param xy_norm Normalized camera coordinate
  /// @param intrinsics camera intrinsics
  /// @return Distorted XY coordinate
  ///
  static Eigen::Vector2d Distort(const Eigen::Vector2d & xy_norm, const Intrinsics & intrinsics);

private:
  bool m_is_cam_extrinsic;
  DataLogger m_msckf_logger;
  DataLogger m_triangulation_logger;
  double m_min_feat_dist{1.0};
  double m_max_feat_dist{100.0};  /// @todo: Get from input
  bool m_is_first_estimate{true};
  bool m_use_true_triangulation{true};
  Intrinsics m_intrinsics;
  Eigen::Vector3d m_pos_c_in_b{0.0, 0.0, 0.0};
  Eigen::Quaterniond m_ang_c_to_b{1.0, 0.0, 0.0, 0.0};
};

#endif  // EKF__UPDATE__MSCKF_UPDATER_HPP_
