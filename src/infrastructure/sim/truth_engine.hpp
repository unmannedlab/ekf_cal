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

#ifndef INFRASTRUCTURE__SIM__TRUTH_ENGINE_HPP_
#define INFRASTRUCTURE__SIM__TRUTH_ENGINE_HPP_

#include <eigen3/Eigen/Eigen>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"
#include "utility/sim/sim_rng.hpp"

///
/// @class TruthEngine
/// @brief Truth for simulation
///
class TruthEngine
{
public:
  ///
  /// @brief TruthEngine constructor
  /// @param max_time Maximum simulation time
  /// @param logger Debug logger pointer
  ///
  explicit TruthEngine(double max_time, std::shared_ptr<DebugLogger> logger);

  virtual ~TruthEngine() = 0;

  ///
  /// @brief True body position getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyPosition(double time) = 0;

  ///
  /// @brief True body velocity getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyVelocity(double time) = 0;

  ///
  /// @brief True body acceleration getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyAcceleration(double time) = 0;

  ///
  /// @brief True body orientation quaternion getter
  /// @param time Simulation time
  ///
  virtual Eigen::Quaterniond GetBodyAngularPosition(double time) = 0;

  ///
  /// @brief True body angular rate getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyAngularRate(double time) = 0;

  ///
  /// @brief True body angular acceleration getter
  /// @param time Simulation time
  ///
  virtual Eigen::Vector3d GetBodyAngularAcceleration(double time) = 0;

  ///
  /// @brief True sensor position getter
  /// @param sensor_id IMU ID
  ///
  Eigen::Vector3d GetImuPosition(unsigned int sensor_id);

  ///
  /// @brief True sensor orientation getter
  /// @param sensor_id IMU ID
  ///
  Eigen::Quaterniond GetImuAngularPosition(unsigned int sensor_id);

  ///
  /// @brief True sensor orientation getter
  /// @param sensor_id IMU ID
  ///
  Eigen::Vector3d GetImuAccelerometerBias(unsigned int sensor_id);

  ///
  /// @brief True sensor orientation getter
  /// @param sensor_id IMU ID
  ///
  Eigen::Vector3d GetImuGyroscopeBias(unsigned int sensor_id);

  ///
  /// @brief True sensor position getter
  /// @param sensor_id Camera ID
  ///
  Eigen::Vector3d GetCameraPosition(unsigned int sensor_id);

  ///
  /// @brief True sensor orientation getter
  /// @param sensor_id Camera ID
  ///
  Eigen::Quaterniond GetCameraAngularPosition(unsigned int sensor_id);

  ///
  /// @brief True camera intrinsics getter
  /// @param sensor_id Camera ID
  ///
  Intrinsics GetCameraIntrinsics(unsigned int sensor_id);

  ///
  /// @brief GPS sensor position getter
  /// @param sensor_id GPS ID
  /// @return Antenna position
  ///
  Eigen::Vector3d GetGpsPosition(unsigned int sensor_id);
  ///
  /// @brief Local frame reference LLA getter
  /// @return Reference LLA
  ///
  Eigen::Vector3d GetLocalPosition();
  ///
  /// @brief Local frame reference heading getter
  /// @return Reference heading
  ///
  double GetLocalHeading();

  ///
  /// @brief True sensor position setter
  /// @param sensor_id sensor ID
  /// @param imu_pos sensor parameter
  ///
  void SetImuPosition(unsigned int sensor_id, Eigen::Vector3d imu_pos);

  ///
  /// @brief True sensor orientation setter
  /// @param sensor_id sensor ID
  /// @param imu_ang_pos sensor parameter
  ///
  void SetImuAngularPosition(unsigned int sensor_id, Eigen::Quaterniond imu_ang_pos);

  ///
  /// @brief True sensor orientation setter
  /// @param sensor_id sensor ID
  /// @param imu_acc_bias sensor parameter
  ///
  void SetImuAccelerometerBias(unsigned int sensor_id, Eigen::Vector3d imu_acc_bias);

  ///
  /// @brief True sensor orientation setter
  /// @param sensor_id sensor ID
  /// @param imu_gyro_bias sensor parameter
  ///
  void SetImuGyroscopeBias(unsigned int sensor_id, Eigen::Vector3d imu_gyro_bias);

  ///
  /// @brief True sensor position setter
  /// @param sensor_id sensor ID
  /// @param cam_pos sensor parameter
  ///
  void SetCameraPosition(unsigned int sensor_id, Eigen::Vector3d cam_pos);

  ///
  /// @brief True sensor orientation setter
  /// @param sensor_id sensor ID
  /// @param cam_ang_pos sensor parameter
  ///
  void SetCameraAngularPosition(unsigned int sensor_id, Eigen::Quaterniond cam_ang_pos);


  ///
  /// @brief True camera intrinsics setter
  /// @param sensor_id Camera ID
  /// @param intrinsics Camera intrinsics
  ///
  void SetCameraIntrinsics(unsigned int sensor_id, Intrinsics intrinsics);

  ///
  /// @brief Fiducial board position setter
  /// @param board_id sensor ID
  /// @param board_position Board position
  ///
  void SetBoardPosition(unsigned int board_id, Eigen::Vector3d board_position);

  ///
  /// @brief Fiducial board orientation setter
  /// @param board_id sensor ID
  /// @param board_orientation Board orientation
  ///
  void SetBoardOrientation(unsigned int board_id, Eigen::Quaterniond board_orientation);

  ///
  /// @brief GPS sensor position setter
  /// @param sensor_id GPS ID
  /// @param gps_position Antenna position
  ///
  void SetGpsPosition(unsigned int sensor_id, Eigen::Vector3d gps_position);

  ///
  /// @brief Local frame reference LLA setter
  /// @param lla_reference Reference LLA
  ///
  void SetLocalPosition(Eigen::Vector3d lla_reference);

  ///
  /// @brief Local frame reference heading setter
  /// @param heading Reference heading
  ///
  void SetLocalHeading(double heading);

  ///
  /// @brief Fiducial board position getter
  /// @param board_id sensor ID
  /// @return Board position
  ///
  Eigen::Vector3d GetBoardPosition(unsigned int board_id);

  ///
  /// @brief Fiducial board orientation getter
  /// @param board_id sensor ID
  /// @return Board orientation
  ///
  Eigen::Quaterniond GetBoardOrientation(unsigned int board_id);

  ///
  /// @brief Write truth data to CSV files
  /// @param data_log_rate Body data rate
  /// @param output_directory Output directory
  ///
  void WriteTruthData(double data_log_rate, std::string output_directory);

  ///
  /// @brief Feature generation function
  /// @param feature_count Number of features to generate
  /// @param room_size Size of room to distribute features in
  /// @param rng Random number generator to use in generation
  ///
  void GenerateGridFeatures();

  ///
  /// @brief Getter function for features
  /// @return Vector of all feature positions
  ///
  std::vector<cv::Point3d> GetFeatures();

  ///
  /// @brief Getter function for a specific feature
  /// @return Feature position
  ///
  cv::Point3d GetFeature(unsigned int feature_id);

  double m_max_time;  ///< @brief Maximum time for truth engine

protected:
  std::shared_ptr<DebugLogger> m_logger;  ///< @brief Debug logger

private:
  std::map<unsigned int, Eigen::Vector3d> m_imu_pos;
  std::map<unsigned int, Eigen::Quaterniond> m_imu_ang_pos;
  std::map<unsigned int, Eigen::Vector3d> m_imu_acc_bias;
  std::map<unsigned int, Eigen::Vector3d> m_imu_gyro_bias;
  std::map<unsigned int, Eigen::Vector3d> m_cam_pos;
  std::map<unsigned int, Eigen::Quaterniond> m_cam_ang_pos;
  std::map<unsigned int, Eigen::Vector3d> m_board_pos;
  std::map<unsigned int, Eigen::Quaterniond> m_board_ang;
  std::map<unsigned int, Intrinsics> m_cam_intrinsics;
  std::vector<cv::Point3d> m_feature_points;
  std::map<unsigned int, cv::Point3d> m_feature_points_map;
  std::map<unsigned int, Eigen::Vector3d> m_gps_pos;
  Eigen::Vector3d m_lla_reference;
  double m_heading;
  double m_room_size{4};
  int m_grid_size{100};
};

#endif  // INFRASTRUCTURE__SIM__TRUTH_ENGINE_HPP_
