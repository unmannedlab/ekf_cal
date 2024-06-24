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

#ifndef EKF__EKF_HPP_
#define EKF__EKF_HPP_

#include <eigen3/Eigen/Eigen>
#include <stddef.h>

#include <memory>
#include <string>

#include "ekf/constants.hpp"
#include "ekf/types.hpp"
#include "infrastructure/data_logger.hpp"
#include "infrastructure/debug_logger.hpp"

///
/// @class EKF
/// @brief Calibration EKF class
/// @todo Implement check for correlation coefficients to be between +/- 1
/// @todo Add gravity initialization/check
/// @todo Create generic function to update(r,H,R)
///
class EKF
{
public:
  ///
  /// @brief EKF class parameters
  ///
  typedef struct Parameters
  {
    std::shared_ptr<DebugLogger> debug_logger;                  ///< @brief
    double body_data_rate{1.0};                                 ///< @brief
    bool data_logging_on{false};                                ///< @brief
    std::string log_directory{""};                              ///< @brief
    AugmentationType augmenting_type{AugmentationType::ALL};    ///< @brief
    double augmenting_time{1.0};                                ///< @brief
    double augmenting_pos_error{0.1};                           ///< @brief
    double augmenting_ang_error{0.1};                           ///< @brief
    Eigen::VectorXd process_noise {Eigen::VectorXd::Ones(18)};  ///< @brief
  } Parameters;

  ///
  /// @brief EKF class constructor
  /// @param params EKF class parameters
  ///
  explicit EKF(Parameters params);

  ///
  /// @brief Getter for state size
  /// @return State size
  ///
  unsigned int GetStateSize();

  ///
  /// @brief Get IMU sensor state
  /// @param imu_id Sensor ID
  /// @return IMU state
  ///
  ImuState GetImuState(unsigned int imu_id);

  ///
  /// @brief Get GPS sensor state
  /// @param gps_id Sensor ID
  /// @return GPS state
  ///
  GpsState GetGpsState(unsigned int gps_id);
  ///
  /// @brief Get camera sensor state
  /// @param cam_id Sensor ID
  /// @return Camera state
  ///
  CamState GetCamState(unsigned int cam_id);

  ///
  /// @brief IMU count getter method
  /// @return IMU count
  ///
  unsigned int GetImuCount();

  ///
  /// @brief IMU state size getter method
  /// @return IMU state size
  ///
  unsigned int GetImuStateSize();

  ///
  /// @brief IMU count getter method
  /// @return IMU count
  ///
  unsigned int GetGpsCount();

  ///
  /// @brief GPS state size getter method
  /// @return GPS state size
  ///
  unsigned int GetGpsStateSize();

  ///
  /// @brief Camera state size getter method
  /// @return Camera state size
  ///
  unsigned int GetCamStateSize();

  ///
  /// @brief Camera count getter method
  /// @return Camera count
  ///
  unsigned int GetCamCount();

  ///
  /// @brief Check if body data should be logged and do so if necessary
  /// @param execution_count
  ///
  void LogBodyStateIfNeeded(int execution_count);

  ///
  /// @brief Process state linearly to given time
  /// @param time Time for prediction
  ///
  void ProcessModel(double time);

  ///
  /// @brief Predict state to given time using IMU measurements
  /// @param time Time of measurement
  /// @param acceleration Acceleration measurement in IMU frame
  /// @param angularRate Angular rate measurement in IMU frame
  ///
  void PredictModel(
    double time,
    Eigen::Vector3d acceleration,
    Eigen::Vector3d angularRate);

  ///
  /// @brief State transition matrix getter method
  /// @param dT State transition time
  /// @return State transition matrix
  ///
  Eigen::MatrixXd GetStateTransition(double dT);

  ///
  /// @brief EKF state initialization method
  /// @param timeInit Initial time
  /// @param bodyStateInit Initial state
  ///
  void Initialize(double timeInit, BodyState bodyStateInit);

  ///
  /// @brief IMU Registration function
  /// @param imu_id IMU ID
  /// @param imu_state Initial IMU state
  /// @param covariance Initial IMU covariance
  ///
  void RegisterIMU(unsigned int imu_id, ImuState imu_state, Eigen::MatrixXd covariance);

  ///
  /// @brief GPS Registration function
  /// @param gps_id GPS ID
  /// @param gps_state Initial GPS state
  /// @param covariance Initial GPS covariance
  ///
  void RegisterGPS(unsigned int gps_id, GpsState gps_state, Eigen::Matrix3d covariance);

  ///
  /// @brief Camera Registration function
  /// @param cam_id IMU ID
  /// @param cam_state Initial camera state
  /// @param covariance Initial camera covariance
  ///
  void RegisterCamera(unsigned int cam_id, CamState cam_state, Eigen::MatrixXd covariance);

  ///
  /// @brief Fiducial Registration function
  /// @param fid_id Fiducial ID
  /// @param fid_state Initial fiducial state
  /// @param covariance Initial fiducial covariance
  ///
  void RegisterFiducial(unsigned int fid_id, FidState fid_state, Eigen::MatrixXd covariance);

  ///
  /// @brief Generate augmented state Jacobian matrix
  /// @param cam_index Camera state start index
  /// @param aug_index Augmented state start index
  /// @return Augmented state Jacobian matrix
  ///
  Eigen::MatrixXd AugmentJacobian(
    unsigned int cam_index,
    unsigned int aug_index);

  ///
  /// @brief Function to augment current state for camera frame
  /// @param camera_id Current camera ID
  /// @param frame_id Current frame ID
  ///
  void AugmentStateIfNeeded(unsigned int camera_id, int frame_id);

  ///
  /// @brief Setter for maximum track length
  /// @param max_track_length maximum track length
  ///
  void SetMaxTrackLength(unsigned int max_track_length);

  ///
  /// @brief Function to add process noise to covariance
  /// @param delta_time delta time over which to add process noise
  ///
  void AddProccessNoise(double delta_time);

  ///
  /// @brief Apply reasonable limits to uncertainties
  ///
  void LimitUncertainty();

  ///
  /// @brief EKF process noise setter
  /// @param process_noise Diagonal terms of process noise
  ///
  void SetProcessNoise(Eigen::VectorXd process_noise);

  ///
  /// @brief GPS reference position setter
  /// @param reference_lla
  /// @param ang_l_to_g
  ///
  void SetGpsReference(Eigen::VectorXd reference_lla, double ang_l_to_g);

  ///
  /// @brief Getter for the LLA reference position
  /// @return Reference LLA position
  ///
  Eigen::VectorXd GetReferenceLLA();

  ///
  /// @brief Getter for the LLA reference frame heading
  /// @return Reference frame heading
  ///
  double GetReferenceAngle();

  ///
  /// @brief Checks if the LLA reference frame has been initialized
  /// @return LLA initialization boolean
  ///
  bool IsLlaInitialized();

  ///
  /// @brief Find augmented state matching a camera and frame ID pair
  /// @param camera_id Desired camera ID
  /// @param frame_id Desired frame ID
  /// @return Matching augmented state
  ///
  AugState GetAugState(int camera_id, int frame_id);

  ///
  /// @brief Get augmented state size
  /// @return Augmented state size
  ///
  unsigned int GetAugStateSize();

  ///
  /// @brief Refresh the sub-state indices
  ///
  void RefreshIndices();

  /// @brief EKF state
  State m_state;

  /// @brief EKF covariance
  Eigen::MatrixXd m_cov = Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size) * 1e-6;

private:
  unsigned int m_state_size{g_body_state_size};
  unsigned int m_imu_state_size{0};
  unsigned int m_gps_state_size{0};
  unsigned int m_cam_state_size{0};
  unsigned int m_aug_state_size{0};
  unsigned int m_fid_state_size{0};
  double m_current_time {0};
  bool m_time_initialized {false};
  std::shared_ptr<DebugLogger> m_debug_logger;
  bool m_data_logging_on;
  unsigned int m_max_track_length{20};
  Eigen::MatrixXd m_process_noise;
  DataLogger m_data_logger;

  bool m_is_lla_initialized{false};
  Eigen::Vector3d m_reference_lla{0, 0, 0};
  double m_ang_l_to_g{0};
};

#endif  // EKF__EKF_HPP_
