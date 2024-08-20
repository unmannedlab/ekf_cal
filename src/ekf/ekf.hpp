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
#include <vector>

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
    std::shared_ptr<DebugLogger> debug_logger;                  ///< @brief Debug logger
    double body_data_rate{1.0};                                 ///< @brief Body data log rate
    bool data_logging_on{false};                                ///< @brief Data logging flag
    std::string log_directory{""};                              ///< @brief Data log directory
    AugmentationType augmenting_type{AugmentationType::ALL};    ///< @brief Augmenting type
    double augmenting_delta_time{1.0};                          ///< @brief Augmenting time
    double augmenting_pos_error{0.1};                           ///< @brief Augmenting pos error
    double augmenting_ang_error{0.1};                           ///< @brief Augmenting ang error
    Eigen::VectorXd process_noise {Eigen::VectorXd::Ones(18)};  ///< @brief Process noise
    Eigen::Vector3d pos_b_in_l{Eigen::Vector3d::Zero()};        ///< @brief Body local position
    Eigen::Quaterniond ang_b_to_l {1, 0, 0, 0};                 ///< @brief Body local orientation
    Eigen::Vector3d pos_l_in_g {Eigen::Vector3d::Zero()};       ///< @brief Local frame position
    double ang_l_to_g{0.0};                                     ///< @brief Local frame heading
    GpsInitType gps_init_type {GpsInitType::CONSTANT};          ///< @brief GPS initialization type
    double gps_init_baseline_dist {100.0};  ///< @brief Minimum pos projection error
    double gps_init_pos_thresh {0.1};       ///< @brief Minimum ang projection error
    double gps_init_ang_thresh {0.1};       ///< @brief Baseline distance threshold
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
  /// @param fid_state Initial fiducial state
  /// @param covariance Initial fiducial covariance
  ///
  void RegisterFiducial(FidState fid_state, Eigen::MatrixXd covariance);

  ///
  /// @brief Augment the state covariance matrix
  /// @param in_cov Original state covariance matrix
  /// @param index Augmented state start index
  /// @return Augmented state covariance matrix
  ///
  Eigen::MatrixXd AugmentCovariance(Eigen::MatrixXd in_cov, unsigned int index);

  ///
  /// @brief Check if state should be augmented using current state
  ///
  void AugmentStateIfNeeded();

  ///
  /// @brief Check if state should be augmented using camera frame
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
  /// @brief Find or interpolate augmented state
  /// @param camera_id Desired camera ID
  /// @param frame_id Desired frame ID
  /// @param time Frame time
  /// @return Augmented state
  ///
  AugState GetAugState(int camera_id, int frame_id, double time);

  ///
  /// @brief Get augmented state size
  /// @return Augmented state size
  ///
  unsigned int GetAugStateSize();

  ///
  /// @brief Refresh the sub-state indices
  ///
  void RefreshIndices();

  ///
  /// @brief GPS LLA to ENU Initialization Routine
  /// @param time GPS measured time
  /// @param gps_lla GPS measured lat-lon-alt
  ///
  void AttemptGpsInitialization(
    double time,
    Eigen::Vector3d gps_lla);

  ///
  /// @brief GPS time vector getter
  /// @return GPS time vector
  ///
  std::vector<double> GetGpsTimeVector();

  ///
  /// @brief GPS ECEF vector getter
  /// @return GPS ECEF vector
  ///
  std::vector<Eigen::Vector3d> GetGpsEcefVector();

  ///
  /// @brief GPS XYZ vector getter
  /// @return GPS XYZ vector
  ///
  std::vector<Eigen::Vector3d> GetGpsXyzVector();

  ///
  /// @brief Current time getter
  /// @return Current time
  ///
  double GetCurrentTime();

  ///
  /// @brief IMU state start getter
  /// @return IMU state start
  ///
  unsigned int get_imu_state_start();

  ///
  /// @brief GPS state start getter
  /// @return GPS state start
  ///
  unsigned int get_gps_state_start();

  ///
  /// @brief Camera state start getter
  /// @return Camera state start
  ///
  unsigned int get_cam_state_start();

  ///
  /// @brief Augmented states start getter
  /// @return Augmented states start
  ///
  unsigned int get_aug_state_start();

  ///
  /// @brief Fiducial state start getter
  /// @return Fiducial state start
  ///
  unsigned int get_fid_state_start();

  bool IsGravityInitialized();

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
  Eigen::MatrixXd m_process_noise {Eigen::MatrixXd::Zero(18, 18)};
  DataLogger m_data_logger;

  GpsInitType m_gps_init_type;
  double m_gps_init_pos_thresh;
  double m_gps_init_ang_thresh;
  double m_gps_init_baseline_dist;
  bool m_is_lla_initialized;
  Eigen::Vector3d m_pos_l_in_g;
  double m_ang_l_to_g;
  std::vector<double> m_gps_time_vec;
  std::vector<Eigen::Vector3d> m_gps_ecef_vec;
  std::vector<Eigen::Vector3d> m_gps_xyz_vec;

  AugmentationType m_augmenting_type;
  double m_augmenting_prev_time;
  double m_augmenting_delta_time;
  double m_augmenting_pos_error;
  double m_augmenting_ang_error;
  double m_primary_camera_id;
  double m_max_frame_period {0.0};
  double m_max_track_duration {0.0};
  double m_min_aug_period {1.0};

  unsigned int m_imu_state_start{0};
  unsigned int m_gps_state_start{0};
  unsigned int m_cam_state_start{0};
  unsigned int m_aug_state_start{0};
  unsigned int m_fid_state_start{0};

  bool m_is_gravity_initialized{false};
};

#endif  // EKF__EKF_HPP_
