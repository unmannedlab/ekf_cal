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
/// @todo Add map of sensor IDs to start index and state size
///
class EKF
{
private:
  static EKF * instance_pointer;
  ///
  /// @brief EKF class constructor
  ///
  EKF();

public:
  ///
  /// @brief Delete copy constructor
  ///
  EKF(const EKF & obj) = delete;

  ///
  /// @brief Singleton reference getter
  /// @return Pointer to singleton instance
  /// @todo Remove singleton pattern
  ///
  static EKF * GetInstance()
  {
    // If there is no instance of class
    // then we can create an instance.
    if (instance_pointer == NULL) {
      // We can access private members
      // within the class.
      instance_pointer = new EKF();

      // returning the instance pointer
      return instance_pointer;
    } else {
      // if instance_pointer != NULL that means
      // the class already have an instance.
      // So, we are returning that instance
      // and not creating new one.
      return instance_pointer;
    }
  }

  ///
  /// @brief Getter for state vector reference
  /// @return State vector reference
  ///
  State & GetState();

  ///
  /// @brief Getter for the body state vector reference
  /// @return Body state vector reference
  ///
  BodyState GetBodyState();

  ///
  /// @brief Get IMU sensor state
  /// @param imu_id Sensor ID
  /// @return IMU state
  ///
  ImuState GetImuState(unsigned int imu_id);
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
  /// @brief IMU count getter method
  /// @return IMU count
  ///
  unsigned int GetImuStateSize();

  ///
  /// @brief Camera count getter method
  /// @return Camera count
  ///
  unsigned int GetCamCount();

  ///
  /// @brief Getter method for state covariance matrix reference
  /// @return State covariance matrix reference
  ///
  Eigen::MatrixXd & GetCov();

  ///
  /// @brief Check if body data should be logged and do so if necessary
  ///
  void LogBodyStateIfNeeded();

  ///
  /// @brief Process state linearly to given time
  /// @param currentTime Time for prediction
  ///
  void ProcessModel(double currentTime);

  ///
  /// @brief Predict state to given time using IMU measurements
  /// @param time Time of measurement
  /// @param acceleration Acceleration measurement in IMU frame
  /// @param accelerationCovariance Acceleration covariance
  /// @param angularRate Angular rate measurement in IMU frame
  /// @param angularRateCovariance Angular rate covariance
  ///
  void PredictModel(
    double time,
    Eigen::Vector3d acceleration,
    Eigen::Matrix3d accelerationCovariance,
    Eigen::Vector3d angularRate,
    Eigen::Matrix3d angularRateCovariance);

  ///
  /// @brief State transition matrix getter method
  /// @param dT State transition time
  /// @return State transition matrix
  ///
  Eigen::MatrixXd GetStateTransition(double dT);

  ///
  /// @brief Getter for IMU state start index
  /// @param imu_id IMU sensor ID
  ///
  unsigned int GetImuStateStartIndex(unsigned int imu_id);

  ///
  /// @brief Getter for camera state start index
  /// @param cam_id Camera sensor ID
  ///
  unsigned int GetCamStateStartIndex(unsigned int cam_id);

  ///
  /// @brief Getter for augmented state start index
  /// @param cam_id Camera sensor ID
  /// @param frame_id Camera frame ID
  ///
  unsigned int GetAugStateStartIndex(unsigned int cam_id, int frame_id);

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
  /// @brief Camera Registration function
  /// @param cam_id IMU ID
  /// @param cam_state Initial camera state
  /// @param covariance Initial camera covariance
  ///
  void RegisterCamera(unsigned int cam_id, CamState cam_state, Eigen::MatrixXd covariance);

  ///
  /// @brief Generate augmented state Jacobian matrix
  /// @param cam_state_start Camera state start index
  /// @param aug_state_start Augmented state start index
  /// @return Augmented state Jacobian matrix
  ///
  Eigen::MatrixXd AugmentJacobian(
    unsigned int cam_state_start,
    unsigned int aug_state_start);

  ///
  /// @brief Function to augment current state for camera frame
  /// @param camera_id Current camera ID
  /// @param frame_id Current frame ID
  ///
  void AugmentState(unsigned int camera_id, int frame_id);

  ///
  /// @brief Body data logger rate setter
  /// @param rate Body data logging rate
  ///
  void SetBodyDataRate(double rate);

  ///
  /// @brief Function to switch the logger on/off
  /// @param value Logger on/off value
  ///
  void SetDataLogging(bool value);

  ///
  /// @brief Setter for maximum track length
  /// @param max_track_length maximum track length
  ///
  void SetMaxTrackLength(unsigned int max_track_length);

  ///
  /// @brief Function to add process noise to covariance
  ///
  void AddProccessNoise();

  ///
  /// @brief Process noise setter
  /// @param process_noise Process noise diagonal elements
  ///
  void SetProcessNoise(Eigen::VectorXd process_noise);

  ///
  /// @brief Find augmented state matching a camera and frame ID pair
  /// @param camera_id Desired camera ID
  /// @param frame_id Desired frame ID
  /// @return Matching augmented state
  ///
  AugmentedState MatchState(int camera_id, int frame_id);

  DataLogger m_data_logger;  ///< @brief Data logger

private:
  unsigned int m_stateSize{g_body_state_size};
  State m_state;
  Eigen::MatrixXd m_cov = Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size);
  double m_current_time {0};
  bool m_time_initialized {false};
  DebugLogger * m_logger = DebugLogger::GetInstance();
  double m_body_data_rate {0};
  bool m_data_logging_on {false};
  unsigned int m_max_track_length{20U};
  Eigen::MatrixXd m_process_noise =
    Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size) * 1e-9;
};

#endif  // EKF__EKF_HPP_
