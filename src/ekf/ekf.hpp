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
#include <string>
#include <vector>

#include "ekf/types.hpp"
#include "ekf/constants.hpp"
#include "infrastructure/debug_logger.hpp"
#include "infrastructure/data_logger.hpp"

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
  /// @brief Predict state to given time
  /// @param currentTime Time for prediction
  ///
  void ProcessModel(double currentTime);

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
  unsigned int GetAugStateStartIndex(unsigned int cam_id, unsigned int frame_id);

  ///
  /// @brief EKF state initialization method
  /// @param timeInit Initial time
  /// @param bodyStateInit Initial state
  ///
  void Initialize(double timeInit, BodyState bodyStateInit);

  ///
  /// @brief
  /// @param imu_id
  /// @param imu_state
  /// @param covariance
  ///
  void RegisterIMU(unsigned int imu_id, ImuState imu_state, Eigen::MatrixXd covariance);

  ///
  /// @brief
  /// @param cam_id
  /// @param cam_state
  /// @param covariance
  ///
  void RegisterCamera(unsigned int cam_id, CamState cam_state, Eigen::MatrixXd covariance);

  ///
  /// @brief Generate augmented state jacobian matrix
  /// @param cam_state_start Camera state start index
  /// @param camera_id Camera ID
  /// @param aug_state_start Augmented state start index
  /// @return Augmented state jacobian matrix
  ///
  Eigen::MatrixXd AugmentJacobian(
    unsigned int cam_state_start,
    unsigned int camera_id,
    unsigned int aug_state_start);

  ///
  /// @brief
  /// @param camera_id
  /// @param frame_id
  ///
  void AugmentState(unsigned int camera_id, unsigned int frame_id);

  ///
  ///
  ///
  void SetBodyDataRate(double rate);

  ///
  /// @brief Function to switch the logger on/off
  /// @param value Logger on/off value
  ///
  void SetDataLogging(bool value);

  DataLogger m_data_logger;

private:
  unsigned int m_stateSize{g_body_state_size};
  State m_state;
  Eigen::MatrixXd m_cov = Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size);
  double m_current_time {0};
  bool m_time_initialized {false};
  DebugLogger * m_logger = DebugLogger::GetInstance();
  double m_body_data_rate {0};
  double m_prev_log_time {0};
  bool m_data_logging_on {false};

  Eigen::MatrixXd m_process_noise =
    Eigen::MatrixXd::Identity(g_body_state_size, g_body_state_size) * 1e-3;
};

#endif  // EKF__EKF_HPP_
