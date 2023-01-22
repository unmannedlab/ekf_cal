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
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "infrastructure/Logger.hpp"
#include "ekf/MSCKF.hpp"

///
/// @class EKF
/// @brief Calibration EKF class
/// @todo Implement check for correlation coefficients to be between +/- 1
/// @todo Add gravity initialization check
///
class EKF
{
private:
  static EKF * instancePointer;
  ///
  /// @brief EKF class constructor
  ///
  EKF() {}

public:
  ///
  /// @brief Delete copy constructor
  ///
  EKF(const EKF & obj) = delete;

  ///
  /// @brief Singleton reference getter
  /// @return Pointer to singleton instance
  ///
  static EKF * getInstance()
  {
    // If there is no instance of class
    // then we can create an instance.
    if (instancePointer == NULL) {
      // We can access private members
      // within the class.
      instancePointer = new EKF();

      // returning the instance pointer
      return instancePointer;
    } else {
      // if instancePointer != NULL that means
      // the class already have an instance.
      // So, we are returning that instance
      // and not creating new one.
      return instancePointer;
    }
  }

  ///
  /// @brief Getter for state vector reference
  /// @return State vector reference
  ///
  Eigen::VectorXd & GetState();

  ///
  /// @brief Getter for the body state vector reference
  /// @return Body state vector reference
  ///
  Eigen::VectorXd GetBodyState();

  ///
  /// @brief Getter method for state covariance matrix reference
  /// @return State covariance matrix reference
  ///
  Eigen::MatrixXd & GetCov();

  ///
  /// @brief Getter method for state size
  /// @return State size
  ///
  unsigned int GetStateSize();

  ///
  /// @brief Predict state to given time
  /// @param currentTime Time for prediction
  ///
  void Predict(double currentTime);

  ///
  /// @brief State transition matrix getter method
  /// @param dT State transition time
  /// @return State transition matrix
  ///
  Eigen::MatrixXd GetStateTransition(double dT);

  ///
  /// @brief Process input matrix getter method
  /// @return Process input matrix
  ///
  Eigen::MatrixXd GetProcessInput();

  ///
  /// @brief EKF state initialization method
  /// @param timeInit Initial time
  /// @param bodyStateInit Initial state
  ///
  void Initialize(double timeInit, Eigen::VectorXd bodyStateInit);

  ///
  /// @brief Get transforms between body and sensors
  /// @param baseIMUName Name of base IMU representing body
  /// @param sensorNames Vector of sensor names
  /// @param sensorPosOffsets Vector of sensor positional offsets
  /// @param sensorAngOffsets Vector of sensor rotational offsets
  ///
  void GetTransforms(
    std::string & baseIMUName,
    std::vector<std::string> & sensorNames,
    std::vector<Eigen::Vector3d> & sensorPosOffsets,
    std::vector<Eigen::Quaterniond> & sensorAngOffsets
  );

  ///
  /// @brief Extend EKF state and covariance
  /// @param sensorStateSize Size of state extension
  /// @param sensorState State extension initialization
  /// @param sensorCov Covariance extension initialization
  ///
  void ExtendState(
    unsigned int sensorStateSize, Eigen::VectorXd sensorState,
    Eigen::MatrixXd sensorCov);

  ///
  ///
  ///
  std::shared_ptr<MSCKF> RegisterMSCKF();

private:
  unsigned int m_stateSize{18U};
  Eigen::VectorXd m_state = Eigen::VectorXd::Zero(18U);
  Eigen::MatrixXd m_cov = Eigen::MatrixXd::Identity(18U, 18U);
  double m_currentTime {0};
  bool m_timeInitialized {false};
  Logger * m_logger = Logger::getInstance();

  Eigen::MatrixXd m_processNoise = Eigen::MatrixXd::Identity(18U, 18U) * 1e-3;
  Eigen::MatrixXd m_processInput = Eigen::MatrixXd::Identity(18U, 18U) * 1e-3;

  /// @todo Create map of MSCKF IDs to vector of body states to update
  typedef struct BodyState
  {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
  } BodyState;

  std::vector<std::shared_ptr<MSCKF>> m_msckfVector;
};

#endif  // EKF__EKF_HPP_
